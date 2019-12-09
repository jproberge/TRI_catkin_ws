/*
 * This is part of the Bayesian Object Tracking (bot),
 * (https://github.com/bayesian-object-tracking)
 *
 * Copyright (c) 2015 Max Planck Society,
 * 				 Autonomous Motion Department,
 * 			     Institute for Intelligent Systems
 *
 * This Source Code Form is subject to the terms of the GNU General Public
 * License License (GNU GPL). A copy of the license can be found in the LICENSE
 * file distributed with this source code.
 */

/*
 * This file implements a part of the algorithm published in:
 *
 * M. Wuthrich, P. Pastor, M. Kalakrishnan, J. Bohg, and S. Schaal.
 * Probabilistic Object Tracking using a Range Camera
 * IEEE Intl Conf on Intelligent Robots and Systems, 2013
 * http://arxiv.org/abs/1505.00241
 *
 */

/**
 * \file cuda_likelihood_evaluator.cu
 * \author Claudia Pfreundt (claudilein@gmail.com)
 * \date November 2015
 */

#define DEBUG

#define VECTOR_DIM 3
#define MATRIX_DIM 9

#include <GL/glut.h>
#include <fl/util/profiling.hpp>
#include <dbot/gpu/cuda_likelihood_evaluator.h>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>


#include <cuda.h>
#include <cuda_gl_interop.h>
#include <math.h>
#include <math_constants.h>


using namespace std;

// ====================== CUDA CONSTANT VALUES ======================= //


// used in propagateOcclusion
__constant__ float g_p_occluded_occluded;
__constant__ float g_one_div_c_minus_one;
__constant__ float g_log_c;


// used in prob
__constant__ float g_one_minus_tail_weight;
__constant__ float g_model_sigma;
__constant__ float g_sigma_factor;
__constant__ float g_tail_weight_div_max_depth;
__constant__ float g_exponential_rate;
__constant__ float g_one_div_sqrt_of_two;
__constant__ float g_one_div_sqrt_of_two_pi;


// used in compare
__constant__ float g_initial_occlusion_prob;

// texture for OpenGL interop
texture<float, cudaTextureType2D, cudaReadModeElementType> texture_reference;





// ************************************************************************************** //
// ************************************************************************************** //
// ================================== CUDA KERNELS ====================================== //
// ************************************************************************************** //
// ************************************************************************************** //

// ============================================================================================= //
// ====================== DEVICE kernels - to be called by other kernels ======================= //
// ============================================================================================= //


// ======================= helper functions for compare (observation model)  ======================= //


__device__ float propagate_occlusion(float initial_p_source, float time) {
    if (isnan(time)) {
        return initial_p_source;
    }
    float pow_c_time = __expf(time * g_log_c);
    return 1 - (pow_c_time * (1 - initial_p_source) + (1. - g_p_occluded_occluded) * (pow_c_time - 1.) * g_one_div_c_minus_one);
}



__device__ float prob(float observation, float prediction, bool occluded)
{
    // todo: if the prediction is infinite, the prob should not depend on occlusion. it does not matter
    // for the algorithm right now, but it should be changed

    float sigma = g_model_sigma + g_sigma_factor * observation * observation;
    float sigma_sq = sigma * sigma;

    if(!occluded)
    {
        if(isinf(prediction)) // if the prediction is infinite we return the limit
            return g_tail_weight_div_max_depth;
        else {
            float pred_minus_obs = prediction - observation;
            return g_tail_weight_div_max_depth
                    + __fdividef(g_one_minus_tail_weight * __expf(- __fdividef(pred_minus_obs * pred_minus_obs, (2 * sigma_sq)))
                    * g_one_div_sqrt_of_two_pi, sigma);
        }
    }
    else
    {
        if(isinf(prediction)) // if the prediction is infinite we return the limit
            return g_tail_weight_div_max_depth +
                    g_one_minus_tail_weight * g_exponential_rate *
                    __expf(0.5 * g_exponential_rate * (-2 * observation + g_exponential_rate * sigma_sq));

        else
            return g_tail_weight_div_max_depth +
                    g_one_minus_tail_weight * g_exponential_rate *
                    __expf(0.5 * g_exponential_rate * (2 * (prediction - observation) + g_exponential_rate * sigma_sq))
                    * __fdividef((1 + erff(__fdividef((prediction - observation + g_exponential_rate * sigma_sq) * g_one_div_sqrt_of_two, sigma))),
                    (2 * (__expf(prediction * g_exponential_rate) - 1)));
    }
}




// ============================================================================================= //
// ========================= GLOBAL kernels - to be called by CPU code ========================= //
// ============================================================================================= //



__global__ void evaluate_kernel(float *observations, float* old_occlusion_probs, float* new_occlusion_probs, int* occlusion_image_indices, int nr_pixels,
                                 float *d_log_likelihoods, float delta_time, int n_poses, int n_rows, int n_cols, bool update_occlusions) {
    int block_id = blockIdx.x + blockIdx.y * gridDim.x;
    if (block_id < n_poses) {

        int pixel_nr = threadIdx.x;

        // OpenGL contructs the texture so that the left lower edge is (0,0), but our observations texture
        // has its (0,0) in the upper left corner, so we need to reverse the reads from the OpenGL texture.
        float texture_array_index_x = blockIdx.x * n_cols + pixel_nr % n_cols;

        float texture_array_index_y = gridDim.y * n_rows - 1 - (blockIdx.y * n_rows + __fdividef(pixel_nr, n_cols));

        float depth;
        float observed_depth;
        float occlusion_prob = g_initial_occlusion_prob;
        float local_sum_of_likelihoods = 0;
        float p_obsIpred_vis, p_obsIpred_occl, p_obsIinf;

        __shared__ float log_likelihoods;
        __shared__ int occlusion_image_index;

        if (threadIdx.x == 0) {
            log_likelihoods = 0;
            occlusion_image_index = occlusion_image_indices[block_id];
        }

        __syncthreads();

        float* occlusion_probs = old_occlusion_probs;
        int occlusion_pixel_index= occlusion_image_index * nr_pixels + pixel_nr;

        if (update_occlusions) {
            // copy occlusion probabilities from the old particles
            int index_from_occlusion = occlusion_image_indices[block_id] * nr_pixels;
            int index_to_occlusion = block_id * nr_pixels;

            while (pixel_nr < nr_pixels) {
                new_occlusion_probs[index_to_occlusion + pixel_nr] = old_occlusion_probs[index_from_occlusion + pixel_nr];
                pixel_nr += blockDim.x;
            }

            // change occlusion prob array to the new one and change the global index
            occlusion_probs = new_occlusion_probs;
            // reset pixel_nr for following loop
            pixel_nr = threadIdx.x;

            occlusion_pixel_index= block_id * nr_pixels + pixel_nr;
        }


        while (pixel_nr < nr_pixels ) {

            depth = tex2D(texture_reference, texture_array_index_x, texture_array_index_y);
            observed_depth = observations[pixel_nr];

            occlusion_prob = propagate_occlusion(occlusion_probs[occlusion_pixel_index], delta_time);
            if (update_occlusions) occlusion_probs[occlusion_pixel_index] = occlusion_prob;


            if (depth != 0 && !isnan(observed_depth)) {

                // prob of observation given prediction, knowing that the object is not occluded
                p_obsIpred_vis = prob(observed_depth, depth, false) * (1 - occlusion_prob);
                // prob of observation given prediction, knowing that the object is occluded
                p_obsIpred_occl = prob(observed_depth, depth, true) * occlusion_prob;
                // prob of observation given no intersection
                p_obsIinf = prob(observed_depth, CUDART_INF_F, true);

                local_sum_of_likelihoods += __logf(__fdividef((p_obsIpred_vis + p_obsIpred_occl), p_obsIinf));


                if(update_occlusions) {
                    // we update the occlusion probability with the observations
                    occlusion_probs[occlusion_pixel_index] = 1 - __fdividef(p_obsIpred_vis, (p_obsIpred_vis + p_obsIpred_occl));
                }
            }

            pixel_nr += blockDim.x;
            occlusion_pixel_index += blockDim.x;
            texture_array_index_x = blockIdx.x * n_cols + pixel_nr % n_cols;
            texture_array_index_y = gridDim.y * n_rows - (blockIdx.y * n_rows + (pixel_nr / n_cols) + 1);
        }

        atomicAdd(&log_likelihoods, local_sum_of_likelihoods);

        __syncthreads();

        if (threadIdx.x == 0) {
            d_log_likelihoods[block_id] = log_likelihoods;
        }
    } else {
        __syncthreads();
    }

}






// ************************************************************************************** //
// ************************************************************************************** //
// ========================== cuda_likelihood_evaluator MEMBER FUNCTIONS ============================== //
// ************************************************************************************** //
// ************************************************************************************** //


CudaEvaluator::CudaEvaluator(const int nr_rows,
                       const int nr_cols) :

    nr_rows_(nr_rows),
    nr_cols_(nr_cols)
{

    cudaDeviceProp  props;
    int device_number;

    memset( &props, 0, sizeof( cudaDeviceProp ) );
    props.major = 2;
    props.minor = 0;
    cudaChooseDevice( &device_number, &props );
    #ifdef DEBUG
        check_cuda_error("No device with compute capability > 2.0 found");
    #endif

    /* tell CUDA which device we will be using for graphic interop.
     * Requires that the CUDA device be specified by
     * cudaGLSetGLDevice() before any other runtime calls. */

//    cudaGLSetGLDevice( device_number );
    #ifdef DEBUG
        check_cuda_error("cudaGLsetGLDevice");
    #endif

    cudaGetDeviceProperties(&props, device_number);     // we will run the program only on one graphics card, the first one we can find = 0

    cuda_device_properties_ = props;

    #ifdef DEBUG
        cout << "Your device has the following properties: " << endl
             << "CUDA Version: " << props.major << "." << props.minor << endl
             << "Number of multiprocessors: " << props.multiProcessorCount << endl;
    #endif

    /* each multiprocessor has various KB of memory (64 KB for the GTX 560 Ti 448) which can be subdivided
     * into L1 cache or shared memory. If you don't need a lot of shared memory set this to prefer L1. */
    cudaDeviceSetCacheConfig(cudaFuncCachePreferL1);


    d_occlusion_probs_ = NULL;
    d_occlusion_probs_copy_ = NULL;
    d_observations_ = NULL;
    d_log_likelihoods_ = NULL;
    d_occlusion_indices_ = NULL;

    set_nr_threads(DEFAULT_NR_THREADS);
}



void CudaEvaluator::init(const float initial_occlusion_prob, const float p_occluded_occluded, const float p_occluded_visible,
                      const float tail_weight, const float model_sigma, const float sigma_factor, const float max_depth, const float exponential_rate) {

    occlusion_time_ = 0;
    occlusion_prob_default_ = initial_occlusion_prob;

    // precompute constants that are used in high-performance kernels later
    float c = p_occluded_occluded - p_occluded_visible;
    float tail_weight_div_max_depth = tail_weight / max_depth;
    float one_minus_tail_weight = 1.0f - tail_weight;
    float one_div_c_minus_one = 1.0f / (c - 1.0f);
    float one_div_sqrt_of_two = 1.0f / sqrt(2);
    float one_div_sqrt_of_two_pi = 1.0f / sqrt(2 * M_PI);
    float log_c = log(c);



    // copy constants to GPU memory
    cudaMemcpyToSymbol(g_initial_occlusion_prob, &initial_occlusion_prob, sizeof(float), 0, cudaMemcpyHostToDevice);
    #ifdef DEBUG
        check_cuda_error("cudaMemcpyToSymbol initial_occlusion_prob -> g_initial_occlusion_prob");
    #endif

    cudaMemcpyToSymbol(g_one_div_c_minus_one, &one_div_c_minus_one, sizeof(float), 0, cudaMemcpyHostToDevice);
    #ifdef DEBUG
        check_cuda_error("cudaMemcpyToSymbol one_div_c_minus_one -> g_one_div_c_minus_one");
    #endif

    cudaMemcpyToSymbol(g_one_div_sqrt_of_two, &one_div_sqrt_of_two, sizeof(float), 0, cudaMemcpyHostToDevice);
    #ifdef DEBUG
        check_cuda_error("cudaMemcpyToSymbol one_div_sqrt_of_two -> g_one_div_sqrt_of_two");
    #endif

    cudaMemcpyToSymbol(g_one_div_sqrt_of_two_pi, &one_div_sqrt_of_two_pi, sizeof(float), 0, cudaMemcpyHostToDevice);
    #ifdef DEBUG
        check_cuda_error("cudaMemcpyToSymbol one_div_sqrt_of_two_pi -> g_one_div_sqrt_of_two_pi");
    #endif

    cudaMemcpyToSymbol(g_log_c, &log_c, sizeof(float), 0, cudaMemcpyHostToDevice);
    #ifdef DEBUG
        check_cuda_error("cudaMemcpyToSymbol log_c -> g_log_c");
    #endif

    cudaMemcpyToSymbol(g_p_occluded_occluded, &p_occluded_occluded, sizeof(float), 0, cudaMemcpyHostToDevice);
    #ifdef DEBUG
        check_cuda_error("cudaMemcpyToSymbol p_occluded_occluded -> g_p_occluded_occluded");
    #endif

    cudaMemcpyToSymbol(g_one_minus_tail_weight, &one_minus_tail_weight, sizeof(float), 0, cudaMemcpyHostToDevice);
    #ifdef DEBUG
        check_cuda_error("cudaMemcpyToSymbol one_minus_tail_weight -> g_one_minus_tail_weight");
    #endif

    cudaMemcpyToSymbol(g_model_sigma, &model_sigma, sizeof(float), 0, cudaMemcpyHostToDevice);
    #ifdef DEBUG
        check_cuda_error("cudaMemcpyToSymbol model_sigma -> g_model_sigma");
    #endif

    cudaMemcpyToSymbol(g_sigma_factor, &sigma_factor, sizeof(float), 0, cudaMemcpyHostToDevice);
    #ifdef DEBUG
        check_cuda_error("cudaMemcpyToSymbol sigma_factor -> g_sigma_factor");
    #endif

    cudaMemcpyToSymbol(g_tail_weight_div_max_depth, &tail_weight_div_max_depth, sizeof(float), 0, cudaMemcpyHostToDevice);
    #ifdef DEBUG
        check_cuda_error("cudaMemcpyToSymbol tail_weight_div_max_depth -> g_tail_weight_div_max_depth");
    #endif

    cudaMemcpyToSymbol(g_exponential_rate, &exponential_rate, sizeof(float), 0, cudaMemcpyHostToDevice);
    #ifdef DEBUG
        check_cuda_error("cudaMemcpyToSymbol exponential_rate -> g_exponential_rate");
    #endif


    constants_initialized_ = true;
}





void CudaEvaluator::weigh_poses(const bool update_occlusions, vector<float> &log_likelihoods) {
    if (observations_set_ && occlusion_indices_set_
            && memory_allocated_ && number_of_poses_set_ && constants_initialized_
            && texture_array_mapped_) {

        double delta_time = observation_time_ - occlusion_time_;
        if(update_occlusions) occlusion_time_ = observation_time_;


        evaluate_kernel <<< grid_dimension_, nr_threads_ >>> (d_observations_, d_occlusion_probs_, d_occlusion_probs_copy_, d_occlusion_indices_, nr_cols_ * nr_rows_,
                                               d_log_likelihoods_, delta_time, nr_poses_, nr_rows_, nr_cols_, update_occlusions);
        #ifdef DEBUG
            check_cuda_error("compare kernel call");
        #endif

        cudaDeviceSynchronize();
        #ifdef DEBUG
            check_cuda_error("cudaDeviceSynchronize compare_multiple");
        #endif

        // switch to new / copied occlusion probabilities
        if (update_occlusions) {
            float *tmp_pointer;
            tmp_pointer = d_occlusion_probs_;
            d_occlusion_probs_ = d_occlusion_probs_copy_;
            d_occlusion_probs_copy_ = tmp_pointer;
        }


        cudaMemcpy(&log_likelihoods[0], d_log_likelihoods_, nr_poses_ * sizeof(float), cudaMemcpyDeviceToHost);
        #ifdef DEBUG
            check_cuda_error("cudaMemcpy d_log_likelihoods -> log_likelihoods");
        #endif

        cudaDeviceSynchronize();
        #ifdef DEBUG
            check_cuda_error("cudaDeviceSynchronize compare");
        #endif
    } else {
        std::cout << "WARNING (CUDA): It seems you forgot to do one of the following: set observation image, set occlusion"
                  << " indices, set number of poses, allocate memory, map texture to texture array or inisitialize constants." << std::endl;
    }

}







// ===================================================================================== //
// =============================== CUDA EVALUATOR SETTERS ================================= //
// ===================================================================================== //

void CudaEvaluator::set_nr_threads(const int nr_threads) {
    if (nr_threads > cuda_device_properties_.maxThreadsDim[0]) {
        std::cout << "ERROR (CUDA): The number of threads you requested ("
                  << nr_threads << ") is not supported my CUDA. Max #threads: "
                  << cuda_device_properties_.maxThreadsDim[0] << std::endl;
        exit(-1);
    }

    nr_threads_ = nr_threads;
}



void CudaEvaluator::set_observations(const float* observations, const float observation_time) {

    if (nr_rows_ * nr_cols_ > observations_size_) {
        std::cout << "ERROR (CUDA) in set_observations: You exceeded "
                  << "(" << nr_rows_ * nr_cols_ << ")"
                  << "the memory space that was allocated for the observation "
                  << "values (" << observations_size_ << ")." << std::endl;
        exit(-1);
    }

    observation_time_ = observation_time;

    cudaMemcpy(d_observations_, observations, nr_cols_ * nr_rows_ * sizeof(float), cudaMemcpyHostToDevice);
    #ifdef DEBUG
        check_cuda_error("cudaMemcpy observations -> d_observations_");
    #endif
    cudaDeviceSynchronize();
    #ifdef DEBUG
        check_cuda_error("cudaDeviceSynchronize set_observations");
    #endif

    observations_set_ = true;
}



void CudaEvaluator::set_occlusion_indices(const int* occlusion_indices,
                                          const int array_size) {

    if (array_size > max_nr_poses_) {
        std::cout << "ERROR (CUDA): The amount of occlusion indices sent to "
                  << "the GPU (" << array_size << ") exceeds the amount of "
                  << "maximum poses (" << max_nr_poses_ << ")." << std::endl;
        exit(-1);
    }

    cudaMemcpy(d_occlusion_indices_, occlusion_indices,
               array_size * sizeof(int), cudaMemcpyHostToDevice);

    #ifdef DEBUG
        check_cuda_error("cudaMemcpy occlusion_indices -> d_occlusion_indices");
    #endif
    cudaDeviceSynchronize();
    #ifdef DEBUG
        check_cuda_error("cudaDeviceSynchronize set_occlusion_indices");
    #endif

    occlusion_indices_set_ = true;
}


void CudaEvaluator::set_resolution(const int nr_rows, const int nr_cols) {

    nr_rows_ = nr_rows;
    nr_cols_ = nr_cols;

}


void CudaEvaluator::set_occlusion_probabilities(const float* occlusion_probabilities,
                                                const int array_size) {

    if (array_size > occlusion_probs_size_) {
        std::cout << "ERROR (CUDA) in set_occlusion_probabilities: You exceeded "
                  << "(" << array_size << ")"
                  << "the memory space that was allocated for the occlusion "
                  << "probabilities (" << occlusion_probs_size_ << "." << std::endl;
        exit(-1);
    }

    cudaMemcpy(d_occlusion_probs_, occlusion_probabilities,
               array_size * sizeof(float), cudaMemcpyHostToDevice);

    #ifdef DEBUG
        check_cuda_error("cudaMemcpy occlusion_probabilities -> d_occlusion_probs_");
    #endif
    cudaDeviceSynchronize();
    #ifdef DEBUG
        check_cuda_error("cudaDeviceSynchronize set_occlusion_probabilities");
    #endif
}


void CudaEvaluator::map_texture_to_texture_array(const cudaArray_t texture_array) {

    d_texture_array_ = texture_array;
    cudaBindTextureToArray(texture_reference, d_texture_array_);

    #ifdef DEBUG
        check_cuda_error("cudaBindTextureToArray");
    #endif

    texture_array_mapped_ = true;
}


void CudaEvaluator::allocate_memory_for_max_poses(int nr_poses,
                                                  int nr_poses_per_row,
                                                  int nr_poses_per_col) {
    if (constants_initialized_) {

        // check limitation by global memory size
        int constant_need, per_pose_need;
        get_memory_need_parameters(nr_rows_, nr_cols_,
                                   constant_need, per_pose_need);
        int memory_needs = constant_need + nr_poses * per_pose_need;

        if (memory_needs > cuda_device_properties_.totalGlobalMem) {
            std::cout << "ERROR (CUDA): Not enough memory to allocate " << nr_poses
                      << " poses." << std::endl;
            exit(-1);
        }

        // check limitation by texture and grid size
        if (nr_poses_per_row * nr_cols_ > cuda_device_properties_.maxTexture2D[0] ||
            nr_poses_per_col * nr_rows_ > cuda_device_properties_.maxTexture2D[1] ||
            nr_poses_per_row > cuda_device_properties_.maxGridSize[0] ||
            nr_poses_per_col > cuda_device_properties_.maxGridSize[1]) {
            std::cout << "ERROR (CUDA): Exceeding maximum texture or grid size with"
                      << nr_poses_per_row << " x " << nr_poses_per_col << " poses"
                      << " at resolution " << nr_rows_ << " x " << nr_cols_ << std::endl;
            exit(-1);
        }

        max_nr_poses_ = nr_poses;
        max_nr_poses_per_row_ = nr_poses_per_row;
        max_nr_poses_per_column_ = nr_poses_per_col;

        grid_dimension_ = dim3(nr_poses_per_row, nr_poses_per_col);


        // reallocate arrays
        allocate(d_log_likelihoods_, sizeof(float) * max_nr_poses_);
        allocate(d_occlusion_indices_, sizeof(int) * max_nr_poses_);
        occlusion_probs_size_ = nr_rows_ * nr_cols_ * max_nr_poses_;
        allocate(d_occlusion_probs_, occlusion_probs_size_ * sizeof(float));
        allocate(d_occlusion_probs_copy_, occlusion_probs_size_ * sizeof(float));
        observations_size_ = nr_rows_ * nr_cols_;
        allocate(d_observations_, observations_size_ * sizeof(float));

        vector<float> initial_occlusion_probs (nr_rows_ * nr_cols_ * max_nr_poses_,
                                               occlusion_prob_default_);

        cudaMemcpy(d_occlusion_probs_, &initial_occlusion_probs[0], occlusion_probs_size_ * sizeof(float), cudaMemcpyHostToDevice);
        #ifdef DEBUG
            check_cuda_error("cudaMemcpy occlusion_prob_default_ -> d_occlusion_probs_");
        #endif

        // initialize log likelihoods with 0
        cudaMemset(d_log_likelihoods_, 0, sizeof(float) * max_nr_poses_);
        #ifdef DEBUG
            check_cuda_error("cudaMemset d_log_likelihoods");
        #endif

        cudaDeviceSynchronize();
        #ifdef DEBUG
            check_cuda_error("cudaDeviceSynchronize allocate_memory_for_max_poses");
        #endif

        memory_allocated_ = true;
    } else {
        std::cout << "WARNING (CUDA): It seems you forgot to call init() to "
                  << "initialize the constants before calling "
                  << "allocate_memory_for_max_poses" << std::endl;
    }
}


void CudaEvaluator::set_number_of_poses(int nr_poses) {
    if (memory_allocated_) {
        if (nr_poses > max_nr_poses_) {
            std::cout << "ERROR (CUDA): You tried to evaluate more poses ("
                      << nr_poses << ") than specified by max_poses ("
                      << max_nr_poses_ << ")." << std::endl;
            exit(-1);
        }

        nr_poses_ = nr_poses;
        int nr_poses_per_row = min(max_nr_poses_per_row_, nr_poses);
        int nr_poses_per_column = min(max_nr_poses_per_column_,
                                   (int) ceil(nr_poses / (float) nr_poses_per_row));

        grid_dimension_ = dim3(nr_poses_per_row, nr_poses_per_column);

        number_of_poses_set_ = true;
    } else {
        std::cout << "WARNING (CUDA): It seems you forgot to call "
                  << "allocate_memory_for_max_poses before calling "
                  << "set_number_of_poses." << std::endl;
    }
}






// ===================================================================================== //
// =============================== CUDA EVALUATOR GETTERS ================================= //
// ===================================================================================== //


int CudaEvaluator::get_max_nr_threads() {
    return cuda_device_properties_.maxThreadsDim[0];
}

int CudaEvaluator::get_default_nr_threads() {
    return DEFAULT_NR_THREADS;
}

int CudaEvaluator::get_warp_size() {
    return cuda_device_properties_.warpSize;
}

cudaDeviceProp CudaEvaluator::get_device_properties() {
    return cuda_device_properties_;
}


void CudaEvaluator::get_memory_need_parameters(int nr_rows, int nr_cols,
                                int& constant_need, int& per_pose_need) {
    constant_need = nr_rows * nr_cols * sizeof(float);
    per_pose_need = (3 + 2 * nr_rows * nr_cols) * sizeof(float);
}

vector<float> CudaEvaluator::get_occlusion_probabilities(int state_id) {
    if (memory_allocated_) {
        float* occlusion_probabilities = (float*) malloc(nr_rows_ * nr_cols_ * sizeof(float));
        int offset = state_id * nr_rows_ * nr_cols_;
        cudaMemcpy(occlusion_probabilities, d_occlusion_probs_ + offset, nr_rows_ * nr_cols_ * sizeof(float), cudaMemcpyDeviceToHost);

        #ifdef DEBUG
            check_cuda_error("cudaMemcpy d_occlusion_probabilities -> occlusion_probabilities");
        #endif

        vector<float> occlusion_probabilities_vector;
        for (int i = 0; i < nr_rows_ * nr_cols_; i++) {
            occlusion_probabilities_vector.push_back(occlusion_probabilities[i]);
        }
        free(occlusion_probabilities);
        return occlusion_probabilities_vector;
    } else {
        std::cout << "WARNING (CUDA): It seems you forgot to call "
                  << "allocate_memory_for_max_poses before calling "
                  << "get_occlusion_probabilities." << std::endl;
        vector<float> dummy;
        return dummy;
    }
}




// ===================================================================================== //
// ========================== CUDA EVALUATOR HELPER FUNCTIONS ============================= //
// ===================================================================================== //




template <typename T> void CudaEvaluator::allocate(T * &pointer, size_t size) {
    cudaFree(pointer);
    cudaMalloc((void **) &pointer, size);
#ifdef DEBUG
    check_cuda_error("cudaMalloc failed");
#endif
}



void CudaEvaluator::check_cuda_error(const char *msg)
{
    cudaError_t err = cudaGetLastError();
    if( cudaSuccess != err)
    {
        fprintf(stderr, "Cuda error: %s: %s.\n", msg, cudaGetErrorString( err) );
        exit(EXIT_FAILURE);
    }
}

// ===================================================================================== //
// ============================ CUDA EVALUATOR DESTRUCTOR  ================================ //
// ===================================================================================== //




CudaEvaluator::~CudaEvaluator() {
    cudaFree(d_occlusion_probs_);
    cudaFree(d_occlusion_probs_copy_);
    cudaFree(d_observations_);
    cudaFree(d_log_likelihoods_);
    cudaFree(d_occlusion_indices_);
    cudaDeviceReset();
}

