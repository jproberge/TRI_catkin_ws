/*
 * This is part of the fl library, a C++ Bayesian filtering library
 * (https://github.com/filtering-library)
 *
 * Copyright (c) 2015 Max Planck Society,
 * 				 Autonomous Motion Department,
 * 			     Institute for Intelligent Systems
 *
 * This Source Code Form is subject to the terms of the MIT License (MIT).
 * A copy of the license can be found in the LICENSE file distributed with this
 * source code.
 */

/**
 * \file random.hpp
 * \date 2014
 * \author Jan Issac (jan.issac@gmail.com)
 * \author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 */

#pragma once


#include <ctime>
#include <chrono>
#include <random>
#include <iostream>
#include <iomanip>
/**
 * \brief global seed
 *
 * \ingroup random
 */
//#ifdef fl_USE_RANDOM_SEED
//    #define RANDOM_SEED (unsigned int) std::time(0)
//#else
//    #define RANDOM_SEED 1
//#endif

#undef RANDOM_SEED
#define RANDOM_SEED (unsigned int) std::time(0)

namespace fl
{

/**
 * \ingroup random
 *
 * \brief Mersenne Twister specialization mt11213b \cite matsumoto1998mersenne
 *
 * mt11213b is slightly faster than mt19937
 */
typedef std::mersenne_twister_engine<
                uint32_t,
                32, 351, 175, 19,
                0xccab8ee7, 11,
                0xffffffff, 7,
                0x31b6ab00, 15,
                0xffe50000, 17,
                1812433253 > mt11213b;

/**
 * \ingroup random
 * \brief A seed. If fl_USE_RANDOM_SEED was set true the seed is set to the
 * current time, otherwise, the seed will be 1.
 *
 */
inline unsigned int seed()
{
//#ifdef fl_USE_RANDOM_SEED
//    static unsigned int seed_inc = std::time(0);
//    return seed_inc++;
//#else
//    return 1;
//#endif

    static unsigned int seed_inc = std::time(0);
    return seed_inc++;
}

}


