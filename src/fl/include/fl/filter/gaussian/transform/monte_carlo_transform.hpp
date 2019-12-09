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
 * \file monte_carlo_transform.hpp
 * \date October 2014
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once


#include <fl/util/traits.hpp>
#include <fl/distribution/gaussian.hpp>
#include <fl/filter/gaussian/transform/point_set_transform.hpp>

namespace fl
{

/**
 * \defgroup monte_carlo_transform Monte Carlo Transform
 * \ingroup point_set_transform
 *
 * \copydoc MonteCarloTransform
 */

/**
 * \defgroup point_count_policy Point Count Policy
 * \ingroup monte_carlo_transform
 * \brief Transform policy which determines the number of samples drawn during
 *        transformation.
 *
 * \details
 * The policy is evaluable at compile time. That is, given the dimension, the
 * number of points is determined at compile time. However, for dynamic sizes
 * or dimensions (Eigen::Dynamic), the number of points falls back to dynamic
 * as well
 */

/**
 * \ingroup point_count_policy
 *
 * The \c MonomialPointCountPolicy determines the number of points as a function
 * of the dimension. The monimial function is of the form
 *
 * \f$ f(x) = a x^p + b\f$
 *
 * where \f$a\f$ is the parameter \c Factor, \f$p\f$ the parameter \c Power,
 * and \f$b\f$ the parameter \c Min.
 *
 * Strictly speaking the function \f$f(x)\f$ is a polynomial of the form
 * \f$g(x) = a x^p + b x^0\f$. However, for simplicity reasons, in most cases
 * the parameter \c Min is 0.
 */
template <int Power, int Factor = 1, int Min = 0>
class MonomialPointCountPolicy
{
public:
    static constexpr int number_of_points(int dimension)
    {
        return (dimension != Eigen::Dynamic)
                    ? monomial(dimension, Power, Factor, Min)
                    : Eigen::Dynamic;
    }

protected:
    static constexpr int monomial(int x, int p, int a, int b)
    {
        return (p > 0) ? a * x * monomial(x, p - 1, 1, 0) + b : 1;
    }
};

/**
 * \ingroup point_count_policy
 *
 * \c LinearPointCountPolicy represents the policy for a linear increase of
 * number of points w.r.t the dimension
 *
 * \f$ f(x) = a x + b\f$
 *
 * where \f$a\f$ is the parameter \c Factor and \f$b\f$ the parameter \c Min.
 *
 * The policy is a special case of the \c MonomialPointCountPolicy for the
 * parameters.
 *
 * \c MonomialPointCountPolicy<1, Factor, \c Min>
 */
template <int Factor = 1, int Min = 0>
struct LinearPointCountPolicy
{
    static constexpr int number_of_points(int dimension)
    {
        return (dimension != Eigen::Dynamic)
                   ? MonomialPointCountPolicy<1, Factor, Min>
                       ::number_of_points(dimension)
                   : Eigen::Dynamic;
    }
};

/**
 * \ingroup point_count_policy
 *
 * \c ConstantPointCountPolicy represents the policy for constant number of
 * points regardless the dimension
 *
 * \f$f(x) = b\f$
 *
 * where \f$b\f$ is the parameter \c NumberOfPoints.
 *
 * The policy is a special case of the
 * \c MonomialPointCountPolicy for the parameters
 *
 * \c MonomialPointCountPolicy<1, 0, \c NumberOfPoints>
 */
template <int NumberOfPoints>
struct ConstantPointCountPolicy
{
    static constexpr int number_of_points(int dimension)
    {
        return (dimension != Eigen::Dynamic)
                   ? NumberOfPoints
                   : Eigen::Dynamic;
    }
};


/**
 * \ingroup monte_carlo_transform
 *
 * This is the Monte Carlo Transform (MCT) used in the Gaussian Filter, the
 * Sigma Point Kalman Filter, in the same sense as using the Unscented Transform
 * within the Unscented Kalman Filter. With a sufficient number of points, the
 * MCT is in practice able to capture relatively high order precision w.r.t the
 * Taylor series expansion. The
 * number of points is determined by the specified PointCountPolicy. The three
 * basic pre-implemented policies are \c MonomialPointCountPolicy,
 * \c ConstantPointCountPolicy, and \c LinearPointCountPolicy. The latter two
 * are a special case of the \c MonomialPointCountPolicy.
 *
 * \tparam PointCountPolicy     The number of points policy. Default is a linear
 *                              policy LinearPointCountPolicy<>
 */
template <
    typename PointCountPolicy = LinearPointCountPolicy<>
>
class MonteCarloTransform
        : public PointSetTransform<MonteCarloTransform<PointCountPolicy>>
{
public:
    /**
     * Creates a MonteCarloTransform
     */
    MonteCarloTransform()
        : PointSetTransform<MonteCarloTransform<PointCountPolicy>>(this)
    { }

    template <typename ... Args>
    MonteCarloTransform(Args...args)
        : PointSetTransform<MonteCarloTransform<PointCountPolicy>>(this)
    { }

    /**
     * \copydoc PointSetTransform::forward(const Gaussian&,
     *                                     PointSet&) const
     *
     * \throws WrongSizeException
     * \throws ResizingFixedSizeEntityException
     */
    template <typename Gaussian_, typename PointSet_>
    void operator()(const Gaussian_& gaussian,
                    PointSet_& point_set) const
    {
        forward(gaussian, gaussian.dimension(), 0, point_set);
    }


    /**
     * \copydoc PointSetTransform::forward(const Gaussian&,
     *                                     int global_dimension,
     *                                     int dimension_offset,
     *                                     PointSet&) const
     *
     * \throws WrongSizeException
     * \throws ResizingFixedSizeEntityException
     */
    template <typename Gaussian_, typename PointSet_>
    void operator()(const Gaussian_& gaussian,
                    int global_dimension,
                    int dimension_offset,
                    PointSet_& point_set) const
    {
        forward(gaussian, global_dimension, dimension_offset, point_set);
    }

    /**
     * \copydoc PointSetTransform::forward(const Gaussian&,
     *                                     PointSet&) const
     *
     * \throws WrongSizeException
     * \throws ResizingFixedSizeEntityException
     */
    template <typename Gaussian_, typename PointSet_>
    void forward(const Gaussian_& gaussian,
                 PointSet_& point_set) const
    {
        forward(gaussian, gaussian.dimension(), 0, point_set);
    }

    /**
     * \copydoc PointSetTransform::forward(const Gaussian&,
     *                                     int global_dimension,
     *                                     int dimension_offset,
     *                                     PointSet&) const
     *
     * \throws WrongSizeException
     * \throws ResizingFixedSizeEntityException
     */
    template <typename Gaussian_, typename PointSet_>
    void forward(const Gaussian_& gaussian,
                 int global_dimension,
                 int dimension_offset,
                 PointSet_& point_set) const
    {

        const int point_count = number_of_points(global_dimension);

        point_set.resize(gaussian.dimension(), point_count);

        for (int i = 0; i < point_count; ++i)
        {
            point_set[i] = gaussian.sample();
        }
    }

    /**
     * \return Number of points generated by this transform
     *
     * \param dimension Dimension of the Gaussian
     */
    static constexpr int number_of_points(int dimension)
    {
        return PointCountPolicy::number_of_points(dimension);
    }

    virtual std::string name() const
    {
        return "MonteCarloTransform";
    }

    virtual std::string description() const
    {
        return name();
    }
};

}
