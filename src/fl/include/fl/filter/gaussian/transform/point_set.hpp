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
 * \file point_set.hpp
 * \date October 2014
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once


#include <map>
#include <cmath>
#include <type_traits>

#include <Eigen/Dense>

#include <fl/util/meta.hpp>
#include <fl/util/traits.hpp>

#include <fl/distribution/gaussian.hpp>
#include <fl/exception/exception.hpp>
#include <fl/filter/filter_interface.hpp>

/** \cond internal */
/**
 * Checks dimensions
 */
#define INLINE_CHECK_POINT_SET_DIMENSIONS() \
    assert(points_.cols() == int(weights_.size())); \
    if (points_.rows() == 0) \
    { \
        fl_throw(ZeroDimensionException("PointSet")); \
    } \
    if (points_.cols() == 0)  \
    { \
        fl_throw(Exception("PointSet contains no points.")); \
    }

/**
 * Checks for index out of bounds and valid dimensions
 */
#define INLINE_CHECK_POINT_SET_BOUNDS(i) \
    if (i >= int(weights_.size())) \
    { \
        fl_throw(OutOfBoundsException(i, weights_.size())); \
    } \
    INLINE_CHECK_POINT_SET_DIMENSIONS();
/** \endcond */

namespace fl
{

// Forward declaration
template <typename Point, int Points_> class PointSet;

/**
 * Trait struct of a PointSet
 */
template <typename Point_, int Points_>
struct Traits<PointSet<Point_, Points_>>
{
    /**
     * \brief Point type
     */
    typedef Point_ Point;

    /**
     * \brief Number of points for fixed-size set
     */
    enum
    {
        /**
         * \brief Number of points which provided by the PointSet.
         *
         * If the number of points is unknown and there for dynamic, then
         * NumberOfPoints is set to Eigen::Dynamic
         */
        NumberOfPoints = IsFixed<Points_>() ? Points_ : Eigen::Dynamic
    };

    /**
     * \brief Weight harbors the weights of a point. For each of the first two
     * moments there is a separate weight.
     *
     *
     * Generally a single weight suffices. However, some transforms utilize
     * different weights for each moment to select a set of points representing
     * the underlying moments.
     */
    struct Weight
    {
        /**
         * First moment (mean) point weight
         */
        double w_mean;

        /**
         * Second centered moment (covariance) point weight
         */
        double w_cov;
    };

    /**
     * \brief Point container type
     *
     * \details
     * The point container type has a fixed-size dimension of a point
     * and the number of the points is statically known.
     */
    typedef Eigen::Matrix<
                typename Point::Scalar,
                Point::RowsAtCompileTime,
                Points_
            > PointMatrix;

    /**
     * \brief WeightVector
     */
    typedef Eigen::Matrix<
                typename Point::Scalar,
                Points_,
                1
            > WeightVector;

    /**
     * \brief Weight list of all points
     */
    typedef Eigen::Array<Weight, Points_, 1> Weights;
};

/**
 * \ingroup nonlinear_gaussian_filter
 *
 * \brief PointSet represents a container of fixed-size or dynamic-size points each
 *        paired with a set of weights.
 *
 * PointSet has two degree-of-freedoms. The first
 * is the dimension of the points. The second is the number of points within the
 * set. Each of the parameter can either be fixed at compile time or left
 * unspecified. That is, the parameter is set to Eigen::Dynamic.
 *
 * \tparam Point    Gaussian variable type
 * \tparam Points_  Number of points representing the gaussian
 */
template <typename Point_, int Points_ = -1>
class PointSet
{
private:
    /** \brief Typdef of \c This for #from_traits(TypeName) helper */
    typedef PointSet<Point_, Points_> This;

public:
    typedef from_traits(Point);
    typedef from_traits(PointMatrix);
    typedef from_traits(Weight);
    typedef from_traits(Weights);
    typedef from_traits(WeightVector);

    static_assert(Points_ != 0, "Invalid point count");

public:
    /**
     * \brief Creates a PointSet
     *
     * \param points_count   Number of points representing the Gaussian
     * \param dimension      Sample space dimension
     */
    PointSet(int dimension,
             int points_count = ToDimension<Points_>())
        : points_(dimension, points_count),
          weights_(points_count, 1)
    {
        assert(points_count >= 0);
        static_assert(Points_ >= Eigen::Dynamic, "Invalid point count");

        points_.setZero();

        double weight = (points_count > 0) ? 1./double(points_count) : 0;
        weights_.fill(Weight{weight, weight});
    }

    /**
     * \brief Creates a PointSet
     */
    PointSet()
    {
        points_.setZero();

        double weight = (weights_.size() > 0) ? 1./double(weights_.size()) : 0;
        weights_.fill(Weight{weight, weight});
    }

    /**
     * \brief Overridable default destructor
     */
    virtual ~PointSet() noexcept { }

    /**
     * \brief Resizes a dynamic-size PointSet
     *
     * \param Points_   Number of points
     *
     * \throws ResizingFixedSizeEntityException
     */
    void resize(int points_count)
    {
        if (int(weights_.size()) == points_count &&
            int(points_.cols()) == points_count) return;

        if (IsFixed<Points_>())
        {
            fl_throw(
                fl::ResizingFixedSizeEntityException(weights_.size(),
                                                     points_count,
                                                     "poit set Gaussian"));
        }

        points_.setZero(points_.rows(), points_count);

        weights_.resize(points_count, 1);
        const double weight = (points_count > 0)? 1. / double(points_count) : 0;
        for (int i = 0; i < points_count; ++i)
        {
            weights_(i).w_mean = weight;
            weights_(i).w_cov = weight;
        }

        // std::fill(weights_.begin(), weights_.end(), Weight{weight, weight});
    }

    /**
     * \brief Resizes a dynamic-size PointSet
     *
     * \param Points_   Number of points
     *
     * \throws ResizingFixedSizeEntityException
     */
    void resize(int dim, int points_count)
    {
        if (dim == dimension() && points_count == count_points()) return;

        points_.setZero(dim, points_count);

        weights_.resize(points_count, 1);
        const double weight = (points_count > 0)? 1. / double(points_count) : 0;
        for (int i = 0; i < points_count; ++i)
        {
            weights_(i).w_mean = weight;
            weights_(i).w_cov = weight;
        }
    }

    /**
     * \brief Sets the new dimension for dynamic-size points (not to confuse
     * with the number of points)
     *
     * \param dim  Dimension of each point
     */
    void dimension(int dim)
    {
        points_.resize(dim, count_points());
    }

    /**
     * \return Dimension of containing points
     */
    int dimension() const
    {
        return points_.rows();
    }

    /**
     * \brief  The number of points
     */
    int count_points() const
    {
        return points_.cols();
    }

    /**
     * \brief  Read only access on i-th point
     *
     * \param i Index of requested point
     *
     * \throws OutOfBoundsException
     * \throws ZeroDimensionException
     */
    Point point(int i) const
    {
        INLINE_CHECK_POINT_SET_BOUNDS(i);

        return points_.col(i);
    }

    auto operator[](int i) -> decltype(PointMatrix().col(i))
    {
        return points_.col(i);
    }

    /**
     * \brief  weight of i-th point assuming both weights are the same
     *
     * \param i Index of requested point
     *
     * \throws OutOfBoundsException
     * \throws ZeroDimensionException
     */
    double weight(int i)
    {
        INLINE_CHECK_POINT_SET_BOUNDS(i);

        return weights_(i).w_mean;
    }

    /**
     * \brief  weights of i-th point
     *
     * \param i Index of requested point
     *
     * \throws OutOfBoundsException
     * \throws ZeroDimensionException
     */
    const Weight& weights(int i) const
    {
        INLINE_CHECK_POINT_SET_BOUNDS(i);

        return weights_(i);
    }

    /**
     * \brief  Point matrix (read only)
     */
    const PointMatrix& points() const noexcept
    {
        return points_;
    }

    /**
     * \brief  Point matrix
     */
    PointMatrix& points()
    {
        return points_;
    }

    /**
     * \brief  point weights vector
     */
    const Weights& weights() const noexcept
    {
        return weights_;
    }

    /**
     * \brief  Returns the weights for the mean of the points as a vector
     */
    WeightVector mean_weights_vector() const noexcept
    {
        const int point_count = count_points();

        WeightVector weight_vec(point_count);

        for (int i = 0; i < point_count; ++i)
        {
            weight_vec(i) = weights_(i).w_mean;
        }

        return weight_vec;
    }

    /**
     * \brief  Returns the weights for the covariance of the points as a vector
     */
    WeightVector covariance_weights_vector() const noexcept
    {
        const int point_count = count_points();

        WeightVector weight_vec(point_count);

        for (int i = 0; i < point_count; ++i)
        {
            weight_vec(i) = weights_(i).w_cov;
        }

        return weight_vec;
    }

    /**
     * \brief Sets the given point matrix
     */
    void points(const PointMatrix& point_matrix)
    {
        points_ = point_matrix;
    }

    /**
     * \brief Sets a given point at position i
     *
     * \param i         Index of point
     * \param p         The new point
     *
     * \throws OutOfBoundsException
     * \throws ZeroDimensionException
     */
    void point(int i, Point p)
    {
        INLINE_CHECK_POINT_SET_BOUNDS(i);

        points_.col(i) = p;
    }

    /**
     * \brief Sets a given point at position i along with its weights
     *
     * \param i         Index of point
     * \param p         The new point
     * \param w         Point weights. The weights determinaing the first two
     *                  moments are the same
     *
     * \throws OutOfBoundsException
     * \throws ZeroDimensionException
     */
    void point(int i, Point p, double w)
    {
        point(i, p, Weight{w, w});
    }

    /**
     * \brief Sets a given point at position i along with its weights
     *
     * \param i         Index of point
     * \param p         The new point
     * \param w_mean    point weight used to compute the first moment
     * \param w_cov     point weight used to compute the second centered moment
     *
     * \throws OutOfBoundsException
     * \throws ZeroDimensionException
     */
    void point(int i, Point p, double w_mean , double w_cov)
    {
        point(i, p, Weight{w_mean, w_cov});
    }

    /**
     * \brief Sets a given point at given position i along with its weights
     *
     * \param i         Index of point
     * \param p         The new point
     * \param weights   point weights
     *
     * \throws OutOfBoundsException
     * \throws ZeroDimensionException
     */
    void point(int i, Point p, Weight weights)
    {
        INLINE_CHECK_POINT_SET_BOUNDS(i);

        points_.col(i) = p;
        weights_(i) = weights;
    }

    /**
     * \brief Sets a given weight of a point at position i
     *
     * \param i         Index of point
     * \param w         Point weights. The weights determinaing the first two
     *                  moments are the same
     *
     * \throws OutOfBoundsException
     * \throws ZeroDimensionException
     */
    void weight(int i, double w)
    {
        weight(i, Weight{w, w});
    }

    /**
     * \brief Sets given weights of a point at position i
     *
     * \param i         Index of point
     * \param w_mean    point weight used to compute the first moment
     * \param w_cov     point weight used to compute the second centered moment
     *
     * \throws OutOfBoundsException
     * \throws ZeroDimensionException
     */
    void weight(int i, double w_mean , double w_cov)
    {
        weight(i, Weight{w_mean, w_cov});
    }

    /**
     * Sets given weights of a point at position i
     *
     * \param i         Index of point
     * \param weights   point weights
     *
     * \throws OutOfBoundsException
     * \throws ZeroDimensionException
     */
    void weight(int i, Weight weights)
    {
        INLINE_CHECK_POINT_SET_BOUNDS(i);

        weights_(i) = weights;
    }

    /**
     * \brief Creates a PointMatrix populated with zero mean points
     *
     * \return Centered points matrix.
     *
     * \throws ZeroDimensionException
     * \throws Exception
     */
    PointMatrix centered_points() const
    {
        INLINE_CHECK_POINT_SET_DIMENSIONS();

        PointMatrix centered(points_.rows(), points_.cols());

        const Point weighted_mean = mean();
        const int point_count = points_.cols();
        for (int i = 0; i < point_count; ++i)
        {
            centered.col(i) = points_.col(i) - weighted_mean;
        }

        return centered;
    }

    /**
     * \brief Centers all points and returns the mean over all points.
     *
     * Computes the weighted mean of all points and subtracts the mean from all
     * points. Finally returns the weighted mean
     *
     * \return Weighted mean
     */
    Point center()
    {
        Point weighted_mean = mean();
        const int point_count = points_.cols();
        for (int i = 0; i < point_count; ++i)
        {
            points_.col(i) -= weighted_mean;
        }

        return weighted_mean;
    }

    /**
     * \brief Returns the weighted mean of all points
     *
     * \throws ZeroDimensionException
     * \throws Exception
     */
    Point mean() const
    {
        INLINE_CHECK_POINT_SET_DIMENSIONS();

        Point weighted_mean;
        weighted_mean.setZero(points_.rows());

        const int point_count = points_.cols();
        for (int i = 0; i < point_count; ++i)
        {
            weighted_mean += weights_(i).w_mean * points_.col(i);
        }

        return weighted_mean;
    }

protected:
    /**
     * \brief point container
     */
    PointMatrix points_;

    /**
     * \brief weight container
     */
    Weights weights_;
};

}
