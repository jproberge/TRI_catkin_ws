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
 * \file scalar_matrix.hpp
 * \date August 2015
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once


#include <Eigen/Dense>
#include <fl/util/types.hpp>

namespace fl
{

/**
 * \ingroup types
 *
 * Represents a matrix of the size 1x1. This Matrix can be used in the same way
 * as a scalar.
 */
class ScalarMatrix
    : public Eigen::Matrix<Real, 1, 1>
{
public:
    /**
     * \brief Default constructor.
     * Creates ScalarMatrix with no arguments. The initial value is zero by
     * default.
     */
    ScalarMatrix()
    {
        this->data()[0] = Real(0);
    }

    /**
     * \brief Creates a ScalarMatrix with an initial value or converts a scalar
     * into a ScalarMatrix.
     * \param value initial value
     */
    ScalarMatrix(Real value)
    {
        this->data()[0] = value;
    }

    /**
     * \brief Constructor copying the value of the ScalarMatrix \a other
     */
    ScalarMatrix(const ScalarMatrix& other)
    {
        this->data()[0] = other.data()[0];
    }

    /**
     * \brief Constructor copying the value of the expression \a other
     */
    template <typename OtherDerived>
    ScalarMatrix(const Eigen::MatrixBase<OtherDerived> &other)
    {
        this->data()[0] = other(0);
    }

    /**
     * \brief operator Real() implicit typecast conversion of a ScalarMatrix
     * into a scalar
     */
    operator Real() const
    {
        return Real(this->data()[0]);
    }

    /**
     * \brief operator+= adds a scalar value to this matrix and assigns
     */
    void operator+=(Real value)
    {
        this->data()[0] += value;
    }

    /**
     * \brief operator-= subtracts a scalar value to this matrix and assigns
     */
    void operator-=(Real value)
    {
        this->data()[0] -= value;
    }

    /**
     * \brief operator*= multiplies a scalar value with this this matrix and
     * assigns
     */
    void operator*=(Real value)
    {
        this->data()[0] *= value;
    }

    /**
     * \brief operator/= divide a scalar value with this this matrix and
     * assigns
     */
    void operator/=(Real value)
    {
        this->data()[0] /= value;
    }

    /**
     * \brief prefix operator ++ which increments the matrix value by 1.
     */
    ScalarMatrix& operator++()
    {
        return (++this->data()[0], *this);
    }

    /**
     * \brief postfix operator ++ which increments the matrix value by 1.
     */
    ScalarMatrix operator++(int)
    {
        auto r = ScalarMatrix(this->data()[0]++); // RVO
        return r;
    }

    /**
     * \brief prefix operator -- which decrements the matrix value by 1.
     */
    ScalarMatrix& operator--()
    {
        return (--this->data()[0], *this);
    }

    /**
     * \brief postfix operator -- which decrements the matrix value by 1.
     */
    ScalarMatrix operator--(int)
    {
        auto r = ScalarMatrix(this->data()[0]--); // RVO
        return r;
    }
};

}


