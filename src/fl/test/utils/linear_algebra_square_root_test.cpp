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
 * \file linear_algebra_square_root_test.cpp
 * \date July 15
 * \author Jan Issac (jan.issac@gmail.com)
 */

#include <gtest/gtest.h>

#include <fl/util/types.hpp>
#include <fl/util/math/linear_algebra.hpp>

static constexpr int N = 50;

template <typename Matrix>
void check_square_root()
{
    Matrix M;
    Matrix L;

    M.resize(N, N);
    M.setRandom();
    M *= M.transpose();
    fl::square_root(M, L );

    EXPECT_TRUE(fl::are_similar(L*L.transpose(), M));
}

TEST(LinearAlgebra, square_root_fixed_size)
{
    check_square_root<Eigen::Matrix<fl::Real, N, N>>();
}

TEST(LinearAlgebra, square_root_dynamic_size)
{
    check_square_root<Eigen::Matrix<fl::Real, Eigen::Dynamic, Eigen::Dynamic>>();
}
