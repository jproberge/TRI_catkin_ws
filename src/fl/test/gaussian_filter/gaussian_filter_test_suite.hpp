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
 * \file gaussian_filter_test_suite.hpp
 * \date June 2015
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once


#include <gtest/gtest.h>
#include "../typecast.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <iostream>

#include <fl/util/profiling.hpp>
#include <fl/util/math/linear_algebra.hpp>
#include <fl/filter/filter_interface.hpp>

#include <fl/model/transition/linear_transition.hpp>
#include <fl/model/sensor/linear_gaussian_sensor.hpp>
#include <fl/model/sensor/linear_decorrelated_gaussian_sensor.hpp>

template <typename TestType>
class GaussianFilterTest
    : public ::testing::Test
{
protected:
    typedef typename TestType::Parameter Configuration;

    enum: signed int
    {
        StateDim = Configuration::StateDim,
        InputDim = Configuration::InputDim,
        ObsrvDim = Configuration::ObsrvDim,

        StateSize = fl::TestSize<StateDim, TestType>::Value,
        InputSize = fl::TestSize<InputDim, TestType>::Value,
        ObsrvSize = fl::TestSize<ObsrvDim, TestType>::Value,

        FilterIterations = Configuration::Iterations
    };

    enum ModelSetup
    {
        Random,
        Identity
    };

    typedef Eigen::Matrix<fl::Real, StateSize, 1> State;
    typedef Eigen::Matrix<fl::Real, InputSize, 1> Input;
    typedef Eigen::Matrix<fl::Real, ObsrvSize, 1> Obsrv;

    GaussianFilterTest()
        : predict_steps_(30),
          predict_update_steps_(FilterIterations)
    { }

    struct ModelFactory
    {
        // Sizes will be 1 of the test type is static and it will fall back to
        // -1 for dynamic size tests. This may be used as an expansion factor
        // using fl::ExpandSizes<MySize, Sizes>::Value. If Sizes is equal to 1
        // (Static) then ExpandSizes will simply multiply MySizes by 1 and
        // everything is defined statically. On the other hand, if the Sizes
        // is -1, fl::ExpandSizes will fallback to -1 indicating that the test
        // type is dynamic.
        enum : signed int { Sizes = fl::TestSize<1, TestType>::Value };

        typedef fl::LinearTransition<
                    State, Input
                > LinearTransition;

        typedef fl::LinearGaussianSensor<
                    Obsrv, State
                > LinearObservation;

        LinearTransition create_linear_state_model()
        {
            auto model = LinearTransition(StateDim, InputDim);

            auto A = model.create_dynamics_matrix();
            auto Q = model.create_noise_matrix();

            switch (setup)
            {
            case Random:
                A.setRandom();
                Q.setRandom();
                break;

            case Identity:
                A.setIdentity();
                Q.setIdentity();
                break;
            }

            model.dynamics_matrix(A);
            model.noise_matrix(Q);

            return model;
        }

        LinearObservation create_sensor()
        {
            auto model = LinearObservation(ObsrvDim, StateDim);

            auto H = model.create_sensor_matrix();
            auto R = model.create_noise_matrix();

            switch (setup)
            {
            case Random:
                H.setRandom();
                R.setRandom();
                break;

            case Identity:
                H.setIdentity();
                R.setIdentity();
                break;
            }

            model.sensor_matrix(H);
            model.noise_matrix(R);

            return model;
        }

        ModelSetup setup;
    };

    typedef typename Configuration::template FilterDefinition<
                ModelFactory
            >::Type Filter;

    Filter create_filter(ModelSetup setup = Identity) const
    {
        return Configuration::create_filter(ModelFactory{setup});
    }

    typename fl::Traits<Filter>::Input zero_input(const Filter& filter)
    {
        return fl::Traits<Filter>::Input::Zero(
            filter.transition().input_dimension());
    }

    typename fl::Traits<Filter>::Obsrv rand_obsrv(const Filter& filter)
    {
        return fl::Traits<Filter>::Obsrv::Random(
            filter.sensor().obsrv_dimension());
    }

protected:
    int predict_steps_;
    int predict_update_steps_;
};

TYPED_TEST_CASE_P(GaussianFilterTest);

//TYPED_TEST_P(GaussianFilterTest, init_predict)
//{
//    typedef TestFixture This;

//    auto filter = This::create_filter();
//    auto belief = filter.create_belief();

//    EXPECT_TRUE(belief.mean().isZero());
//    EXPECT_TRUE(belief.covariance().isIdentity());

//    std::cout << filter.name() << std::endl;
//    std::cout << filter.description() << std::endl;

//    filter.predict(belief, This::zero_input(), belief);

//    auto Q = filter.transition().noise_covariance();

//    EXPECT_TRUE(belief.mean().isZero());
//    EXPECT_TRUE(fl::are_similar(belief.covariance(), 2. * Q));
//}

TYPED_TEST_P(GaussianFilterTest, predict_then_update)
{
    typedef TestFixture This;

    auto filter = This::create_filter(This::Identity);

    PV(filter.name());

    auto belief = filter.create_belief();

    EXPECT_TRUE(belief.covariance().ldlt().isPositive());

    for (int i = 0; i < This::predict_update_steps_; ++i)
    {
        PF(i);
//        PV(belief.mean());
//        PV(belief.covariance());

        filter.predict(belief, This::zero_input(filter), belief);

//        if (!belief.covariance().ldlt().isPositive())
//        {
//            PV(belief.mean());
//            PV(belief.covariance());
//        }

        ASSERT_TRUE(belief.covariance().ldlt().isPositive());


        filter.update(belief, This::rand_obsrv(filter), belief);

//        if (!belief.covariance().ldlt().isPositive())
//        {
//            PV(belief.mean());
//            PV(belief.covariance());
//        }

        ASSERT_TRUE(belief.covariance().ldlt().isPositive());
    }

    PV(belief.mean());
    PV(belief.covariance());
}

TYPED_TEST_P(GaussianFilterTest, predict_loop)
{
    typedef TestFixture This;

    auto filter = This::create_filter(This::Identity);
    auto belief = filter.create_belief();

    EXPECT_TRUE(belief.covariance().ldlt().isPositive());

    for (int i = 0; i < This::predict_steps_; ++i)
    {
        filter.predict(belief, This::zero_input(filter), belief);
    }

    EXPECT_TRUE(belief.covariance().ldlt().isPositive());
}

REGISTER_TYPED_TEST_CASE_P(GaussianFilterTest,
                           //init_predict,
                           predict_then_update,
                           predict_loop);



