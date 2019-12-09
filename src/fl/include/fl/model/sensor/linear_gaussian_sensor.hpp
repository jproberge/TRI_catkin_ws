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
 * \file linear_gaussian_sensor.hpp
 * \date October 2014
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once


#include <fl/util/traits.hpp>
#include <fl/util/descriptor.hpp>
#include <fl/distribution/gaussian.hpp>
#include <fl/model/sensor/linear_sensor.hpp>

namespace fl
{

/**
 * \ingroup sensors
 *
 * This represents the linear gaussian observation model \f$p(y_t \mid x_t)\f$
 * governed by the linear equation
 *
 * \f$ y_t  = H_t x_t + N_t v_t \f$
 *
 * where \f$ y_t\f$ is the observation, \f$ x_t\f$ is the state. The matrix
 * \f$H_t\f$ is the sensor model mapping the state into the observation space.
 * The vector \f$ v_t \sim {\cal N}(v_t; 0, I)\f$ is a standard normal variate
 * which is mapped into the the observation space via the noise model matrix
 * \f$N_t\f$. Any Gaussian noise \f$\tilde{v}_t \sim {\cal N}(\tilde{v}_t ; 0,
 * R_t) \f$ can be represented via \f$\tilde{v}_t = N_t v_t\f$ with\f$N_t
 * = \sqrt{R_t}\f$. Hence, the linear equation may be restated as the more
 * familiar but equivalent form
 *
 * \f$ y_t  = H_t x_t + \tilde{v}_t \f$.
 */
template <typename Obsrv, typename State>
class LinearGaussianSensor
    : public LinearSensor<Obsrv, State, Gaussian<Obsrv>>,
      public Descriptor
{
public:
    /**
     * Constructs a linear gaussian observation model
     * \param obsrv_dim     observation dimension if dynamic size
     * \param state_dim     state dimension if dynamic size
     */
    explicit
    LinearGaussianSensor(int obsrv_dim = DimensionOf<Obsrv>(),
                                   int state_dim = DimensionOf<State>())
        : LinearSensor<Obsrv, State, Gaussian<Obsrv>>(
              obsrv_dim, state_dim)
    { }

    virtual std::string name() const
    {
        return "LinearGaussianSensor";
    }

    virtual std::string description() const
    {
        return "Linear observation model with additive Gaussian noise";
    }
};

}


