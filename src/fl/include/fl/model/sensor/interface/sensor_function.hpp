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
 * \file sensor_function.hpp
 * \date October 2014
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once


#include <fl/util/types.hpp>
#include <fl/util/traits.hpp>

namespace fl
{

/**
 * \ingroup sensors
 */
template <
    typename Obsrv_,
    typename State_,
    typename Noise_,
    int Id = 0
>
class SensorFunction
    : private internal::NonAdditiveNoiseModelType
{
public:
    typedef internal::NonAdditiveNoiseModelType Type;

    typedef Obsrv_ Obsrv;
    typedef State_ State;
    typedef Noise_ Noise;

    /**
     * \brief Overridable default destructor
     */
    virtual ~SensorFunction() noexcept { }

    /**
     * Evaluates the model function \f$y = h(x, w)\f$ where \f$x\f$ is the state
     * and \f$w\sim {\cal N}(0, 1)\f$ is a white noise parameter. Put
     * differently, \f$y = h(x, w)\f$ is a sample from the conditional model
     * distribution \f$p(y \mid x)\f$.
     *
     * \param state         The state variable \f$x\f$
     * \param noise         The noise term \f$w\f$
     * \param delta_time    Prediction time
     */
    virtual Obsrv observation(const State& state,
                              const Noise& noise) const = 0;

    /**
     * \brief Returns the dimension of the state variable \f$x\f$
     */
    virtual int state_dimension() const = 0;

    /**
     * \brief Returns the dimension of the noise term \f$w\f$
     */
    virtual int noise_dimension() const = 0;

    /**
     * \brief Returns the dimension of the measurement \f$h(x, w)\f$
     */
    virtual int obsrv_dimension() const = 0;

    /**
     * \return Model id number
     *
     * In case of multiple sensors of the same kind, this function returns the
     * id of the individual model.
     */
    virtual int id() const { return Id; }

    /**
     * \brief Sets the model id for mulit-sensor use
     *
     * In some cases a single observation model may be used for multiple sensor
     * of the same kind. It often suffices to alter the sensor id before
     * evaluating the model.
     *
     * \param new_id    Model's new ID
     */
    virtual void id(int) { /* const ID */ }
};

}


