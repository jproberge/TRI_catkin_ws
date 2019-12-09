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
 * \file particle_tracker.h
 * \date November 2015
 * \author Jan Issac (jan.issac@gmail.com)
 * \author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 */

#pragma once

#include <fl/model/transition/interface/transition_function.hpp>

#include <dbot/tracker/tracker.h>
#include <dbot/filter/rao_blackwell_coordinate_particle_filter.h>

namespace dbot
{
/**
 * \brief ParticleTracker
 */
class ParticleTracker : public Tracker
{
public:
    typedef fl::TransitionFunction<State, Noise, Input> Transition;
    typedef RbSensor<State> Sensor;

    typedef RaoBlackwellCoordinateParticleFilter<Transition, Sensor> Filter;

public:
    /**
     * \brief Creates the tracker
     *
     * \param filter
     *     Rbc particle filter instance
     * \param object_model
     *     Object model instance
     * \param camera_data
     *     Camera data container
     * \param update_rate
     *     Moving average update rate
     */
    ParticleTracker(
        const std::shared_ptr<Filter>& filter,
        const std::shared_ptr<ObjectModel>& object_model,
        int evaluation_count,
        double update_rate,
        bool center_object_frame);
    

    virtual ~ParticleTracker() { }

    /**
     * \brief perform a single filter step
     *
     * \param image
     *     Current observation image
     */
    State on_track(const Obsrv& image);

    /**
     * \brief Initializes the particle filter with the given initial states and
     *    the number of evaluations
     * @param initial_states
     * @param evaluation_count
     */
    State on_initialize(const std::vector<State>& initial_states);

private:
    std::shared_ptr<Filter> filter_;
    int evaluation_count_;
};
}
