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

#include <dbot/tracker/particle_tracker.h>

namespace dbot
{
ParticleTracker::ParticleTracker(
    const std::shared_ptr<Filter>& filter,
    const std::shared_ptr<ObjectModel>& object_model,
    int evaluation_count,
    double update_rate,
    bool center_object_frame)
    : Tracker(object_model, update_rate, center_object_frame),
      filter_(filter),
      evaluation_count_(evaluation_count)
{
}

auto ParticleTracker::on_initialize(
    const std::vector<State>& initial_states) -> State
{
    filter_->set_particles(initial_states);
    filter_->resample(evaluation_count_ / filter_->sampling_blocks().size());

    State delta_mean = filter_->belief().mean();

    for (size_t i = 0; i < filter_->belief().size(); i++)
    {
        filter_->belief().location(i).subtract(delta_mean);
    }

    auto& integrated_poses = filter_->sensor()->integrated_poses();
    integrated_poses.apply_delta(delta_mean);

    return integrated_poses;
}

auto ParticleTracker::on_track(const Obsrv& image) -> State
{
    filter_->filter(image, zero_input());

    State delta_mean = filter_->belief().mean();

    for (size_t i = 0; i < filter_->belief().size(); i++)
    {
        filter_->belief().location(i).subtract(delta_mean);
    }

    auto& integrated_poses = filter_->sensor()->integrated_poses();
    integrated_poses.apply_delta(delta_mean);

    return integrated_poses;
}
}
