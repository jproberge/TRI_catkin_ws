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

#include <fl/util/profiling.hpp>
#include <dbot/tracker/tracker.h>

namespace dbot
{
Tracker::Tracker(const std::shared_ptr<ObjectModel> &object_model,
                             double update_rate,
                             bool center_object_frame)
    : object_model_(object_model),
      update_rate_(update_rate),
      center_object_frame_(center_object_frame),
      moving_average_(object_model_->count_parts())
{
}

void Tracker::initialize(const std::vector<State>& initial_states)
{
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<State> states;
    for (auto state : initial_states)
    {
        states.push_back(to_center_coordinate_system(state));
    }

    moving_average_ = to_model_coordinate_system(on_initialize(states));
}

void Tracker::move_average(const Tracker::State& new_state,
                                 Tracker::State& moving_average,
                                 double update_rate)
{
    for (int i = 0; i < moving_average.count(); i++)
    {
        auto partial_moving_average = moving_average.component(i);
        auto partial_new_state = new_state.component(i);

        Eigen::Vector4d average_q = partial_moving_average
                                        .orientation()
                                        .quaternion()
                                        .coeffs();
        Eigen::Vector4d new_q =
            partial_new_state.orientation().quaternion().coeffs();

        if (average_q.dot(new_q) < 0) new_q = -new_q;

        Eigen::Quaterniond q;
        q.coeffs() = (1.0 - update_rate) * average_q + update_rate * new_q;
        q.normalize();

        // taking weighted average
        partial_moving_average = (1.0 - update_rate) * partial_moving_average +
                                 update_rate * partial_new_state;
        partial_moving_average.orientation().quaternion(q);
    }
}

auto Tracker::track(const Obsrv& image) -> State
{
    std::lock_guard<std::mutex> lock(mutex_);

    move_average(to_model_coordinate_system(on_track(image)),
                 moving_average_,
                 update_rate_);

    return moving_average_;
}

auto Tracker::to_center_coordinate_system(
    const Tracker::State& state) -> State
{
    if (!center_object_frame_) return state;

    State centered_state = state;
    for (size_t j = 0; j < state.count(); j++)
    {
        centered_state.component(j).position() +=
            state.component(j).orientation().rotation_matrix() *
            object_model_->centers()[j];
    }

    return centered_state;
}
    

auto Tracker::to_model_coordinate_system(
    const Tracker::State& state) -> State
{
    if (!center_object_frame_) return state;

    State model_state = state;
    for (size_t j = 0; j < state.count(); j++)
    {
        model_state.component(j).position() -=
            state.component(j).orientation().rotation_matrix() *
            object_model_->centers()[j];
    }

    return model_state;
}

Tracker::Input Tracker::zero_input() const
{
    return Input::Zero(1);
}
}
