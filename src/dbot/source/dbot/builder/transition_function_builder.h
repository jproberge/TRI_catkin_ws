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

/**
 * \file transition_function_builder.h
 * \date November 2015
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once

#include <memory>
#include <Eigen/Dense>
#include <fl/model/transition/interface/transition_function.hpp>

namespace dbot
{

template <typename State, typename Noise, typename Input>
class TransitionFunctionBuilder
{
public:
    typedef fl::TransitionFunction<State, Noise, Input> Model;

public:
    virtual std::shared_ptr<Model> build() const = 0;
};

}
