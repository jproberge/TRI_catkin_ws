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
 * \file forward_adaptive.hpp
 * \date Febuary 2015
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once


#include <type_traits>

#include <fl/util/meta/operator/not_adaptive.hpp>
#include <fl/util/meta/options_argument.hpp>
#include <fl/model/adaptive_model.hpp>

namespace fl
{

// Forward declaration
template <typename ... > struct ForwardAdaptive;

/**
 * \ingroup meta
 *
 * \brief ForwardAdaptive operator injects the AdaptiveModel interface into the
 * model if the specified \c Model does not implement it. If the model already
 * implements the interface, no changes will be made.
 *
 * The resulting model of ForwardAdaptive<Model>::Type can be used
 * within the adaptive context. For instance, the JointSensor<...>
 * is an adaptive model and therefore its sum-models must be adaptive too. If
 * one of the sub-models does not implement AdaptiveModel it will be
 * translated into an adaptive model using the ForwardAdaptive operator. This is
 * achieved by applying the NotAdaptive decorating operator on the specific
 * model.
 */
template <typename Model>
struct ForwardAdaptive<Model>
    : ForwardAdaptive<
          Model,
          Options<
            !std::is_base_of<internal::AdaptiveModelType, Model>::value
            &
            (std::is_base_of<internal::SensorType, Model>::value |
             std::is_base_of<internal::TransitionType, Model>::value)
          >
      >
{ };

/**
 * \internal
 * \ingroup meta
 *
 * ForwardAdaptive specialization forwarding an adaptive model of \c Model which
 * is decorated by the NotAdaptive operator
 */
template <typename Model>
struct ForwardAdaptive<Model, Options<1>>
{
    typedef NotAdaptive<Model> Type;
};

/**
 * \internal
 * \ingroup meta
 *
 * ForwardAdaptive specialization forwarding the unmodified \c Model which
 * already implements the AdaptiveModel interface.
 */
template <typename Model>
struct ForwardAdaptive<Model, Options<0>>
{
    typedef Model Type;
};



}


