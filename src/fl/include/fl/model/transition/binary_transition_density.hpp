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
 * \file binary_transition_density.hpp
 * \date June 2015
 * \author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 */

#pragma once


#include <fl/util/types.hpp>
#include <iostream>



namespace fl
{

class BinaryTransitionDensity /// \todo not really a density since it is discrete
{
public:
    BinaryTransitionDensity(const Real& p_1to1,
                            const Real& p_0to1):   p_1to1_(p_1to1),
                                                            p_0to1_(p_0to1)
    {
        /// \todo: this case could be handled properly, but it would be
        /// a little bit more involved
        if(p_0to1 > p_1to1)
        {
            std::cout << "the case of p_0to1 > p_1to1 is not handled in the "
                      << "binary transition density." << std::endl;
        }
    }

    Real probability(const bool& next_state,
                              const bool& state,
                              const Real& dt)
    {
        Real a = std::pow(p_1to1_ - p_0to1_, dt);
        Real prob_1 =  a * Real(state)
                                + (a - 1.) * p_0to1_ / (p_1to1_ - 1. - p_0to1_);

        // limit of p_0to1_ -> 0 and p_1to1_ -> 1
        if(!std::isfinite(prob_1)) prob_1 = Real(state);

        return next_state ? prob_1 : 1. - prob_1;
    }


private:
    Real p_1to1_;
    Real p_0to1_;
};


}


