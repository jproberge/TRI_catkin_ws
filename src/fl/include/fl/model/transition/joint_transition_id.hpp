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
 * \file joint_transition_id.hpp
 * \date Febuary 2015
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once


#include <Eigen/Dense>

#include <tuple>
#include <memory>

#include <fl/util/traits.hpp>
#include <fl/util/meta.hpp>

#include <fl/model/transition/interface/transition_function.hpp>

namespace fl
{

// Forward declarations
template <typename...Models> class JointTransition;

/**
 * Traits of JointTransition
 */
template <typename...Models>
struct Traits<JointTransition<Models...>>
{
    enum : signed int
    {
        StateDim = JoinSizes<Traits<Models>::State::SizeAtCompileTime...>::Size,
        NoiseDim = JoinSizes<Traits<Models>::Noise::SizeAtCompileTime...>::Size,
        InputDim = JoinSizes<Traits<Models>::Input::SizeAtCompileTime...>::Size
    };

    typedef typename Traits<
                         typename FirstTypeIn<Models...>::Type
                     >::State::Scalar Scalar;

    typedef Eigen::Matrix<Scalar, StateDim, 1> State;
    typedef Eigen::Matrix<Scalar, NoiseDim, 1> Noise;
    typedef Eigen::Matrix<Scalar, InputDim, 1> Input;

    typedef TransitionInterface<
                State,
                Noise,
                Input
            > TransitionBase;
};

/**
 * \ingroup transitions
 */
template <typename ... Models>
class JointTransition
    : public Traits<
                 JointTransition<Models...>
             >::TransitionBase
{
private:
    /** Typdef of \c This for #from_traits(TypeName) helper */
    typedef JointTransition<Models...> This;

public:
    typedef from_traits(State);
    typedef from_traits(Noise);
    typedef from_traits(Input);

public:
    /**
     * Constructor a JointTransition which is a composition of mixture of
     * fixed and dynamic-size process models.
     *
     * \param models    Variadic list of shared pointers of the models
     */
    JointTransition(const Models& ... models)
        : models_(models...)
    { }

    /**
     * \copydoc TransitionInterface::predict_state
     */
    virtual State predict_state(double delta_time,
                                const State& state,
                                const Noise& noise,
                                const Input& input)
    {
        State prediction = State(state_dimension(), 1);

        predict_state<sizeof...(Models)>(
            models_,
            delta_time,
            state,
            noise,
            input,
            prediction);

        return prediction;
    }

    /**
     * \brief Overridable default destructor
     */
    virtual ~JointTransition() noexcept { }

    /**
     * \copydoc TransitionInterface::state_dimension
     *
     * Determines the joint size of the state vector of all models
     */
    virtual int state_dimension() const
    {
        return expand_state_dimension(CreateIndexSequence<sizeof...(Models)>());
    }

    /**
     * \copydoc TransitionInterface::noise_dimension
     *
     * Determines the joint size of the noise vector of all models
     */
    virtual int noise_dimension() const
    {
        return expand_noise_dimension(CreateIndexSequence<sizeof...(Models)>());
    }

    /**
     * \copydoc TransitionInterface::input_dimension
     *
     * Determines the joint size of the input vector of all models
     */
    virtual int input_dimension() const
    {
        return expand_input_dimension(CreateIndexSequence<sizeof...(Models)>());
    }

    std::tuple<Models...>& models()
    {
        return models_;
    }

    const std::tuple<Models...>& models() const
    {
        return models_;
    }

protected:
    /**
     * \brief Contains the points to the sub-models which this joint model
     *        is composed of.
     */
    std::tuple<Models...> models_;


private:
    /** \cond internal */
    template <int...Indices>
    constexpr int expand_state_dimension(IndexSequence<Indices...>) const
    {
        const auto& dims = { std::get<Indices>(models_).state_dimension()... };

        int joint_dim = 0;
        for (auto dim : dims) { joint_dim += dim; }

        return joint_dim;
    }

    template <int...Indices>
    constexpr int expand_noise_dimension(IndexSequence<Indices...>) const
    {
        const auto& dims = { std::get<Indices>(models_).noise_dimension()... };

        int joint_dim = 0;
        for (auto dim : dims) { joint_dim += dim; }

        return joint_dim;
    }

    template <int...Indices>
    constexpr int expand_input_dimension(IndexSequence<Indices...>) const
    {
        const auto& dims = { std::get<Indices>(models_).input_dimension()... };

        int joint_dim = 0;
        for (auto dim : dims) { joint_dim += dim; }

        return joint_dim;
    }

    template <
        int Size,
        int k = 0,
        typename Tuple,
        typename State,
        typename Noise,
        typename Input
    >
    void predict_state(Tuple& models_tuple,
                       double delta_time,
                       const State& state,
                       const Noise& noise,
                       const Input& input,
                       State& prediction,
                       const int state_offset = 0,
                       const int noise_offset = 0,
                       const int input_offset = 0)
    {
        auto&& model = std::get<k>(models_tuple);

        const auto state_dim = model.state_dimension();
        const auto noise_dim = model.noise_dimension();
        const auto input_dim = model.input_dimension();

        prediction.middleRows(state_offset, state_dim) =
            model.predict_state(
                delta_time,
                state.middleRows(state_offset, state_dim),
                noise.middleRows(noise_offset, noise_dim),
                input.middleRows(input_offset, input_dim)
            );

        if (Size == k + 1) return;

        predict_state<Size, k + (k + 1 < Size ? 1 : 0)>(
                    models_tuple,
                    delta_time,
                    state,
                    noise,
                    input,
                    prediction,
                    state_offset + state_dim,
                    noise_offset + noise_dim,
                    input_offset + input_dim);
    }
    /** \endcond */
};

}



