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
 * \file joint_distribution_iid.hpp
 * \date Febuary 2015
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once


#include <Eigen/Dense>

#include <fl/util/meta.hpp>
#include <fl/util/traits.hpp>

#include <fl/distribution/interface/moments.hpp>

namespace fl
{

// Forward declaration
template <typename...Distributions> class JointDistribution;

/**
 * \internal
 * Traits of JointDistribution<Distribution, Count>
 */
template <typename Distribution, int Count>
struct Traits<JointDistribution<MultipleOf<Distribution, Count>>>
{
    typedef typename Distribution::Variate MarginalVariate;

    enum : signed int
    {
        MarginalCount = Count,
        JointSize = ExpandSizes<SizeOf<MarginalVariate>::Value, Count>::Size
    };

    typedef typename MarginalVariate::Scalar Scalar;

    typedef Eigen::Matrix<Scalar, JointSize, 1> Variate;
};

/**
 * \ingroup distributions
 */
template <typename MarginalDistribution, int Count>
class JointDistribution<MultipleOf<MarginalDistribution, Count>>
    : public Moments<
                typename Traits<
                    JointDistribution<MultipleOf<MarginalDistribution, Count>>
                >::Variate>
{
public:
    /** Typdef of \c This for #from_traits(TypeName) helper */
    typedef JointDistribution This;

    typedef typename Traits<This>::Variate Variate;
    typedef typename Moments<Variate>::SecondMoment SecondMoment;
    typedef Eigen::Array<MarginalDistribution, Count, 1> MarginalDistributions;

public:
    explicit
    JointDistribution(MarginalDistribution marginal,
                      int count = ToDimension<Count>::Value)
        : distributions_(MarginalDistributions(count, 1))
    {
        assert(count > 0);

        for (int i = 0; i < distributions_.rows(); ++i)
        {
            distributions_(i) = marginal;
        }

        dimension_ = marginal.dimension() * count;
    }

    /**
     * \brief Overridable default destructor
     */
    virtual ~JointDistribution() noexcept { }

    virtual Variate mean() const
    {
        Variate mu = Variate(dimension(), 1);

        int offset = 0;
        for (int i = 0; i < distributions_.size(); ++i)
        {
            const MarginalDistribution& marginal = distributions_(i);
            int dim =  marginal.dimension();

            mu.middleRows(offset, dim) = marginal.mean();

            offset += dim;
        }

        return mu;
    }

    virtual SecondMoment covariance() const
    {
        SecondMoment cov = SecondMoment::Zero(dimension(), dimension());

        int offset = 0;
        for (int i = 0; i < distributions_.size(); ++i)
        {
            const MarginalDistribution& marginal = distributions_(i);
            int dim =  marginal.dimension();

            cov.block(offset, offset, dim, dim) = marginal.covariance();

            offset += dim;
        }

        return cov;
    }

    virtual int dimension() const
    {
        return dimension_;
    }

    MarginalDistributions& distributions()
    {
        return distributions_;
    }

    const MarginalDistributions& distributions() const
    {
        return distributions_;
    }

    MarginalDistribution& distribution(int index)
    {
        assert(index < distributions_.size());
        return distributions_(index);
    }

    const MarginalDistribution& distribution(int index) const
    {
        assert(index < distributions_.size());
        return distributions_(index);
    }

protected:
    MarginalDistributions distributions_;
    int dimension_;
};

}


