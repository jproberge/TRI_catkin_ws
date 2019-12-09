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
 * \file gaussian.hpp
 * \date October 2014
 * \author Jan Issac (jan.issac@gmail.com)
 * \author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 */

#pragma once


#include <Eigen/Dense>

#include <array>
#include <string>
#include <cstddef>
#include <type_traits>

#include <fl/util/traits.hpp>
#include <fl/exception/exception.hpp>
#include <fl/distribution/interface/moments.hpp>
#include <fl/distribution/interface/evaluation.hpp>
#include <fl/distribution/interface/standard_gaussian_mapping.hpp>

namespace fl
{

/**
 * \ingroup exceptions
 *
 * Exception used in case of accessing dynamic-size distribution attributes
 * without initializing the Gaussian using a dimension greater 0.
 */
class GaussianUninitializedException
    : public Exception
{
public:
    /**
     * Creates a GaussianUninitializedException
     */
    GaussianUninitializedException()
        : Exception("Accessing uninitialized dynamic-size distribution. "
                    "Gaussian dimension is 0. "
                    "Use ::dimension(dimension) to initialize the "
                    "distribution!") { }

    /**
     * \return Exception name
     */
    virtual std::string name() const noexcept
    {
        return "fl::GaussianUninitializedException";
    }
};

/**
 * \ingroup exceptions
 *
 * Exception representing a unsupported representation ID
 */
class InvalidGaussianRepresentationException
    : public Exception
{
public:
    /**
     * Creates an InvalidGaussianRepresentationException
     */
    InvalidGaussianRepresentationException()
        : Exception("Invalid Gaussian covariance representation") { }

    /**
     * \return Exception name
     */
    virtual std::string name() const noexcept
    {
        return "fl::InvalidGaussianRepresentationException";
    }
};


/**
 * \class Gaussian
 *
 * \brief General Gaussian Distribution
 * \ingroup distributions
 * \{
 *
 * The Gaussian is a general purpose distribution representing a multi-variate
 * \f${\cal N}(x; \mu, \Sigma)\f$. It can be used in various
 * ways while maintaining efficienty at the same time. This is due to it's
 * multi-representation structure. The distribution can be represented either by
 *
 *  - the covariance matrix \f$\Sigma\f$,
 *  - the precision matrix \f$\Sigma^{-1} = \Lambda\f$,
 *  - the covariance square root matrix (Cholesky decomposition or LDLT)
 *    \f$\sqrt{\Sigma} = L\sqrt{D}\f$,
 *  - or the diagonal form of the previous three options
 *    \f$diag(\sigma_1, \ldots, \sigma_n)\f$.
 *
 * A change in one representation results in change of all other
 * representations.
 *
 * Two key features of the distribution are its aibility to evaluation the
 * probability of a given sample and to map a noise sample into the distribution
 * sample space.
 *
 * \cond internal
 * The Gaussian internal structure uses lazy assignments or write on read
 * technique. Due to the multi-representation of the Gaussian, modifying one
 * representation affects all remaining ones. If one of the representation is
 * modified, the other representations are only then updated when needed. This
 * minimizes redundant computation and increases efficienty.
 * \endcond
 */
template <typename Variate>
class Gaussian
    : public Moments<Variate>,
      public Evaluation<Variate>,
      public StandardGaussianMapping<Variate, SizeOf<Variate>::Value>
{
private:
    typedef StandardGaussianMapping<
                Variate,
                SizeOf<Variate>::Value
            > StdGaussianMappingBase;

public:
    /**
     * \brief Second moment matrix type, i.e covariance matrix, precision
     *        matrix, and their diagonal and square root representations
     */
    typedef typename Moments<Variate>::SecondMoment SecondMoment;

    /**
     * \brief Represents the StandardGaussianMapping standard variate type which
     *        is of the same dimension as the Gaussian Variate. The
     *        StandardVariate type is used to sample from a standard normal
     *        Gaussian and map it to this Gaussian
     */
    typedef typename StdGaussianMappingBase::StandardVariate StandardVariate;

protected:
    /** \cond internal */
    /**
     * \enum Attribute
     * Implementation attributes. The enumeration lists the different
     * representations along with other properties such as the rank of the
     * second moment and the log normalizer.
     */
    enum Attribute
    {
        CovarianceMatrix = 0,     /**< Covariance mat. */
        PrecisionMatrix,          /**< Inverse of the cov. mat. */
        SquareRootMatrix,         /**< Cholesky decomp. of the cov. mat. */
        DiagonalCovarianceMatrix, /**< Diagonal form of the of cov. mat. */
        DiagonalPrecisionMatrix,  /**< Diagonal form of the inv cov. mat. */
        DiagonalSquareRootMatrix, /**< Diagonal form of the Cholesky decomp. */
        Rank,                     /**< Covariance Rank */
        Normalizer,               /**< Log probability normalizer */
        Determinant,              /**< Determinant of covariance */

        Attributes                /**< Total number of attribute */
    };

    /**
     * \brief Flags array type which contains the content status if different
     *        distribution representation
     */
    typedef std::array<bool, Attributes> FlagArray;
    /** \endcond */

public:
    /**
     * Creates a dynamic or fixed size Gaussian.
     *
     * \param dimension Dimension of the Gaussian. The default is defined by the
     *                  dimension of the variable type \em Vector. If the size
     *                  of the Variate at compile time is fixed, this will be
     *                  adapted. For dynamic-sized Variable the dimension is
     *                  initialized to 0.
     */
    explicit Gaussian(int dim = DimensionOf<Variate>()):
        StdGaussianMappingBase(dim)
    {
        static_assert(Variate::SizeAtCompileTime != 0,
                      "Illegal static dimension");

        std::fill(dirty_.begin(), dirty_.end(), true);
        set_standard();
    }

    /**
     * \brief Overridable default destructor
     */
    virtual ~Gaussian() noexcept { }

    /**
     * \return Gaussian dimension
     */
    virtual int dimension() const
    {
        return StdGaussianMappingBase::standard_variate_dimension();
    }

    /**
     * \return Gaussian first moment
     */
    virtual const Variate& mean() const
    {
        return mean_;
    }

    /**
     * \return Gaussian second centered moment
     *
     * Computes the covariance from other representation of not available
     *
     * \throws GaussianUninitializedException if the Gaussian is of dynamic-size
     *         and has not been initialized using SetStandard(dimension).
     * \throws InvalidGaussianRepresentationException if non-of theember function)
     *         representation can be used as a source
     */
    virtual const SecondMoment& covariance() const
    {
        if (dimension() == 0)
        {
            fl_throw(GaussianUninitializedException());
        }

        if (is_dirty(CovarianceMatrix) && is_dirty(DiagonalCovarianceMatrix))
        {
            switch (select_first_representation<4>({{DiagonalSquareRootMatrix,
                                                    DiagonalPrecisionMatrix,
                                                    SquareRootMatrix,
                                                    PrecisionMatrix}}))
            {
            case SquareRootMatrix:
                covariance_ = square_root_ * square_root_.transpose();
                break;

            case PrecisionMatrix:
                covariance_ = precision_.inverse();
                break;

            case DiagonalSquareRootMatrix:
                covariance_.setZero(dimension(), dimension());
                for (int i = 0; i < square_root_.diagonalSize(); ++i)
                {
                    covariance_(i, i) = square_root_(i, i) * square_root_(i, i);
                }
                break;

            case DiagonalPrecisionMatrix:
                covariance_.setZero(dimension(), dimension());
                for (int i = 0; i < precision_.diagonalSize(); ++i)
                {
                    covariance_(i, i) = 1./precision_(i, i);
                }
                break;

            default:
                fl_throw(InvalidGaussianRepresentationException());
                break;
            }

            updated_internally(CovarianceMatrix);
        }

        return covariance_;
    }

    /**
     * \return Gaussian second centered moment in the precision form (inverse
     * of the covariance)
     *
     * Computes the precision from other representation of not available
     *
     * \throws GaussianUninitializedException if the Gaussian is of dynamic-size
     *         and has not been initialized using SetStandard(dimension).
     * \throws InvalidGaussianRepresentationException if non-of the
     *         representation can be used as a source
     */
    virtual const SecondMoment& precision() const
    {
        if (dimension() == 0)
        {
            fl_throw(GaussianUninitializedException());
        }

        if (is_dirty(PrecisionMatrix) && is_dirty(DiagonalPrecisionMatrix))
        {
            const SecondMoment& cov = covariance();

            switch (select_first_representation<4>({{DiagonalCovarianceMatrix,
                                                    DiagonalSquareRootMatrix,
                                                    CovarianceMatrix,
                                                    SquareRootMatrix}}))
            {
            case CovarianceMatrix:
            case SquareRootMatrix:
                precision_ = covariance().inverse();
                break;

            case DiagonalCovarianceMatrix:
            case DiagonalSquareRootMatrix:
                precision_.setZero(dimension(), dimension());
                for (int i = 0; i < cov.rows(); ++i)
                {
                    precision_(i, i) = 1./cov(i, i);
                }
                break;

            default:
                fl_throw(InvalidGaussianRepresentationException());
                break;
            }

            updated_internally(PrecisionMatrix);
        }

        return precision_;
    }


    /**
     * \return Gaussian second centered moment in the square root form (
     * Cholesky decomposition)
     *
     * Computes the square root from other representation of not available
     *
     * \throws GaussianUninitializedException if the Gaussian is of dynamic-size
     *         and has not been initialized using SetStandard(dimension).
     * \throws InvalidGaussianRepresentationException if non-of the
     *         representation can be used as a source
     */
    virtual const SecondMoment& square_root() const
    {
        if (dimension() == 0)
        {
            fl_throw(GaussianUninitializedException());
        }

        if (is_dirty(SquareRootMatrix) && is_dirty(DiagonalSquareRootMatrix))
        {
            const SecondMoment& cov = covariance();

            switch (select_first_representation<4>({{DiagonalCovarianceMatrix,
                                                    DiagonalPrecisionMatrix,
                                                    CovarianceMatrix,
                                                    PrecisionMatrix}}))
            {
            case CovarianceMatrix:
            case PrecisionMatrix:
                fl::square_root(cov, square_root_);
                break;

            case DiagonalCovarianceMatrix:
            case DiagonalPrecisionMatrix:
                square_root_.setZero(dimension(), dimension());
                for (int i = 0; i < square_root_.rows(); ++i)
                {
                    square_root_(i, i) = std::sqrt(cov(i, i));
                }
                break;

            default:
                fl_throw(InvalidGaussianRepresentationException());
                break;
            }

            updated_internally(SquareRootMatrix);
        }

        return square_root_;
    }

    /**
     * \return True if the covariance matrix has a full rank
     *
     * \throws see covariance()
     */
    virtual bool has_full_rank() const
    {
        if (is_dirty(Rank))
        {
            full_rank_ =
               covariance().colPivHouseholderQr().rank() == covariance().rows();

            updated_internally(Rank);
        }

        return full_rank_;
    }

    /**
     * \return Log normalizing constant
     *
     * \throws see has_full_rank()
     */
    virtual Real log_normalizer() const
    {
        if (is_dirty(Normalizer))
        {
            if (has_full_rank())
            {
                log_norm_ =
                    -0.5
                    * (std::log(covariance_determinant())
                       + Real(dimension()) * std::log(2.0 * M_PI));
            }
            else
            {
                log_norm_ = 0.0; // FIXME
            }

            updated_internally(Normalizer);
        }

        return log_norm_;
    }

    /**
     * \return Covariance determinant
     *
     * \throws see covariance
     */
    virtual Real covariance_determinant() const
    {
        if (is_dirty(Determinant))
        {
            determinant_ = covariance().determinant();

            updated_internally(Determinant);
        }

        return determinant_;
    }

    /**
     * \return Log of the probability of the given sample \c vector
     *
     * \param vector sample which should be evaluated
     *
     * \throws see has_full_rank()
     */
    Real log_probability(const Variate& vector) const override
    {
        // assert(has_full_rank());

        if(has_full_rank())
        {
            return log_normalizer() - 0.5
                    * (vector - mean()).dot(precision() * (vector - mean()));
        }

        return -std::numeric_limits<Real>::infinity();
    }

    /**
     * \return a Gaussian sample of the type \c Variate determined by mapping a
     * noise sample into the Gaussian sample space
     *
     * \param sample    Noise Sample
     *
     * \throws see square_root()
     */
    Variate map_standard_normal(const StandardVariate& sample) const override
    {
        return mean() + square_root() * sample;
    }

    /**
     * Sets the Gaussian to a standard distribution with zero mean and identity
     * covariance.
     */
    virtual void set_standard()
    {
        mean_.resize(dimension());
        covariance_.resize(dimension(), dimension());
        precision_.resize(dimension(), dimension());
        square_root_.resize(dimension(), dimension());

        mean(Variate::Zero(dimension()));
        covariance(SecondMoment::Identity(dimension(), dimension()));

        full_rank_ = true;
        updated_internally(Rank);
    }

    /**
     * Changes the dimension of the dynamic-size Gaussian and sets it to a
     * standard distribution with zero mean and identity covariance.
     *
     * \param new_dimension New dimension of the Gaussian
     *
     * \throws ResizingFixedSizeEntityException
     *         see GaussianMap::standard_variate_dimension(int)
     */
    virtual void dimension(int new_dimension)
    {
        StdGaussianMappingBase::standard_variate_dimension(new_dimension);
        set_standard();
    }

    /**
     * Sets the mean
     *
     * \param mean New Gaussian mean
     *
     * \throws WrongSizeException
     */
    virtual void mean(const Variate& mean) noexcept
    {
        if (mean_.size() != mean.size())
        {
            fl_throw(fl::WrongSizeException(mean.size(), mean_.size()));
        }

        mean_ = mean;
    }

    /**
     * Sets the covariance matrix
     * \param covariance New covariance matrix
     *
     * \throws WrongSizeException
     */

    virtual void covariance(const SecondMoment& covariance)
    {
        if (covariance_.size() != covariance.size())
        {
            fl_throw(fl::WrongSizeException(
                         covariance.size(), covariance_.size()));
        }

        covariance_ = covariance;
        updated_externally(CovarianceMatrix);
    }

    /**
     * Sets the covariance matrix in the form of its square root (Cholesky f
     * actor)
     *
     * \param square_root New covariance square root
     *
     * \throws WrongSizeException
     */
    virtual void square_root(const SecondMoment& square_root)
    {
        if (square_root_.size() != square_root.size())
        {
            fl_throw(fl::WrongSizeException(
                         square_root.size(), square_root_.size()));
        }

        square_root_ = square_root;
        updated_externally(SquareRootMatrix);
    }

    /**
     * Sets the covariance matrix in the precision form (inverse of covariance)
     *
     * \param precision New precision matrix
     *
     * \throws WrongSizeException
     */
    virtual void precision(const SecondMoment& precision) noexcept
    {
        if (precision_.size() != precision.size())
        {
            fl_throw(fl::WrongSizeException(
                         precision.size(), precision_.size()));
        }

        precision_ = precision;
        updated_externally(PrecisionMatrix);
    }

    /**
     * Sets the covariance matrix as a diagonal matrix
     *
     * \param diag_covariance New diagonal covariance matrix
     *
     * \throws WrongSizeException
     */
    virtual void diagonal_covariance(const SecondMoment& diag_covariance)
    {
        if (diag_covariance.size() != covariance_.size())
        {
            fl_throw(
                fl::WrongSizeException(
                    diag_covariance.size(), covariance_.size()));
        }

        covariance_ = diag_covariance.diagonal().asDiagonal();
        updated_externally(DiagonalCovarianceMatrix);
    }

    /**
     * Sets the covariance matrix in its diagonal square root form
     *
     * \param diag_square_root New diagonal square root of the covariance
     *
     * \throws WrongSizeException
     */
    virtual void diagonal_square_root(const SecondMoment& diag_square_root)
    {
        if (diag_square_root.size() != square_root_.size())
        {
            fl_throw(
                fl::WrongSizeException(
                    diag_square_root.size(), square_root_.size()));
        }

        square_root_ = diag_square_root.diagonal().asDiagonal();
        updated_externally(DiagonalSquareRootMatrix);
    }

    /**
     * Sets the covariance matrix in its diagonal precision form
     *
     * \param diag_precision New diagonal precision matrix
     *
     * \throws WrongSizeException
     */
    virtual void diagonal_precision(const SecondMoment& diag_precision)
    {
        if (diag_precision.size() != precision_.size())
        {
            fl_throw(
                fl::WrongSizeException(
                    diag_precision.size(), precision_.size()));
        }

        precision_ = diag_precision.diagonal().asDiagonal();
        updated_externally(DiagonalPrecisionMatrix);
    }

    /**
     * \brief Returns true if the specified Gaussian is close to this one w.r.t.
     *        \a an epsilon.
     *
     * \throws See covariance() and mean()
     */
    template <typename OtherVariate>
    bool is_approx(const Gaussian<OtherVariate>& other)
    {


        return fl::are_similar(mean(), other.mean()) &&
               fl::are_similar(covariance(), other.covariance());
    }

    /**
     * \brief Returns true if the specified Gaussian is close to this one w.r.t.
     *        \a an epsilon.
     *
     * \throws See covariance() and mean()
     */
    template <typename OtherVariate>
    bool is_approx(const Gaussian<OtherVariate>& other,
                   Real eps = 1.e-9,
                   bool scale_with_size = false)
    {
        Real eps_mean = eps;
        Real eps_cov = eps;

        if (scale_with_size)
        {
            eps_mean *= mean().size();
            eps_cov *= covariance().size();
        }

        return fl::are_similar(mean(), other.mean(), eps_mean) &&
               fl::are_similar(covariance(), other.covariance(), eps_cov);
    }

protected:
    /** \cond internal */
    /**
     * Flags the specified attribute as valid and the rest of attributes as
     * dirty.
     *
     * \param attribute Modified attribute
     */
    void updated_externally(Attribute attribute) const noexcept
    {
        std::fill(dirty_.begin(), dirty_.end(), true);
        updated_internally(attribute);
    }

    /**
     * Flags the specified attribute as valid.
     *
     * \param attribute Modified attribute
     */
    void updated_internally(Attribute attribute) const noexcept
    {
        dirty_[attribute] = false;
    }

    /**
     * \return True if any of the other representation was modified.
     * \param attribute     Attribute in question
     */
    bool is_dirty(Attribute attribute) const noexcept
    {
        return dirty_[int(attribute)];
    }

    /**
     * \return First representation ID that is available
     *
     * \param representations   Representation list
     *
     * Example:
     * If the last invoked functions were
     *
     * \code
     * diagonal_covariance(my_diagonal);
     * my_covariance = covariance();
     * \endcode
     *
     * Now, the representation is set to \c DiagonalCovarianceMatrix and
     * \c CovarianceMatrix since \c diagonal_covariance() was used to set the
     * covariance matrix followed by requesting \c covariance().
     * The following subsequent call
     *
     * \code
     * Attribute att = SelectRepresentation({SquareRoot,
     *                                       DiagonalCovarianceMatrix,
     *                                       CovarianceMatrix});
     * \endcode
     *
     * will assign att to DiagonalCovarianceMatrix since that is the first
     * available representation within the initializer-list
     * <tt>{#SquareRoot, #DiagonalCovarianceMatrix, #CovarianceMatrix}</tt>.
     *
     * This method is used to determine the best suitable representation
     * for conversion. It is recommanded to put the diagonal forms at the
     * beginning of the initialization-list. Diagonal forms can be converted
     * most efficiently other  representations.
     */
    template <int AttributeCount>
    Attribute select_first_representation(
        const std::array<Attribute, AttributeCount>& representations
    ) const noexcept
    {
        for (auto& rep: representations)  if (!is_dirty(rep)) return rep;
        return Attributes;
    }
    /** \endcond */

protected:
    /** \cond internal */
    Variate mean_;                     /**< \brief first moment vector */
    mutable SecondMoment covariance_;  /**< \brief cov. form */
    mutable SecondMoment precision_;   /**< \brief cov. inverse form */
    mutable SecondMoment square_root_; /**< \brief cov. square root form */
    mutable bool full_rank_;           /**< \brief full rank flag */
    mutable Real log_norm_;            /**< \brief log normalizing constant */
    mutable Real determinant_;         /**< \brief determinant of covariance */
    mutable FlagArray dirty_;          /**< \brief data validity flags */
    /** \endcond */
};

/** \} */

}
