/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *  Copyright (c) 2016, ETH Zurich, Wyss Zurich, Zurich Eye
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Sep 12, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Zurich Eye
 *********************************************************************************/

/**
 * @file RelativePoseError.hpp
 * @brief Header file for the RelativePoseError class.
 * @author Stefan Leutenegger
 */

#pragma once

#pragma diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// Eigen 3.2.7 uses std::binder1st and std::binder2nd which are deprecated since c++11
// Fix is in 3.3 devel (http://eigen.tuxfamily.org/bz/show_bug.cgi?id=872).
#include <ceres/ceres.h>
#pragma diagnostic pop

#include "gici/estimate/error_interface.h"

namespace gici
{

/// \brief Relative error between two poses.
class RelativePoseError : public ceres::SizedCostFunction<6 /* number of residuals */, 7, /* size of first parameter */
                                                          7 /* size of second parameter */>,
                          public ErrorInterface
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// \brief The base class type.
    typedef ceres::SizedCostFunction<6, 7, 7> base_t;

    /// \brief Number of residuals (6).
    static const int kNumResiduals = 6;

    /// \brief The information matrix type (6x6).
    typedef Eigen::Matrix<double, 6, 6> information_t;

    /// \brief The covariance matrix type (same as information).
    typedef Eigen::Matrix<double, 6, 6> covariance_t;

    /// \brief Default constructor.
    RelativePoseError();

    /// \brief Construct with measurement and information matrix
    /// @param[in] information The information (weight) matrix.
    RelativePoseError(const Eigen::Matrix<double, 6, 6> &information);

    /// \brief Construct with measurement and variance.
    /// @param[in] translationVariance The (relative) translation variance.
    /// @param[in] rotationVariance The (relative) rotation variance.
    RelativePoseError(double translationVariance, double rotationVariance);

    /// \brief Trivial destructor.
    virtual ~RelativePoseError()
    {
    }

    // setters
    /// \brief Set the information.
    /// @param[in] information The information (weight) matrix.
    void setInformation(const information_t &information);

    // getters
    /// \brief Get the information matrix.
    /// \return The information (weight) matrix.
    const information_t &information() const
    {
        return information_;
    }

    /// \brief Get the covariance matrix.
    /// \return The inverse information (covariance) matrix.
    const information_t &covariance() const
    {
        return covariance_;
    }

    // error term and Jacobian implementation
    /**
     * @brief This evaluates the error term and additionally computes the Jacobians.
     * @param parameters Pointer to the parameters (see ceres)
     * @param residuals Pointer to the residual vector (see ceres)
     * @param jacobians Pointer to the Jacobians (see ceres)
     * @return success of th evaluation.
     */
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    /**
     * @brief This evaluates the error term and additionally computes
     *        the Jacobians in the minimal internal representation.
     * @param parameters Pointer to the parameters (see ceres)
     * @param residuals Pointer to the residual vector (see ceres)
     * @param jacobians Pointer to the Jacobians (see ceres)
     * @param jacobians_minimal Pointer to the minimal Jacobians (equivalent to jacobians).
     * @return Success of the evaluation.
     */
    bool EvaluateWithMinimalJacobians(double const *const *parameters, double *residuals, double **jacobians,
                                      double **jacobians_minimal) const;

    // sizes
    /// \brief Residual dimension.
    size_t residualDim() const
    {
        return kNumResiduals;
    }

    /// \brief Number of parameter blocks.
    size_t parameterBlocks() const
    {
        return parameter_block_sizes().size();
    }

    /// \brief Dimension of an individual parameter block.
    size_t parameterBlockDim(size_t parameter_block_idx) const
    {
        return base_t::parameter_block_sizes().at(parameter_block_idx);
    }

    /// @brief Residual block type as string
    virtual ErrorType typeInfo() const
    {
        return ErrorType::kRelativePoseError;
    }

    // Convert normalized residual to raw residual
    virtual void deNormalizeResidual(double *residuals) const
    {
        Eigen::Map<Eigen::Matrix<double, 6, 1>> Residual(residuals);
        Residual = square_root_information_inverse_ * Residual;
    }

  protected:
    // weighting related
    information_t information_;             ///< The 6x6 information matrix.
    information_t square_root_information_; ///< The 6x6 square root information matrix.
    information_t square_root_information_inverse_;
    covariance_t covariance_; ///< The 6x6 covariance matrix.
};

} // namespace gici
