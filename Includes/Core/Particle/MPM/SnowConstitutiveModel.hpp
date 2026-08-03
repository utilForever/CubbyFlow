// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_SNOW_CONSTITUTIVE_MODEL_HPP
#define CUBBYFLOW_SNOW_CONSTITUTIVE_MODEL_HPP

#include <Core/Matrix/Matrix.hpp>

namespace CubbyFlow
{
//!
//! \brief Per-particle multiplicative deformation state for snow.
//!
//! Decomposes the total deformation gradient as `F = F_E F_P`, where
//! `elastic` and `plastic` store `F_E` and `F_P`, respectively.
//!
template <size_t N>
struct SnowDeformationState
{
    using MatrixType = Matrix<double, N, N>;

    MatrixType elastic = MatrixType::MakeIdentity();
    MatrixType plastic = MatrixType::MakeIdentity();
};

//!
//! \brief Snow-specific elastoplastic constitutive model.
//!
//! Implements the fixed-corotated hyperelastic snow model from "A Material
//! Point Method for Snow Simulation" by A. Stomakhin et al. (2013) for MPM
//! simulations.
//!
template <size_t N>
class SnowConstitutiveModel final
{
 public:
    static_assert(N == 2 || N == 3, "Snow model supports only 2-D and 3-D.");

    using MatrixType = Matrix<double, N, N>;
    using State = SnowDeformationState<N>;

    //!
    //! \brief Constructs a snow constitutive model.
    //!
    //! \param[in] youngsModulus Initial Young's modulus; must be positive and
    //! finite.
    //! \param[in] poissonRatio Poisson's ratio in `(-1, 0.5)`.
    //! \param[in] criticalCompression Principal-stretch compression offset in
    //! `[0, 1)`.
    //! \param[in] criticalStretch Non-negative principal-stretch extension
    //! offset.
    //! \param[in] hardeningCoefficient Non-negative hardening coefficient.
    //!
    explicit SnowConstitutiveModel(double youngsModulus = 1.4e5,
                                   double poissonRatio = 0.2,
                                   double criticalCompression = 2.5e-2,
                                   double criticalStretch = 7.5e-3,
                                   double hardeningCoefficient = 10.0);

    //!
    //! \brief Projects a deformation increment into elastic and plastic parts.
    //!
    //! Forms `F_E_trial = D F_E`, clamps its principal stretches, and transfers
    //! the remainder to `F_P` while preserving the trial total deformation.
    //!
    //! \param[in] deformationGradientIncrement Multiplicative increment `D`
    //! applied on the left.
    //! \param[in] state Current deformation state.
    //!
    //! \return Updated deformation state.
    //!
    [[nodiscard]] State Update(const MatrixType& deformationGradientIncrement,
                               const State& state) const;

    //!
    //! \brief Computes the fixed-corotated Kirchhoff stress for the state.
    //!
    //! Applies the `det(F_P)`-dependent hardening/softening factor to the
    //! elastic response.
    //!
    //! \param[in] state Current deformation state.
    //!
    //! \return Fixed-corotated Kirchhoff stress.
    [[nodiscard]] MatrixType ComputeKirchhoffStress(const State& state) const;

 private:
    [[nodiscard]] static bool IsFinite(const MatrixType& matrix);

    static void ValidateDeformation(const MatrixType& matrix);

    double m_mu0;
    double m_lambda0;
    double m_criticalCompression;
    double m_criticalStretch;
    double m_hardeningCoefficient;
};

using SnowDeformationState2 = SnowDeformationState<2>;
using SnowDeformationState3 = SnowDeformationState<3>;
using SnowConstitutiveModel2 = SnowConstitutiveModel<2>;
using SnowConstitutiveModel3 = SnowConstitutiveModel<3>;
}  // namespace CubbyFlow

#include <Core/Particle/MPM/SnowConstitutiveModel-Impl.hpp>

#endif
