// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_MPM_FLUID_CONSTITUTIVE_MODEL_HPP
#define CUBBYFLOW_MPM_FLUID_CONSTITUTIVE_MODEL_HPP

#include <Core/Matrix/Matrix.hpp>
#include <Core/Utils/Constants.hpp>

#include <cstddef>

namespace CubbyFlow
{
//!
//! \brief Weakly compressible fluid constitutive model for MPM.
//!
//! Computes particle density, Murnaghan-Tait equation-of-state pressure, and
//! the corresponding isotropic Cauchy stress in 2-D or 3-D.
//!
template <size_t N>
class MPMFluidConstitutiveModel final
{
 public:
    static_assert(N == 2 || N == 3,
                  "MPM fluid model supports only 2-D and 3-D.");

    using MatrixType = Matrix<double, N, N>;

    //!
    //! \brief Constructs a weakly compressible MPM fluid model.
    //!
    //! \param[in] targetDensity Positive rest density.
    //! \param[in] speedOfSound Positive artificial speed of sound.
    //! \param[in] eosExponent Equation-of-state exponent, at least one.
    //! \param[in] negativePressureScale Expansion-pressure scale in `[0, 1]`.
    //!
    //! \throws std::invalid_argument If a parameter or the resulting
    //! equation-of-state scale is invalid.
    //!
    explicit MPMFluidConstitutiveModel(double targetDensity = WATER_DENSITY,
                                       double speedOfSound = 100.0,
                                       double eosExponent = 7.0,
                                       double negativePressureScale = 0.0);

    //! Returns the target density.
    [[nodiscard]] double GetTargetDensity() const;

    //! Returns the speed of sound.
    [[nodiscard]] double GetSpeedOfSound() const;

    //! Returns the equation-of-state exponent.
    [[nodiscard]] double GetEosExponent() const;

    //! Returns the negative-pressure scale.
    [[nodiscard]] double GetNegativePressureScale() const;

    //!
    //! \brief Computes density from particle mass and current volume.
    //!
    //! \throws std::invalid_argument If either input or the resulting density
    //! is not finite and positive.
    //!
    [[nodiscard]] double ComputeDensity(double mass,
                                        double currentVolume) const;

    //!
    //! \brief Computes equation-of-state pressure for a density.
    //!
    //! \throws std::invalid_argument If the density is not finite and positive
    //! or the resulting pressure is not finite.
    //!
    [[nodiscard]] double ComputePressure(double density) const;

    //!
    //! \brief Converts pressure to the isotropic Cauchy stress `-p I`.
    //!
    //! \throws std::invalid_argument If the pressure is not finite.
    //!
    [[nodiscard]] MatrixType ComputeCauchyStress(double pressure) const;

 private:
    double m_targetDensity;
    double m_speedOfSound;
    double m_eosExponent;
    double m_negativePressureScale;
    double m_eosScale;
};

using MPMFluidConstitutiveModel2 = MPMFluidConstitutiveModel<2>;
using MPMFluidConstitutiveModel3 = MPMFluidConstitutiveModel<3>;
}  // namespace CubbyFlow

#include <Core/Particle/MPM/MPMFluidConstitutiveModel-Impl.hpp>

#endif
