// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_MPM_FLUID_CONSTITUTIVE_MODEL_IMPL_HPP
#define CUBBYFLOW_MPM_FLUID_CONSTITUTIVE_MODEL_IMPL_HPP

#include <Core/Utils/PhysicsHelpers.hpp>

#include <cmath>
#include <stdexcept>

namespace CubbyFlow
{
template <size_t N>
MPMFluidConstitutiveModel<N>::MPMFluidConstitutiveModel(
    double targetDensity, double speedOfSound, double eosExponent,
    double negativePressureScale)
    : m_targetDensity(targetDensity),
      m_speedOfSound(speedOfSound),
      m_eosExponent(eosExponent),
      m_negativePressureScale(negativePressureScale),
      m_eosScale(targetDensity * speedOfSound * speedOfSound)
{
    if (!std::isfinite(m_targetDensity) || m_targetDensity <= 0.0 ||
        !std::isfinite(m_speedOfSound) || m_speedOfSound <= 0.0 ||
        !std::isfinite(m_eosExponent) || m_eosExponent < 1.0 ||
        !std::isfinite(m_negativePressureScale) ||
        m_negativePressureScale < 0.0 || m_negativePressureScale > 1.0 ||
        !std::isfinite(m_eosScale) || m_eosScale <= 0.0)
    {
        throw std::invalid_argument{ "Invalid MPM fluid model parameters." };
    }
}

template <size_t N>
double MPMFluidConstitutiveModel<N>::GetTargetDensity() const
{
    return m_targetDensity;
}

template <size_t N>
double MPMFluidConstitutiveModel<N>::GetSpeedOfSound() const
{
    return m_speedOfSound;
}

template <size_t N>
double MPMFluidConstitutiveModel<N>::GetEosExponent() const
{
    return m_eosExponent;
}

template <size_t N>
double MPMFluidConstitutiveModel<N>::GetNegativePressureScale() const
{
    return m_negativePressureScale;
}

template <size_t N>
double MPMFluidConstitutiveModel<N>::ComputeDensity(double mass,
                                                    double currentVolume) const
{
    if (!std::isfinite(mass) || mass <= 0.0 || !std::isfinite(currentVolume) ||
        currentVolume <= 0.0)
    {
        throw std::invalid_argument{ "Invalid MPM fluid mass or volume." };
    }

    const double density = mass / currentVolume;

    if (!std::isfinite(density) || density <= 0.0)
    {
        throw std::invalid_argument{ "Invalid MPM fluid density." };
    }

    return density;
}

template <size_t N>
double MPMFluidConstitutiveModel<N>::ComputePressure(double density) const
{
    if (!std::isfinite(density) || density <= 0.0)
    {
        throw std::invalid_argument{ "Invalid MPM fluid density." };
    }

    const double pressure =
        ComputePressureFromEos(density, m_targetDensity, m_eosScale,
                               m_eosExponent, m_negativePressureScale);

    if (!std::isfinite(pressure))
    {
        throw std::invalid_argument{ "Non-finite MPM fluid pressure." };
    }

    return pressure;
}

template <size_t N>
MPMFluidConstitutiveModel<N>::MatrixType
MPMFluidConstitutiveModel<N>::ComputeCauchyStress(double pressure) const
{
    if (!std::isfinite(pressure))
    {
        throw std::invalid_argument{ "Invalid MPM fluid pressure." };
    }

    return -pressure * MatrixType::MakeIdentity();
}
}  // namespace CubbyFlow

#endif
