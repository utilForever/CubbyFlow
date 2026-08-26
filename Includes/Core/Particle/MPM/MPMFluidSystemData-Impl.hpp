// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_MPM_FLUID_SYSTEM_DATA_IMPL_HPP
#define CUBBYFLOW_MPM_FLUID_SYSTEM_DATA_IMPL_HPP

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace CubbyFlow
{
template <size_t N>
MPMFluidSystemData<N>::MPMFluidSystemData(const Vector<size_t, N>& resolution,
                                          const Vector<double, N>& gridSpacing,
                                          const Vector<double, N>& gridOrigin,
                                          size_t numberOfParticles)
    : Base{ resolution, gridSpacing, gridOrigin, numberOfParticles },
      m_volumeRatios(numberOfParticles, 1.0),
      m_velocityGradients(numberOfParticles, MatrixType{})
{
    // Do nothing
}

template <size_t N>
void MPMFluidSystemData<N>::Resize(size_t newNumberOfParticles)
{
    Base::Resize(newNumberOfParticles);

    m_volumeRatios.Resize(newNumberOfParticles, 1.0);
    m_velocityGradients.Resize(newNumberOfParticles, MatrixType{});
}

template <size_t N>
void MPMFluidSystemData<N>::Deserialize(const std::vector<uint8_t>& buffer)
{
    Base::Deserialize(buffer);
    ResetFluidState();
}

template <size_t N>
void MPMFluidSystemData<N>::Set(const ParticleSystemData<N>& other)
{
    Base::Set(other);
    ResetFluidState();
}

template <size_t N>
ConstArrayView1<double> MPMFluidSystemData<N>::VolumeRatios() const
{
    return m_volumeRatios.View();
}

template <size_t N>
ArrayView1<double> MPMFluidSystemData<N>::VolumeRatios()
{
    return m_volumeRatios.View();
}

template <size_t N>
ConstArrayView1<typename MPMFluidSystemData<N>::MatrixType>
MPMFluidSystemData<N>::VelocityGradients() const
{
    return m_velocityGradients.View();
}

template <size_t N>
ArrayView1<typename MPMFluidSystemData<N>::MatrixType>
MPMFluidSystemData<N>::VelocityGradients()
{
    return m_velocityGradients.View();
}

template <size_t N>
void MPMFluidSystemData<N>::TransferFromGridToParticles(
    double timeStepInSeconds)
{
    if (!std::isfinite(timeStepInSeconds) || timeStepInSeconds < 0.0)
    {
        throw std::invalid_argument("Invalid MPM fluid time step.");
    }

    this->ValidateGridToParticleState();

    Array1<double> nextVolumeRatios(this->NumberOfParticles(), 1.0);
    Array1<MatrixType> nextVelocityGradients(this->NumberOfParticles(),
                                             MatrixType{});

    for (size_t i = 0; i < this->NumberOfParticles(); ++i)
    {
        if (!std::isfinite(m_volumeRatios[i]) || m_volumeRatios[i] <= 0.0)
        {
            throw std::invalid_argument("Invalid MPM fluid volume ratio.");
        }

        const MatrixType gradient = this->ComputeVelocityGradient(i);
        const double volumeIncrement =
            (MatrixType::MakeIdentity() + timeStepInSeconds * gradient)
                .Determinant();
        const double volumeRatio = m_volumeRatios[i] * volumeIncrement;

        if (!std::isfinite(volumeIncrement) || volumeIncrement <= 0.0 ||
            !std::isfinite(volumeRatio) || volumeRatio <= 0.0)
        {
            throw std::invalid_argument("Invalid MPM fluid volume update.");
        }

        nextVelocityGradients[i] = gradient;
        nextVolumeRatios[i] = volumeRatio;
    }

    this->TransferFromGridToParticlesUnchecked();

    std::ranges::copy(nextVolumeRatios, m_volumeRatios.begin());
    std::ranges::copy(nextVelocityGradients, m_velocityGradients.begin());
}

template <size_t N>
void MPMFluidSystemData<N>::ResetFluidState()
{
    m_volumeRatios.Fill(1.0);
    m_velocityGradients.Fill(MatrixType{});
}

}  // namespace CubbyFlow

#endif
