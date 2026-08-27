// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_MPM_FLUID_SOLVER_IMPL_HPP
#define CUBBYFLOW_MPM_FLUID_SOLVER_IMPL_HPP

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace CubbyFlow
{
template <size_t N>
MPMFluidSolver<N>::MPMFluidSolver(const SizeType& resolution,
                                  const VectorType& gridSpacing,
                                  const VectorType& gridOrigin, double radius,
                                  double mass, double targetDensity,
                                  double speedOfSound, double eosExponent,
                                  double negativePressureScale)
    : Base{ radius, mass },
      m_mpmSystemData{ std::make_shared<MPMFluidSystemData<N>>(
          resolution, gridSpacing, gridOrigin) },
      m_constitutiveModel{ targetDensity, speedOfSound, eosExponent,
                           negativePressureScale }
{
    if (!std::isfinite(radius) || radius < 0.0 || !std::isfinite(mass) ||
        mass <= 0.0)
    {
        throw std::invalid_argument{ "Invalid MPM fluid particle parameters." };
    }

    m_mpmSystemData->SetRadius(radius);
    m_mpmSystemData->SetMass(mass);
    this->SetParticleSystemData(m_mpmSystemData);
    this->SetIsUsingFixedSubTimeSteps(false);
}

template <size_t N>
std::shared_ptr<MPMFluidSystemData<N>> MPMFluidSolver<N>::GetMPMSystemData()
    const
{
    return m_mpmSystemData;
}

template <size_t N>
const MPMFluidConstitutiveModel<N>& MPMFluidSolver<N>::GetConstitutiveModel()
    const
{
    return m_constitutiveModel;
}

template <size_t N>
double MPMFluidSolver<N>::GetTimeStepLimitScale() const
{
    return m_timeStepLimitScale;
}

template <size_t N>
void MPMFluidSolver<N>::SetTimeStepLimitScale(double newScale)
{
    if (!std::isfinite(newScale) || newScale <= 0.0 || newScale > 1.0)
    {
        throw std::invalid_argument{
            "Time-step limit scale must be in (0, 1]."
        };
    }

    m_timeStepLimitScale = newScale;
}

template <size_t N>
MPMFluidSolver<N>::Builder MPMFluidSolver<N>::GetBuilder()
{
    return Builder{};
}

template <size_t N>
void MPMFluidSolver<N>::OnInitialize()
{
    Base::OnInitialize();
    m_mpmSystemData->TransferFromParticlesToGrid();
    InitializeReferenceVolumes();
}

template <size_t N>
unsigned int MPMFluidSolver<N>::GetNumberOfSubTimeSteps(
    double timeIntervalInSeconds) const
{
    if (!std::isfinite(timeIntervalInSeconds) || timeIntervalInSeconds <= 0.0 ||
        m_mpmSystemData->NumberOfParticles() == 0)
    {
        return 1;
    }

    const auto spacing = m_mpmSystemData->GridMass().GridSpacing();
    const auto velocities = m_mpmSystemData->Velocities();
    double minSpacing = spacing[0];
    double maxSpeed = 0.0;

    for (size_t axis = 1; axis < N; ++axis)
    {
        minSpacing = std::min(minSpacing, spacing[axis]);
    }

    for (const auto& velocity : velocities)
    {
        for (double component : velocity)
        {
            if (!std::isfinite(component))
            {
                throw std::invalid_argument{
                    "Invalid MPM fluid particle velocity."
                };
            }
        }
        maxSpeed = std::max(maxSpeed, velocity.Length());
    }

    const double desiredTimeStep =
        minSpacing / (m_constitutiveModel.GetSpeedOfSound() + maxSpeed);
    const double count = std::ceil(timeIntervalInSeconds /
                                   (m_timeStepLimitScale * desiredTimeStep));
    const double maxCount =
        static_cast<double>(std::numeric_limits<unsigned int>::max());

    return static_cast<unsigned int>(std::clamp(count, 1.0, maxCount));
}

template <size_t N>
void MPMFluidSolver<N>::AccumulateForces(double timeStepInSeconds)
{
    (void)timeStepInSeconds;
}

template <size_t N>
void MPMFluidSolver<N>::OnBeginAdvanceTimeStep(double timeStepInSeconds)
{
    m_mpmSystemData->TransferFromParticlesToGrid();
    InitializeReferenceVolumes();
    UpdateGridVelocities(timeStepInSeconds);
    m_mpmSystemData->TransferFromGridToParticles(timeStepInSeconds);
}

template <size_t N>
MPMFluidSolver<N>::SizeType MPMFluidSolver<N>::ClampIndex(
    const Vector<ssize_t, N>& index, const SizeType& dataSize)
{
    SizeType result;

    for (size_t axis = 0; axis < N; ++axis)
    {
        result[axis] = static_cast<size_t>(std::clamp<ssize_t>(
            index[axis], 0, static_cast<ssize_t>(dataSize[axis] - 1)));
    }

    return result;
}

template <size_t N>
void MPMFluidSolver<N>::InitializeReferenceVolumes()
{
    const auto masses = m_mpmSystemData->ParticleMasses();
    auto volumes = m_mpmSystemData->InitialVolumes();

    for (size_t i = 0; i < volumes.Length(); ++i)
    {
        if (volumes[i] == 0.0)
        {
            volumes[i] = masses[i] / m_constitutiveModel.GetTargetDensity();
        }
        else if (!std::isfinite(volumes[i]) || volumes[i] < 0.0)
        {
            throw std::invalid_argument{
                "Invalid MPM fluid reference volume."
            };
        }
    }
}

template <size_t N>
void MPMFluidSolver<N>::UpdateGridVelocities(double timeStepInSeconds)
{
    const auto positions = m_mpmSystemData->Positions();
    const auto velocities = m_mpmSystemData->Velocities();
    const auto masses = m_mpmSystemData->ParticleMasses();
    const auto initialVolumes = m_mpmSystemData->InitialVolumes();
    const auto volumeRatios = m_mpmSystemData->VolumeRatios();
    const auto& gridMass = m_mpmSystemData->GridMass();
    auto& gridVelocities = m_mpmSystemData->GridVelocities();
    const auto dataSize = gridMass.DataSize();
    const auto spacing = gridMass.GridSpacing();
    const auto dataOrigin = gridMass.DataOrigin();

    for (size_t i = 0; i < positions.Length(); ++i)
    {
        const double currentVolume = initialVolumes[i] * volumeRatios[i];
        const double density =
            m_constitutiveModel.ComputeDensity(masses[i], currentVolume);
        const double pressure = m_constitutiveModel.ComputePressure(density);
        const MatrixType stress =
            m_constitutiveModel.ComputeCauchyStress(pressure);
        const VectorType relativeVelocity =
            velocities[i] - this->GetWind()->Sample(positions[i]);
        const VectorType externalForce =
            masses[i] * this->GetGravity() -
            this->GetDragCoefficient() * relativeVelocity;
        const auto stencil = CubicBSplineKernel<N>::GetStencil(
            positions[i], spacing, dataOrigin);

        for (const auto& entry : stencil)
        {
            if (entry.weight == 0.0)
            {
                continue;
            }

            const auto index = ClampIndex(entry.index, dataSize);
            const double nodeMass = gridMass(index);

            if (nodeMass > 0.0)
            {
                const VectorType increment =
                    timeStepInSeconds *
                    (entry.weight * externalForce -
                     currentVolume * stress * entry.gradient) /
                    nodeMass;

                for (double component : increment)
                {
                    if (!std::isfinite(component))
                    {
                        throw std::invalid_argument{
                            "Invalid MPM fluid grid velocity update."
                        };
                    }
                }

                gridVelocities(index) += increment;
            }
        }
    }
}

template <size_t N>
MPMFluidSolver<N>::Builder& MPMFluidSolver<N>::Builder::WithResolution(
    const SizeType& resolution)
{
    m_resolution = resolution;
    return *this;
}

template <size_t N>
MPMFluidSolver<N>::Builder& MPMFluidSolver<N>::Builder::WithGridSpacing(
    const VectorType& gridSpacing)
{
    m_gridSpacing = gridSpacing;
    return *this;
}

template <size_t N>
MPMFluidSolver<N>::Builder& MPMFluidSolver<N>::Builder::WithOrigin(
    const VectorType& gridOrigin)
{
    m_gridOrigin = gridOrigin;
    return *this;
}

template <size_t N>
MPMFluidSolver<N>::Builder& MPMFluidSolver<N>::Builder::WithRadius(
    double radius)
{
    m_radius = radius;
    return *this;
}

template <size_t N>
MPMFluidSolver<N>::Builder& MPMFluidSolver<N>::Builder::WithMass(double mass)
{
    m_mass = mass;
    return *this;
}

template <size_t N>
MPMFluidSolver<N>::Builder& MPMFluidSolver<N>::Builder::WithTargetDensity(
    double targetDensity)
{
    m_targetDensity = targetDensity;
    return *this;
}

template <size_t N>
MPMFluidSolver<N>::Builder& MPMFluidSolver<N>::Builder::WithSpeedOfSound(
    double speedOfSound)
{
    m_speedOfSound = speedOfSound;
    return *this;
}

template <size_t N>
MPMFluidSolver<N>::Builder& MPMFluidSolver<N>::Builder::WithEosExponent(
    double eosExponent)
{
    m_eosExponent = eosExponent;
    return *this;
}

template <size_t N>
MPMFluidSolver<N>::Builder&
MPMFluidSolver<N>::Builder::WithNegativePressureScale(
    double negativePressureScale)
{
    m_negativePressureScale = negativePressureScale;
    return *this;
}

template <size_t N>
MPMFluidSolver<N> MPMFluidSolver<N>::Builder::Build() const
{
    return MPMFluidSolver{
        m_resolution,   m_gridSpacing, m_gridOrigin,
        m_radius,       m_mass,        m_targetDensity,
        m_speedOfSound, m_eosExponent, m_negativePressureScale
    };
}

template <size_t N>
std::shared_ptr<MPMFluidSolver<N>> MPMFluidSolver<N>::Builder::MakeShared()
    const
{
    return std::make_shared<MPMFluidSolver>(
        m_resolution, m_gridSpacing, m_gridOrigin, m_radius, m_mass,
        m_targetDensity, m_speedOfSound, m_eosExponent,
        m_negativePressureScale);
}
}  // namespace CubbyFlow

#endif
