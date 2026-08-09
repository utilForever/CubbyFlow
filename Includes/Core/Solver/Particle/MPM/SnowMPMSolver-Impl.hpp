// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_SNOW_MPM_SOLVER_IMPL_HPP
#define CUBBYFLOW_SNOW_MPM_SOLVER_IMPL_HPP

#include <Core/Utils/Parallel.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace CubbyFlow
{
template <size_t N>
SnowMPMSolver<N>::SnowMPMSolver(const SizeType& resolution,
                                const VectorType& gridSpacing,
                                const VectorType& gridOrigin, double radius,
                                double mass)
    : Base{ radius, mass },
      m_mpmSystemData{ std::make_shared<MPMSystemData<N>>(
          resolution, gridSpacing, gridOrigin) }
{
    if (!std::isfinite(radius) || radius < 0.0 || !std::isfinite(mass) ||
        mass <= 0.0)
    {
        throw std::invalid_argument{ "Invalid snow MPM particle parameters." };
    }

    m_mpmSystemData->SetRadius(radius);
    m_mpmSystemData->SetMass(mass);
    this->SetParticleSystemData(m_mpmSystemData);
    this->SetIsUsingFixedSubTimeSteps(false);
}

template <size_t N>
std::shared_ptr<MPMSystemData<N>> SnowMPMSolver<N>::GetMPMSystemData() const
{
    return m_mpmSystemData;
}

template <size_t N>
double SnowMPMSolver<N>::GetTimeStepLimitScale() const
{
    return m_timeStepLimitScale;
}

template <size_t N>
void SnowMPMSolver<N>::SetTimeStepLimitScale(double newScale)
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
int SnowMPMSolver<N>::GetClosedDomainBoundaryFlag() const
{
    return m_closedDomainBoundaryFlag;
}

template <size_t N>
void SnowMPMSolver<N>::SetClosedDomainBoundaryFlag(int flag)
{
    m_closedDomainBoundaryFlag = flag;
}

template <size_t N>
typename SnowMPMSolver<N>::Builder SnowMPMSolver<N>::GetBuilder()
{
    return Builder{};
}

template <size_t N>
void SnowMPMSolver<N>::OnInitialize()
{
    Base::OnInitialize();
    m_mpmSystemData->TransferFromParticlesToGrid();
    InitializeReferenceVolumes();
    m_maxVelocityGradient = ComputeMaxVelocityGradient();
}

template <size_t N>
unsigned int SnowMPMSolver<N>::GetNumberOfSubTimeSteps(
    double timeIntervalInSeconds) const
{
    if (!std::isfinite(timeIntervalInSeconds) || timeIntervalInSeconds <= 0.0)
    {
        return 1;
    }

    const auto velocities = m_mpmSystemData->Velocities();
    const auto masses = m_mpmSystemData->ParticleMasses();
    const auto volumes = m_mpmSystemData->InitialVolumes();
    const auto states = m_mpmSystemData->DeformationStates();
    const auto spacing = m_mpmSystemData->GridMass().GridSpacing();
    double minSpacing = spacing[0];
    double maxVelocity = 0.0;
    double maxWaveSpeed = 0.0;

    for (size_t axis = 1; axis < N; ++axis)
    {
        minSpacing = std::min(minSpacing, spacing[axis]);
    }

    for (size_t i = 0; i < velocities.Length(); ++i)
    {
        maxVelocity = std::max(maxVelocity, velocities[i].Length());

        if (volumes[i] == 0.0)
        {
            continue;
        }

        if (!std::isfinite(volumes[i]) || volumes[i] < 0.0)
        {
            throw std::invalid_argument{ "Invalid snow reference volume." };
        }

        maxWaveSpeed =
            std::max(maxWaveSpeed, m_constitutiveModel.ComputeWaveSpeed(
                                       states[i], masses[i] / volumes[i]));
    }

    double desiredTimeStep = std::numeric_limits<double>::infinity();
    if (maxVelocity > 0.0)
    {
        desiredTimeStep = std::min(desiredTimeStep, minSpacing / maxVelocity);
    }
    if (maxWaveSpeed > 0.0)
    {
        desiredTimeStep = std::min(desiredTimeStep, minSpacing / maxWaveSpeed);
    }
    if (m_maxVelocityGradient > 0.0)
    {
        desiredTimeStep =
            std::min(desiredTimeStep, 0.2 / m_maxVelocityGradient);
    }

    if (!std::isfinite(desiredTimeStep))
    {
        return 1;
    }

    const double count = std::ceil(timeIntervalInSeconds /
                                   (m_timeStepLimitScale * desiredTimeStep));
    const double maxCount =
        static_cast<double>(std::numeric_limits<unsigned int>::max());

    return static_cast<unsigned int>(std::clamp(count, 1.0, maxCount));
}

template <size_t N>
void SnowMPMSolver<N>::AccumulateForces(double timeStepInSeconds)
{
    (void)timeStepInSeconds;
}

template <size_t N>
void SnowMPMSolver<N>::OnBeginAdvanceTimeStep(double timeStepInSeconds)
{
    m_mpmSystemData->TransferFromParticlesToGrid();
    InitializeReferenceVolumes();
    UpdateGridVelocities(timeStepInSeconds);
    ConstrainGridVelocities();
    UpdateDeformation(timeStepInSeconds);
    m_mpmSystemData->TransferFromGridToParticles();
}

template <size_t N>
void SnowMPMSolver<N>::OnEndAdvanceTimeStep(double timeStepInSeconds)
{
    Base::OnEndAdvanceTimeStep(timeStepInSeconds);
    ConstrainParticlesToDomain();
}

template <size_t N>
typename SnowMPMSolver<N>::SizeType SnowMPMSolver<N>::ClampIndex(
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
void SnowMPMSolver<N>::InitializeReferenceVolumes()
{
    const auto positions = m_mpmSystemData->Positions();
    const auto masses = m_mpmSystemData->ParticleMasses();
    auto volumes = m_mpmSystemData->InitialVolumes();
    const auto& gridMass = m_mpmSystemData->GridMass();
    const auto dataSize = gridMass.DataSize();
    const auto spacing = gridMass.GridSpacing();
    const auto dataOrigin = gridMass.DataOrigin();
    double cellVolume = 1.0;

    for (size_t axis = 0; axis < N; ++axis)
    {
        cellVolume *= spacing[axis];
    }

    if (!std::isfinite(cellVolume) || cellVolume <= 0.0)
    {
        throw std::invalid_argument{ "Invalid snow MPM cell volume." };
    }

    for (size_t i = 0; i < volumes.Length(); ++i)
    {
        if (volumes[i] != 0.0)
        {
            if (!std::isfinite(volumes[i]) || volumes[i] < 0.0)
            {
                throw std::invalid_argument{ "Invalid snow reference volume." };
            }
            continue;
        }

        double referenceDensity = 0.0;
        const auto stencil = CubicBSplineKernel<N>::GetStencil(
            positions[i], spacing, dataOrigin);

        for (const auto& entry : stencil)
        {
            referenceDensity += gridMass(ClampIndex(entry.index, dataSize)) *
                                entry.weight / cellVolume;
        }

        if (!std::isfinite(referenceDensity) || referenceDensity <= 0.0)
        {
            throw std::invalid_argument{ "Invalid snow reference density." };
        }

        volumes[i] = masses[i] / referenceDensity;
    }
}

template <size_t N>
void SnowMPMSolver<N>::UpdateGridVelocities(double timeStepInSeconds)
{
    const auto positions = m_mpmSystemData->Positions();
    const auto velocities = m_mpmSystemData->Velocities();
    const auto masses = m_mpmSystemData->ParticleMasses();
    const auto volumes = m_mpmSystemData->InitialVolumes();
    const auto states = m_mpmSystemData->DeformationStates();
    const auto& gridMass = m_mpmSystemData->GridMass();
    auto& gridVelocities = m_mpmSystemData->GridVelocities();
    const auto dataSize = gridMass.DataSize();
    const auto spacing = gridMass.GridSpacing();
    const auto dataOrigin = gridMass.DataOrigin();

    for (size_t i = 0; i < positions.Length(); ++i)
    {
        const VectorType relativeVelocity =
            velocities[i] - this->GetWind()->Sample(positions[i]);
        const VectorType externalForce =
            masses[i] * this->GetGravity() -
            this->GetDragCoefficient() * relativeVelocity;
        const MatrixType stress =
            m_constitutiveModel.ComputeKirchhoffStress(states[i]);
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
                gridVelocities(index) +=
                    timeStepInSeconds *
                    (entry.weight * externalForce -
                     volumes[i] * stress * entry.gradient) /
                    nodeMass;
            }
        }
    }
}

template <size_t N>
void SnowMPMSolver<N>::ConstrainGridVelocities()
{
    static constexpr std::array lowerFlags{ DIRECTION_LEFT, DIRECTION_DOWN,
                                            DIRECTION_BACK };
    static constexpr std::array upperFlags{ DIRECTION_RIGHT, DIRECTION_UP,
                                            DIRECTION_FRONT };
    const auto& gridMass = m_mpmSystemData->GridMass();
    auto& gridVelocities = m_mpmSystemData->GridVelocities();
    const auto dataSize = gridVelocities.DataSize();
    const auto dataPosition = gridVelocities.DataPosition();
    const auto collider = this->GetCollider();

    gridVelocities.ParallelForEachDataPointIndex(
        [&gridMass, &gridVelocities, &dataSize, &dataPosition, &collider,
         this](const SizeType& index) {
            if (gridMass(index) <= 0.0)
            {
                return;
            }

            VectorType velocity = gridVelocities(index);
            if (collider != nullptr)
            {
                VectorType position = dataPosition(index);
                collider->ResolveCollision(0.0, 0.0, &position, &velocity);
            }

            for (size_t axis = 0; axis < N; ++axis)
            {
                if ((m_closedDomainBoundaryFlag & lowerFlags[axis]) != 0 &&
                    index[axis] == 0 && velocity[axis] < 0.0)
                {
                    velocity[axis] = 0.0;
                }
                if ((m_closedDomainBoundaryFlag & upperFlags[axis]) != 0 &&
                    index[axis] == dataSize[axis] - 1 && velocity[axis] > 0.0)
                {
                    velocity[axis] = 0.0;
                }
            }

            gridVelocities(index) = velocity;
        });
}

template <size_t N>
typename SnowMPMSolver<N>::MatrixType SnowMPMSolver<N>::ComputeVelocityGradient(
    size_t particleIndex) const
{
    const auto positions = m_mpmSystemData->Positions();
    const auto& gridVelocities = m_mpmSystemData->GridVelocities();
    const auto dataSize = gridVelocities.DataSize();
    const auto stencil = CubicBSplineKernel<N>::GetStencil(
        positions[particleIndex], gridVelocities.GridSpacing(),
        gridVelocities.DataOrigin());
    MatrixType result;

    for (const auto& entry : stencil)
    {
        if (entry.weight != 0.0)
        {
            const VectorType velocity =
                gridVelocities(ClampIndex(entry.index, dataSize));
            for (size_t row = 0; row < N; ++row)
            {
                for (size_t column = 0; column < N; ++column)
                {
                    result(row, column) +=
                        velocity[row] * entry.gradient[column];
                }
            }
        }
    }

    return result;
}

template <size_t N>
double SnowMPMSolver<N>::ComputeMaxVelocityGradient() const
{
    double result = 0.0;

    for (size_t i = 0; i < m_mpmSystemData->NumberOfParticles(); ++i)
    {
        result = std::max(result, ComputeVelocityGradient(i).AbsMax());
    }

    return result;
}

template <size_t N>
void SnowMPMSolver<N>::UpdateDeformation(double timeStepInSeconds)
{
    auto states = m_mpmSystemData->DeformationStates();
    m_maxVelocityGradient = 0.0;

    for (size_t i = 0; i < states.Length(); ++i)
    {
        const MatrixType velocityGradient = ComputeVelocityGradient(i);
        m_maxVelocityGradient =
            std::max(m_maxVelocityGradient, velocityGradient.AbsMax());
        states[i] = m_constitutiveModel.Update(
            MatrixType::MakeIdentity() + timeStepInSeconds * velocityGradient,
            states[i]);
    }
}

template <size_t N>
void SnowMPMSolver<N>::ConstrainParticlesToDomain()
{
    static constexpr std::array lowerFlags{ DIRECTION_LEFT, DIRECTION_DOWN,
                                            DIRECTION_BACK };
    static constexpr std::array upperFlags{ DIRECTION_RIGHT, DIRECTION_UP,
                                            DIRECTION_FRONT };
    const auto domain = m_mpmSystemData->GridMass().GetBoundingBox();
    auto positions = m_mpmSystemData->Positions();
    auto velocities = m_mpmSystemData->Velocities();

    ParallelFor(
        ZERO_SIZE, positions.Length(),
        [&domain, &positions, &velocities, this](size_t i) {
            for (size_t axis = 0; axis < N; ++axis)
            {
                if ((m_closedDomainBoundaryFlag & lowerFlags[axis]) != 0 &&
                    positions[i][axis] <= domain.lowerCorner[axis])
                {
                    positions[i][axis] = domain.lowerCorner[axis];
                    velocities[i][axis] = std::max(velocities[i][axis], 0.0);
                }
                if ((m_closedDomainBoundaryFlag & upperFlags[axis]) != 0 &&
                    positions[i][axis] >= domain.upperCorner[axis])
                {
                    positions[i][axis] = domain.upperCorner[axis];
                    velocities[i][axis] = std::min(velocities[i][axis], 0.0);
                }
            }
        });
}

template <size_t N>
typename SnowMPMSolver<N>::Builder& SnowMPMSolver<N>::Builder::WithResolution(
    const SizeType& resolution)
{
    m_resolution = resolution;
    return *this;
}

template <size_t N>
typename SnowMPMSolver<N>::Builder& SnowMPMSolver<N>::Builder::WithGridSpacing(
    const VectorType& gridSpacing)
{
    m_gridSpacing = gridSpacing;
    return *this;
}

template <size_t N>
typename SnowMPMSolver<N>::Builder& SnowMPMSolver<N>::Builder::WithOrigin(
    const VectorType& gridOrigin)
{
    m_gridOrigin = gridOrigin;
    return *this;
}

template <size_t N>
typename SnowMPMSolver<N>::Builder& SnowMPMSolver<N>::Builder::WithRadius(
    double radius)
{
    m_radius = radius;
    return *this;
}

template <size_t N>
typename SnowMPMSolver<N>::Builder& SnowMPMSolver<N>::Builder::WithMass(
    double mass)
{
    m_mass = mass;
    return *this;
}

template <size_t N>
SnowMPMSolver<N> SnowMPMSolver<N>::Builder::Build() const
{
    return SnowMPMSolver{ m_resolution, m_gridSpacing, m_gridOrigin, m_radius,
                          m_mass };
}

template <size_t N>
std::shared_ptr<SnowMPMSolver<N>> SnowMPMSolver<N>::Builder::MakeShared() const
{
    return std::make_shared<SnowMPMSolver>(m_resolution, m_gridSpacing,
                                           m_gridOrigin, m_radius, m_mass);
}
}  // namespace CubbyFlow

#endif
