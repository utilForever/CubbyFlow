// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_MPM_SYSTEM_DATA_IMPL_HPP
#define CUBBYFLOW_MPM_SYSTEM_DATA_IMPL_HPP

#include <Core/Utils/IterationUtils.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace CubbyFlow
{
template <size_t N>
double CubicBSplineKernel<N>::Weight(double x)
{
    const double ax = std::abs(x);

    if (ax < 1.0)
    {
        return 0.5 * ax * ax * ax - ax * ax + 2.0 / 3.0;
    }

    if (ax < 2.0)
    {
        const double d = 2.0 - ax;
        return d * d * d / 6.0;
    }

    return 0.0;
}

template <size_t N>
double CubicBSplineKernel<N>::Gradient(double x)
{
    const double ax = std::abs(x);

    if (ax < 1.0)
    {
        return x * (1.5 * ax - 2.0);
    }

    if (ax < 2.0)
    {
        const double d = 2.0 - ax;
        return -0.5 * d * d * std::copysign(1.0, x);
    }

    return 0.0;
}

template <size_t N>
CubicBSplineKernel<N>::Stencil CubicBSplineKernel<N>::GetStencil(
    const Vector<double, N>& position, const Vector<double, N>& gridSpacing,
    const Vector<double, N>& dataOrigin)
{
    Vector<double, N> normalized;
    Vector<ssize_t, N> firstIndex;

    for (size_t axis = 0; axis < N; ++axis)
    {
        if (!std::isfinite(position[axis]) ||
            !std::isfinite(gridSpacing[axis]) ||
            !std::isfinite(dataOrigin[axis]) || gridSpacing[axis] <= 0.0)
        {
            throw std::invalid_argument("Invalid cubic B-spline input.");
        }

        normalized[axis] =
            (position[axis] - dataOrigin[axis]) / gridSpacing[axis];

        const double lowestIndex = std::nextafter(
            static_cast<double>(std::numeric_limits<ssize_t>::lowest()) + 1.0,
            0.0);
        const double highestIndex = std::nextafter(
            static_cast<double>(std::numeric_limits<ssize_t>::max()) - 2.0,
            0.0);

        if (!std::isfinite(normalized[axis]) ||
            normalized[axis] < lowestIndex || normalized[axis] > highestIndex)
        {
            throw std::invalid_argument(
                "Cubic B-spline index is out of range.");
        }

        firstIndex[axis] =
            static_cast<ssize_t>(std::floor(normalized[axis])) - 1;
    }

    std::array<Entry, STENCIL_SIZE> result;
    size_t flatIndex = 0;

    ForEachIndex(Vector<size_t, N>::MakeConstant(4),
                 [&result, &flatIndex, &normalized, &firstIndex,
                  &gridSpacing](auto... rawIndices) {
                     const Vector<size_t, N> offset{ rawIndices... };
                     Entry& entry = result[flatIndex++];
                     std::array<double, N> axisWeights;

                     entry.weight = 1.0;

                     for (size_t axis = 0; axis < N; ++axis)
                     {
                         entry.index[axis] = firstIndex[axis] +
                                             static_cast<ssize_t>(offset[axis]);
                         axisWeights[axis] =
                             Weight(normalized[axis] -
                                    static_cast<double>(entry.index[axis]));
                         entry.weight *= axisWeights[axis];
                     }

                     for (size_t axis = 0; axis < N; ++axis)
                     {
                         entry.gradient[axis] =
                             Gradient(normalized[axis] -
                                      static_cast<double>(entry.index[axis])) /
                             gridSpacing[axis];

                         for (size_t other = 0; other < N; ++other)
                         {
                             if (other != axis)
                             {
                                 entry.gradient[axis] *= axisWeights[other];
                             }
                         }
                     }
                 });

    return result;
}

template <size_t N>
MPMSystemData<N>::MPMSystemData(const Vector<size_t, N>& resolution,
                                const Vector<double, N>& gridSpacing,
                                const Vector<double, N>& gridOrigin,
                                size_t numberOfParticles)
    : Base{}
{
    ResizeGrid(resolution, gridSpacing, gridOrigin);
    Resize(numberOfParticles);
}

template <size_t N>
void MPMSystemData<N>::Resize(size_t newNumberOfParticles)
{
    Base::Resize(newNumberOfParticles);

    m_particleMasses.Resize(newNumberOfParticles, Base::Mass());
    m_initialVolumes.Resize(newNumberOfParticles, 0.0);
    m_deformationStates.Resize(newNumberOfParticles, DeformationState{});
}

template <size_t N>
void MPMSystemData<N>::ResizeGrid(const Vector<size_t, N>& resolution,
                                  const Vector<double, N>& gridSpacing,
                                  const Vector<double, N>& gridOrigin)
{
    ValidateGridParameters(resolution, gridSpacing, gridOrigin);

    m_gridMass.Resize(resolution, gridSpacing, gridOrigin);
    m_gridVelocities.Resize(resolution, gridSpacing, gridOrigin);
    m_gridVelocitiesBeforeUpdate.Resize(resolution, gridSpacing, gridOrigin);
}

template <size_t N>
ConstArrayView1<double> MPMSystemData<N>::ParticleMasses() const
{
    return m_particleMasses.View();
}

template <size_t N>
ArrayView1<double> MPMSystemData<N>::ParticleMasses()
{
    return m_particleMasses.View();
}

template <size_t N>
ConstArrayView1<double> MPMSystemData<N>::InitialVolumes() const
{
    return m_initialVolumes.View();
}

template <size_t N>
ArrayView1<double> MPMSystemData<N>::InitialVolumes()
{
    return m_initialVolumes.View();
}

template <size_t N>
ConstArrayView1<typename MPMSystemData<N>::DeformationState>
MPMSystemData<N>::DeformationStates() const
{
    return m_deformationStates.View();
}

template <size_t N>
ArrayView1<typename MPMSystemData<N>::DeformationState>
MPMSystemData<N>::DeformationStates()
{
    return m_deformationStates.View();
}

template <size_t N>
const VertexCenteredScalarGrid<N>& MPMSystemData<N>::GridMass() const
{
    return m_gridMass;
}

template <size_t N>
VertexCenteredScalarGrid<N>& MPMSystemData<N>::GridMass()
{
    return m_gridMass;
}

template <size_t N>
const VertexCenteredVectorGrid<N>& MPMSystemData<N>::GridVelocities() const
{
    return m_gridVelocities;
}

template <size_t N>
VertexCenteredVectorGrid<N>& MPMSystemData<N>::GridVelocities()
{
    return m_gridVelocities;
}

template <size_t N>
const VertexCenteredVectorGrid<N>&
MPMSystemData<N>::GridVelocitiesBeforeUpdate() const
{
    return m_gridVelocitiesBeforeUpdate;
}

template <size_t N>
VertexCenteredVectorGrid<N>& MPMSystemData<N>::GridVelocitiesBeforeUpdate()
{
    return m_gridVelocitiesBeforeUpdate;
}

template <size_t N>
double MPMSystemData<N>::GetFLIPBlendingFactor() const
{
    return m_flipBlendingFactor;
}

template <size_t N>
void MPMSystemData<N>::SetFLIPBlendingFactor(double factor)
{
    if (!std::isfinite(factor) || factor < 0.0 || factor > 1.0)
    {
        throw std::invalid_argument("FLIP blending factor must be in [0, 1].");
    }

    m_flipBlendingFactor = factor;
}

template <size_t N>
void MPMSystemData<N>::TransferFromParticlesToGrid()
{
    ValidateGridState();

    const auto positions = this->Positions();
    const auto velocities = this->Velocities();

    for (size_t i = 0; i < this->NumberOfParticles(); ++i)
    {
        if (!std::isfinite(m_particleMasses[i]) || m_particleMasses[i] <= 0.0 ||
            !IsFinite(positions[i]) || !IsFinite(velocities[i]))
        {
            throw std::invalid_argument("Invalid MPM particle state.");
        }
    }

    m_gridMass.Fill(0.0, ExecutionPolicy::Serial);
    m_gridVelocities.Fill(Vector<double, N>{}, ExecutionPolicy::Serial);

    const auto dataSize = m_gridMass.DataSize();
    const auto gridSpacing = m_gridMass.GridSpacing();
    const auto dataOrigin = m_gridMass.DataOrigin();

    for (size_t i = 0; i < this->NumberOfParticles(); ++i)
    {
        const auto stencil = CubicBSplineKernel<N>::GetStencil(
            positions[i], gridSpacing, dataOrigin);

        for (const auto& entry : stencil)
        {
            if (entry.weight == 0.0)
            {
                continue;
            }

            const auto index = FoldIndex(entry.index, dataSize);
            const double mass = entry.weight * m_particleMasses[i];

            m_gridMass(index) += mass;
            m_gridVelocities(index) += mass * velocities[i];
        }
    }

    m_gridMass.ForEachDataPointIndex([&](const Vector<size_t, N>& index) {
        const double mass = m_gridMass(index);
        if (mass > 0.0)
        {
            m_gridVelocities(index) /= mass;
        }
    });

    m_gridVelocitiesBeforeUpdate.Set(m_gridVelocities);
}

template <size_t N>
void MPMSystemData<N>::TransferFromGridToParticles()
{
    ValidateGridState();

    const auto positions = this->Positions();
    auto velocities = this->Velocities();

    for (size_t i = 0; i < this->NumberOfParticles(); ++i)
    {
        if (!IsFinite(positions[i]) || !IsFinite(velocities[i]))
        {
            throw std::invalid_argument("Invalid MPM particle state.");
        }
    }

    const auto dataSize = m_gridVelocities.DataSize();
    const auto gridSpacing = m_gridVelocities.GridSpacing();
    const auto dataOrigin = m_gridVelocities.DataOrigin();

    for (size_t i = 0; i < this->NumberOfParticles(); ++i)
    {
        Vector<double, N> picVelocity;
        Vector<double, N> flipDelta;
        const auto stencil = CubicBSplineKernel<N>::GetStencil(
            positions[i], gridSpacing, dataOrigin);

        for (const auto& entry : stencil)
        {
            if (entry.weight == 0.0)
            {
                continue;
            }

            const auto index = FoldIndex(entry.index, dataSize);
            picVelocity += entry.weight * m_gridVelocities(index);
            flipDelta += entry.weight * (m_gridVelocities(index) -
                                         m_gridVelocitiesBeforeUpdate(index));
        }

        const Vector<double, N> flipVelocity = velocities[i] + flipDelta;
        const Vector<double, N> result =
            (1.0 - m_flipBlendingFactor) * picVelocity +
            m_flipBlendingFactor * flipVelocity;

        if (!IsFinite(result))
        {
            throw std::invalid_argument("Invalid MPM grid velocity.");
        }

        velocities[i] = result;
    }
}

template <size_t N>
Vector<size_t, N> MPMSystemData<N>::FoldIndex(const Vector<ssize_t, N>& index,
                                              const Vector<size_t, N>& dataSize)
{
    Vector<size_t, N> result;

    for (size_t axis = 0; axis < N; ++axis)
    {
        result[axis] = index[axis] < 0
                           ? 0
                           : std::min(static_cast<size_t>(index[axis]),
                                      dataSize[axis] - 1);
    }

    return result;
}

template <size_t N>
bool MPMSystemData<N>::IsFinite(const Vector<double, N>& value)
{
    for (size_t axis = 0; axis < N; ++axis)
    {
        if (!std::isfinite(value[axis]))
        {
            return false;
        }
    }

    return true;
}

template <size_t N>
void MPMSystemData<N>::ValidateGridParameters(
    const Vector<size_t, N>& resolution, const Vector<double, N>& gridSpacing,
    const Vector<double, N>& gridOrigin)
{
    for (size_t axis = 0; axis < N; ++axis)
    {
        if (resolution[axis] == 0 ||
            resolution[axis] == std::numeric_limits<size_t>::max() ||
            !std::isfinite(gridSpacing[axis]) || gridSpacing[axis] <= 0.0 ||
            !std::isfinite(gridOrigin[axis]))
        {
            throw std::invalid_argument("Invalid MPM grid parameters.");
        }
    }
}

template <size_t N>
void MPMSystemData<N>::ValidateGridState() const
{
    ValidateGridParameters(m_gridMass.Resolution(), m_gridMass.GridSpacing(),
                           m_gridMass.Origin());
    ValidateGridParameters(m_gridVelocities.Resolution(),
                           m_gridVelocities.GridSpacing(),
                           m_gridVelocities.Origin());
    ValidateGridParameters(m_gridVelocitiesBeforeUpdate.Resolution(),
                           m_gridVelocitiesBeforeUpdate.GridSpacing(),
                           m_gridVelocitiesBeforeUpdate.Origin());

    if (m_gridMass.Resolution() != m_gridVelocities.Resolution() ||
        m_gridMass.Resolution() != m_gridVelocitiesBeforeUpdate.Resolution() ||
        m_gridMass.GridSpacing() != m_gridVelocities.GridSpacing() ||
        m_gridMass.GridSpacing() !=
            m_gridVelocitiesBeforeUpdate.GridSpacing() ||
        m_gridMass.Origin() != m_gridVelocities.Origin() ||
        m_gridMass.Origin() != m_gridVelocitiesBeforeUpdate.Origin())
    {
        throw std::invalid_argument("MPM grids must have matching geometry.");
    }
}
}  // namespace CubbyFlow

#endif
