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

#include <cmath>
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
typename CubicBSplineKernel<N>::Stencil CubicBSplineKernel<N>::GetStencil(
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
        firstIndex[axis] =
            static_cast<ssize_t>(std::floor(normalized[axis])) - 1;
    }

    std::array<Entry, STENCIL_SIZE> result;
    size_t flatIndex = 0;
    ForEachIndex(Vector<size_t, N>::MakeConstant(4), [&](auto... rawIndices) {
        const Vector<size_t, N> offset{ rawIndices... };
        Entry& entry = result[flatIndex++];
        std::array<double, N> axisWeights;
        entry.weight = 1.0;

        for (size_t axis = 0; axis < N; ++axis)
        {
            entry.index[axis] =
                firstIndex[axis] + static_cast<ssize_t>(offset[axis]);
            axisWeights[axis] = Weight(normalized[axis] -
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
    : Base{},
      m_gridMass{ resolution, gridSpacing, gridOrigin },
      m_gridVelocities{ resolution, gridSpacing, gridOrigin },
      m_gridVelocitiesBeforeUpdate{ resolution, gridSpacing, gridOrigin }
{
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
}  // namespace CubbyFlow

#endif
