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

namespace CubbyFlow
{
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
