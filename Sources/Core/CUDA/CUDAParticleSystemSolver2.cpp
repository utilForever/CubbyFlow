// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#if CUBBYFLOW_USE_CUDA

#include <Core/CUDA/CUDAParticleSystemSolver2.hpp>

using namespace CubbyFlow;

CUDAParticleSystemSolver2::CUDAParticleSystemSolver2()
    : CUDAParticleSystemSolver2{ 1e-3f, 1e-3f }
{
    // Do nothing
}

CUDAParticleSystemSolver2::CUDAParticleSystemSolver2(float radius, float mass)
    : m_radius{ radius }, m_mass{ mass }
{
    // Do nothing
}

float CUDAParticleSystemSolver2::Radius() const
{
    return m_radius;
}

void CUDAParticleSystemSolver2::SetRadius(float newRadius)
{
    m_radius = newRadius;
}

float CUDAParticleSystemSolver2::Mass() const
{
    return m_mass;
}

void CUDAParticleSystemSolver2::SetMass(float newMass)
{
    m_mass = newMass;
}

CUDAParticleSystemSolver2::Builder CUDAParticleSystemSolver2::GetBuilder()
{
    return Builder{};
}

CUDAParticleSystemSolver2::Builder&
CUDAParticleSystemSolver2::Builder::WithRadius(float radius)
{
    m_radius = radius;
    return *this;
}

CUDAParticleSystemSolver2::Builder&
CUDAParticleSystemSolver2::Builder::WithMass(float mass)
{
    m_mass = mass;
    return *this;
}

CUDAParticleSystemSolver2 CUDAParticleSystemSolver2::Builder::Build() const
{
    return CUDAParticleSystemSolver2{ m_radius, m_mass };
}

CUDAParticleSystemSolver2Ptr CUDAParticleSystemSolver2::Builder::MakeShared()
    const
{
    return std::make_shared<CUDAParticleSystemSolver2>(m_radius, m_mass);
}

#endif
