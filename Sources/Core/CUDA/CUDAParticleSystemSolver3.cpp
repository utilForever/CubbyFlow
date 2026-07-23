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

#include <Core/CUDA/CUDAParticleSystemSolver3.hpp>

using namespace CubbyFlow;

CUDAParticleSystemSolver3::CUDAParticleSystemSolver3()
    : CUDAParticleSystemSolver3{ 1e-3f, 1e-3f }
{
    // Do nothing
}

CUDAParticleSystemSolver3::CUDAParticleSystemSolver3(float radius, float mass)
    : m_radius{ radius }, m_mass{ mass }
{
    // Do nothing
}

float CUDAParticleSystemSolver3::Radius() const
{
    return m_radius;
}

void CUDAParticleSystemSolver3::SetRadius(float newRadius)
{
    m_radius = newRadius;
}

float CUDAParticleSystemSolver3::Mass() const
{
    return m_mass;
}

void CUDAParticleSystemSolver3::SetMass(float newMass)
{
    m_mass = newMass;
}

CUDAParticleSystemSolver3::Builder CUDAParticleSystemSolver3::GetBuilder()
{
    return Builder{};
}

CUDAParticleSystemSolver3::Builder&
CUDAParticleSystemSolver3::Builder::WithRadius(float radius)
{
    m_radius = radius;
    return *this;
}

CUDAParticleSystemSolver3::Builder&
CUDAParticleSystemSolver3::Builder::WithMass(float mass)
{
    m_mass = mass;
    return *this;
}

CUDAParticleSystemSolver3 CUDAParticleSystemSolver3::Builder::Build() const
{
    return CUDAParticleSystemSolver3{ m_radius, m_mass };
}

CUDAParticleSystemSolver3Ptr CUDAParticleSystemSolver3::Builder::MakeShared()
    const
{
    return std::make_shared<CUDAParticleSystemSolver3>(m_radius, m_mass);
}

#endif
