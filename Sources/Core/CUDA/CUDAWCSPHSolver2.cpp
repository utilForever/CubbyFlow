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

#include <Core/CUDA/CUDAWCSPHSolver2.hpp>

using namespace CubbyFlow;

CUDAWCSPHSolver2::CUDAWCSPHSolver2()
    : CUDAWCSPHSolver2(WATER_DENSITY_FLOAT, 0.1f, 1.8f)
{
    // Do nothing
}

CUDAWCSPHSolver2::CUDAWCSPHSolver2(float targetDensity, float targetSpacing,
                                   float relativeKernelRadius)
    : CUDASPHSolverBase2{}
{
    const auto sph = SPHSystemData();
    sph->SetTargetDensity(targetDensity);
    sph->SetTargetSpacing(targetSpacing);
    sph->SetRelativeKernelRadius(relativeKernelRadius);
}

float CUDAWCSPHSolver2::EOSExponent() const
{
    return m_eosExponent;
}

void CUDAWCSPHSolver2::SetEOSExponent(float newEosExponent)
{
    m_eosExponent = std::max(newEosExponent, 1.0f);
}

CUDAWCSPHSolver2::Builder CUDAWCSPHSolver2::GetBuilder()
{
    return Builder{};
}

CUDAWCSPHSolver2 CUDAWCSPHSolver2::Builder::Build() const
{
    return CUDAWCSPHSolver2{ m_targetDensity, m_targetSpacing,
                             m_relativeKernelRadius };
}

CUDAWCSPHSolver2Ptr CUDAWCSPHSolver2::Builder::MakeShared() const
{
    return std::make_shared<CUDAWCSPHSolver2>(m_targetDensity, m_targetSpacing,
                                              m_relativeKernelRadius);
}

#endif