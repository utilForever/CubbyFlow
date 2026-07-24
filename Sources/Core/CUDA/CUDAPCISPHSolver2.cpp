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

#include <Core/CUDA/CUDAPCISPHSolver2.hpp>
#include <Core/Particle/SPHKernels.hpp>
#include <Core/PointGenerator/TrianglePointGenerator.hpp>

using namespace CubbyFlow;

// Heuristically chosen value.
constexpr float DEFAULT_TIME_STEP_LIMIT_SCALE = 5.0f;

CUDAPCISPHSolver2::CUDAPCISPHSolver2()
{
    SetTimeStepLimitScale(DEFAULT_TIME_STEP_LIMIT_SCALE);
}

CUDAPCISPHSolver2::CUDAPCISPHSolver2(float targetDensity, float targetSpacing,
                                     float relativeKernelRadius)
    : CUDASPHSolverBase2{}
{
    const auto sph = SPHSystemData();
    sph->SetTargetDensity(targetDensity);
    sph->SetTargetSpacing(targetSpacing);
    sph->SetRelativeKernelRadius(relativeKernelRadius);

    m_tempPositionsIdx = sph->AddVectorData();
    m_tempVelocitiesIdx = sph->AddVectorData();
    m_tempDensitiesIdx = sph->AddFloatData();
    m_pressureForcesIdx = sph->AddVectorData();
    m_densityErrorsIdx = sph->AddFloatData();

    SetTimeStepLimitScale(DEFAULT_TIME_STEP_LIMIT_SCALE);
}

float CUDAPCISPHSolver2::MaxDensityErrorRatio() const
{
    return m_maxDensityErrorRatio;
}

void CUDAPCISPHSolver2::SetMaxDensityErrorRatio(float ratio)
{
    m_maxDensityErrorRatio = std::max(ratio, 0.0f);
}

unsigned int CUDAPCISPHSolver2::MaxNumberOfIterations() const
{
    return m_maxNumberOfIterations;
}

void CUDAPCISPHSolver2::SetMaxNumberOfIterations(unsigned int n)
{
    m_maxNumberOfIterations = n;
}

CUDAArrayView1<float2> CUDAPCISPHSolver2::TempPositions()
{
    return SPHSystemData()->VectorDataAt(m_tempPositionsIdx);
}

CUDAArrayView1<float2> CUDAPCISPHSolver2::TempVelocities()
{
    return SPHSystemData()->VectorDataAt(m_tempVelocitiesIdx);
}

CUDAArrayView1<float> CUDAPCISPHSolver2::TempDensities()
{
    return SPHSystemData()->FloatDataAt(m_tempDensitiesIdx);
}

CUDAArrayView1<float2> CUDAPCISPHSolver2::PressureForces()
{
    return SPHSystemData()->VectorDataAt(m_pressureForcesIdx);
}

CUDAArrayView1<float> CUDAPCISPHSolver2::DensityErrors()
{
    return SPHSystemData()->FloatDataAt(m_densityErrorsIdx);
}

float CUDAPCISPHSolver2::ComputeDelta(float timeStepInSeconds)
{
    const auto particles = SPHSystemData();
    const auto kernelRadius = static_cast<double>(particles->KernelRadius());

    Array1<Vector2D> points;
    const TrianglePointGenerator pointsGenerator;
    const Vector2D origin;
    BoundingBox2D sampleBound(origin, origin);
    sampleBound.Expand(1.5 * kernelRadius);

    pointsGenerator.Generate(
        sampleBound, static_cast<double>(particles->TargetSpacing()), &points);

    const SPHSpikyKernel2 kernel(kernelRadius);

    double denom = 0;
    Vector2D denom1;
    double denom2 = 0;

    for (const auto& point : points)
    {
        const double distanceSquared = point.LengthSquared();

        if (distanceSquared < kernelRadius * kernelRadius)
        {
            double distance = std::sqrt(distanceSquared);
            Vector2D direction =
                (distance > 0.0) ? point / distance : Vector2D{};

            // grad(Wij)
            Vector2D gradWij = kernel.Gradient(distance, direction);
            denom1 += gradWij;
            denom2 += gradWij.Dot(gradWij);
        }
    }

    denom += -denom1.Dot(denom1) - denom2;

    const auto beta = static_cast<double>(ComputeBeta(timeStepInSeconds));

    return static_cast<float>((std::fabs(denom) > 0.0) ? -1 / (beta * denom)
                                                       : 0);
}

float CUDAPCISPHSolver2::ComputeBeta(float timeStepInSeconds)
{
    const auto particles = SPHSystemData();
    return 2.0f * Square(particles->Mass() * timeStepInSeconds /
                         particles->TargetDensity());
}

CUDAPCISPHSolver2::Builder CUDAPCISPHSolver2::GetBuilder()
{
    return Builder{};
}

CUDAPCISPHSolver2 CUDAPCISPHSolver2::Builder::Build() const
{
    return CUDAPCISPHSolver2{ m_targetDensity, m_targetSpacing,
                              m_relativeKernelRadius };
}

CUDAPCISPHSolver2Ptr CUDAPCISPHSolver2::Builder::MakeShared() const
{
    return std::make_shared<CUDAPCISPHSolver2>(m_targetDensity, m_targetSpacing,
                                               m_relativeKernelRadius);
}

#endif
