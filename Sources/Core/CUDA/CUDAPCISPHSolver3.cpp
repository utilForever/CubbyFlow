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

#include <Core/CUDA/CUDAPCISPHSolver3.hpp>
#include <Core/Particle/SPHKernels.hpp>
#include <Core/PointGenerator/BccLatticePointGenerator.hpp>

using namespace CubbyFlow;

// Heuristically chosen value.
constexpr float DEFAULT_TIME_STEP_LIMIT_SCALE = 5.0f;

CUDAPCISPHSolver3::CUDAPCISPHSolver3()
{
    SetTimeStepLimitScale(DEFAULT_TIME_STEP_LIMIT_SCALE);
}

CUDAPCISPHSolver3::CUDAPCISPHSolver3(float targetDensity, float targetSpacing,
                                     float relativeKernelRadius)
    : CUDASPHSolverBase3{}
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

float CUDAPCISPHSolver3::MaxDensityErrorRatio() const
{
    return m_maxDensityErrorRatio;
}

void CUDAPCISPHSolver3::SetMaxDensityErrorRatio(float ratio)
{
    m_maxDensityErrorRatio = std::max(ratio, 0.0f);
}

unsigned int CUDAPCISPHSolver3::MaxNumberOfIterations() const
{
    return m_maxNumberOfIterations;
}

void CUDAPCISPHSolver3::SetMaxNumberOfIterations(unsigned int n)
{
    m_maxNumberOfIterations = n;
}

CUDAArrayView1<float4> CUDAPCISPHSolver3::TempPositions()
{
    return SPHSystemData()->VectorDataAt(m_tempPositionsIdx);
}

CUDAArrayView1<float4> CUDAPCISPHSolver3::TempVelocities()
{
    return SPHSystemData()->VectorDataAt(m_tempVelocitiesIdx);
}

CUDAArrayView1<float> CUDAPCISPHSolver3::TempDensities()
{
    return SPHSystemData()->FloatDataAt(m_tempDensitiesIdx);
}

CUDAArrayView1<float4> CUDAPCISPHSolver3::PressureForces()
{
    return SPHSystemData()->VectorDataAt(m_pressureForcesIdx);
}

CUDAArrayView1<float> CUDAPCISPHSolver3::DensityErrors()
{
    return SPHSystemData()->FloatDataAt(m_densityErrorsIdx);
}

float CUDAPCISPHSolver3::ComputeDelta(float timeStepInSeconds)
{
    const auto particles = SPHSystemData();
    const auto kernelRadius = static_cast<double>(particles->KernelRadius());

    Array1<Vector3D> points;
    const BccLatticePointGenerator pointsGenerator;
    const Vector3D origin;
    BoundingBox3D sampleBound(origin, origin);
    sampleBound.Expand(1.5 * kernelRadius);

    pointsGenerator.Generate(
        sampleBound, static_cast<double>(particles->TargetSpacing()), &points);

    const SPHSpikyKernel3 kernel(kernelRadius);

    double denom = 0;
    Vector3D denom1;
    double denom2 = 0;

    for (const auto& point : points)
    {
        const double distanceSquared = point.LengthSquared();

        if (distanceSquared < kernelRadius * kernelRadius)
        {
            double distance = std::sqrt(distanceSquared);
            Vector3D direction =
                (distance > 0.0) ? point / distance : Vector3D{};

            // grad(Wij)
            Vector3D gradWij = kernel.Gradient(distance, direction);
            denom1 += gradWij;
            denom2 += gradWij.Dot(gradWij);
        }
    }

    denom += -denom1.Dot(denom1) - denom2;

    const auto beta = static_cast<double>(ComputeBeta(timeStepInSeconds));

    return static_cast<float>((std::fabs(denom) > 0.0) ? -1 / (beta * denom)
                                                       : 0);
}

float CUDAPCISPHSolver3::ComputeBeta(float timeStepInSeconds)
{
    const auto particles = SPHSystemData();
    return 2.0f * Square(particles->Mass() * timeStepInSeconds /
                         particles->TargetDensity());
}

CUDAPCISPHSolver3::Builder CUDAPCISPHSolver3::GetBuilder()
{
    return Builder{};
}

CUDAPCISPHSolver3 CUDAPCISPHSolver3::Builder::Build() const
{
    return CUDAPCISPHSolver3{ m_targetDensity, m_targetSpacing,
                              m_relativeKernelRadius };
}

CUDAPCISPHSolver3Ptr CUDAPCISPHSolver3::Builder::MakeShared() const
{
    return std::make_shared<CUDAPCISPHSolver3>(m_targetDensity, m_targetSpacing,
                                               m_relativeKernelRadius);
}

#endif
