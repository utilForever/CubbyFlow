// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Particle/SPHKernels.hpp>
#include <Core/Solver/Particle/SPH/SPHSolver3.hpp>
#include <Core/Utils/Logging.hpp>
#include <Core/Utils/PhysicsHelpers.hpp>
#include <Core/Utils/Timer.hpp>

namespace CubbyFlow
{
constexpr double TIME_STEP_LIMIT_BY_SPEED_FACTOR = 0.4;
constexpr double TIME_STEP_LIMIT_BY_FORCE_FACTOR = 0.25;

SPHSolver3::SPHSolver3()
{
    SetParticleSystemData(std::make_shared<SPHSystemData3>());
    SetIsUsingFixedSubTimeSteps(false);
}

SPHSolver3::SPHSolver3(double targetDensity, double targetSpacing,
                       double relativeKernelRadius)
{
    auto sphParticles = std::make_shared<SPHSystemData3>();
    SetParticleSystemData(sphParticles);
    sphParticles->SetTargetDensity(targetDensity);
    sphParticles->SetTargetSpacing(targetSpacing);
    sphParticles->SetRelativeKernelRadius(relativeKernelRadius);
    SetIsUsingFixedSubTimeSteps(false);
}

double SPHSolver3::GetEosExponent() const
{
    return m_eosExponent;
}

void SPHSolver3::SetEosExponent(double newEosExponent)
{
    m_eosExponent = std::max(newEosExponent, 1.0);
}

double SPHSolver3::GetNegativePressureScale() const
{
    return m_negativePressureScale;
}

void SPHSolver3::SetNegativePressureScale(double newNegativePressureScale)
{
    m_negativePressureScale = std::clamp(newNegativePressureScale, 0.0, 1.0);
}

double SPHSolver3::GetViscosityCoefficient() const
{
    return m_viscosityCoefficient;
}

void SPHSolver3::SetViscosityCoefficient(double newViscosityCoefficient)
{
    m_viscosityCoefficient = std::max(newViscosityCoefficient, 0.0);
}

double SPHSolver3::GetPseudoViscosityCoefficient() const
{
    return m_pseudoViscosityCoefficient;
}

void SPHSolver3::SetPseudoViscosityCoefficient(
    double newPseudoViscosityCoefficient)
{
    m_pseudoViscosityCoefficient = std::max(newPseudoViscosityCoefficient, 0.0);
}

double SPHSolver3::GetSpeedOfSound() const
{
    return m_speedOfSound;
}

void SPHSolver3::SetSpeedOfSound(double newSpeedOfSound)
{
    m_speedOfSound =
        std::max(newSpeedOfSound, std::numeric_limits<double>::epsilon());
}

double SPHSolver3::GetTimeStepLimitScale() const
{
    return m_timeStepLimitScale;
}

void SPHSolver3::SetTimeStepLimitScale(double newScale)
{
    m_timeStepLimitScale = std::max(newScale, 0.0);
}

SPHSystemData3Ptr SPHSolver3::GetSPHSystemData() const
{
    return std::dynamic_pointer_cast<SPHSystemData3>(GetParticleSystemData());
}

unsigned int SPHSolver3::GetNumberOfSubTimeSteps(
    double timeIntervalInSeconds) const
{
    SPHSystemData3Ptr particles = GetSPHSystemData();
    const size_t numberOfParticles = particles->NumberOfParticles();
    ArrayView1<Vector3D> f = particles->Forces();

    const double kernelRadius = particles->KernelRadius();
    const double mass = particles->Mass();

    double maxForceMagnitude = 0.0;

    for (size_t i = 0; i < numberOfParticles; ++i)
    {
        maxForceMagnitude = std::max(maxForceMagnitude, f[i].Length());
    }

    const double timeStepLimitBySpeed =
        TIME_STEP_LIMIT_BY_SPEED_FACTOR * kernelRadius / m_speedOfSound;
    const double timeStepLimitByForce =
        TIME_STEP_LIMIT_BY_FORCE_FACTOR *
        std::sqrt(kernelRadius * mass / maxForceMagnitude);

    const double desiredTimeStep =
        m_timeStepLimitScale *
        std::min(timeStepLimitBySpeed, timeStepLimitByForce);

    return static_cast<unsigned int>(
        std::ceil(timeIntervalInSeconds / desiredTimeStep));
}

void SPHSolver3::AccumulateForces(double timeStepInSeconds)
{
    AccumulateNonPressureForces(timeStepInSeconds);
    AccumulatePressureForce(timeStepInSeconds);
}

void SPHSolver3::OnBeginAdvanceTimeStep(double timeStepInSeconds)
{
    UNUSED_VARIABLE(timeStepInSeconds);

    SPHSystemData3Ptr particles = GetSPHSystemData();

    const Timer timer;
    particles->BuildNeighborSearcher();
    particles->BuildNeighborLists();
    particles->UpdateDensities();

    CUBBYFLOW_INFO << "Building neighbor lists and updating densities took "
                   << timer.DurationInSeconds() << " seconds";
}

void SPHSolver3::OnEndAdvanceTimeStep(double timeStepInSeconds)
{
    ComputePseudoViscosity(timeStepInSeconds);

    SPHSystemData3Ptr particles = GetSPHSystemData();
    const size_t numberOfParticles = particles->NumberOfParticles();
    ArrayView1<double> densities = particles->Densities();

    double maxDensity = 0.0;
    for (size_t i = 0; i < numberOfParticles; ++i)
    {
        maxDensity = std::max(maxDensity, densities[i]);
    }

    CUBBYFLOW_INFO << "Max density: " << maxDensity << " "
                   << "Max density / target density ratio: "
                   << maxDensity / particles->TargetDensity();
}

void SPHSolver3::AccumulateNonPressureForces(double timeStepInSeconds)
{
    ParticleSystemSolver3::AccumulateForces(timeStepInSeconds);
    AccumulateViscosityForce();
}

void SPHSolver3::AccumulatePressureForce(double timeStepInSeconds)
{
    UNUSED_VARIABLE(timeStepInSeconds);

    SPHSystemData3Ptr particles = GetSPHSystemData();
    const ArrayView1<Vector3D> x = particles->Positions();
    const ArrayView1<double> d = particles->Densities();
    const ArrayView1<double> p = particles->Pressures();
    const ArrayView1<Vector3D> f = particles->Forces();

    ComputePressure();
    AccumulatePressureForce(x, d, p, f);
}

void SPHSolver3::ComputePressure()
{
    SPHSystemData3Ptr particles = GetSPHSystemData();
    const size_t numberOfParticles = particles->NumberOfParticles();
    ArrayView1<double> d = particles->Densities();
    ArrayView1<double> p = particles->Pressures();

    // See Murnaghan-Tait equation of state from
    // https://en.wikipedia.org/wiki/Tait_equation
    const double targetDensity = particles->TargetDensity();
    const double eosScale = targetDensity * Square(m_speedOfSound);

    ParallelFor(ZERO_SIZE, numberOfParticles,
                [&p, &d, &targetDensity, &eosScale, this](size_t i) {
                    p[i] = ComputePressureFromEos(d[i], targetDensity, eosScale,
                                                  GetEosExponent(),
                                                  GetNegativePressureScale());
                });
}

void SPHSolver3::AccumulatePressureForce(
    const ConstArrayView1<Vector3D>& positions,
    const ConstArrayView1<double>& densities,
    const ConstArrayView1<double>& pressures,
    ArrayView1<Vector3D> pressureForces)
{
    SPHSystemData3Ptr particles = GetSPHSystemData();
    const size_t numberOfParticles = particles->NumberOfParticles();

    const double massSquared = Square(particles->Mass());
    const SPHSpikyKernel3 kernel{ particles->KernelRadius() };

    ParallelFor(ZERO_SIZE, numberOfParticles,
                [&particles, &positions, &pressureForces, &massSquared,
                 &pressures, &densities, &kernel](size_t i) {
                    const auto& neighbors = particles->NeighborLists()[i];
                    for (size_t j : neighbors)
                    {
                        const double dist =
                            positions[i].DistanceTo(positions[j]);
                        if (dist > 0.0)
                        {
                            Vector3D dir = (positions[j] - positions[i]) / dist;
                            pressureForces[i] -=
                                massSquared *
                                (pressures[i] / (densities[i] * densities[i]) +
                                 pressures[j] / (densities[j] * densities[j])) *
                                kernel.Gradient(dist, dir);
                        }
                    }
                });
}

void SPHSolver3::AccumulateViscosityForce()
{
    SPHSystemData3Ptr particles = GetSPHSystemData();
    const size_t numberOfParticles = particles->NumberOfParticles();
    ArrayView1<Vector3D> x = particles->Positions();
    ArrayView1<Vector3D> v = particles->Velocities();
    ArrayView1<double> d = particles->Densities();
    ArrayView1<Vector3D> f = particles->Forces();

    const double massSquared = Square(particles->Mass());
    const SPHSpikyKernel3 kernel{ particles->KernelRadius() };

    ParallelFor(
        ZERO_SIZE, numberOfParticles,
        [&particles, &x, &f, this, &massSquared, &v, &d, &kernel](size_t i) {
            const auto& neighbors = particles->NeighborLists()[i];
            for (size_t j : neighbors)
            {
                const double dist = x[i].DistanceTo(x[j]);

                f[i] += GetViscosityCoefficient() * massSquared *
                        (v[j] - v[i]) / d[j] * kernel.SecondDerivative(dist);
            }
        });
}

void SPHSolver3::ComputePseudoViscosity(double timeStepInSeconds)
{
    SPHSystemData3Ptr particles = GetSPHSystemData();
    const size_t numberOfParticles = particles->NumberOfParticles();
    ArrayView1<Vector3D> x = particles->Positions();
    ArrayView1<Vector3D> v = particles->Velocities();
    ArrayView1<double> d = particles->Densities();

    const double mass = particles->Mass();
    const SPHSpikyKernel3 kernel{ particles->KernelRadius() };

    Array1<Vector3D> smoothedVelocities{ numberOfParticles };

    ParallelFor(ZERO_SIZE, numberOfParticles,
                [&particles, &x, &mass, &d, &kernel, &v,
                 &smoothedVelocities](size_t i) {
                    double weightSum = 0.0;
                    Vector3D smoothedVelocity;

                    const auto& neighbors = particles->NeighborLists()[i];
                    for (size_t j : neighbors)
                    {
                        const double dist = x[i].DistanceTo(x[j]);
                        const double wj = mass / d[j] * kernel(dist);
                        weightSum += wj;
                        smoothedVelocity += wj * v[j];
                    }

                    const double wi = mass / d[i];
                    weightSum += wi;
                    smoothedVelocity += wi * v[i];

                    if (weightSum > 0.0)
                    {
                        smoothedVelocity /= weightSum;
                    }

                    smoothedVelocities[i] = smoothedVelocity;
                });

    double factor = timeStepInSeconds * m_pseudoViscosityCoefficient;
    factor = std::clamp(factor, 0.0, 1.0);

    ParallelFor(ZERO_SIZE, numberOfParticles,
                [&v, &smoothedVelocities, &factor](size_t i) {
                    v[i] = Lerp(v[i], smoothedVelocities[i], factor);
                });
}

SPHSolver3::Builder SPHSolver3::GetBuilder()
{
    return Builder{};
}

SPHSolver3 SPHSolver3::Builder::Build() const
{
    return SPHSolver3{ m_targetDensity, m_targetSpacing,
                       m_relativeKernelRadius };
}

SPHSolver3Ptr SPHSolver3::Builder::MakeShared() const
{
    return std::make_shared<SPHSolver3>(m_targetDensity, m_targetSpacing,
                                        m_relativeKernelRadius);
}
}  // namespace CubbyFlow
