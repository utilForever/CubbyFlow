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
#include <Core/PointGenerator/BccLatticePointGenerator.hpp>
#include <Core/Solver/Particle/PCISPH/PCISPHSolver3.hpp>
#include <Core/Utils/Logging.hpp>

namespace CubbyFlow
{
// Heuristically chosen
const double DEFAULT_TIME_STEP_LIMIT_SCALE = 5.0;

PCISPHSolver3::PCISPHSolver3()
{
    SetTimeStepLimitScale(DEFAULT_TIME_STEP_LIMIT_SCALE);
}

PCISPHSolver3::PCISPHSolver3(double targetDensity, double targetSpacing,
                             double relativeKernelRadius)
    : SPHSolver3{ targetDensity, targetSpacing, relativeKernelRadius }
{
    SetTimeStepLimitScale(DEFAULT_TIME_STEP_LIMIT_SCALE);
}

double PCISPHSolver3::GetMaxDensityErrorRatio() const
{
    return m_maxDensityErrorRatio;
}

void PCISPHSolver3::SetMaxDensityErrorRatio(double ratio)
{
    m_maxDensityErrorRatio = std::max(ratio, 0.0);
}

unsigned int PCISPHSolver3::GetMaxNumberOfIterations() const
{
    return m_maxNumberOfIterations;
}

void PCISPHSolver3::SetMaxNumberOfIterations(unsigned int n)
{
    m_maxNumberOfIterations = n;
}

void PCISPHSolver3::AccumulatePressureForce(double timeIntervalInSeconds)
{
    SPHSystemData3Ptr particles = GetSPHSystemData();
    const size_t numberOfParticles = particles->NumberOfParticles();
    const double delta = ComputeDelta(timeIntervalInSeconds);
    const double targetDensity = particles->TargetDensity();
    const double mass = particles->Mass();

    ArrayView1<double> p = particles->Pressures();
    ArrayView1<double> d = particles->Densities();
    ArrayView1<Vector3D> x = particles->Positions();
    ArrayView1<Vector3D> v = particles->Velocities();
    ArrayView1<Vector3D> f = particles->Forces();

    // Predicted density ds
    Array1<double> ds(numberOfParticles, 0.0);

    SPHStdKernel3 kernel{ particles->KernelRadius() };

    // Initialize buffers
    ParallelFor(ZERO_SIZE, numberOfParticles, [&](size_t i) {
        p[i] = 0.0;
        m_pressureForces[i] = Vector3D{};
        m_densityErrors[i] = 0.0;
        ds[i] = d[i];
    });

    unsigned int maxNumIter = 0;
    double maxDensityError = 0.0;
    double densityErrorRatio = 0.0;

    for (unsigned int k = 0; k < m_maxNumberOfIterations; ++k)
    {
        // Predict velocity and position
        ParallelFor(ZERO_SIZE, numberOfParticles, [&](size_t i) {
            m_tempVelocities[i] = v[i] + timeIntervalInSeconds / mass *
                                             (f[i] + m_pressureForces[i]);
            m_tempPositions[i] =
                x[i] + timeIntervalInSeconds * m_tempVelocities[i];
        });

        // Resolve collisions
        ResolveCollision(m_tempPositions, m_tempVelocities);

        // Compute pressure from density error
        ParallelFor(ZERO_SIZE, numberOfParticles, [&](size_t i) {
            double weightSum = 0.0;
            const auto& neighbors = particles->NeighborLists()[i];

            for (size_t j : neighbors)
            {
                const double dist =
                    m_tempPositions[j].DistanceTo(m_tempPositions[i]);
                weightSum += kernel(dist);
            }
            weightSum += kernel(0);

            const double density = mass * weightSum;
            double densityError = (density - targetDensity);
            double pressure = delta * densityError;

            if (pressure < 0.0)
            {
                pressure *= GetNegativePressureScale();
                densityError *= GetNegativePressureScale();
            }

            p[i] += pressure;
            ds[i] = density;
            m_densityErrors[i] = densityError;
        });

        // Compute pressure gradient force
        m_pressureForces.Fill(Vector3D{});
        SPHSolver3::AccumulatePressureForce(x, ds, p, m_pressureForces);

        // Compute max density error
        maxDensityError = 0.0;
        for (size_t i = 0; i < numberOfParticles; ++i)
        {
            maxDensityError = AbsMax(maxDensityError, m_densityErrors[i]);
        }

        densityErrorRatio = maxDensityError / targetDensity;
        maxNumIter = k + 1;

        if (std::fabs(densityErrorRatio) < m_maxDensityErrorRatio)
        {
            break;
        }
    }

    CUBBYFLOW_INFO << "Number of PCI iterations: " << maxNumIter;
    CUBBYFLOW_INFO << "Max density error after PCI iteration: "
                   << maxDensityError;

    if (std::fabs(densityErrorRatio) > m_maxDensityErrorRatio)
    {
        CUBBYFLOW_WARN
            << "Max density error ratio is greater than the threshold!";
        CUBBYFLOW_WARN << "Ratio: " << densityErrorRatio
                       << " Threshold: " << m_maxDensityErrorRatio;
    }

    // Accumulate pressure force
    ParallelFor(ZERO_SIZE, numberOfParticles,
                [this, &f](size_t i) { f[i] += m_pressureForces[i]; });
}

void PCISPHSolver3::OnBeginAdvanceTimeStep(double timeStepInSeconds)
{
    SPHSolver3::OnBeginAdvanceTimeStep(timeStepInSeconds);

    // Allocate temp buffers
    const size_t numberOfParticles =
        GetParticleSystemData()->NumberOfParticles();
    m_tempPositions.Resize(numberOfParticles);
    m_tempVelocities.Resize(numberOfParticles);
    m_pressureForces.Resize(numberOfParticles);
    m_densityErrors.Resize(numberOfParticles);
}

double PCISPHSolver3::ComputeDelta(double timeStepInSeconds) const
{
    const SPHSystemData3Ptr particles = GetSPHSystemData();
    const double kernelRadius = particles->KernelRadius();

    Array1<Vector3D> points;
    const BccLatticePointGenerator pointsGenerator;
    const Vector3D origin;
    BoundingBox3D sampleBound{ origin, origin };
    sampleBound.Expand(1.5 * kernelRadius);

    pointsGenerator.Generate(sampleBound, particles->TargetSpacing(), &points);

    const SPHSpikyKernel3 kernel{ kernelRadius };

    double denom = 0;
    Vector3D denom1;
    double denom2 = 0;

    for (const auto& point : points)
    {
        const double distanceSquared = point.LengthSquared();

        if (distanceSquared < kernelRadius * kernelRadius)
        {
            const double distance = std::sqrt(distanceSquared);
            Vector3D direction =
                (distance > 0.0) ? point / distance : Vector3D{};

            // grad(Wij)
            Vector3D gradWij = kernel.Gradient(distance, direction);
            denom1 += gradWij;
            denom2 += gradWij.Dot(gradWij);
        }
    }

    denom += -denom1.Dot(denom1) - denom2;

    return (std::fabs(denom) > 0.0)
               ? -1 / (ComputeBeta(timeStepInSeconds) * denom)
               : 0;
}

double PCISPHSolver3::ComputeBeta(double timeStepInSeconds) const
{
    const SPHSystemData3Ptr particles = GetSPHSystemData();
    return 2.0 * Square(particles->Mass() * timeStepInSeconds /
                        particles->TargetDensity());
}

PCISPHSolver3::Builder PCISPHSolver3::GetBuilder()
{
    return Builder{};
}

PCISPHSolver3 PCISPHSolver3::Builder::Build() const
{
    return PCISPHSolver3{ m_targetDensity, m_targetSpacing,
                          m_relativeKernelRadius };
}

PCISPHSolver3Ptr PCISPHSolver3::Builder::MakeShared() const
{
    return std::make_shared<PCISPHSolver3>(m_targetDensity, m_targetSpacing,
                                           m_relativeKernelRadius);
}
}  // namespace CubbyFlow