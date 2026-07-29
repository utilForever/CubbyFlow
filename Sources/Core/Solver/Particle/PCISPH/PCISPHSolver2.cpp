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
#include <Core/PointGenerator/TrianglePointGenerator.hpp>
#include <Core/Solver/Particle/PCISPH/PCISPHSolver2.hpp>
#include <Core/Utils/Logging.hpp>

namespace CubbyFlow
{
// Heuristically chosen
const double DEFAULT_TIME_STEP_LIMIT_SCALE = 5.0;

PCISPHSolver2::PCISPHSolver2()
{
    SetTimeStepLimitScale(DEFAULT_TIME_STEP_LIMIT_SCALE);
}

PCISPHSolver2::PCISPHSolver2(double targetDensity, double targetSpacing,
                             double relativeKernelRadius)
    : SPHSolver2{ targetDensity, targetSpacing, relativeKernelRadius }
{
    SetTimeStepLimitScale(DEFAULT_TIME_STEP_LIMIT_SCALE);
}

double PCISPHSolver2::GetMaxDensityErrorRatio() const
{
    return m_maxDensityErrorRatio;
}

void PCISPHSolver2::SetMaxDensityErrorRatio(double ratio)
{
    m_maxDensityErrorRatio = std::max(ratio, 0.0);
}

unsigned int PCISPHSolver2::GetMaxNumberOfIterations() const
{
    return m_maxNumberOfIterations;
}

void PCISPHSolver2::SetMaxNumberOfIterations(unsigned int n)
{
    m_maxNumberOfIterations = n;
}

void PCISPHSolver2::AccumulatePressureForce(double timeIntervalInSeconds)
{
    SPHSystemData2Ptr particles = GetSPHSystemData();
    const size_t numberOfParticles = particles->NumberOfParticles();
    const double delta = ComputeDelta(timeIntervalInSeconds);
    const double targetDensity = particles->TargetDensity();
    const double mass = particles->Mass();

    ArrayView1<double> p = particles->Pressures();
    ArrayView1<double> d = particles->Densities();
    ArrayView1<Vector2D> x = particles->Positions();
    ArrayView1<Vector2D> v = particles->Velocities();
    ArrayView1<Vector2D> f = particles->Forces();

    // Predicted density ds
    Array1<double> ds(numberOfParticles, 0.0);

    SPHStdKernel2 kernel{ particles->KernelRadius() };

    // Initialize buffers
    ParallelFor(ZERO_SIZE, numberOfParticles, [&p, this, &ds, &d](size_t i) {
        p[i] = 0.0;
        m_pressureForces[i] = Vector2D{};
        m_densityErrors[i] = 0.0;
        ds[i] = d[i];
    });

    unsigned int maxNumIter = 0;
    double maxDensityError = 0.0;
    double densityErrorRatio = 0.0;

    for (unsigned int k = 0; k < m_maxNumberOfIterations; ++k)
    {
        // Predict velocity and position
        ParallelFor(
            ZERO_SIZE, numberOfParticles,
            [this, &v, &timeIntervalInSeconds, &mass, &f, &x](size_t i) {
                m_tempVelocities[i] = v[i] + timeIntervalInSeconds / mass *
                                                 (f[i] + m_pressureForces[i]);
                m_tempPositions[i] =
                    x[i] + timeIntervalInSeconds * m_tempVelocities[i];
            });

        // Resolve collisions
        ResolveCollision(m_tempPositions, m_tempVelocities);

        // Compute pressure from density error
        ParallelFor(ZERO_SIZE, numberOfParticles,
                    [&particles, this, &kernel, &mass, &targetDensity, &delta,
                     &p, &ds](size_t i) {
                        double weightSum = 0.0;
                        const auto& neighbors = particles->NeighborLists()[i];
                        for (size_t j : neighbors)
                        {
                            const double dist = m_tempPositions[j].DistanceTo(
                                m_tempPositions[i]);
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
        m_pressureForces.Fill(Vector2D{});
        SPHSolver2::AccumulatePressureForce(x, ds, p, m_pressureForces);

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

void PCISPHSolver2::OnBeginAdvanceTimeStep(double timeStepInSeconds)
{
    SPHSolver2::OnBeginAdvanceTimeStep(timeStepInSeconds);

    // Allocate temp buffers
    const size_t numberOfParticles =
        GetParticleSystemData()->NumberOfParticles();
    m_tempPositions.Resize(numberOfParticles);
    m_tempVelocities.Resize(numberOfParticles);
    m_pressureForces.Resize(numberOfParticles);
    m_densityErrors.Resize(numberOfParticles);
}

double PCISPHSolver2::ComputeDelta(double timeStepInSeconds) const
{
    const SPHSystemData2Ptr particles = GetSPHSystemData();
    const double kernelRadius = particles->KernelRadius();

    Array1<Vector2D> points;
    const TrianglePointGenerator pointsGenerator;
    const Vector2D origin;
    BoundingBox2D sampleBound{ origin, origin };
    sampleBound.Expand(1.5 * kernelRadius);

    pointsGenerator.Generate(sampleBound, particles->TargetSpacing(), &points);

    const SPHSpikyKernel2 kernel{ kernelRadius };

    double denom = 0;
    Vector2D denom1;
    double denom2 = 0;

    for (const auto& point : points)
    {
        const double distanceSquared = point.LengthSquared();

        if (distanceSquared < kernelRadius * kernelRadius)
        {
            const double distance = std::sqrt(distanceSquared);
            Vector2D direction =
                (distance > 0.0) ? point / distance : Vector2D{};

            // grad(Wij)
            Vector2D gradWij = kernel.Gradient(distance, direction);
            denom1 += gradWij;
            denom2 += gradWij.Dot(gradWij);
        }
    }

    denom += -denom1.Dot(denom1) - denom2;

    return (std::fabs(denom) > 0.0)
               ? -1 / (ComputeBeta(timeStepInSeconds) * denom)
               : 0;
}

double PCISPHSolver2::ComputeBeta(double timeStepInSeconds) const
{
    const SPHSystemData2Ptr particles = GetSPHSystemData();
    return 2.0 * Square(particles->Mass() * timeStepInSeconds /
                        particles->TargetDensity());
}

PCISPHSolver2::Builder PCISPHSolver2::GetBuilder()
{
    return Builder{};
}

PCISPHSolver2 PCISPHSolver2::Builder::Build() const
{
    return PCISPHSolver2{ m_targetDensity, m_targetSpacing,
                          m_relativeKernelRadius };
}

PCISPHSolver2Ptr PCISPHSolver2::Builder::MakeShared() const
{
    return std::make_shared<PCISPHSolver2>(m_targetDensity, m_targetSpacing,
                                           m_relativeKernelRadius);
}
}  // namespace CubbyFlow
