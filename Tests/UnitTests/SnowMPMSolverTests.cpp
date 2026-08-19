#include "gtest/gtest.h"

#include "../../Examples/SnowMPMSim/SnowBall.hpp"

#include <Core/Geometry/Plane.hpp>
#include <Core/Geometry/RigidBodyCollider.hpp>
#include <Core/Solver/Particle/MPM/SnowMPMSolver.hpp>
#include <Core/Utils/Constants.hpp>
#include <Core/Utils/IterationUtils.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

using namespace CubbyFlow;

namespace
{
template <size_t N>
using VectorD = Vector<double, N>;

template <size_t N>
using VectorUZ = Vector<size_t, N>;

template <size_t N>
class TestableSnowMPMSolver final : public SnowMPMSolver<N>
{
 public:
    using SnowMPMSolver<N>::SnowMPMSolver;

    void Initialize()
    {
        this->OnInitialize();
    }

    void BeginStep(double timeStepInSeconds)
    {
        this->OnBeginAdvanceTimeStep(timeStepInSeconds);
    }

    [[nodiscard]] unsigned int NumberOfSubTimeSteps(double interval) const
    {
        return this->GetNumberOfSubTimeSteps(interval);
    }
};

template <size_t N>
VectorD<N> InteriorPosition(const VectorD<N>& spacing)
{
    VectorD<N> result;
    for (size_t axis = 0; axis < N; ++axis)
    {
        result[axis] = 3.0 * spacing[axis];
    }
    return result;
}

template <size_t N>
void UseOneFixedStep(SnowMPMSolver<N>* solver)
{
    solver->SetIsUsingFixedSubTimeSteps(true);
    solver->SetNumberOfFixedSubTimeSteps(1);
}

template <size_t N>
void AddLinearParticleLattice(SnowMPMSolver<N>* solver, double rate)
{
    auto data = solver->GetMPMSystemData();
    const auto dataSize = data->GridMass().DataSize();
    const auto spacing = data->GridMass().GridSpacing();
    const auto origin = data->GridMass().DataOrigin();
    const double center =
        origin[0] + 0.5 * static_cast<double>(dataSize[0] - 1) * spacing[0];
    Array1<VectorD<N>> positions;
    Array1<VectorD<N>> velocities;

    ForEachIndex(dataSize, [center, origin, &positions, rate, spacing,
                            &velocities](auto... rawIndices) {
        const VectorUZ<N> index{ rawIndices... };
        VectorD<N> position = origin;
        for (size_t axis = 0; axis < N; ++axis)
        {
            position[axis] += spacing[axis] * static_cast<double>(index[axis]);
        }

        VectorD<N> velocity;
        velocity[0] = rate * (position[0] - center);
        positions.Append(position);
        velocities.Append(velocity);
    });

    data->AddParticles(positions, velocities);
}

template <size_t N>
void AddCompressedParticleLattice(SnowMPMSolver<N>* solver)
{
    auto data = solver->GetMPMSystemData();
    const auto spacing = data->GridMass().GridSpacing();
    const auto origin = data->GridMass().DataOrigin();
    Array1<VectorD<N>> positions;

    ForEachIndex(VectorUZ<N>::MakeConstant(2), VectorUZ<N>::MakeConstant(8),
                 [origin, &positions, spacing](auto... rawIndices) {
                     const VectorUZ<N> index{ rawIndices... };
                     VectorD<N> position = origin;

                     for (size_t axis = 0; axis < N; ++axis)
                     {
                         position[axis] +=
                             spacing[axis] * static_cast<double>(index[axis]);
                     }

                     positions.Append(position);
                 });

    data->AddParticles(positions);

    const auto masses = data->ParticleMasses();
    auto volumes = data->InitialVolumes();
    auto states = data->DeformationStates();

    for (size_t i = 0; i < positions.Length(); ++i)
    {
        volumes[i] = masses[i] / 400.0;
        states[i].elastic(0, 0) = 0.98;
    }
}

template <size_t N>
std::shared_ptr<MPMSystemData<N>> RunCompressedCase(bool semiImplicit)
{
    SnowMPMSolver<N> solver{ VectorUZ<N>::MakeConstant(10),
                             VectorD<N>::MakeConstant(0.01) };

    UseOneFixedStep(&solver);
    solver.SetClosedDomainBoundaryFlag(DIRECTION_NONE);
    solver.SetIsUsingSemiImplicit(semiImplicit);

    if (semiImplicit)
    {
        solver.SetTolerance(1e-4);
    }

    solver.SetGravity({});
    solver.SetDragCoefficient(0.0);
    AddCompressedParticleLattice(&solver);

    for (int frame = 0; frame < 3; ++frame)
    {
        solver.Update(Frame{ frame, 0.002 });
    }

    return solver.GetMPMSystemData();
}

template <size_t N>
double MaxParticleSpeed(const MPMSystemData<N>& data)
{
    double result = 0.0;

    for (const auto& velocity : data.Velocities())
    {
        for (double value : velocity)
        {
            if (!std::isfinite(value))
            {
                return std::numeric_limits<double>::infinity();
            }
        }

        result = std::max(result, velocity.Length());
    }

    return result;
}

template <size_t N>
size_t FindParticle(const MPMSystemData<N>& data, const VectorD<N>& position)
{
    const auto positions = data.Positions();
    for (size_t i = 0; i < positions.Length(); ++i)
    {
        if (positions[i].IsSimilar(position, 1e-12))
        {
            return i;
        }
    }

    return positions.Length();
}

template <size_t N>
void ExpectEmptyUpdate()
{
    SnowMPMSolver<N> solver;
    solver.Update(Frame{ 0, 0.01 });
    EXPECT_EQ(solver.GetMPMSystemData()->NumberOfParticles(), 0u);
}

template <size_t N>
void ExpectEmptySemiImplicitUpdate()
{
    SnowMPMSolver<N> solver;

    solver.SetIsUsingSemiImplicit(true);
    solver.Update(Frame{ 0, 0.01 });

    EXPECT_EQ(solver.GetMPMSystemData()->NumberOfParticles(), 0u);
    EXPECT_EQ(solver.GetLastNumberOfIterations(), 0u);
    EXPECT_DOUBLE_EQ(solver.GetLastResidual(), 0.0);
}

template <size_t N>
void ExpectZeroResidualSemiImplicitUpdate()
{
    SnowMPMSolver<N> solver{ VectorUZ<N>::MakeConstant(8) };
    UseOneFixedStep(&solver);
    solver.SetIsUsingSemiImplicit(true);
    solver.SetGravity({});
    solver.SetDragCoefficient(0.0);

    auto data = solver.GetMPMSystemData();
    data->AddParticle(InteriorPosition(VectorD<N>::MakeConstant(1.0)));
    solver.Update(Frame{ 0, 0.01 });

    EXPECT_TRUE(data->Velocities()[0].IsSimilar(VectorD<N>{}, 0.0));
    EXPECT_EQ(solver.GetLastNumberOfIterations(), 0u);
    EXPECT_DOUBLE_EQ(solver.GetLastResidual(), 0.0);
}

template <size_t N>
void ExpectSmallStepMatchesExplicit()
{
    const auto run = [](bool semiImplicit) {
        SnowMPMSolver<N> solver{ VectorUZ<N>::MakeConstant(8) };
        UseOneFixedStep(&solver);
        solver.SetIsUsingSemiImplicit(semiImplicit);
        solver.SetGravity({});
        solver.SetDragCoefficient(0.0);

        AddLinearParticleLattice(&solver, -0.1);

        solver.Update(Frame{ 0, 1e-8 });
        return solver.GetMPMSystemData();
    };

    const auto explicitData = run(false);
    const auto implicitData = run(true);

    ASSERT_EQ(explicitData->NumberOfParticles(),
              implicitData->NumberOfParticles());

    for (size_t i = 0; i < explicitData->NumberOfParticles(); ++i)
    {
        EXPECT_TRUE(explicitData->Velocities()[i].IsSimilar(
            implicitData->Velocities()[i], 1e-8));
        EXPECT_TRUE(explicitData->DeformationStates()[i].elastic.IsSimilar(
            implicitData->DeformationStates()[i].elastic, 1e-8));
    }
}

template <size_t N>
void ExpectSemiImplicitStiffStability()
{
    const auto explicitData = RunCompressedCase<N>(false);
    const auto implicitData = RunCompressedCase<N>(true);

    for (const auto& position : implicitData->Positions())
    {
        for (double value : position)
        {
            EXPECT_TRUE(std::isfinite(value));
        }
    }
    for (const auto& velocity : implicitData->Velocities())
    {
        for (double value : velocity)
        {
            EXPECT_TRUE(std::isfinite(value));
        }
    }
    for (const auto& state : implicitData->DeformationStates())
    {
        for (double value : state.elastic)
        {
            EXPECT_TRUE(std::isfinite(value));
        }
        for (double value : state.plastic)
        {
            EXPECT_TRUE(std::isfinite(value));
        }
    }

    const double explicitSpeed = MaxParticleSpeed(*explicitData);
    const double implicitSpeed = MaxParticleSpeed(*implicitData);

    EXPECT_GT(explicitSpeed, 1.0);
    EXPECT_LT(implicitSpeed, explicitSpeed);
    EXPECT_LT(implicitSpeed, 1.0);
}

template <size_t N>
void ExpectSemiImplicitConvergesAcrossMassScales()
{
    SnowMPMSolver<N> solver{ VectorUZ<N>::MakeConstant(16),
                             VectorD<N>::MakeConstant(0.1) };
    UseOneFixedStep(&solver);
    solver.SetClosedDomainBoundaryFlag(DIRECTION_NONE);
    solver.SetIsUsingSemiImplicit(true);
    solver.SetMaxNumberOfIterations(10);
    solver.SetTolerance(1e-3);
    solver.SetGravity({});
    solver.SetDragCoefficient(0.0);

    Array1<VectorD<N>> positions;
    for (size_t cluster = 0; cluster < 2; ++cluster)
    {
        ForEachIndex(VectorUZ<N>::MakeConstant(2),
                     [cluster, &positions](auto... rawIndices) {
                         const VectorUZ<N> index{ rawIndices... };
                         VectorD<N> position;
                         for (size_t axis = 0; axis < N; ++axis)
                         {
                             position[axis] =
                                 0.1 * static_cast<double>(index[axis] + 3) +
                                 0.7 * static_cast<double>(cluster);
                         }
                         positions.Append(position);
                     });
    }

    auto data = solver.GetMPMSystemData();
    data->AddParticles(positions);
    auto masses = data->ParticleMasses();
    auto volumes = data->InitialVolumes();
    auto states = data->DeformationStates();
    const size_t clusterSize = size_t{ 1 } << N;

    for (size_t i = 0; i < positions.Length(); ++i)
    {
        const size_t local = i % clusterSize;
        masses[i] = std::pow(1e-12, static_cast<double>(local) /
                                        static_cast<double>(clusterSize - 1));
        volumes[i] = masses[i] / 400.0;
        states[i].elastic(0, 0) = 0.98;
    }

    EXPECT_NO_THROW(solver.Update(Frame{ 0, 5e-4 }));
    EXPECT_LE(solver.GetLastNumberOfIterations(), 10u);
    EXPECT_LE(solver.GetLastResidual(), solver.GetTolerance());
}

template <size_t N>
void ExpectSemiImplicitFailureRollback()
{
    SnowMPMSolver<N> solver{ VectorUZ<N>::MakeConstant(10),
                             VectorD<N>::MakeConstant(0.01) };
    UseOneFixedStep(&solver);
    solver.SetClosedDomainBoundaryFlag(DIRECTION_NONE);
    solver.SetIsUsingSemiImplicit(true);
    solver.SetMaxNumberOfIterations(0);
    solver.SetTolerance(1e-12);
    solver.SetGravity({});
    solver.SetDragCoefficient(0.0);

    AddCompressedParticleLattice(&solver);

    auto data = solver.GetMPMSystemData();
    const Array1<VectorD<N>> positionsBefore(data->Positions());
    const Array1<VectorD<N>> velocitiesBefore(data->Velocities());
    const Array1<SnowDeformationState<N>> statesBefore(
        data->DeformationStates());

    EXPECT_THROW(solver.Update(Frame{ 0, 0.002 }), std::runtime_error);
    EXPECT_EQ(solver.GetLastNumberOfIterations(), 0u);
    EXPECT_GT(solver.GetLastResidual(), solver.GetTolerance());

    for (size_t i = 0; i < data->NumberOfParticles(); ++i)
    {
        EXPECT_EQ(data->Positions()[i], positionsBefore[i]);
        EXPECT_EQ(data->Velocities()[i], velocitiesBefore[i]);
        EXPECT_EQ(data->DeformationStates()[i].elastic,
                  statesBefore[i].elastic);
        EXPECT_EQ(data->DeformationStates()[i].plastic,
                  statesBefore[i].plastic);
    }
}

template <size_t N>
void ExpectReferenceVolumeUsesCellVolume()
{
    VectorD<N> spacing = VectorD<N>::MakeConstant(1.0);
    spacing[0] = 0.5;

    if constexpr (N == 3)
    {
        spacing[2] = 2.0;
    }

    SnowMPMSolver<N> solver{
        VectorUZ<N>::MakeConstant(8), spacing, {}, 0.1, 2.0
    };

    UseOneFixedStep(&solver);
    solver.SetGravity({});

    auto data = solver.GetMPMSystemData();
    data->AddParticle(InteriorPosition(spacing));
    solver.Update(Frame{ 0, 0.001 });

    double cellVolume = 1.0;

    for (size_t axis = 0; axis < N; ++axis)
    {
        cellVolume *= spacing[axis];
    }

    EXPECT_NEAR(data->InitialVolumes()[0],
                cellVolume * static_cast<double>(size_t{ 1 } << N), 1e-12);
    EXPECT_TRUE(data->DeformationStates()[0].elastic.IsSimilar(
        Matrix<double, N, N>::MakeIdentity(), 1e-12));
    EXPECT_TRUE(data->DeformationStates()[0].plastic.IsSimilar(
        Matrix<double, N, N>::MakeIdentity(), 1e-12));
}

template <size_t N>
void ExpectUniformMotion()
{
    SnowMPMSolver<N> solver{ VectorUZ<N>::MakeConstant(8) };
    UseOneFixedStep(&solver);
    solver.SetGravity({});
    solver.SetDragCoefficient(0.0);

    VectorD<N> velocity;
    velocity[0] = 2.0;

    const auto initialPosition =
        InteriorPosition(VectorD<N>::MakeConstant(1.0));
    auto data = solver.GetMPMSystemData();
    data->AddParticle(initialPosition, velocity);

    solver.Update(Frame{ 0, 0.01 });

    EXPECT_TRUE(data->Velocities()[0].IsSimilar(velocity, 1e-12));
    EXPECT_TRUE(data->Positions()[0].IsSimilar(
        initialPosition + 0.01 * velocity, 1e-12));
}

template <size_t N>
void ExpectExternalForcesAppliedOnce()
{
    SnowMPMSolver<N> gravitySolver{ VectorUZ<N>::MakeConstant(8),
                                    VectorD<N>::MakeConstant(1.0),
                                    {},
                                    0.1,
                                    2.0 };

    UseOneFixedStep(&gravitySolver);
    gravitySolver.SetDragCoefficient(0.0);

    VectorD<N> gravity;
    gravity[1] = -2.0;

    gravitySolver.SetGravity(gravity);

    auto gravityData = gravitySolver.GetMPMSystemData();
    gravityData->AddParticle(InteriorPosition(VectorD<N>::MakeConstant(1.0)));
    gravitySolver.Update(Frame{ 0, 0.1 });

    EXPECT_TRUE(gravityData->Velocities()[0].IsSimilar(0.1 * gravity, 1e-12));

    SnowMPMSolver<N> dragSolver{ VectorUZ<N>::MakeConstant(8),
                                 VectorD<N>::MakeConstant(1.0),
                                 {},
                                 0.1,
                                 2.0 };
    UseOneFixedStep(&dragSolver);
    dragSolver.SetGravity({});
    dragSolver.SetDragCoefficient(2.0);

    VectorD<N> velocity;
    velocity[0] = 1.0;

    auto dragData = dragSolver.GetMPMSystemData();
    dragData->AddParticle(InteriorPosition(VectorD<N>::MakeConstant(1.0)),
                          velocity);
    dragSolver.Update(Frame{ 0, 0.1 });

    velocity[0] = 0.9;

    EXPECT_TRUE(dragData->Velocities()[0].IsSimilar(velocity, 1e-12));
}

template <size_t N>
void ExpectLinearVelocityUpdatesDeformation()
{
    SnowMPMSolver<N> solver{ VectorUZ<N>::MakeConstant(8) };
    UseOneFixedStep(&solver);
    solver.SetGravity({});
    solver.SetDragCoefficient(0.0);

    AddLinearParticleLattice(&solver, 1.0);

    auto data = solver.GetMPMSystemData();
    const auto target = InteriorPosition(VectorD<N>::MakeConstant(1.0));
    const size_t targetIndex = FindParticle(*data, target);

    ASSERT_LT(targetIndex, data->NumberOfParticles());

    solver.Update(Frame{ 0, 0.001 });

    auto expected = Matrix<double, N, N>::MakeIdentity();
    expected(0, 0) = 1.001;

    EXPECT_TRUE(data->DeformationStates()[targetIndex].elastic.IsSimilar(
        expected, 1e-10));
}

template <size_t N>
void ExpectAdaptiveRestrictions()
{
    const auto resolution = VectorUZ<N>::MakeConstant(8);
    const auto unit = VectorD<N>::MakeConstant(1.0);
    const auto position = InteriorPosition(unit);

    TestableSnowMPMSolver<N> baseline{ resolution, unit };
    baseline.SetGravity({});
    baseline.GetMPMSystemData()->AddParticle(position);
    baseline.GetMPMSystemData()->InitialVolumes()[0] =
        baseline.GetMPMSystemData()->ParticleMasses()[0] / 400.0;
    baseline.Initialize();

    const unsigned int baselineSteps = baseline.NumberOfSubTimeSteps(0.1);
    baseline.SetIsUsingSemiImplicit(true);

    EXPECT_EQ(baseline.NumberOfSubTimeSteps(0.1), 1u);

    TestableSnowMPMSolver<N> finer{ resolution, VectorD<N>::MakeConstant(0.5) };
    finer.SetGravity({});
    finer.GetMPMSystemData()->AddParticle(0.5 * position);
    finer.GetMPMSystemData()->InitialVolumes()[0] =
        finer.GetMPMSystemData()->ParticleMasses()[0] / 400.0;
    finer.Initialize();

    EXPECT_GT(finer.NumberOfSubTimeSteps(0.1), baselineSteps);

    TestableSnowMPMSolver<N> fast{ resolution, unit };
    fast.SetGravity({});

    VectorD<N> fastVelocity;
    fastVelocity[0] = 100.0;

    fast.GetMPMSystemData()->AddParticle(position, fastVelocity);
    fast.GetMPMSystemData()->InitialVolumes()[0] =
        fast.GetMPMSystemData()->ParticleMasses()[0] / 400.0;
    fast.Initialize();

    EXPECT_GT(fast.NumberOfSubTimeSteps(0.1), baselineSteps);

    fast.SetIsUsingSemiImplicit(true);

    EXPECT_GT(fast.NumberOfSubTimeSteps(0.1), 1u);

    TestableSnowMPMSolver<N> softerDensity{ resolution, unit };
    softerDensity.SetGravity({});
    softerDensity.GetMPMSystemData()->AddParticle(position);
    softerDensity.GetMPMSystemData()->InitialVolumes()[0] =
        softerDensity.GetMPMSystemData()->ParticleMasses()[0] / 100.0;
    softerDensity.Initialize();

    EXPECT_GT(softerDensity.NumberOfSubTimeSteps(0.1), baselineSteps);

    for (double rate : { -100.0, 100.0 })
    {
        SCOPED_TRACE(rate);
        TestableSnowMPMSolver<N> deforming{ resolution, unit };
        deforming.SetClosedDomainBoundaryFlag(DIRECTION_NONE);
        deforming.SetGravity({});

        AddLinearParticleLattice(&deforming, rate);

        auto volumes = deforming.GetMPMSystemData()->InitialVolumes();
        const auto masses = deforming.GetMPMSystemData()->ParticleMasses();

        for (size_t i = 0; i < volumes.Length(); ++i)
        {
            volumes[i] = masses[i] / 400.0;
        }

        deforming.Initialize();

        EXPECT_EQ(deforming.NumberOfSubTimeSteps(0.1), 56u);

        deforming.BeginStep(0.001);
        EXPECT_EQ(deforming.NumberOfSubTimeSteps(0.1), 56u);

        deforming.SetIsUsingSemiImplicit(true);
        EXPECT_EQ(deforming.NumberOfSubTimeSteps(0.1), 56u);
    }
}

template <size_t N>
void ExpectParametersAndBuilder()
{
    SnowMPMSolver<N> solver;
    EXPECT_FALSE(solver.GetIsUsingSemiImplicit());

    solver.SetIsUsingSemiImplicit(true);
    EXPECT_TRUE(solver.GetIsUsingSemiImplicit());
    EXPECT_EQ(solver.GetMaxNumberOfIterations(), 100u);

    solver.SetMaxNumberOfIterations(25);
    EXPECT_EQ(solver.GetMaxNumberOfIterations(), 25u);
    EXPECT_DOUBLE_EQ(solver.GetTolerance(), 1e-6);

    solver.SetTolerance(1e-8);
    EXPECT_DOUBLE_EQ(solver.GetTolerance(), 1e-8);
    EXPECT_THROW(solver.SetTolerance(0.0), std::invalid_argument);
    EXPECT_THROW(solver.SetTolerance(std::numeric_limits<double>::quiet_NaN()),
                 std::invalid_argument);
    EXPECT_EQ(solver.GetLastNumberOfIterations(), 0u);
    EXPECT_DOUBLE_EQ(solver.GetLastResidual(), 0.0);
    EXPECT_EQ(solver.GetClosedDomainBoundaryFlag(), DIRECTION_ALL);

    solver.SetClosedDomainBoundaryFlag(DIRECTION_LEFT | DIRECTION_UP);
    EXPECT_EQ(solver.GetClosedDomainBoundaryFlag(),
              DIRECTION_LEFT | DIRECTION_UP);
    EXPECT_DOUBLE_EQ(solver.GetTimeStepLimitScale(), 0.9);

    solver.SetTimeStepLimitScale(0.5);
    EXPECT_DOUBLE_EQ(solver.GetTimeStepLimitScale(), 0.5);
    EXPECT_THROW(solver.SetTimeStepLimitScale(0.0), std::invalid_argument);
    EXPECT_THROW(solver.SetTimeStepLimitScale(1.1), std::invalid_argument);
    EXPECT_THROW(
        solver.SetTimeStepLimitScale(std::numeric_limits<double>::quiet_NaN()),
        std::invalid_argument);

    const auto resolution = VectorUZ<N>::MakeConstant(7);
    const auto spacing = VectorD<N>::MakeConstant(0.25);
    const auto origin = VectorD<N>::MakeConstant(-1.0);
    auto built = SnowMPMSolver<N>::GetBuilder()
                     .WithResolution(resolution)
                     .WithGridSpacing(spacing)
                     .WithOrigin(origin)
                     .WithRadius(0.2)
                     .WithMass(3.0)
                     .Build();

    const auto data = built.GetMPMSystemData();
    EXPECT_EQ(data->GridMass().Resolution(), resolution);
    EXPECT_EQ(data->GridMass().GridSpacing(), spacing);
    EXPECT_EQ(data->GridMass().Origin(), origin);
    EXPECT_DOUBLE_EQ(data->Radius(), 0.2);
    EXPECT_DOUBLE_EQ(data->Mass(), 3.0);

    EXPECT_NE(SnowMPMSolver<N>::GetBuilder().MakeShared(), nullptr);
}

template <size_t N>
void ExpectClosedDomainWallStopsVelocity(bool semiImplicit)
{
    constexpr std::array lowerFlags{ DIRECTION_LEFT, DIRECTION_DOWN,
                                     DIRECTION_BACK };
    constexpr std::array upperFlags{ DIRECTION_RIGHT, DIRECTION_UP,
                                     DIRECTION_FRONT };
    const auto resolution = VectorUZ<N>::MakeConstant(4);
    const auto spacing = VectorD<N>::MakeConstant(1.0);

    for (size_t axis = 0; axis < N; ++axis)
    {
        for (bool isUpper : { false, true })
        {
            TestableSnowMPMSolver<N> solver{ resolution, spacing };
            solver.SetClosedDomainBoundaryFlag(isUpper ? upperFlags[axis]
                                                       : lowerFlags[axis]);
            solver.SetIsUsingSemiImplicit(semiImplicit);
            solver.SetGravity({});
            solver.SetDragCoefficient(0.0);

            VectorD<N> position = VectorD<N>::MakeConstant(2.0);
            VectorD<N> velocity;

            position[axis] = isUpper ? 3.75 : 0.25;
            velocity[axis] = isUpper ? 1.0 : -1.0;

            auto data = solver.GetMPMSystemData();
            data->AddParticle(position, velocity);

            solver.BeginStep(1e-3);

            VectorUZ<N> nodeIndex = VectorUZ<N>::MakeConstant(2);
            nodeIndex[axis] =
                isUpper ? data->GridMass().DataSize()[axis] - 1 : 0;

            ASSERT_GT(data->GridMass()(nodeIndex), 0.0);
            EXPECT_DOUBLE_EQ(data->GridVelocities()(nodeIndex)[axis], 0.0);
        }
    }
}

template <size_t N>
void ExpectUnconstrainedDomainWallPreservesVelocity(bool semiImplicit)
{
    const auto resolution = VectorUZ<N>::MakeConstant(4);
    const auto spacing = VectorD<N>::MakeConstant(1.0);

    for (int boundaryFlag : { DIRECTION_NONE, DIRECTION_LEFT })
    {
        TestableSnowMPMSolver<N> solver{ resolution, spacing };
        solver.SetClosedDomainBoundaryFlag(boundaryFlag);
        solver.SetIsUsingSemiImplicit(semiImplicit);
        solver.SetGravity({});
        solver.SetDragCoefficient(0.0);

        VectorD<N> position = VectorD<N>::MakeConstant(2.0);
        VectorD<N> velocity;

        position[0] = 0.25;
        velocity[0] = boundaryFlag == DIRECTION_NONE ? -1.0 : 1.0;

        auto data = solver.GetMPMSystemData();
        data->AddParticle(position, velocity);

        solver.BeginStep(1e-3);

        VectorUZ<N> nodeIndex = VectorUZ<N>::MakeConstant(2);
        nodeIndex[0] = 0;

        ASSERT_GT(data->GridMass()(nodeIndex), 0.0);
        EXPECT_DOUBLE_EQ(data->GridVelocities()(nodeIndex)[0], velocity[0]);
    }
}

template <size_t N>
void ExpectClosedDomainWalls(bool semiImplicit = false)
{
    ExpectClosedDomainWallStopsVelocity<N>(semiImplicit);
    ExpectUnconstrainedDomainWallPreservesVelocity<N>(semiImplicit);
}

template <size_t N>
void ExpectSemiImplicitClosedDomainWall()
{
    ExpectClosedDomainWalls<N>(true);
}

template <size_t N>
void ExpectMovingColliderAffectsGrid()
{
    TestableSnowMPMSolver<N> solver{ VectorUZ<N>::MakeConstant(4),
                                     VectorD<N>::MakeConstant(1.0) };
    solver.SetClosedDomainBoundaryFlag(DIRECTION_NONE);
    solver.SetGravity({});
    solver.SetDragCoefficient(0.0);

    VectorD<N> normal;
    normal[0] = 1.0;

    auto collider = std::make_shared<RigidBodyCollider<N>>(
        std::make_shared<Plane<N>>(normal, VectorD<N>{}));
    collider->linearVelocity[0] = 1.0;

    solver.SetCollider(collider);

    VectorD<N> position = VectorD<N>::MakeConstant(2.0);
    position[0] = 0.25;

    auto data = solver.GetMPMSystemData();
    data->AddParticle(position);

    solver.BeginStep(1e-3);

    VectorUZ<N> nodeIndex = VectorUZ<N>::MakeConstant(2);
    nodeIndex[0] = 0;

    ASSERT_GT(data->GridMass()(nodeIndex), 0.0);
    EXPECT_DOUBLE_EQ(data->GridVelocities()(nodeIndex)[0], 1.0);
}

template <size_t N>
void ExpectFrictionAffectsGrid()
{
    const auto runCase = [](double frictionCoefficient) {
        TestableSnowMPMSolver<N> solver{ VectorUZ<N>::MakeConstant(4),
                                         VectorD<N>::MakeConstant(1.0) };
        solver.SetClosedDomainBoundaryFlag(DIRECTION_NONE);
        solver.SetGravity({});
        solver.SetDragCoefficient(0.0);

        VectorD<N> normal;
        normal[1] = 1.0;

        VectorD<N> point;
        point[1] = 2.0;

        auto collider = std::make_shared<RigidBodyCollider<N>>(
            std::make_shared<Plane<N>>(normal, point));
        collider->SetFrictionCoefficient(frictionCoefficient);

        solver.SetCollider(collider);

        VectorD<N> position = VectorD<N>::MakeConstant(2.25);
        VectorD<N> velocity;
        velocity[0] = 1.0;
        velocity[1] = -1.0;

        auto data = solver.GetMPMSystemData();
        data->AddParticle(position, velocity);

        solver.BeginStep(1e-3);

        const VectorUZ<N> nodeIndex = VectorUZ<N>::MakeConstant(2);
        EXPECT_GT(data->GridMass()(nodeIndex), 0.0);

        return data->GridVelocities()(nodeIndex);
    };

    const auto frictionless = runCase(0.0);
    EXPECT_DOUBLE_EQ(frictionless[0], 1.0);
    EXPECT_DOUBLE_EQ(frictionless[1], 0.0);

    const auto frictional = runCase(1.0);
    EXPECT_DOUBLE_EQ(frictional[0], 0.0);
    EXPECT_DOUBLE_EQ(frictional[1], 0.0);
}

template <size_t N>
void ExpectParticleDomainProjection()
{
    const auto runCase = [](int boundaryFlag) {
        SnowMPMSolver<N> solver{ VectorUZ<N>::MakeConstant(4),
                                 VectorD<N>::MakeConstant(1.0) };
        UseOneFixedStep(&solver);
        solver.SetClosedDomainBoundaryFlag(boundaryFlag);
        solver.SetGravity({});
        solver.SetDragCoefficient(0.0);

        VectorD<N> position = VectorD<N>::MakeConstant(2.0);
        VectorD<N> velocity;

        position[0] = -0.01;
        velocity[0] = -1.0;

        auto data = solver.GetMPMSystemData();
        data->AddParticle(position, velocity);

        solver.Update(Frame{ 0, 1e-3 });

        for (size_t axis = 0; axis < N; ++axis)
        {
            EXPECT_TRUE(std::isfinite(data->Positions()[0][axis]));
            EXPECT_TRUE(std::isfinite(data->Velocities()[0][axis]));
        }

        return std::array{ data->Positions()[0][0], data->Velocities()[0][0] };
    };

    const auto closed = runCase(DIRECTION_LEFT);
    EXPECT_DOUBLE_EQ(closed[0], 0.0);
    EXPECT_DOUBLE_EQ(closed[1], 0.0);

    const auto open = runCase(DIRECTION_NONE);
    EXPECT_NEAR(open[0], -0.011, 1e-12);
    EXPECT_NEAR(open[1], -1.0, 1e-12);
}
}  // namespace

#define RUN_FOR_2D_AND_3D(function) \
    do                              \
    {                               \
        {                           \
            SCOPED_TRACE("N = 2");  \
            function<2>();          \
        }                           \
        {                           \
            SCOPED_TRACE("N = 3");  \
            function<3>();          \
        }                           \
    } while (false)

TEST(SnowMPMSolver, EmptyUpdate)
{
    RUN_FOR_2D_AND_3D(ExpectEmptyUpdate);
}

TEST(SnowMPMSolver, EmptySemiImplicitUpdate)
{
    RUN_FOR_2D_AND_3D(ExpectEmptySemiImplicitUpdate);
}

TEST(SnowMPMSolver, ZeroResidualSemiImplicitUpdate)
{
    RUN_FOR_2D_AND_3D(ExpectZeroResidualSemiImplicitUpdate);
}

TEST(SnowMPMSolver, SmallStepImplicitAgreement)
{
    RUN_FOR_2D_AND_3D(ExpectSmallStepMatchesExplicit);
}

TEST(SnowMPMSolver, SemiImplicitStiffStability)
{
    RUN_FOR_2D_AND_3D(ExpectSemiImplicitStiffStability);
}

TEST(SnowMPMSolver, SemiImplicitMassScaling)
{
    RUN_FOR_2D_AND_3D(ExpectSemiImplicitConvergesAcrossMassScales);
}

TEST(SnowMPMSolver, SemiImplicitFailureRollback)
{
    RUN_FOR_2D_AND_3D(ExpectSemiImplicitFailureRollback);
}

TEST(SnowMPMSolver, SemiImplicitClosedDomainWall)
{
    RUN_FOR_2D_AND_3D(ExpectSemiImplicitClosedDomainWall);
}

TEST(SnowMPMSolver, ReferenceVolume)
{
    RUN_FOR_2D_AND_3D(ExpectReferenceVolumeUsesCellVolume);
}

TEST(SnowMPMSolver, UniformMotion)
{
    RUN_FOR_2D_AND_3D(ExpectUniformMotion);
}

TEST(SnowMPMSolver, ExternalForces)
{
    RUN_FOR_2D_AND_3D(ExpectExternalForcesAppliedOnce);
}

TEST(SnowMPMSolver, LinearVelocityGradient)
{
    RUN_FOR_2D_AND_3D(ExpectLinearVelocityUpdatesDeformation);
}

TEST(SnowMPMSolver, AdaptiveRestrictions)
{
    RUN_FOR_2D_AND_3D(ExpectAdaptiveRestrictions);
}

TEST(SnowMPMSolver, ParametersAndBuilder)
{
    RUN_FOR_2D_AND_3D(ExpectParametersAndBuilder);
}

TEST(SnowMPMSolver, ClosedDomainWalls)
{
    RUN_FOR_2D_AND_3D(ExpectClosedDomainWalls);
}

TEST(SnowMPMSolver, MovingCollider)
{
    RUN_FOR_2D_AND_3D(ExpectMovingColliderAffectsGrid);
}

TEST(SnowMPMSolver, FrictionalCollider)
{
    RUN_FOR_2D_AND_3D(ExpectFrictionAffectsGrid);
}

TEST(SnowMPMSolver, ParticleDomainProjection)
{
    RUN_FOR_2D_AND_3D(ExpectParticleDomainProjection);
}

TEST(SnowMPMExample, PaperSnowBallSampling)
{
    const Vector3D center{ 0.5, 0.5, 0.5 };
    constexpr double radius = 0.2;
    const auto first = GeneratePaperSnowBall(center, radius, 0.05, 4, 7);
    const auto second = GeneratePaperSnowBall(center, radius, 0.05, 4, 7);

    ASSERT_GT(first.positions.Length(), 500u);
    ASSERT_EQ(first.positions.Length(), second.positions.Length());
    ASSERT_EQ(first.positions.Length(), first.massScales.Length());
    ASSERT_EQ(first.positions.Length(), first.hardeningScales.Length());

    bool foundOuterParticle = false;
    for (size_t i = 0; i < first.positions.Length(); ++i)
    {
        EXPECT_EQ(first.positions[i], second.positions[i]);
        EXPECT_LE(first.positions[i].DistanceTo(center), radius);

        if (first.positions[i].DistanceTo(center) >= 0.75 * radius)
        {
            foundOuterParticle = true;
            EXPECT_DOUBLE_EQ(first.massScales[i], 1.5);
            EXPECT_GT(first.hardeningScales[i], 1.0);
        }
    }

    EXPECT_TRUE(foundOuterParticle);
}

TEST(SnowMPMExample, ConfiguresPaperSemiImplicitSteps)
{
    SnowMPMSolver3 automatic;
    ConfigurePaperSemiImplicit(automatic, 0);

    EXPECT_TRUE(automatic.GetIsUsingSemiImplicit());
    EXPECT_FALSE(automatic.GetIsUsingFixedSubTimeSteps());
    EXPECT_EQ(automatic.GetMaxNumberOfIterations(), 200u);
    EXPECT_DOUBLE_EQ(automatic.GetTolerance(), 1e-3);

    SnowMPMSolver3 overridden;
    ConfigurePaperSemiImplicit(overridden, 8);
    EXPECT_TRUE(overridden.GetIsUsingFixedSubTimeSteps());
    EXPECT_EQ(overridden.GetNumberOfFixedSubTimeSteps(), 8u);
}
