#include "gtest/gtest.h"

#include <Core/Solver/Particle/MPM/SnowMPMSolver.hpp>
#include <Core/Utils/IterationUtils.hpp>

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

    ForEachIndex(dataSize, [&](auto... rawIndices) {
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

    TestableSnowMPMSolver<N> softerDensity{ resolution, unit };
    softerDensity.SetGravity({});
    softerDensity.GetMPMSystemData()->AddParticle(position);
    softerDensity.GetMPMSystemData()->InitialVolumes()[0] =
        softerDensity.GetMPMSystemData()->ParticleMasses()[0] / 100.0;
    softerDensity.Initialize();
    EXPECT_GT(softerDensity.NumberOfSubTimeSteps(0.1), baselineSteps);

    TestableSnowMPMSolver<N> deforming{ resolution, unit };
    deforming.SetGravity({});
    AddLinearParticleLattice(&deforming, 100.0);
    auto volumes = deforming.GetMPMSystemData()->InitialVolumes();
    const auto masses = deforming.GetMPMSystemData()->ParticleMasses();
    for (size_t i = 0; i < volumes.Length(); ++i)
    {
        volumes[i] = masses[i] / 400.0;
    }
    deforming.Initialize();
    EXPECT_EQ(deforming.NumberOfSubTimeSteps(0.1), 56u);
}

template <size_t N>
void ExpectParametersAndBuilder()
{
    SnowMPMSolver<N> solver;
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
