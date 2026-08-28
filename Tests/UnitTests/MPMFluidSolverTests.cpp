// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include "gtest/gtest.h"

#include <Core/Emitter/PointParticleEmitter2.hpp>
#include <Core/Emitter/PointParticleEmitter3.hpp>
#include <Core/Solver/Particle/MPM/MPMFluidSolver.hpp>
#include <Core/Utils/Constants.hpp>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <type_traits>

using namespace CubbyFlow;

namespace
{
template <size_t N>
using VectorD = Vector<double, N>;

template <size_t N>
using VectorUZ = Vector<size_t, N>;

template <size_t N>
using PointEmitter =
    std::conditional_t<N == 2, PointParticleEmitter2, PointParticleEmitter3>;

template <size_t N>
class TestableMPMFluidSolver final : public MPMFluidSolver<N>
{
 public:
    using MPMFluidSolver<N>::MPMFluidSolver;

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
void UseOneFixedStep(MPMFluidSolver<N>* solver)
{
    solver->SetIsUsingFixedSubTimeSteps(true);
    solver->SetNumberOfFixedSubTimeSteps(1);
}

template <size_t N>
void ExpectParameters()
{
    MPMFluidSolver<N> solver;
    EXPECT_DOUBLE_EQ(solver.GetTimeStepLimitScale(), 0.9);
    EXPECT_DOUBLE_EQ(solver.GetConstitutiveModel().GetTargetDensity(),
                     WATER_DENSITY);
    EXPECT_DOUBLE_EQ(solver.GetConstitutiveModel().GetSpeedOfSound(), 100.0);

    solver.SetTimeStepLimitScale(0.5);
    EXPECT_DOUBLE_EQ(solver.GetTimeStepLimitScale(), 0.5);
    EXPECT_THROW(solver.SetTimeStepLimitScale(0.0), std::invalid_argument);
    EXPECT_THROW(solver.SetTimeStepLimitScale(1.1), std::invalid_argument);
    EXPECT_THROW(
        solver.SetTimeStepLimitScale(std::numeric_limits<double>::quiet_NaN()),
        std::invalid_argument);
    EXPECT_THROW((MPMFluidSolver<N>{ VectorUZ<N>::MakeConstant(4),
                                     VectorD<N>::MakeConstant(1.0),
                                     {},
                                     -1.0,
                                     1.0 }),
                 std::invalid_argument);
    EXPECT_THROW((MPMFluidSolver<N>{ VectorUZ<N>::MakeConstant(4),
                                     VectorD<N>::MakeConstant(1.0),
                                     {},
                                     0.1,
                                     0.0 }),
                 std::invalid_argument);
}

template <size_t N>
void ExpectBuilder()
{
    const auto resolution = VectorUZ<N>::MakeConstant(7);
    const auto spacing = VectorD<N>::MakeConstant(0.25);
    const auto origin = VectorD<N>::MakeConstant(-1.0);
    auto built = MPMFluidSolver<N>::GetBuilder()
                     .WithResolution(resolution)
                     .WithGridSpacing(spacing)
                     .WithOrigin(origin)
                     .WithRadius(0.2)
                     .WithMass(3.0)
                     .WithTargetDensity(900.0)
                     .WithSpeedOfSound(20.0)
                     .WithEosExponent(5.0)
                     .WithNegativePressureScale(0.25)
                     .Build();

    const auto data = built.GetMPMSystemData();
    EXPECT_EQ(data->GridMass().Resolution(), resolution);
    EXPECT_EQ(data->GridMass().GridSpacing(), spacing);
    EXPECT_EQ(data->GridMass().Origin(), origin);
    EXPECT_DOUBLE_EQ(data->Radius(), 0.2);
    EXPECT_DOUBLE_EQ(data->Mass(), 3.0);
    EXPECT_DOUBLE_EQ(built.GetConstitutiveModel().GetTargetDensity(), 900.0);
    EXPECT_DOUBLE_EQ(built.GetConstitutiveModel().GetSpeedOfSound(), 20.0);
    EXPECT_DOUBLE_EQ(built.GetConstitutiveModel().GetEosExponent(), 5.0);
    EXPECT_DOUBLE_EQ(built.GetConstitutiveModel().GetNegativePressureScale(),
                     0.25);
    EXPECT_NE(MPMFluidSolver<N>::GetBuilder().MakeShared(), nullptr);
}

template <size_t N>
void ExpectEmitter()
{
    const auto resolution = VectorUZ<N>::MakeConstant(8);
    const auto spacing = VectorD<N>::MakeConstant(0.1);
    const auto position = spacing;
    VectorD<N> direction;
    direction[0] = 1.0;

    MPMFluidSolver<N> solver{ resolution, spacing };
    solver.SetGravity({});
    solver.SetDragCoefficient(0.0);
    auto emitter =
        std::make_shared<PointEmitter<N>>(position, direction, 0.0, 0.0, 1, 1);
    solver.SetEmitter(emitter);

    solver.Update(Frame{ 0, 0.001 });

    const auto data = solver.GetMPMSystemData();
    EXPECT_EQ(data->NumberOfParticles(), 1u);
    EXPECT_DOUBLE_EQ(data->InitialVolumes()[0], 1e-6);
}

template <size_t N>
void ExpectDefensiveChecks()
{
    const auto resolution = VectorUZ<N>::MakeConstant(8);
    const auto spacing = VectorD<N>::MakeConstant(1.0);
    const auto position = VectorD<N>::MakeConstant(3.5);

    TestableMPMFluidSolver<N> invalidVelocity{ resolution, spacing };
    auto invalidVelocityData = invalidVelocity.GetMPMSystemData();
    invalidVelocityData->AddParticle(position);
    invalidVelocity.Initialize();
    invalidVelocityData->Velocities()[0][0] =
        std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(static_cast<void>(invalidVelocity.NumberOfSubTimeSteps(0.1)),
                 std::invalid_argument);

    TestableMPMFluidSolver<N> invalidIncrement{ resolution, spacing };
    invalidIncrement.GetMPMSystemData()->AddParticle(position);
    invalidIncrement.Initialize();
    EXPECT_THROW(invalidIncrement.BeginStep(std::numeric_limits<double>::max()),
                 std::invalid_argument);
}

template <size_t N>
void ExpectReferenceVolumesAndAdaptiveSteps()
{
    const auto resolution = VectorUZ<N>::MakeConstant(8);
    const auto spacing = VectorD<N>::MakeConstant(0.1);

    TestableMPMFluidSolver<N> empty{ resolution, spacing };
    EXPECT_NO_THROW(empty.Initialize());
    EXPECT_EQ(empty.NumberOfSubTimeSteps(0.1), 1u);
    EXPECT_NO_THROW(empty.Update(Frame{ 0, 0.001 }));

    auto emittedData = empty.GetMPMSystemData();
    emittedData->AddParticle(VectorD<N>::MakeConstant(0.35));
    EXPECT_NO_THROW(empty.Update(Frame{ 1, 0.001 }));
    EXPECT_DOUBLE_EQ(emittedData->InitialVolumes()[0], 1e-6);

    TestableMPMFluidSolver<N> solver{ resolution, spacing, {},  0.01,
                                      2.0,        1000.0,  10.0 };
    const VectorD<N> position = VectorD<N>::MakeConstant(0.35);

    VectorD<N> velocity;
    velocity[0] = 3.0;

    auto data = solver.GetMPMSystemData();
    data->AddParticle(position, velocity);

    solver.Initialize();
    EXPECT_DOUBLE_EQ(data->InitialVolumes()[0], 0.002);
    EXPECT_EQ(solver.NumberOfSubTimeSteps(0.1), 15u);

    solver.SetTimeStepLimitScale(0.5);
    EXPECT_EQ(solver.NumberOfSubTimeSteps(0.1), 26u);

    data->InitialVolumes()[0] = 0.003;
    solver.BeginStep(0.001);
    EXPECT_DOUBLE_EQ(data->InitialVolumes()[0], 0.003);

    TestableMPMFluidSolver<N> fasterSound{ resolution, spacing, {},  0.01,
                                           2.0,        1000.0,  20.0 };
    fasterSound.GetMPMSystemData()->AddParticle(position, velocity);
    fasterSound.Initialize();
    EXPECT_EQ(fasterSound.NumberOfSubTimeSteps(0.1), 26u);

    TestableMPMFluidSolver<N> finerSpacing{
        resolution, VectorD<N>::MakeConstant(0.05), {}, 0.01, 2.0, 1000.0, 10.0
    };
    finerSpacing.GetMPMSystemData()->AddParticle(VectorD<N>::MakeConstant(0.2),
                                                 velocity);
    finerSpacing.Initialize();
    EXPECT_EQ(finerSpacing.NumberOfSubTimeSteps(0.1), 29u);

    TestableMPMFluidSolver<N> invalid{ resolution, spacing };
    invalid.GetMPMSystemData()->AddParticle(position);
    invalid.GetMPMSystemData()->InitialVolumes()[0] = -1.0;
    EXPECT_THROW(invalid.Initialize(), std::invalid_argument);
}

template <size_t N>
void ExpectUniformMotionAndExternalForces()
{
    const auto resolution = VectorUZ<N>::MakeConstant(8);
    const auto spacing = VectorD<N>::MakeConstant(1.0);
    const auto position = VectorD<N>::MakeConstant(3.5);

    MPMFluidSolver<N> uniform{ resolution, spacing };
    UseOneFixedStep(&uniform);
    uniform.SetGravity({});
    uniform.SetDragCoefficient(0.0);

    VectorD<N> velocity;
    velocity[0] = 0.2;

    auto uniformData = uniform.GetMPMSystemData();
    uniformData->AddParticle(position, velocity);
    uniform.Update(Frame{ 0, 0.001 });

    EXPECT_TRUE(uniformData->Velocities()[0].IsSimilar(velocity, 1e-12));
    EXPECT_TRUE(uniformData->Positions()[0].IsSimilar(
        position + 0.001 * velocity, 1e-12));

    MPMFluidSolver<N> gravitySolver{ resolution, spacing };
    UseOneFixedStep(&gravitySolver);
    gravitySolver.SetDragCoefficient(0.0);

    VectorD<N> gravity;
    gravity[1] = -2.0;
    gravitySolver.SetGravity(gravity);

    auto gravityData = gravitySolver.GetMPMSystemData();
    gravityData->AddParticle(position);
    gravitySolver.Update(Frame{ 0, 0.001 });

    EXPECT_TRUE(gravityData->Velocities()[0].IsSimilar(0.001 * gravity, 1e-12));

    MPMFluidSolver<N> dragSolver{ resolution, spacing, {}, 0.1, 2.0 };
    UseOneFixedStep(&dragSolver);
    dragSolver.SetGravity({});
    dragSolver.SetDragCoefficient(4.0);

    velocity = {};
    velocity[0] = 1.0;

    auto dragData = dragSolver.GetMPMSystemData();
    dragData->AddParticle(position, velocity);
    dragSolver.Update(Frame{ 0, 0.001 });

    velocity[0] = 0.998;
    EXPECT_TRUE(dragData->Velocities()[0].IsSimilar(velocity, 1e-12));
}

template <size_t N>
void ExpectCompressedParticlesMoveOutwardAndStayFinite()
{
    const auto resolution = VectorUZ<N>::MakeConstant(8);
    const auto spacing = VectorD<N>::MakeConstant(1.0);
    MPMFluidSolver<N> solver{ resolution, spacing, {}, 0.1, 1.0, 1000.0, 10.0 };

    UseOneFixedStep(&solver);
    solver.SetGravity({});
    solver.SetDragCoefficient(0.0);

    VectorD<N> left = VectorD<N>::MakeConstant(3.5);
    VectorD<N> right = left;
    left[0] = 3.25;
    right[0] = 3.75;

    auto data = solver.GetMPMSystemData();
    data->AddParticle(left);
    data->AddParticle(right);
    data->InitialVolumes()[0] = 0.001;
    data->InitialVolumes()[1] = 0.001;
    data->VolumeRatios()[0] = 0.5;
    data->VolumeRatios()[1] = 0.5;

    for (int frame = 0; frame < 4; ++frame)
    {
        solver.Update(Frame{ frame, 1e-5 });
    }

    EXPECT_LT(data->Velocities()[0][0], 0.0);
    EXPECT_GT(data->Velocities()[1][0], 0.0);

    for (size_t i = 0; i < data->NumberOfParticles(); ++i)
    {
        EXPECT_GT(data->VolumeRatios()[i], 0.0);
        EXPECT_TRUE(std::isfinite(data->VolumeRatios()[i]));

        for (double component : data->Positions()[i])
        {
            EXPECT_TRUE(std::isfinite(component));
        }
        for (double component : data->Velocities()[i])
        {
            EXPECT_TRUE(std::isfinite(component));
        }
    }
}
}  // namespace

TEST(MPMFluidSolver, ParametersAndBuilder)
{
    ExpectParameters<2>();
    ExpectParameters<3>();
    ExpectBuilder<2>();
    ExpectBuilder<3>();
}

TEST(MPMFluidSolver, Emitter)
{
    ExpectEmitter<2>();
    ExpectEmitter<3>();
}

TEST(MPMFluidSolver, DefensiveChecks)
{
    ExpectDefensiveChecks<2>();
    ExpectDefensiveChecks<3>();
}

TEST(MPMFluidSolver, ReferenceVolumesAndAdaptiveSteps)
{
    ExpectReferenceVolumesAndAdaptiveSteps<2>();
    ExpectReferenceVolumesAndAdaptiveSteps<3>();
}

TEST(MPMFluidSolver, UniformMotionAndExternalForces)
{
    ExpectUniformMotionAndExternalForces<2>();
    ExpectUniformMotionAndExternalForces<3>();
}

TEST(MPMFluidSolver, CompressedParticlesMoveOutwardAndStayFinite)
{
    ExpectCompressedParticlesMoveOutwardAndStayFinite<2>();
    ExpectCompressedParticlesMoveOutwardAndStayFinite<3>();
}
