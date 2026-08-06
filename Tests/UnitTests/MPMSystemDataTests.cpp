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

#include <Core/Particle/MPM/MPMSystemData.hpp>

#include <limits>

using namespace CubbyFlow;

namespace
{
template <size_t N>
void ExpectParticleStateResizes()
{
    MPMSystemData<N> data{ Vector<size_t, N>::MakeConstant(4),
                           Vector<double, N>::MakeConstant(1.0),
                           {},
                           1 };

    data.ParticleMasses()[0] = 1.25;
    data.InitialVolumes()[0] = 0.5;
    data.SetMass(2.5);
    data.AddParticle(Vector<double, N>::MakeConstant(0.25));
    ASSERT_EQ(data.NumberOfParticles(), 2u);
    EXPECT_DOUBLE_EQ(data.ParticleMasses()[0], 1.25);
    EXPECT_DOUBLE_EQ(data.ParticleMasses()[1], 2.5);
    EXPECT_DOUBLE_EQ(data.InitialVolumes()[0], 0.5);
    EXPECT_DOUBLE_EQ(data.InitialVolumes()[1], 0.0);

    const auto identity = Matrix<double, N, N>::MakeIdentity();
    EXPECT_TRUE(data.DeformationStates()[0].elastic.IsSimilar(identity));
    EXPECT_TRUE(data.DeformationStates()[1].elastic.IsSimilar(identity));
    EXPECT_TRUE(data.DeformationStates()[0].plastic.IsSimilar(identity));
    EXPECT_TRUE(data.DeformationStates()[1].plastic.IsSimilar(identity));
}

template <size_t N>
void ExpectBaseSetResizesMPMState()
{
    MPMSystemData<N> data{ Vector<size_t, N>::MakeConstant(2),
                           Vector<double, N>::MakeConstant(1.0),
                           {},
                           1 };
    ParticleSystemData<N> source{ 3 };

    data.Set(source);

    EXPECT_EQ(data.NumberOfParticles(), 3u);
    EXPECT_EQ(data.ParticleMasses().Length(), 3u);
    EXPECT_EQ(data.InitialVolumes().Length(), 3u);
    EXPECT_EQ(data.DeformationStates().Length(), 3u);
}

template <size_t N>
void ExpectBaseDeserializeResizesMPMState()
{
    ParticleSystemData<N> source{ 3 };
    std::vector<uint8_t> buffer;
    source.Serialize(&buffer);

    MPMSystemData<N> data{ Vector<size_t, N>::MakeConstant(2),
                           Vector<double, N>::MakeConstant(1.0),
                           {},
                           1 };
    data.Deserialize(buffer);

    EXPECT_EQ(data.NumberOfParticles(), 3u);
    EXPECT_EQ(data.ParticleMasses().Length(), 3u);
    EXPECT_EQ(data.InitialVolumes().Length(), 3u);
    EXPECT_EQ(data.DeformationStates().Length(), 3u);
}

template <size_t N>
void ExpectGridStateResizes()
{
    MPMSystemData<N> data;
    const auto resolution = Vector<size_t, N>::MakeConstant(3);
    const auto spacing = Vector<double, N>::MakeConstant(0.5);
    const auto origin = Vector<double, N>::MakeConstant(-1.0);

    data.ResizeGrid(resolution, spacing, origin);

    for (size_t axis = 0; axis < N; ++axis)
    {
        EXPECT_EQ(data.GridMass().Resolution()[axis], resolution[axis]);
        EXPECT_EQ(data.GridVelocities().Resolution()[axis], resolution[axis]);
        EXPECT_EQ(data.GridVelocitiesBeforeUpdate().Resolution()[axis],
                  resolution[axis]);
        EXPECT_DOUBLE_EQ(data.GridMass().GridSpacing()[axis], spacing[axis]);
        EXPECT_DOUBLE_EQ(data.GridMass().Origin()[axis], origin[axis]);
    }
}

template <size_t N>
void ExpectStencilPartitionAndGradient()
{
    const Vector<double, N> spacing = Vector<double, N>::MakeConstant(0.5);
    const Vector<double, N> position = Vector<double, N>::MakeConstant(1.125);
    const auto stencil = CubicBSplineKernel<N>::GetStencil(position, spacing,
                                                           Vector<double, N>{});

    double weightSum = 0.0;
    Vector<double, N> gradientSum;

    for (const auto& entry : stencil)
    {
        weightSum += entry.weight;
        gradientSum += entry.gradient;
    }

    EXPECT_NEAR(weightSum, 1.0, 1e-12);
    EXPECT_NEAR(gradientSum.Length(), 0.0, 1e-12);

    constexpr double epsilon = 1e-6;
    auto shifted = position;

    shifted[0] += epsilon;

    const auto shiftedStencil = CubicBSplineKernel<N>::GetStencil(
        shifted, spacing, Vector<double, N>{});

    for (size_t i = 0; i < stencil.size(); ++i)
    {
        EXPECT_NEAR((shiftedStencil[i].weight - stencil[i].weight) / epsilon,
                    stencil[i].gradient[0], 1e-5);
    }
}

template <size_t N>
void ExpectStencilRejectsUnrepresentableCoordinates()
{
    const auto zero = Vector<double, N>{};
    const auto unit = Vector<double, N>::MakeConstant(1.0);
    const auto largest =
        Vector<double, N>::MakeConstant(std::numeric_limits<double>::max());
    const auto lowest =
        Vector<double, N>::MakeConstant(std::numeric_limits<double>::lowest());

    EXPECT_THROW((void)CubicBSplineKernel<N>::GetStencil(unit, zero, {}),
                 std::invalid_argument);
    EXPECT_THROW((void)CubicBSplineKernel<N>::GetStencil(largest, unit, {}),
                 std::invalid_argument);
    EXPECT_THROW((void)CubicBSplineKernel<N>::GetStencil(largest, unit, lowest),
                 std::invalid_argument);
}

template <size_t N>
void ExpectParticleToGridConservation(const Vector<double, N>& firstPosition)
{
    MPMSystemData<N> data{ Vector<size_t, N>::MakeConstant(4),
                           Vector<double, N>::MakeConstant(1.0),
                           {},
                           2 };
    data.Positions()[0] = firstPosition;
    data.Positions()[1] = Vector<double, N>::MakeConstant(2.25);
    data.Velocities()[0] = Vector<double, N>::MakeConstant(2.0);
    data.Velocities()[1] = Vector<double, N>::MakeConstant(-1.0);
    data.ParticleMasses()[0] = 2.0;
    data.ParticleMasses()[1] = 3.0;

    data.TransferFromParticlesToGrid();

    double gridMass = 0.0;
    Vector<double, N> gridMomentum;

    data.GridMass().ForEachDataPointIndex(
        [&data, &gridMass, &gridMomentum](const Vector<size_t, N>& index) {
            const double mass = data.GridMass()(index);
            gridMass += mass;
            gridMomentum += mass * data.GridVelocities()(index);
            EXPECT_TRUE(data.GridVelocities()(index).IsSimilar(
                data.GridVelocitiesBeforeUpdate()(index), 1e-12));
        });

    EXPECT_NEAR(gridMass, 5.0, 1e-11);
    EXPECT_TRUE(
        gridMomentum.IsSimilar(Vector<double, N>::MakeConstant(1.0), 1e-11));
}

template <size_t N>
void ExpectGridToParticleBlend(double factor, double expected,
                               const Vector<double, N>& position,
                               bool useDefault = false)
{
    MPMSystemData<N> data{ Vector<size_t, N>::MakeConstant(4),
                           Vector<double, N>::MakeConstant(1.0),
                           {},
                           1 };
    data.Positions()[0] = position;
    data.Velocities()[0] = Vector<double, N>::MakeConstant(10.0);

    data.GridVelocitiesBeforeUpdate().Fill(Vector<double, N>::MakeConstant(1.0),
                                           ExecutionPolicy::Serial);
    data.GridVelocities().Fill(Vector<double, N>::MakeConstant(3.0),
                               ExecutionPolicy::Serial);

    if (!useDefault)
    {
        data.SetFLIPBlendingFactor(factor);
    }

    data.TransferFromGridToParticles();

    EXPECT_TRUE(data.Velocities()[0].IsSimilar(
        Vector<double, N>::MakeConstant(expected), 1e-12));
}

template <size_t N>
void ExpectRejectsDivergentGridState()
{
    MPMSystemData<N> data{ Vector<size_t, N>::MakeConstant(4) };
    data.GridMass().Clear();
    EXPECT_THROW(data.TransferFromParticlesToGrid(), std::invalid_argument);

    data.ResizeGrid(Vector<size_t, N>::MakeConstant(4),
                    Vector<double, N>::MakeConstant(1.0), {});
    data.GridVelocitiesBeforeUpdate().Resize(
        Vector<size_t, N>::MakeConstant(2),
        Vector<double, N>::MakeConstant(1.0), {});
    EXPECT_THROW(data.TransferFromGridToParticles(), std::invalid_argument);
}

template <size_t N>
void ExpectInvalidGridLeavesParticleVelocitiesUnchanged()
{
    MPMSystemData<N> data{ Vector<size_t, N>::MakeConstant(8),
                           Vector<double, N>::MakeConstant(1.0),
                           {},
                           2 };
    data.Positions()[0] = Vector<double, N>::MakeConstant(1.0);
    data.Positions()[1] = Vector<double, N>::MakeConstant(7.0);
    data.Velocities()[0] = Vector<double, N>::MakeConstant(10.0);
    data.Velocities()[1] = Vector<double, N>::MakeConstant(20.0);
    data.GridVelocitiesBeforeUpdate().Fill(Vector<double, N>::MakeConstant(1.0),
                                           ExecutionPolicy::Serial);
    data.GridVelocities().Fill(Vector<double, N>::MakeConstant(2.0),
                               ExecutionPolicy::Serial);

    auto invalidVelocity = Vector<double, N>{};
    invalidVelocity[0] = std::numeric_limits<double>::infinity();
    data.GridVelocities()(Vector<size_t, N>::MakeConstant(7)) = invalidVelocity;

    EXPECT_THROW(data.TransferFromGridToParticles(), std::invalid_argument);
    EXPECT_TRUE(data.Velocities()[0].IsSimilar(
        Vector<double, N>::MakeConstant(10.0), 1e-12));
    EXPECT_TRUE(data.Velocities()[1].IsSimilar(
        Vector<double, N>::MakeConstant(20.0), 1e-12));
}

template <size_t N>
void ExpectRejectsInvalidInput()
{
    MPMSystemData<N> data{ Vector<size_t, N>::MakeConstant(2) };
    const Vector<size_t, N> zeroResolution{};
    const auto unitSpacing = Vector<double, N>::MakeConstant(1.0);
    auto negativeSpacing = unitSpacing;
    negativeSpacing[0] = -1.0;
    const auto overflowingResolution =
        Vector<size_t, N>::MakeConstant(std::numeric_limits<size_t>::max());

    EXPECT_THROW(data.ResizeGrid(zeroResolution, unitSpacing, {}),
                 std::invalid_argument);
    EXPECT_THROW(data.ResizeGrid(Vector<size_t, N>::MakeConstant(2),
                                 negativeSpacing, {}),
                 std::invalid_argument);
    EXPECT_THROW(data.ResizeGrid(overflowingResolution, unitSpacing, {}),
                 std::invalid_argument);
    EXPECT_THROW(data.SetFLIPBlendingFactor(-0.1), std::invalid_argument);
    EXPECT_THROW(data.SetFLIPBlendingFactor(1.1), std::invalid_argument);
    EXPECT_THROW(
        data.SetFLIPBlendingFactor(std::numeric_limits<double>::quiet_NaN()),
        std::invalid_argument);

    data.Resize(1);
    data.ParticleMasses()[0] = 0.0;
    EXPECT_THROW(data.TransferFromParticlesToGrid(), std::invalid_argument);
    data.ParticleMasses()[0] = 1.0;
    data.Positions()[0][0] = std::numeric_limits<double>::infinity();
    EXPECT_THROW(data.TransferFromParticlesToGrid(), std::invalid_argument);
    EXPECT_THROW(data.TransferFromGridToParticles(), std::invalid_argument);

    data.Positions()[0] = Vector<double, N>{};
    data.Velocities()[0] = Vector<double, N>{};
    auto invalidGridVelocity = Vector<double, N>{};
    invalidGridVelocity[0] = std::numeric_limits<double>::infinity();
    data.GridVelocities().Fill(invalidGridVelocity, ExecutionPolicy::Serial);
    EXPECT_THROW(data.TransferFromGridToParticles(), std::invalid_argument);
}
}  // namespace

TEST(CubicBSplineKernel, Values)
{
    EXPECT_NEAR(CubicBSplineKernel<2>::Weight(0.0), 2.0 / 3.0, 1e-12);
    EXPECT_NEAR(CubicBSplineKernel<2>::Weight(1.0), 1.0 / 6.0, 1e-12);
    EXPECT_DOUBLE_EQ(CubicBSplineKernel<2>::Weight(2.0), 0.0);
    EXPECT_NEAR(CubicBSplineKernel<2>::Gradient(0.5), -0.625, 1e-12);
    EXPECT_NEAR(CubicBSplineKernel<2>::Gradient(-0.5), 0.625, 1e-12);
}

TEST(CubicBSplineKernel, Stencil)
{
    ExpectStencilPartitionAndGradient<2>();
    ExpectStencilPartitionAndGradient<3>();
}

TEST(CubicBSplineKernel, RejectsUnrepresentableCoordinates)
{
    ExpectStencilRejectsUnrepresentableCoordinates<2>();
    ExpectStencilRejectsUnrepresentableCoordinates<3>();
}

TEST(MPMSystemData, ParticleStateResizes)
{
    ExpectParticleStateResizes<2>();
    ExpectParticleStateResizes<3>();
}

TEST(MPMSystemData, BaseSetResizesMPMState)
{
    ExpectBaseSetResizesMPMState<2>();
    ExpectBaseSetResizesMPMState<3>();
}

TEST(MPMSystemData, BaseDeserializeResizesMPMState)
{
    ExpectBaseDeserializeResizesMPMState<2>();
    ExpectBaseDeserializeResizesMPMState<3>();
}

TEST(MPMSystemData, GridStateResizes)
{
    ExpectGridStateResizes<2>();
    ExpectGridStateResizes<3>();
}

TEST(MPMSystemData, ParticleToGridConservesMassAndMomentum)
{
    ExpectParticleToGridConservation<2>(Vector2D::MakeConstant(1.25));
    ExpectParticleToGridConservation<3>(Vector3D::MakeConstant(1.25));
    ExpectParticleToGridConservation<2>(Vector2D{});
    ExpectParticleToGridConservation<3>(Vector3D{});
}

TEST(MPMSystemData, GridToParticleBlendsPICAndFLIP)
{
    ExpectGridToParticleBlend<2>(0.0, 3.0, Vector2D::MakeConstant(1.25));
    ExpectGridToParticleBlend<3>(0.0, 3.0, Vector3D::MakeConstant(1.25));
    ExpectGridToParticleBlend<2>(1.0, 12.0, Vector2D{});
    ExpectGridToParticleBlend<3>(1.0, 12.0, Vector3D{});
    ExpectGridToParticleBlend<2>(0.95, 11.55, Vector2D::MakeConstant(1.25),
                                 true);
    ExpectGridToParticleBlend<3>(0.95, 11.55, Vector3D::MakeConstant(1.25),
                                 true);
}

TEST(MPMSystemData, EmptyTransfersAreSafe)
{
    MPMSystemData2 data2;
    MPMSystemData3 data3;

    EXPECT_NO_THROW(data2.TransferFromParticlesToGrid());
    EXPECT_NO_THROW(data2.TransferFromGridToParticles());
    EXPECT_NO_THROW(data3.TransferFromParticlesToGrid());
    EXPECT_NO_THROW(data3.TransferFromGridToParticles());
}

TEST(MPMSystemData, RejectsDivergentGridState)
{
    ExpectRejectsDivergentGridState<2>();
    ExpectRejectsDivergentGridState<3>();
}

TEST(MPMSystemData, InvalidGridLeavesParticleVelocitiesUnchanged)
{
    ExpectInvalidGridLeavesParticleVelocitiesUnchanged<2>();
    ExpectInvalidGridLeavesParticleVelocitiesUnchanged<3>();
}

TEST(MPMSystemData, RejectsInvalidInput)
{
    ExpectRejectsInvalidInput<2>();
    ExpectRejectsInvalidInput<3>();
}
