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

#include <Core/Particle/MPM/MPMFluidSystemData.hpp>

#include <limits>
#include <type_traits>

using namespace CubbyFlow;

namespace
{
template <typename T>
concept HasDeformationStates = requires(T& data) { data.DeformationStates(); };

template <typename T>
concept HasNoArgumentGridToParticleTransfer =
    requires(T& data) { data.TransferFromGridToParticles(); };

static_assert(!HasDeformationStates<MPMFluidSystemData2>);
static_assert(!HasDeformationStates<MPMFluidSystemData3>);
static_assert(!HasNoArgumentGridToParticleTransfer<MPMTransferSystemData<2>>);
static_assert(!HasNoArgumentGridToParticleTransfer<MPMTransferSystemData<3>>);
static_assert(HasNoArgumentGridToParticleTransfer<MPMSystemData2>);
static_assert(HasNoArgumentGridToParticleTransfer<MPMSystemData3>);
static_assert(std::is_same_v<MPMSystemData2::Base, ParticleSystemData2>);
static_assert(std::is_same_v<MPMSystemData3::Base, ParticleSystemData3>);

template <size_t N>
void ExpectParticleStateResizes()
{
    MPMFluidSystemData<N> data{ Vector<size_t, N>::MakeConstant(4),
                                Vector<double, N>::MakeConstant(1.0),
                                {},
                                1 };

    EXPECT_DOUBLE_EQ(data.VolumeRatios()[0], 1.0);
    EXPECT_TRUE(data.VelocityGradients()[0].IsSimilar(Matrix<double, N, N>{}));

    data.VolumeRatios()[0] = 2.0;
    data.VelocityGradients()[0](0, 0) = 3.0;
    data.AddParticle(Vector<double, N>::MakeConstant(0.25));

    ASSERT_EQ(data.VolumeRatios().Length(), 2u);
    ASSERT_EQ(data.VelocityGradients().Length(), 2u);
    EXPECT_DOUBLE_EQ(data.VolumeRatios()[0], 2.0);
    EXPECT_DOUBLE_EQ(data.VolumeRatios()[1], 1.0);
    EXPECT_DOUBLE_EQ(data.VelocityGradients()[0](0, 0), 3.0);
    EXPECT_TRUE(data.VelocityGradients()[1].IsSimilar(Matrix<double, N, N>{}));
}

template <size_t N>
void DirtyFluidState(MPMFluidSystemData<N>* data)
{
    data->VolumeRatios()[0] = 2.0;
    data->VelocityGradients()[0](0, 0) = 3.0;
}

template <size_t N>
void ExpectFluidStateReset(const MPMFluidSystemData<N>& data)
{
    ASSERT_EQ(data.VolumeRatios().Length(), 3u);
    ASSERT_EQ(data.VelocityGradients().Length(), 3u);

    for (size_t i = 0; i < 3; ++i)
    {
        EXPECT_DOUBLE_EQ(data.VolumeRatios()[i], 1.0);
        EXPECT_TRUE(
            data.VelocityGradients()[i].IsSimilar(Matrix<double, N, N>{}));
    }
}

template <size_t N>
void ExpectBaseSetResetsFluidState()
{
    MPMFluidSystemData<N> data{ Vector<size_t, N>::MakeConstant(2),
                                Vector<double, N>::MakeConstant(1.0),
                                {},
                                1 };
    ParticleSystemData<N> source{ 3 };

    DirtyFluidState(&data);
    data.Set(source);
    ExpectFluidStateReset(data);
}

template <size_t N>
void ExpectBaseDeserializeResetsFluidState()
{
    ParticleSystemData<N> source{ 3 };

    std::vector<uint8_t> buffer;
    source.Serialize(&buffer);

    MPMFluidSystemData<N> data{ Vector<size_t, N>::MakeConstant(2),
                                Vector<double, N>::MakeConstant(1.0),
                                {},
                                1 };

    DirtyFluidState(&data);
    data.Deserialize(buffer);
    ExpectFluidStateReset(data);
}

template <size_t N>
void ExpectParticleToGridConservation()
{
    MPMFluidSystemData<N> data{ Vector<size_t, N>::MakeConstant(4),
                                Vector<double, N>::MakeConstant(1.0),
                                {},
                                2 };
    data.Positions()[0] = Vector<double, N>::MakeConstant(1.25);
    data.Positions()[1] = Vector<double, N>::MakeConstant(2.25);
    data.Velocities()[0] = Vector<double, N>::MakeConstant(2.0);
    data.Velocities()[1] = Vector<double, N>::MakeConstant(-1.0);
    data.ParticleMasses()[0] = 2.0;
    data.ParticleMasses()[1] = 3.0;

    data.TransferFromParticlesToGrid();

    double totalMass = 0.0;
    Vector<double, N> totalMomentum;
    data.GridMass().ForEachDataPointIndex(
        [&data, &totalMass, &totalMomentum](const Vector<size_t, N>& index) {
            const double mass = data.GridMass()(index);
            totalMass += mass;
            totalMomentum += mass * data.GridVelocities()(index);
        });

    EXPECT_NEAR(totalMass, 5.0, 1e-11);
    EXPECT_TRUE(
        totalMomentum.IsSimilar(Vector<double, N>::MakeConstant(1.0), 1e-11));
}

template <size_t N>
void ExpectParticleToGridFailureDoesNotCommit()
{
    MPMFluidSystemData<N> data{ Vector<size_t, N>::MakeConstant(4),
                                Vector<double, N>::MakeConstant(1.0),
                                {},
                                2 };
    const Vector<double, N> gridVelocity = Vector<double, N>::MakeConstant(3.0);
    const Vector<double, N> oldGridVelocity =
        Vector<double, N>::MakeConstant(4.0);
    data.Positions()[0] = Vector<double, N>::MakeConstant(1.25);
    data.Positions()[1] =
        Vector<double, N>::MakeConstant(std::numeric_limits<double>::max());
    data.GridMass().Fill(2.0, ExecutionPolicy::Serial);
    data.GridVelocities().Fill(gridVelocity, ExecutionPolicy::Serial);
    data.GridVelocitiesBeforeUpdate().Fill(oldGridVelocity,
                                           ExecutionPolicy::Serial);

    EXPECT_THROW(data.TransferFromParticlesToGrid(), std::invalid_argument);

    data.GridMass().ForEachDataPointIndex(
        [&data, &gridVelocity,
         &oldGridVelocity](const Vector<size_t, N>& index) {
            EXPECT_DOUBLE_EQ(data.GridMass()(index), 2.0);
            EXPECT_TRUE(data.GridVelocities()(index).IsSimilar(gridVelocity));
            EXPECT_TRUE(data.GridVelocitiesBeforeUpdate()(index).IsSimilar(
                oldGridVelocity));
        });
}

template <size_t N>
void ExpectOverflowingParticleToGridTransferDoesNotCommit()
{
    MPMFluidSystemData<N> data{ Vector<size_t, N>::MakeConstant(4),
                                Vector<double, N>::MakeConstant(1.0),
                                {},
                                1 };
    const Vector<double, N> gridVelocity = Vector<double, N>::MakeConstant(3.0);
    const Vector<double, N> oldGridVelocity =
        Vector<double, N>::MakeConstant(4.0);

    data.Positions()[0] = Vector<double, N>::MakeConstant(1.25);
    data.Velocities()[0] =
        Vector<double, N>::MakeConstant(std::numeric_limits<double>::max());
    data.ParticleMasses()[0] = std::numeric_limits<double>::max();
    data.GridMass().Fill(2.0, ExecutionPolicy::Serial);
    data.GridVelocities().Fill(gridVelocity, ExecutionPolicy::Serial);
    data.GridVelocitiesBeforeUpdate().Fill(oldGridVelocity,
                                           ExecutionPolicy::Serial);

    EXPECT_THROW(data.TransferFromParticlesToGrid(), std::invalid_argument);

    data.GridMass().ForEachDataPointIndex(
        [&data, &gridVelocity,
         &oldGridVelocity](const Vector<size_t, N>& index) {
            EXPECT_DOUBLE_EQ(data.GridMass()(index), 2.0);
            EXPECT_TRUE(data.GridVelocities()(index).IsSimilar(gridVelocity));
            EXPECT_TRUE(data.GridVelocitiesBeforeUpdate()(index).IsSimilar(
                oldGridVelocity));
        });
}

template <size_t N>
void ExpectUniformVelocityRoundTrip()
{
    const Vector<double, N> velocity = Vector<double, N>::MakeConstant(2.5);
    MPMFluidSystemData<N> data{ Vector<size_t, N>::MakeConstant(6),
                                Vector<double, N>::MakeConstant(1.0),
                                {},
                                2 };
    data.Positions()[0] = Vector<double, N>::MakeConstant(2.25);
    data.Positions()[1] = Vector<double, N>::MakeConstant(3.25);
    data.Velocities()[0] = velocity;
    data.Velocities()[1] = velocity;

    data.TransferFromParticlesToGrid();
    data.TransferFromGridToParticles(0.1);

    EXPECT_TRUE(data.Velocities()[0].IsSimilar(velocity, 1e-12));
    EXPECT_TRUE(data.Velocities()[1].IsSimilar(velocity, 1e-12));
    EXPECT_NEAR(data.VelocityGradients()[0].AbsMax(), 0.0, 1e-12);
    EXPECT_NEAR(data.VelocityGradients()[1].AbsMax(), 0.0, 1e-12);
    EXPECT_NEAR(data.VolumeRatios()[0], 1.0, 1e-12);
    EXPECT_NEAR(data.VolumeRatios()[1], 1.0, 1e-12);
}

template <size_t N>
void ExpectUniformDilation(double rate, double expectedVolumeRatio)
{
    MPMFluidSystemData<N> data{ Vector<size_t, N>::MakeConstant(6),
                                Vector<double, N>::MakeConstant(1.0),
                                {},
                                1 };
    data.Positions()[0] = Vector<double, N>::MakeConstant(2.25);
    data.GridVelocities().Fill(
        [rate](const Vector<double, N>& position) { return rate * position; },
        ExecutionPolicy::Serial);
    data.GridVelocitiesBeforeUpdate().Set(data.GridVelocities());

    data.TransferFromGridToParticles(1.0);

    const auto gradient = data.VelocityGradients()[0];

    for (size_t row = 0; row < N; ++row)
    {
        for (size_t column = 0; column < N; ++column)
        {
            EXPECT_NEAR(gradient(row, column), row == column ? rate : 0.0,
                        1e-12);
        }
    }

    EXPECT_NEAR(data.VolumeRatios()[0], expectedVolumeRatio, 1e-12);
}

template <size_t N>
void FillCrossVelocityField(MPMFluidSystemData<N>* data)
{
    data->GridVelocities().Fill(
        [](const Vector<double, N>& position) {
            Vector<double, N> velocity;
            velocity[0] = 2.0 * position[1];
            velocity[1] = 3.0 * position[0];
            return velocity;
        },
        ExecutionPolicy::Serial);
    data->GridVelocitiesBeforeUpdate().Set(data->GridVelocities());
}

template <size_t N>
void ExpectVelocityGradientPreservesComponentOrientation()
{
    MPMFluidSystemData<N> data{ Vector<size_t, N>::MakeConstant(6),
                                Vector<double, N>::MakeConstant(1.0),
                                {},
                                1 };
    data.Positions()[0] = Vector<double, N>::MakeConstant(2.25);

    FillCrossVelocityField(&data);
    data.TransferFromGridToParticles(0.0);

    const auto gradient = data.VelocityGradients()[0];

    for (size_t row = 0; row < N; ++row)
    {
        for (size_t column = 0; column < N; ++column)
        {
            double expected = 0.0;

            if (row == 0 && column == 1)
            {
                expected = 2.0;
            }
            else if (row == 1 && column == 0)
            {
                expected = 3.0;
            }

            EXPECT_NEAR(gradient(row, column), expected, 1e-12);
        }
    }
}

template <size_t N>
void ExpectVolumeRatioMultipliesNonDiagonalIncrement()
{
    MPMFluidSystemData<N> data{ Vector<size_t, N>::MakeConstant(6),
                                Vector<double, N>::MakeConstant(1.0),
                                {},
                                1 };
    data.Positions()[0] = Vector<double, N>::MakeConstant(2.25);
    data.VolumeRatios()[0] = 2.0;

    FillCrossVelocityField(&data);

    data.TransferFromGridToParticles(0.1);
    EXPECT_NEAR(data.VolumeRatios()[0], 1.88, 1e-12);

    data.TransferFromGridToParticles(0.1);
    EXPECT_NEAR(data.VolumeRatios()[0], 1.7672, 1e-12);
}

template <size_t N>
void ExpectEmptyAndBoundaryTransfersAreSafe()
{
    MPMFluidSystemData<N> empty;
    EXPECT_NO_THROW(empty.TransferFromParticlesToGrid());
    EXPECT_NO_THROW(empty.TransferFromGridToParticles(0.1));

    MPMFluidSystemData<N> boundary{ Vector<size_t, N>::MakeConstant(2),
                                    Vector<double, N>::MakeConstant(1.0),
                                    {},
                                    1 };
    boundary.Velocities()[0] = Vector<double, N>::MakeConstant(1.0);

    EXPECT_NO_THROW(boundary.TransferFromParticlesToGrid());
    EXPECT_NO_THROW(boundary.TransferFromGridToParticles(0.1));
    EXPECT_TRUE(boundary.Velocities()[0].IsSimilar(
        Vector<double, N>::MakeConstant(1.0), 1e-12));
    EXPECT_NEAR(boundary.VolumeRatios()[0], 1.0, 1e-12);
}

template <size_t N>
void ExpectInvalidFluidStateIsNotCommitted()
{
    MPMFluidSystemData<N> data{ Vector<size_t, N>::MakeConstant(4),
                                Vector<double, N>::MakeConstant(1.0),
                                {},
                                1 };
    const Vector<double, N> velocity = Vector<double, N>::MakeConstant(4.0);

    data.Positions()[0] = Vector<double, N>::MakeConstant(1.25);
    data.Velocities()[0] = velocity;
    data.VolumeRatios()[0] = 0.0;
    data.GridVelocities().Fill(Vector<double, N>::MakeConstant(2.0),
                               ExecutionPolicy::Serial);
    data.GridVelocitiesBeforeUpdate().Fill(Vector<double, N>::MakeConstant(1.0),
                                           ExecutionPolicy::Serial);

    EXPECT_THROW(data.TransferFromGridToParticles(0.1), std::invalid_argument);
    EXPECT_TRUE(data.Velocities()[0].IsSimilar(velocity));
    EXPECT_DOUBLE_EQ(data.VolumeRatios()[0], 0.0);
    EXPECT_TRUE(data.VelocityGradients()[0].IsSimilar(Matrix<double, N, N>{}));
    EXPECT_THROW(data.TransferFromGridToParticles(-0.1), std::invalid_argument);
}

template <size_t N>
void ExpectOverflowingGridTransferIsNotCommitted()
{
    MPMFluidSystemData<N> data{ Vector<size_t, N>::MakeConstant(4),
                                Vector<double, N>::MakeConstant(1.0),
                                {},
                                1 };
    const Vector<double, N> velocity = Vector<double, N>::MakeConstant(4.0);
    const double large = 0.6 * std::numeric_limits<double>::max();
    data.Positions()[0] = Vector<double, N>::MakeConstant(1.25);
    data.Velocities()[0] = velocity;
    data.VolumeRatios()[0] = 2.0;
    data.VelocityGradients()[0](0, 0) = 3.0;
    data.GridVelocities().Fill(Vector<double, N>::MakeConstant(large),
                               ExecutionPolicy::Serial);
    data.GridVelocitiesBeforeUpdate().Fill(
        Vector<double, N>::MakeConstant(-large), ExecutionPolicy::Serial);
    data.SetFLIPBlendingFactor(1.0);

    EXPECT_THROW(data.TransferFromGridToParticles(0.0), std::invalid_argument);
    EXPECT_TRUE(data.Velocities()[0].IsSimilar(velocity));
    EXPECT_DOUBLE_EQ(data.VolumeRatios()[0], 2.0);
    EXPECT_DOUBLE_EQ(data.VelocityGradients()[0](0, 0), 3.0);
}
}  // namespace

TEST(MPMFluidSystemData, ParticleStateResizes)
{
    ExpectParticleStateResizes<2>();
    ExpectParticleStateResizes<3>();
}

TEST(MPMFluidSystemData, ParticleToGridConservesMassAndMomentum)
{
    ExpectParticleToGridConservation<2>();
    ExpectParticleToGridConservation<3>();
}

TEST(MPMFluidSystemData, ParticleToGridFailureDoesNotCommit)
{
    ExpectParticleToGridFailureDoesNotCommit<2>();
    ExpectParticleToGridFailureDoesNotCommit<3>();
}

TEST(MPMFluidSystemData, OverflowingParticleToGridTransferDoesNotCommit)
{
    ExpectOverflowingParticleToGridTransferDoesNotCommit<2>();
    ExpectOverflowingParticleToGridTransferDoesNotCommit<3>();
}

TEST(MPMFluidSystemData, BaseSetResetsFluidState)
{
    ExpectBaseSetResetsFluidState<2>();
    ExpectBaseSetResetsFluidState<3>();
}

TEST(MPMFluidSystemData, BaseDeserializeResetsFluidState)
{
    ExpectBaseDeserializeResetsFluidState<2>();
    ExpectBaseDeserializeResetsFluidState<3>();
}

TEST(MPMFluidSystemData, UniformVelocityRoundTrip)
{
    ExpectUniformVelocityRoundTrip<2>();
    ExpectUniformVelocityRoundTrip<3>();
}

TEST(MPMFluidSystemData, UniformExpansionAndCompression)
{
    ExpectUniformDilation<2>(0.1, 1.21);
    ExpectUniformDilation<3>(0.1, 1.331);
    ExpectUniformDilation<2>(-0.1, 0.81);
    ExpectUniformDilation<3>(-0.1, 0.729);
}

TEST(MPMFluidSystemData, VelocityGradientPreservesComponentOrientation)
{
    ExpectVelocityGradientPreservesComponentOrientation<2>();
    ExpectVelocityGradientPreservesComponentOrientation<3>();
}

TEST(MPMFluidSystemData, VolumeRatioMultipliesNonDiagonalIncrement)
{
    ExpectVolumeRatioMultipliesNonDiagonalIncrement<2>();
    ExpectVolumeRatioMultipliesNonDiagonalIncrement<3>();
}

TEST(MPMFluidSystemData, EmptyAndBoundaryTransfersAreSafe)
{
    ExpectEmptyAndBoundaryTransfersAreSafe<2>();
    ExpectEmptyAndBoundaryTransfersAreSafe<3>();
}

TEST(MPMFluidSystemData, InvalidFluidStateIsNotCommitted)
{
    ExpectInvalidFluidStateIsNotCommitted<2>();
    ExpectInvalidFluidStateIsNotCommitted<3>();
}

TEST(MPMFluidSystemData, OverflowingGridTransferIsNotCommitted)
{
    ExpectOverflowingGridTransferIsNotCommitted<2>();
    ExpectOverflowingGridTransferIsNotCommitted<3>();
}
