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
}  // namespace

TEST(MPMSystemData, ParticleStateResizes)
{
    ExpectParticleStateResizes<2>();
    ExpectParticleStateResizes<3>();
}

TEST(MPMSystemData, GridStateResizes)
{
    ExpectGridStateResizes<2>();
    ExpectGridStateResizes<3>();
}
