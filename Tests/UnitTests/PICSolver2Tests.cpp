#include "gtest/gtest.h"

#include <Core/Solver/Hybrid/PIC/PICSolver2.hpp>

using namespace CubbyFlow;

TEST(PICSolver2, UpdateEmpty)
{
    PICSolver2 solver;

    for (Frame frame; frame.index < 2; ++frame)
    {
        solver.Update(frame);
    }
}

TEST(PICSolver2, UpdateParticles)
{
    PICSolver2 solver{ { 4, 4 }, { 1, 1 }, {} };
    solver.SetGravity({});

    const ParticleSystemData2Ptr particles = solver.GetParticleSystemData();
    particles->AddParticle({ 1.5, 1.5 }, { 1.0, 0.5 });

    solver.Update(Frame{ 0, 0.01 });

    ASSERT_EQ(1u, particles->NumberOfParticles());
    EXPECT_TRUE(
        solver.GetGridSystemData()->Velocity()->GetBoundingBox().Contains(
            particles->Positions()[0]));
}
