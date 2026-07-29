#include "gtest/gtest.h"

#include <Core/Solver/Hybrid/PIC/PICSolver3.hpp>

using namespace CubbyFlow;

TEST(PICSolver3, UpdateEmpty)
{
    PICSolver3 solver;

    for (Frame frame; frame.index < 2; ++frame)
    {
        solver.Update(frame);
    }
}

TEST(PICSolver3, UpdateParticles)
{
    PICSolver3 solver{ { 4, 4, 4 }, { 1, 1, 1 }, {} };
    solver.SetGravity({});

    const ParticleSystemData3Ptr particles = solver.GetParticleSystemData();
    particles->AddParticle({ 1.5, 1.5, 1.5 }, { 1.0, 0.5, 0.25 });

    solver.Update(Frame{ 0, 0.01 });

    ASSERT_EQ(1u, particles->NumberOfParticles());
    EXPECT_TRUE(
        solver.GetGridSystemData()->Velocity()->GetBoundingBox().Contains(
            particles->Positions()[0]));
}
