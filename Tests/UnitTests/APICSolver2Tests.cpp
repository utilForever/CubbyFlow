#include "gtest/gtest.h"

#include <Core/Solver/Hybrid/APIC/APICSolver2.hpp>

using namespace CubbyFlow;

TEST(APICSolver2, UpdateEmpty)
{
    APICSolver2 solver;

    for (Frame frame; frame.index < 2; ++frame)
    {
        solver.Update(frame);
    }
}

TEST(APICSolver2, UpdateParticles)
{
    APICSolver2 solver{ { 4, 4 }, { 1, 1 }, {} };
    solver.SetGravity({});

    const ParticleSystemData2Ptr particles = solver.GetParticleSystemData();
    particles->AddParticle({ 1.5, 1.5 }, { 1.0, 0.5 });

    solver.Update(Frame{ 0, 0.01 });

    ASSERT_EQ(1u, particles->NumberOfParticles());
    EXPECT_TRUE(
        solver.GetGridSystemData()->Velocity()->GetBoundingBox().Contains(
            particles->Positions()[0]));
}
