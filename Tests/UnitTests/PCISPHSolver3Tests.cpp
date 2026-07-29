#include "gtest/gtest.h"

#include <Core/Solver/Particle/PCISPH/PCISPHSolver3.hpp>

using namespace CubbyFlow;

TEST(PCISPHSolver3, UpdateEmpty)
{
    // Empty solver test
    PCISPHSolver3 solver;
    Frame frame(0, 0.01);
    solver.Update(frame++);
    solver.Update(frame);
}

TEST(PCISPHSolver3, UpdateParticles)
{
    PCISPHSolver3 solver;
    solver.SetGravity({});

    const SPHSystemData3Ptr particles = solver.GetSPHSystemData();
    particles->AddParticle({ 0.0, 0.0, 0.0 });
    particles->AddParticle({ 0.05, 0.0, 0.0 });

    solver.Update(Frame{ 0, 0.001 });

    EXPECT_GT(particles->Densities()[0], 0.0);
    EXPECT_GT(particles->Densities()[1], 0.0);
}

TEST(PCISPHSolver3, Parameters)
{
    PCISPHSolver3 solver;

    solver.SetMaxDensityErrorRatio(5.0);
    EXPECT_DOUBLE_EQ(5.0, solver.GetMaxDensityErrorRatio());

    solver.SetMaxDensityErrorRatio(-1.0);
    EXPECT_DOUBLE_EQ(0.0, solver.GetMaxDensityErrorRatio());

    solver.SetMaxNumberOfIterations(10);
    EXPECT_DOUBLE_EQ(10, solver.GetMaxNumberOfIterations());
}
