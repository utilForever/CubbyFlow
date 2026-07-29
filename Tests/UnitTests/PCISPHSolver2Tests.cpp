#include "gtest/gtest.h"

#include <Core/Solver/Particle/PCISPH/PCISPHSolver2.hpp>

using namespace CubbyFlow;

TEST(PCISPHSolver2, UpdateEmpty)
{
    // Empty solver test
    PCISPHSolver2 solver;
    Frame frame(0, 0.01);
    solver.Update(frame++);
    solver.Update(frame);
}

TEST(PCISPHSolver2, UpdateParticles)
{
    PCISPHSolver2 solver;
    solver.SetGravity({});

    const SPHSystemData2Ptr particles = solver.GetSPHSystemData();
    particles->AddParticle({ 0.0, 0.0 });
    particles->AddParticle({ 0.05, 0.0 });

    solver.Update(Frame{ 0, 0.001 });

    EXPECT_GT(particles->Densities()[0], 0.0);
    EXPECT_GT(particles->Densities()[1], 0.0);
}

TEST(PCISPHSolver2, Parameters)
{
    PCISPHSolver2 solver;

    solver.SetMaxDensityErrorRatio(5.0);
    EXPECT_DOUBLE_EQ(5.0, solver.GetMaxDensityErrorRatio());

    solver.SetMaxDensityErrorRatio(-1.0);
    EXPECT_DOUBLE_EQ(0.0, solver.GetMaxDensityErrorRatio());

    solver.SetMaxNumberOfIterations(10);
    EXPECT_DOUBLE_EQ(10, solver.GetMaxNumberOfIterations());
}
