#include "gtest/gtest.h"

#include <Core/Solver/Particle/SPH/SPHSolver3.hpp>

using namespace CubbyFlow;

TEST(SPHSolver3, UpdateEmpty)
{
    // Empty solver test
    SPHSolver3 solver;
    Frame frame(0, 0.01);
    solver.Update(frame++);
    solver.Update(frame);
}

TEST(SPHSolver3, UpdateParticles)
{
    SPHSolver3 solver;
    solver.SetGravity({});

    const SPHSystemData3Ptr particles = solver.GetSPHSystemData();
    particles->AddParticle({ 0.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 });
    particles->AddParticle({ 0.05, 0.0, 0.0 }, { 0.0, 1.0, 0.0 });

    solver.Update(Frame{ 0, 0.001 });

    EXPECT_GT(particles->Densities()[0], 0.0);
    EXPECT_GT(particles->Densities()[1], 0.0);
}

TEST(SPHSolver3, Parameters)
{
    SPHSolver3 solver;

    solver.SetEosExponent(5.0);
    EXPECT_DOUBLE_EQ(5.0, solver.GetEosExponent());

    solver.SetEosExponent(-1.0);
    EXPECT_DOUBLE_EQ(1.0, solver.GetEosExponent());

    solver.SetNegativePressureScale(0.3);
    EXPECT_DOUBLE_EQ(0.3, solver.GetNegativePressureScale());

    solver.SetNegativePressureScale(-1.0);
    EXPECT_DOUBLE_EQ(0.0, solver.GetNegativePressureScale());

    solver.SetNegativePressureScale(3.0);
    EXPECT_DOUBLE_EQ(1.0, solver.GetNegativePressureScale());

    solver.SetViscosityCoefficient(0.3);
    EXPECT_DOUBLE_EQ(0.3, solver.GetViscosityCoefficient());

    solver.SetViscosityCoefficient(-1.0);
    EXPECT_DOUBLE_EQ(0.0, solver.GetViscosityCoefficient());

    solver.SetPseudoViscosityCoefficient(0.3);
    EXPECT_DOUBLE_EQ(0.3, solver.GetPseudoViscosityCoefficient());

    solver.SetPseudoViscosityCoefficient(-1.0);
    EXPECT_DOUBLE_EQ(0.0, solver.GetPseudoViscosityCoefficient());

    solver.SetSpeedOfSound(0.3);
    EXPECT_DOUBLE_EQ(0.3, solver.GetSpeedOfSound());

    solver.SetSpeedOfSound(-1.0);
    EXPECT_GT(solver.GetSpeedOfSound(), 0.0);

    solver.SetTimeStepLimitScale(0.3);
    EXPECT_DOUBLE_EQ(0.3, solver.GetTimeStepLimitScale());

    solver.SetTimeStepLimitScale(-1.0);
    EXPECT_DOUBLE_EQ(0.0, solver.GetTimeStepLimitScale());

    EXPECT_TRUE(solver.GetSPHSystemData() != nullptr);
}
