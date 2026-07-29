#include "gtest/gtest.h"

#include <Core/Solver/Grid/GridFluidSolver2.hpp>

using namespace CubbyFlow;

TEST(GridFluidSolver2, Constructor)
{
    GridFluidSolver2 solver;

    // Check if the sub-step solvers are present
    EXPECT_TRUE(solver.GetAdvectionSolver() != nullptr);
    EXPECT_TRUE(solver.GetDiffusionSolver() != nullptr);
    EXPECT_TRUE(solver.GetPressureSolver() != nullptr);

    // Check default parameters
    EXPECT_GE(solver.GetViscosityCoefficient(), 0.0);
    EXPECT_GT(solver.GetMaxCFL(), 0.0);
    EXPECT_EQ(DIRECTION_ALL, solver.GetClosedDomainBoundaryFlag());

    // Check grid system data
    EXPECT_TRUE(solver.GetGridSystemData() != nullptr);
    EXPECT_EQ(1u, solver.GetGridSystemData()->Resolution().x);
    EXPECT_EQ(1u, solver.GetGridSystemData()->Resolution().y);
    EXPECT_EQ(solver.GetGridSystemData()->Velocity(), solver.GetVelocity());

    // Collider should be null
    EXPECT_TRUE(solver.GetCollider() == nullptr);
}

TEST(GridFluidSolver2, ResizeGridSystemData)
{
    GridFluidSolver2 solver;

    solver.ResizeGrid(Vector2UZ(1, 2), Vector2D(3.0, 4.0), Vector2D(5.0, 6.0));

    EXPECT_EQ(1u, solver.GetResolution().x);
    EXPECT_EQ(2u, solver.GetResolution().y);
    EXPECT_EQ(3.0, solver.GetGridSpacing().x);
    EXPECT_EQ(4.0, solver.GetGridSpacing().y);
    EXPECT_EQ(5.0, solver.GetGridOrigin().x);
    EXPECT_EQ(6.0, solver.GetGridOrigin().y);
}

TEST(GridFluidSolver2, MinimumResolution)
{
    GridFluidSolver2 solver;

    solver.ResizeGrid(Vector2UZ(1, 1), Vector2D(1.0, 1.0), Vector2D());
    solver.GetVelocity()->Fill(Vector2D());

    Frame frame(0, 1.0 / 60.0);
    frame.timeIntervalInSeconds = 0.01;
    solver.Update(frame);
}

TEST(GridFluidSolver2, GravityOnly)
{
    GridFluidSolver2 solver;
    solver.SetGravity(Vector2D(0, -10));
    solver.SetAdvectionSolver(nullptr);
    solver.SetDiffusionSolver(nullptr);
    solver.SetPressureSolver(nullptr);

    solver.ResizeGrid(Vector2UZ(3, 3), Vector2D(1.0 / 3.0, 1.0 / 3.0),
                      Vector2D());
    solver.GetVelocity()->Fill(Vector2D());

    Frame frame(0, 1.0 / 60.0);
    frame.timeIntervalInSeconds = 0.01;
    solver.Update(frame);

    solver.GetVelocity()->ForEachUIndex([&solver](const Vector2UZ& idx) {
        EXPECT_NEAR(0.0, solver.GetVelocity()->U(idx), 1e-8);
    });

    solver.GetVelocity()->ForEachVIndex([&solver](const Vector2UZ& idx) {
        if (idx.y == 0 || idx.y == 3)
        {
            EXPECT_NEAR(0.0, solver.GetVelocity()->V(idx), 1e-8);
        }
        else
        {
            EXPECT_NEAR(-0.1, solver.GetVelocity()->V(idx), 1e-8);
        }
    });
}