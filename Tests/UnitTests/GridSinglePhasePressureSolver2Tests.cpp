#include "gtest/gtest.h"

#include <Core/Grid/CellCenteredScalarGrid.hpp>
#include <Core/Solver/Grid/GridSinglePhasePressureSolver2.hpp>

using namespace CubbyFlow;

TEST(GridSinglePhasePressureSolver2, SolveSinglePhase)
{
    FaceCenteredGrid2 vel({ 3, 3 });

    for (size_t j = 0; j < 3; ++j)
    {
        for (size_t i = 0; i < 4; ++i)
        {
            vel.U(i, j) = 0.0;
        }
    }

    for (size_t j = 0; j < 4; ++j)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            if (j == 0 || j == 3)
            {
                vel.V(i, j) = 0.0;
            }
            else
            {
                vel.V(i, j) = 1.0;
            }
        }
    }

    GridSinglePhasePressureSolver2 solver;
    solver.Solve(vel, 1.0, &vel,
                 ConstantScalarField2(std::numeric_limits<double>::max()),
                 ConstantVectorField2({ 0, 0 }),
                 ConstantScalarField2(-std::numeric_limits<double>::max()),
                 false);

    for (size_t j = 0; j < 3; ++j)
    {
        for (size_t i = 0; i < 4; ++i)
        {
            EXPECT_NEAR(0.0, vel.U(i, j), 1e-6);
        }
    }

    for (size_t j = 0; j < 4; ++j)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            EXPECT_NEAR(0.0, vel.V(i, j), 1e-6);
        }
    }

    const auto& pressure = solver.GetPressure();
    for (size_t j = 0; j < 2; ++j)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            EXPECT_NEAR(pressure(i, j + 1) - pressure(i, j), -1.0, 1e-6);
        }
    }
}

TEST(GridSinglePhasePressureSolver2, SolveSinglePhaseCompressed)
{
    FaceCenteredGrid2 vel({ 3, 3 });

    for (size_t j = 0; j < 3; ++j)
    {
        for (size_t i = 0; i < 4; ++i)
        {
            vel.U(i, j) = 0.0;
        }
    }

    for (size_t j = 0; j < 4; ++j)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            if (j == 0 || j == 3)
            {
                vel.V(i, j) = 0.0;
            }
            else
            {
                vel.V(i, j) = 1.0;
            }
        }
    }

    GridSinglePhasePressureSolver2 solver;
    solver.Solve(vel, 1.0, &vel,
                 ConstantScalarField2(std::numeric_limits<double>::max()),
                 ConstantVectorField2({ 0, 0 }),
                 ConstantScalarField2(-std::numeric_limits<double>::max()),
                 true);

    for (size_t j = 0; j < 3; ++j)
    {
        for (size_t i = 0; i < 4; ++i)
        {
            EXPECT_NEAR(0.0, vel.U(i, j), 1e-6);
        }
    }

    for (size_t j = 0; j < 4; ++j)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            EXPECT_NEAR(0.0, vel.V(i, j), 1e-6);
        }
    }

    const auto& pressure = solver.GetPressure();
    for (size_t j = 0; j < 2; ++j)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            EXPECT_NEAR(pressure(i, j + 1) - pressure(i, j), -1.0, 1e-6);
        }
    }
}

TEST(GridSinglePhasePressureSolver2, SolveSinglePhaseWithBoundary)
{
    FaceCenteredGrid2 vel({ 3, 3 });
    CellCenteredScalarGrid2 boundarySDF({ 3, 3 });

    for (size_t j = 0; j < 3; ++j)
    {
        for (size_t i = 0; i < 4; ++i)
        {
            vel.U(i, j) = 0.0;
        }
    }

    for (size_t j = 0; j < 4; ++j)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            if (j == 0 || j == 3)
            {
                vel.V(i, j) = 0.0;
            }
            else
            {
                vel.V(i, j) = 1.0;
            }
        }
    }

    // Wall on the right-most column
    boundarySDF.Fill([](const Vector2D& x) { return -x.x + 2.0; });

    GridSinglePhasePressureSolver2 solver;
    solver.Solve(vel, 1.0, &vel, boundarySDF);

    for (size_t j = 0; j < 3; ++j)
    {
        for (size_t i = 0; i < 4; ++i)
        {
            EXPECT_NEAR(0.0, vel.U(i, j), 1e-6);
        }
    }

    for (size_t j = 0; j < 4; ++j)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            if (i == 2 && (j == 1 || j == 2))
            {
                EXPECT_NEAR(1.0, vel.V(i, j), 1e-6);
            }
            else
            {
                EXPECT_NEAR(0.0, vel.V(i, j), 1e-6);
            }
        }
    }

    const auto& pressure = solver.GetPressure();
    for (size_t j = 0; j < 2; ++j)
    {
        for (size_t i = 0; i < 2; ++i)
        {
            EXPECT_NEAR(pressure(i, j + 1) - pressure(i, j), -1.0, 1e-6);
        }
    }
}

TEST(GridSinglePhasePressureSolver2, SolveFreeSurface)
{
    FaceCenteredGrid2 vel({ 3, 3 });
    CellCenteredScalarGrid2 fluidSDF({ 3, 3 });

    for (size_t j = 0; j < 3; ++j)
    {
        for (size_t i = 0; i < 4; ++i)
        {
            vel.U(i, j) = 0.0;
        }
    }

    for (size_t j = 0; j < 4; ++j)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            if (j == 0 || j == 3)
            {
                vel.V(i, j) = 0.0;
            }
            else
            {
                vel.V(i, j) = 1.0;
            }
        }
    }

    fluidSDF.Fill([](const Vector2D& x) { return x.y - 2.0; });

    GridSinglePhasePressureSolver2 solver;
    solver.Solve(vel, 1.0, &vel,
                 ConstantScalarField2(std::numeric_limits<double>::max()),
                 ConstantVectorField2({ 0, 0 }), fluidSDF);

    for (size_t j = 0; j < 3; ++j)
    {
        for (size_t i = 0; i < 4; ++i)
        {
            EXPECT_NEAR(0.0, vel.U(i, j), 1e-6);
        }
    }

    for (size_t j = 0; j < 4; ++j)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            EXPECT_NEAR(0.0, vel.V(i, j), 1e-6);
        }
    }

    const auto& pressure = solver.GetPressure();
    for (size_t j = 0; j < 3; ++j)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            double p = static_cast<double>(2 - j);
            EXPECT_NEAR(p, pressure(i, j), 1e-6);
        }
    }
}

TEST(GridSinglePhasePressureSolver2, SolveFreeSurfaceCompressed)
{
    FaceCenteredGrid2 vel({ 3, 3 });
    CellCenteredScalarGrid2 fluidSDF({ 3, 3 });

    for (size_t j = 0; j < 3; ++j)
    {
        for (size_t i = 0; i < 4; ++i)
        {
            vel.U(i, j) = 0.0;
        }
    }

    for (size_t j = 0; j < 4; ++j)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            if (j == 0 || j == 3)
            {
                vel.V(i, j) = 0.0;
            }
            else
            {
                vel.V(i, j) = 1.0;
            }
        }
    }

    fluidSDF.Fill([](const Vector2D& x) { return x.y - 2.0; });

    GridSinglePhasePressureSolver2 solver;
    solver.Solve(vel, 1.0, &vel,
                 ConstantScalarField2(std::numeric_limits<double>::max()),
                 ConstantVectorField2({ 0, 0 }), fluidSDF, true);

    for (size_t j = 0; j < 3; ++j)
    {
        for (size_t i = 0; i < 4; ++i)
        {
            EXPECT_NEAR(0.0, vel.U(i, j), 1e-6);
        }
    }

    for (size_t j = 0; j < 4; ++j)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            EXPECT_NEAR(0.0, vel.V(i, j), 1e-6);
        }
    }

    const auto& pressure = solver.GetPressure();
    for (size_t j = 0; j < 3; ++j)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            double p = static_cast<double>(2 - j);
            EXPECT_NEAR(p, pressure(i, j), 1e-6);
        }
    }
}

TEST(GridSinglePhasePressureSolver2, SolveFreeSurfaceWithBoundary)
{
    FaceCenteredGrid2 vel({ 3, 3 });
    CellCenteredScalarGrid2 fluidSDF({ 3, 3 });
    CellCenteredScalarGrid2 boundarySDF({ 3, 3 });

    for (size_t j = 0; j < 3; ++j)
    {
        for (size_t i = 0; i < 4; ++i)
        {
            vel.U(i, j) = 0.0;
        }
    }

    for (size_t j = 0; j < 4; ++j)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            if (j == 0 || j == 3)
            {
                vel.V(i, j) = 0.0;
            }
            else
            {
                vel.V(i, j) = 1.0;
            }
        }
    }

    // Wall on the right-most column
    boundarySDF.Fill([](const Vector2D& x) { return -x.x + 2.0; });
    fluidSDF.Fill([](const Vector2D& x) { return x.y - 2.0; });

    GridSinglePhasePressureSolver2 solver;
    solver.Solve(vel, 1.0, &vel, boundarySDF, ConstantVectorField2({ 0, 0 }),
                 fluidSDF);

    for (size_t j = 0; j < 3; ++j)
    {
        for (size_t i = 0; i < 4; ++i)
        {
            EXPECT_NEAR(0.0, vel.U(i, j), 1e-6);
        }
    }

    for (size_t j = 0; j < 4; ++j)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            if (i == 2 && (j == 1 || j == 2))
            {
                EXPECT_NEAR(1.0, vel.V(i, j), 1e-6);
            }
            else
            {
                EXPECT_NEAR(0.0, vel.V(i, j), 1e-6);
            }
        }
    }

    const auto& pressure = solver.GetPressure();
    for (size_t j = 0; j < 3; ++j)
    {
        for (size_t i = 0; i < 2; ++i)
        {
            double p = static_cast<double>(2 - j);
            EXPECT_NEAR(p, pressure(i, j), 1e-6);
        }
    }
}

TEST(GridSinglePhasePressureSolver2, SolveSinglePhaseWithMG)
{
    const size_t n = 64;
    FaceCenteredGrid2 vel({ n, n });

    for (size_t j = 0; j < n; ++j)
    {
        for (size_t i = 0; i < n + 1; ++i)
        {
            vel.U(i, j) = 0.0;
        }
    }

    for (size_t j = 0; j < n + 1; ++j)
    {
        for (size_t i = 0; i < n; ++i)
        {
            if (j == 0 || j == n)
            {
                vel.V(i, j) = 0.0;
            }
            else
            {
                vel.V(i, j) = 1.0;
            }
        }
    }

    GridSinglePhasePressureSolver2 solver;
    solver.SetLinearSystemSolver(
        std::make_shared<FDMMGSolver2>(5, 10, 10, 40, 10));

    solver.Solve(vel, 1.0, &vel);

    for (size_t j = 0; j < n; ++j)
    {
        for (size_t i = 0; i < n + 1; ++i)
        {
            EXPECT_NEAR(0.0, vel.U(i, j), 0.01);
        }
    }

    for (size_t j = 0; j < n + 1; ++j)
    {
        for (size_t i = 0; i < n; ++i)
        {
            EXPECT_NEAR(0.0, vel.V(i, j), 0.05);
        }
    }
}