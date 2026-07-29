#include "gtest/gtest.h"

#include <Core/Grid/CellCenteredScalarGrid.hpp>
#include <Core/Solver/Grid/GridFractionalSinglePhasePressureSolver2.hpp>

using namespace CubbyFlow;

TEST(GridFractionalSinglePhasePressureSolver2, SolveFreeSurface)
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

    GridFractionalSinglePhasePressureSolver2 solver;
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
    for (size_t i = 0; i < 3; ++i)
    {
        EXPECT_NEAR(1.5, pressure(i, 0), 1e-6);
        EXPECT_NEAR(0.5, pressure(i, 1), 1e-6);
        EXPECT_NEAR(0.0, pressure(i, 2), 1e-6);
    }
}

TEST(GridFractionalSinglePhasePressureSolver2, SolveFreeSurfaceCompressed)
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

    GridFractionalSinglePhasePressureSolver2 solver;
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
    for (size_t i = 0; i < 3; ++i)
    {
        EXPECT_NEAR(1.5, pressure(i, 0), 1e-6);
        EXPECT_NEAR(0.5, pressure(i, 1), 1e-6);
        EXPECT_NEAR(0.0, pressure(i, 2), 1e-6);
    }
}

TEST(GridFractionalSinglePhasePressureSolver2, SolveFreeSurfaceMG)
{
    FaceCenteredGrid2 vel({ 32, 32 });
    CellCenteredScalarGrid2 fluidSDF({ 32, 32 });

    for (size_t j = 0; j < 32; ++j)
    {
        for (size_t i = 0; i < 33; ++i)
        {
            vel.U(i, j) = 0.0;
        }
    }

    for (size_t j = 0; j < 33; ++j)
    {
        for (size_t i = 0; i < 32; ++i)
        {
            if (j == 0 || j == 32)
            {
                vel.V(i, j) = 0.0;
            }
            else
            {
                vel.V(i, j) = 1.0;
            }
        }
    }

    fluidSDF.Fill([](const Vector2D& x) { return x.y - 16.0; });

    GridFractionalSinglePhasePressureSolver2 solver;
    solver.SetLinearSystemSolver(
        std::make_shared<FDMMGSolver2>(5, 50, 50, 50, 50));
    solver.Solve(vel, 1.0, &vel,
                 ConstantScalarField2(std::numeric_limits<double>::max()),
                 ConstantVectorField2({ 0, 0 }), fluidSDF, true);

    for (size_t j = 0; j < 32; ++j)
    {
        for (size_t i = 0; i < 33; ++i)
        {
            EXPECT_NEAR(0.0, vel.U(i, j), 0.002);
        }
    }

    for (size_t j = 0; j < 16; ++j)
    {
        for (size_t i = 0; i < 32; ++i)
        {
            EXPECT_NEAR(0.0, vel.V(i, j), 0.002);
        }
    }

    const auto& pressure = solver.GetPressure();
    for (size_t j = 0; j < 32; ++j)
    {
        for (size_t i = 16; i < 17; ++i)
        {
            if (j < 16)
            {
                EXPECT_NEAR(15.5 - static_cast<double>(j), pressure(i, j), 0.1);
            }
            else
            {
                EXPECT_NEAR(0.0, pressure(i, j), 0.1);
            }
        }
    }
}