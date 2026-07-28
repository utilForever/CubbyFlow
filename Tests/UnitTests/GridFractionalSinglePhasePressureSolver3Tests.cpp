#include "gtest/gtest.h"

#include <Core/Grid/CellCenteredScalarGrid.hpp>
#include <Core/Solver/Grid/GridFractionalSinglePhasePressureSolver3.hpp>

using namespace CubbyFlow;

TEST(GridFractionalSinglePhasePressureSolver3, SolveFreeSurface)
{
    FaceCenteredGrid3 vel({ 3, 3, 3 });
    CellCenteredScalarGrid3 fluidSDF({ 3, 3, 3 });

    vel.Fill(Vector3D());

    for (size_t k = 0; k < 3; ++k)
    {
        for (size_t j = 0; j < 4; ++j)
        {
            for (size_t i = 0; i < 3; ++i)
            {
                if (j == 0 || j == 3)
                {
                    vel.V(i, j, k) = 0.0;
                }
                else
                {
                    vel.V(i, j, k) = 1.0;
                }
            }
        }
    }

    fluidSDF.Fill([](const Vector3D& x) { return x.y - 2.0; });

    GridFractionalSinglePhasePressureSolver3 solver;
    solver.Solve(vel, 1.0, &vel,
                 ConstantScalarField3(std::numeric_limits<double>::max()),
                 ConstantVectorField3({ 0, 0, 0 }), fluidSDF);

    for (size_t k = 0; k < 3; ++k)
    {
        for (size_t j = 0; j < 3; ++j)
        {
            for (size_t i = 0; i < 4; ++i)
            {
                EXPECT_NEAR(0.0, vel.U(i, j, k), 1e-6);
            }
        }
    }

    for (size_t k = 0; k < 3; ++k)
    {
        for (size_t j = 0; j < 4; ++j)
        {
            for (size_t i = 0; i < 3; ++i)
            {
                EXPECT_NEAR(0.0, vel.V(i, j, k), 1e-6);
            }
        }
    }

    for (size_t k = 0; k < 4; ++k)
    {
        for (size_t j = 0; j < 3; ++j)
        {
            for (size_t i = 0; i < 3; ++i)
            {
                EXPECT_NEAR(0.0, vel.W(i, j, k), 1e-6);
            }
        }
    }

    const auto& pressure = solver.GetPressure();
    for (size_t k = 0; k < 3; ++k)
    {
        for (size_t j = 0; j < 2; ++j)
        {
            for (size_t i = 0; i < 3; ++i)
            {
                double p = static_cast<double>(1.5 - j);
                EXPECT_NEAR(p, pressure(i, j, k), 1e-6);
            }
        }
    }
}

TEST(GridFractionalSinglePhasePressureSolver3, SolveFreeSurfaceCompressed)
{
    FaceCenteredGrid3 vel({ 3, 3, 3 });
    CellCenteredScalarGrid3 fluidSDF({ 3, 3, 3 });

    vel.Fill(Vector3D());

    for (size_t k = 0; k < 3; ++k)
    {
        for (size_t j = 0; j < 4; ++j)
        {
            for (size_t i = 0; i < 3; ++i)
            {
                if (j == 0 || j == 3)
                {
                    vel.V(i, j, k) = 0.0;
                }
                else
                {
                    vel.V(i, j, k) = 1.0;
                }
            }
        }
    }

    fluidSDF.Fill([](const Vector3D& x) { return x.y - 2.0; });

    GridFractionalSinglePhasePressureSolver3 solver;
    solver.Solve(vel, 1.0, &vel,
                 ConstantScalarField3(std::numeric_limits<double>::max()),
                 ConstantVectorField3({ 0, 0, 0 }), fluidSDF, true);

    for (size_t k = 0; k < 3; ++k)
    {
        for (size_t j = 0; j < 3; ++j)
        {
            for (size_t i = 0; i < 4; ++i)
            {
                EXPECT_NEAR(0.0, vel.U(i, j, k), 1e-6);
            }
        }
    }

    for (size_t k = 0; k < 3; ++k)
    {
        for (size_t j = 0; j < 4; ++j)
        {
            for (size_t i = 0; i < 3; ++i)
            {
                EXPECT_NEAR(0.0, vel.V(i, j, k), 1e-6);
            }
        }
    }

    for (size_t k = 0; k < 4; ++k)
    {
        for (size_t j = 0; j < 3; ++j)
        {
            for (size_t i = 0; i < 3; ++i)
            {
                EXPECT_NEAR(0.0, vel.W(i, j, k), 1e-6);
            }
        }
    }

    const auto& pressure = solver.GetPressure();
    for (size_t k = 0; k < 3; ++k)
    {
        for (size_t j = 0; j < 2; ++j)
        {
            for (size_t i = 0; i < 3; ++i)
            {
                double p = static_cast<double>(1.5 - j);
                EXPECT_NEAR(p, pressure(i, j, k), 1e-6);
            }
        }
    }
}