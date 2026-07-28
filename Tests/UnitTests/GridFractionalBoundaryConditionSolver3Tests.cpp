#include "gtest/gtest.h"

#include <Core/Solver/Grid/GridFractionalBoundaryConditionSolver3.hpp>

using namespace CubbyFlow;

TEST(GridFractionalBoundaryConditionSolver3, ClosedDomain)
{
    GridFractionalBoundaryConditionSolver3 bndSolver;
    Vector3UZ gridSize(10, 10, 10);
    Vector3D gridSpacing(1.0, 1.0, 1.0);
    Vector3D gridOrigin(-5.0, -5.0, -5.0);

    bndSolver.UpdateCollider(nullptr, gridSize, gridSpacing, gridOrigin);

    FaceCenteredGrid3 velocity(gridSize, gridSpacing, gridOrigin);
    velocity.Fill(Vector3D(1.0, 1.0, 1.0));

    bndSolver.ConstrainVelocity(&velocity);

    velocity.ForEachUIndex([&gridSize, &velocity](const Vector3UZ& idx) {
        if (idx.x == 0 || idx.x == gridSize.x)
        {
            EXPECT_DOUBLE_EQ(0.0, velocity.U(idx));
        }
        else
        {
            EXPECT_DOUBLE_EQ(1.0, velocity.U(idx));
        }
    });

    velocity.ForEachVIndex([&gridSize, &velocity](const Vector3UZ& idx) {
        if (idx.y == 0 || idx.y == gridSize.y)
        {
            EXPECT_DOUBLE_EQ(0.0, velocity.V(idx));
        }
        else
        {
            EXPECT_DOUBLE_EQ(1.0, velocity.V(idx));
        }
    });

    velocity.ForEachWIndex([&gridSize, &velocity](const Vector3UZ& idx) {
        if (idx.z == 0 || idx.z == gridSize.z)
        {
            EXPECT_DOUBLE_EQ(0.0, velocity.W(idx));
        }
        else
        {
            EXPECT_DOUBLE_EQ(1.0, velocity.W(idx));
        }
    });
}

TEST(GridFractionalBoundaryConditionSolver3, OpenDomain)
{
    GridFractionalBoundaryConditionSolver3 bndSolver;
    Vector3UZ gridSize(10, 10, 10);
    Vector3D gridSpacing(1.0, 1.0, 1.0);
    Vector3D gridOrigin(-5.0, -5.0, -5.0);

    // Partially open domain
    bndSolver.SetClosedDomainBoundaryFlag(DIRECTION_LEFT | DIRECTION_UP |
                                          DIRECTION_FRONT);
    bndSolver.UpdateCollider(nullptr, gridSize, gridSpacing, gridOrigin);

    FaceCenteredGrid3 velocity(gridSize, gridSpacing, gridOrigin);
    velocity.Fill(Vector3D(1.0, 1.0, 1.0));

    bndSolver.ConstrainVelocity(&velocity);

    velocity.ForEachUIndex([&velocity](const Vector3UZ& idx) {
        if (idx.x == 0)
        {
            EXPECT_DOUBLE_EQ(0.0, velocity.U(idx));
        }
        else
        {
            EXPECT_DOUBLE_EQ(1.0, velocity.U(idx));
        }
    });

    velocity.ForEachVIndex([&gridSize, &velocity](const Vector3UZ& idx) {
        if (idx.y == gridSize.y)
        {
            EXPECT_DOUBLE_EQ(0.0, velocity.V(idx));
        }
        else
        {
            EXPECT_DOUBLE_EQ(1.0, velocity.V(idx));
        }
    });

    velocity.ForEachWIndex([&gridSize, &velocity](const Vector3UZ& idx) {
        if (idx.z == gridSize.z)
        {
            EXPECT_DOUBLE_EQ(0.0, velocity.W(idx));
        }
        else
        {
            EXPECT_DOUBLE_EQ(1.0, velocity.W(idx));
        }
    });
}