#include "gtest/gtest.h"

#include <Core/Geometry/RigidBodyCollider.hpp>
#include <Core/Geometry/Sphere.hpp>
#include <Core/Solver/Grid/GridFractionalBoundaryConditionSolver2.hpp>

using namespace CubbyFlow;

TEST(GridFractionalBoundaryConditionSolver2, ClosedDomain)
{
    GridFractionalBoundaryConditionSolver2 bndSolver;
    Vector2UZ gridSize(10, 10);
    Vector2D gridSpacing(1.0, 1.0);
    Vector2D gridOrigin(-5.0, -5.0);

    bndSolver.UpdateCollider(nullptr, gridSize, gridSpacing, gridOrigin);

    FaceCenteredGrid2 velocity(gridSize, gridSpacing, gridOrigin);
    velocity.Fill(Vector2D(1.0, 1.0));

    bndSolver.ConstrainVelocity(&velocity);

    velocity.ForEachUIndex([&gridSize, &velocity](const Vector2UZ& idx) {
        if (idx.x == 0 || idx.x == gridSize.x)
        {
            EXPECT_DOUBLE_EQ(0.0, velocity.U(idx));
        }
        else
        {
            EXPECT_DOUBLE_EQ(1.0, velocity.U(idx));
        }
    });

    velocity.ForEachVIndex([&gridSize, &velocity](const Vector2UZ& idx) {
        if (idx.y == 0 || idx.y == gridSize.y)
        {
            EXPECT_DOUBLE_EQ(0.0, velocity.V(idx));
        }
        else
        {
            EXPECT_DOUBLE_EQ(1.0, velocity.V(idx));
        }
    });
}

TEST(GridFractionalBoundaryConditionSolver2, OpenDomain)
{
    GridFractionalBoundaryConditionSolver2 bndSolver;
    Vector2UZ gridSize(10, 10);
    Vector2D gridSpacing(1.0, 1.0);
    Vector2D gridOrigin(-5.0, -5.0);

    // Partially open domain
    bndSolver.SetClosedDomainBoundaryFlag(DIRECTION_LEFT | DIRECTION_UP);
    bndSolver.UpdateCollider(nullptr, gridSize, gridSpacing, gridOrigin);

    FaceCenteredGrid2 velocity(gridSize, gridSpacing, gridOrigin);
    velocity.Fill(Vector2D(1.0, 1.0));

    bndSolver.ConstrainVelocity(&velocity);

    velocity.ForEachUIndex([&velocity](const Vector2UZ& idx) {
        if (idx.x == 0)
        {
            EXPECT_DOUBLE_EQ(0.0, velocity.U(idx));
        }
        else
        {
            EXPECT_DOUBLE_EQ(1.0, velocity.U(idx));
        }
    });

    velocity.ForEachVIndex([&gridSize, &velocity](const Vector2UZ& idx) {
        if (idx.y == gridSize.y)
        {
            EXPECT_DOUBLE_EQ(0.0, velocity.V(idx));
        }
        else
        {
            EXPECT_DOUBLE_EQ(1.0, velocity.V(idx));
        }
    });
}

TEST(GridFractionalBoundaryConditionSolver2, MovingCollider)
{
    FaceCenteredGrid2 velocity({ 8, 8 }, { 0.125, 0.125 });
    velocity.Fill(Vector2D{ 1.0, 1.0 });

    auto collider = std::make_shared<RigidBodyCollider2>(
        std::make_shared<Sphere2>(Vector2D{ 0.5, 0.5 }, 0.3));
    collider->linearVelocity = Vector2D{ -1.0, 0.0 };

    GridFractionalBoundaryConditionSolver2 solver;
    solver.SetClosedDomainBoundaryFlag(0);
    solver.UpdateCollider(collider, velocity.Resolution(),
                          velocity.GridSpacing(), velocity.Origin());
    solver.ConstrainVelocity(&velocity);

    EXPECT_LT(velocity.U(6, 3), 1.0);
    EXPECT_LT(velocity.V(3, 6), 1.0);
}
