#include "gtest/gtest.h"

#include <ManualTests.hpp>

#include <Core/Geometry/ImplicitSurfaceSet.hpp>
#include <Core/Geometry/Plane.hpp>
#include <Core/Geometry/RigidBodyCollider.hpp>
#include <Core/Geometry/Sphere.hpp>
#include <Core/Solver/Grid/GridBlockedBoundaryConditionSolver2.hpp>
#include <Core/Solver/Grid/GridFractionalBoundaryConditionSolver2.hpp>

#include <algorithm>

using namespace CubbyFlow;

CUBBYFLOW_TESTS(GridBlockedBoundaryConditionSolver2);

CUBBYFLOW_BEGIN_TEST_F(GridBlockedBoundaryConditionSolver2,
                       ConstrainVelocitySmall)
{
    GridBlockedBoundaryConditionSolver2 solver;
    auto collider = std::make_shared<RigidBodyCollider2>(
        std::make_shared<Plane2>(Vector2D(1, 1).Normalized(), Vector2D()));
    Vector2UZ gridSize(10, 10);
    Vector2D gridSpacing(1.0, 1.0);
    Vector2D gridOrigin(-5.0, -5.0);

    solver.UpdateCollider(collider, gridSize, gridSpacing, gridOrigin);

    FaceCenteredGrid2 velocity(gridSize, gridSpacing, gridOrigin);
    velocity.Fill(Vector2D(1.0, 1.0));

    solver.ConstrainVelocity(&velocity);

    // Output
    Array2<double> dataU(10, 10);
    Array2<double> dataV(10, 10);
    Array2<double> dataM(10, 10);

    ForEachIndex(dataU.Size(), [&velocity, &dataU, &dataV, &dataM, &solver](
                                   size_t i, size_t j) {
        Vector2D vel = velocity.ValueAtCellCenter(i, j);
        dataU(i, j) = vel.x;
        dataV(i, j) = vel.y;
        dataM(i, j) = static_cast<double>(solver.GetMarker()(i, j));
    });

    SaveData(dataU.View(), "data_#grid2,x.npy");
    SaveData(dataV.View(), "data_#grid2,y.npy");
    SaveData(dataM.View(), "marker_#grid2.npy");
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(GridBlockedBoundaryConditionSolver2, ConstrainVelocity)
{
    double dx = 1.0 / 32.0;
    FaceCenteredGrid2 velocity(Vector2UZ(64, 32), Vector2D(dx, dx), Vector2D());
    velocity.Fill(Vector2D(1.0, 0.0));
    BoundingBox2D domain = velocity.GetBoundingBox();

    // Collider setting
    auto surfaceSet = std::make_shared<ImplicitSurfaceSet2>();
    surfaceSet->AddExplicitSurface(
        std::make_shared<Sphere2>(domain.MidPoint(), 0.25));
    surfaceSet->AddExplicitSurface(
        std::make_shared<Sphere2>(domain.upperCorner, 0.25));
    surfaceSet->AddExplicitSurface(
        std::make_shared<Sphere2>(domain.lowerCorner, 0.25));
    RigidBodyCollider2Ptr collider =
        std::make_shared<RigidBodyCollider2>(surfaceSet);
    collider->linearVelocity = Vector2D(-1.0, 0.0);

    // Solver setting
    GridBlockedBoundaryConditionSolver2 solver;
    solver.UpdateCollider(collider, velocity.Resolution(),
                          velocity.GridSpacing(), velocity.Origin());
    solver.SetClosedDomainBoundaryFlag(DIRECTION_RIGHT | DIRECTION_DOWN |
                                       DIRECTION_UP);

    // Constrain velocity
    solver.ConstrainVelocity(&velocity, 5);

    // Output
    Array2<double> dataU(64, 32);
    Array2<double> dataV(64, 32);
    Array2<double> dataM(64, 32);

    ForEachIndex(dataU.Size(), [&velocity, &dataU, &dataV, &dataM, &solver](
                                   size_t i, size_t j) {
        Vector2D vel = velocity.ValueAtCellCenter(i, j);
        dataU(i, j) = vel.x;
        dataV(i, j) = vel.y;
        dataM(i, j) = static_cast<double>(solver.GetMarker()(i, j));
    });

    SaveData(dataU.View(), "data_#grid2,x.npy");
    SaveData(dataV.View(), "data_#grid2,y.npy");
    SaveData(dataM.View(), "marker_#grid2.npy");
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(GridBlockedBoundaryConditionSolver2,
                       ConstrainVelocityWithFriction)
{
    double dx = 1.0 / 32.0;
    FaceCenteredGrid2 velocity(Vector2UZ(64, 32), Vector2D(dx, dx), Vector2D());
    velocity.Fill(Vector2D(1.0, 0.0));
    BoundingBox2D domain = velocity.GetBoundingBox();

    // Collider setting
    auto surfaceSet = std::make_shared<ImplicitSurfaceSet2>();
    surfaceSet->AddExplicitSurface(
        std::make_shared<Sphere2>(domain.MidPoint(), 0.25));
    surfaceSet->AddExplicitSurface(
        std::make_shared<Sphere2>(domain.upperCorner, 0.25));
    surfaceSet->AddExplicitSurface(
        std::make_shared<Sphere2>(domain.lowerCorner, 0.25));
    RigidBodyCollider2Ptr collider =
        std::make_shared<RigidBodyCollider2>(surfaceSet);
    collider->linearVelocity = Vector2D(-1.0, 0.0);
    collider->SetFrictionCoefficient(1.0);

    // Solver setting
    GridBlockedBoundaryConditionSolver2 solver;
    solver.UpdateCollider(collider, velocity.Resolution(),
                          velocity.GridSpacing(), velocity.Origin());
    solver.SetClosedDomainBoundaryFlag(DIRECTION_RIGHT | DIRECTION_DOWN |
                                       DIRECTION_UP);

    // Constrain velocity
    solver.ConstrainVelocity(&velocity, 5);

    // Output
    Array2<double> dataU(64, 32);
    Array2<double> dataV(64, 32);
    Array2<double> dataM(64, 32);

    ForEachIndex(dataU.Size(), [&velocity, &dataU, &dataV, &dataM, &solver](
                                   size_t i, size_t j) {
        Vector2D vel = velocity.ValueAtCellCenter(i, j);
        dataU(i, j) = vel.x;
        dataV(i, j) = vel.y;
        dataM(i, j) = static_cast<double>(solver.GetMarker()(i, j));
    });

    SaveData(dataU.View(), "data_#grid2,x.npy");
    SaveData(dataV.View(), "data_#grid2,y.npy");
    SaveData(dataM.View(), "marker_#grid2.npy");
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_TESTS(GridFractionalBoundaryConditionSolver2);

CUBBYFLOW_BEGIN_TEST_F(GridFractionalBoundaryConditionSolver2,
                       ConstrainVelocity)
{
    double dx = 1.0 / 32.0;
    FaceCenteredGrid2 velocity(Vector2UZ(64, 32), Vector2D(dx, dx), Vector2D());
    velocity.Fill(Vector2D(1.0, 0.0));
    BoundingBox2D domain = velocity.GetBoundingBox();

    // Collider setting
    auto surfaceSet = std::make_shared<ImplicitSurfaceSet2>();
    surfaceSet->AddExplicitSurface(
        std::make_shared<Sphere2>(domain.MidPoint(), 0.25));
    surfaceSet->AddExplicitSurface(
        std::make_shared<Sphere2>(domain.upperCorner, 0.25));
    surfaceSet->AddExplicitSurface(
        std::make_shared<Sphere2>(domain.lowerCorner, 0.25));
    RigidBodyCollider2Ptr collider =
        std::make_shared<RigidBodyCollider2>(surfaceSet);
    collider->linearVelocity = Vector2D(-1.0, 0.0);

    // Solver setting
    GridFractionalBoundaryConditionSolver2 solver;
    solver.UpdateCollider(collider, velocity.Resolution(),
                          velocity.GridSpacing(), velocity.Origin());
    solver.SetClosedDomainBoundaryFlag(DIRECTION_RIGHT | DIRECTION_DOWN |
                                       DIRECTION_UP);

    // Constrain velocity
    solver.ConstrainVelocity(&velocity, 5);

    // Output
    Array2<double> dataU(64, 32);
    Array2<double> dataV(64, 32);

    ForEachIndex(dataU.Size(), [&velocity, &dataU, &dataV](size_t i, size_t j) {
        Vector2D vel = velocity.ValueAtCellCenter(i, j);
        dataU(i, j) = vel.x;
        dataV(i, j) = vel.y;
    });

    SaveData(dataU.View(), "data_#grid2,x.npy");
    SaveData(dataV.View(), "data_#grid2,y.npy");
}
CUBBYFLOW_END_TEST_F