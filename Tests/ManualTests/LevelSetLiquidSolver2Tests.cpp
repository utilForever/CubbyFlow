#include "gtest/gtest.h"

#include <Core/Emitter/VolumeGridEmitter2.hpp>
#include <Core/Geometry/Box.hpp>
#include <Core/Geometry/ImplicitSurfaceSet.hpp>
#include <Core/Geometry/Plane.hpp>
#include <Core/Geometry/RigidBodyCollider.hpp>
#include <Core/Geometry/Sphere.hpp>
#include <Core/Geometry/SurfaceToImplicit.hpp>
#include <Core/Solver/Grid/GridFractionalSinglePhasePressureSolver2.hpp>
#include <Core/Solver/LevelSet/LevelSetLiquidSolver2.hpp>
#include <Core/Utils/LevelSetUtils.hpp>

#include <ManualTests.hpp>

using namespace CubbyFlow;

CUBBYFLOW_TESTS(LevelSetLiquidSolver2);

CUBBYFLOW_BEGIN_TEST_F(LevelSetLiquidSolver2, Drop)
{
    LevelSetLiquidSolver2 solver;

    auto data = solver.GetGridSystemData();
    double dx = 1.0 / 32.0;
    data->Resize(Vector2UZ(32, 64), Vector2D(dx, dx), Vector2D());

    // Source Setting
    BoundingBox2D domain = data->GetBoundingBox();
    ImplicitSurfaceSet2 surfaceSet;
    surfaceSet.AddExplicitSurface(
        std::make_shared<Plane2>(Vector2D(0, 1), Vector2D(0, 0.5)));
    surfaceSet.AddExplicitSurface(
        std::make_shared<Sphere2>(domain.MidPoint(), 0.15));

    auto sdf = solver.GetSignedDistanceField();
    sdf->Fill([&surfaceSet](const Vector2D& x) {
        return surfaceSet.SignedDistance(x);
    });

    Array2<double> output(32, 64);
    ForEachIndex(output.Size(), [&output, &sdf, &dx](size_t i, size_t j) {
        output(i, j) = 1.0 - SmearedHeavisideSDF((*sdf)(i, j) / dx);
    });

    char fileName[256];
    snprintf(fileName, sizeof(fileName), "data.#grid2,0000.npy");
    SaveData(output.View(), fileName);

    for (Frame frame(0, 1.0 / 60.0); frame.index < 120; frame.Advance())
    {
        solver.Update(frame);

        ForEachIndex(output.Size(), [&output, &sdf, &dx](size_t i, size_t j) {
            output(i, j) = 1.0 - SmearedHeavisideSDF((*sdf)(i, j) / dx);
        });
        snprintf(fileName, sizeof(fileName), "data.#grid2,%04d.npy",
                 frame.index);
        SaveData(output.View(), fileName);
    }
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(LevelSetLiquidSolver2, DropStopAndGo)
{
    Frame frame(0, 1.0 / 60.0);
    Array2<double> output(32, 64);
    std::vector<uint8_t> dump;
    char fileName[256];

    {
        // Build solver
        auto solver = LevelSetLiquidSolver2::Builder()
                          .WithResolution({ 32, 64 })
                          .WithDomainSizeX(1.0)
                          .MakeShared();

        auto grids = solver->GetGridSystemData();
        auto domain = grids->GetBoundingBox();
        auto sdf = solver->GetSignedDistanceField();
        double dx = grids->GridSpacing().x;

        // Build emitter
        auto plane = Plane2::Builder()
                         .WithNormal({ 0, 1 })
                         .WithPoint({ 0, 0.5 })
                         .MakeShared();

        auto sphere = Sphere2::Builder()
                          .WithCenter(domain.MidPoint())
                          .WithRadius(0.15)
                          .MakeShared();

        auto surfaceSet =
            ImplicitSurfaceSet2::Builder()
                .WithExplicitSurfaces(Array1<Surface2Ptr>{ plane, sphere })
                .MakeShared();

        auto emitter = VolumeGridEmitter2::Builder()
                           .WithSourceRegion(surfaceSet)
                           .MakeShared();

        solver->SetEmitter(emitter);
        emitter->AddSignedDistanceTarget(solver->GetSignedDistanceField());

        for (; frame.index < 60; ++frame)
        {
            solver->Update(frame);

            ForEachIndex(
                output.Size(), [&output, &sdf, &dx](size_t i, size_t j) {
                    output(i, j) = 1.0 - SmearedHeavisideSDF((*sdf)(i, j) / dx);
                });

            snprintf(fileName, sizeof(fileName), "data.#grid2,%04d.npy",
                     frame.index);
            SaveData(output.View(), fileName);
        }

        grids->Serialize(&dump);
    }

    {
        // Build solver
        auto solver = LevelSetLiquidSolver2::Builder().MakeShared();
        solver->SetCurrentFrame(frame);

        auto grids = solver->GetGridSystemData();
        grids->Deserialize(dump);

        double dx = grids->GridSpacing().x;
        auto sdf = solver->GetSignedDistanceField();

        for (; frame.index < 120; ++frame)
        {
            solver->Update(frame);

            ForEachIndex(
                output.Size(), [&output, &sdf, &dx](size_t i, size_t j) {
                    output(i, j) = 1.0 - SmearedHeavisideSDF((*sdf)(i, j) / dx);
                });

            snprintf(fileName, sizeof(fileName), "data.#grid2,%04d.npy",
                     frame.index);
            SaveData(output.View(), fileName);
        }
    }
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(LevelSetLiquidSolver2, DropHighRes)
{
    LevelSetLiquidSolver2 solver;

    auto data = solver.GetGridSystemData();
    double dx = 1.0 / 128.0;
    data->Resize(Vector2UZ(128, 256), Vector2D(dx, dx), Vector2D());

    // Source Setting
    BoundingBox2D domain = data->GetBoundingBox();
    ImplicitSurfaceSet2 surfaceSet;
    surfaceSet.AddExplicitSurface(
        std::make_shared<Plane2>(Vector2D(0, 1), Vector2D(0, 0.5)));
    surfaceSet.AddExplicitSurface(
        std::make_shared<Sphere2>(domain.MidPoint(), 0.15));

    auto sdf = solver.GetSignedDistanceField();
    sdf->Fill([&surfaceSet](const Vector2D& x) {
        return surfaceSet.SignedDistance(x);
    });

    Array2<double> output(128, 256);
    ForEachIndex(output.Size(), [&output, &sdf, &dx](size_t i, size_t j) {
        output(i, j) = 1.0 - SmearedHeavisideSDF((*sdf)(i, j) / dx);
    });

    char fileName[256];
    snprintf(fileName, sizeof(fileName), "data.#grid2,0000.npy");
    SaveData(output.View(), fileName);

    for (Frame frame(0, 1.0 / 60.0); frame.index < 120; frame.Advance())
    {
        solver.Update(frame);

        ForEachIndex(output.Size(), [&output, &sdf, &dx](size_t i, size_t j) {
            output(i, j) = 1.0 - SmearedHeavisideSDF((*sdf)(i, j) / dx);
        });
        snprintf(fileName, sizeof(fileName), "data.#grid2,%04d.npy",
                 frame.index);
        SaveData(output.View(), fileName);
    }
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(LevelSetLiquidSolver2, DropWithCollider)
{
    LevelSetLiquidSolver2 solver;

    auto data = solver.GetGridSystemData();
    double dx = 1.0 / 32.0;
    data->Resize(Vector2UZ(32, 150), Vector2D(dx, dx), Vector2D());

    // Source Setting
    BoundingBox2D domain = data->GetBoundingBox();
    ImplicitSurfaceSet2 surfaceSet;
    surfaceSet.AddExplicitSurface(
        std::make_shared<Plane2>(Vector2D(0, 1), Vector2D(0, 0.5)));

    auto sdf = solver.GetSignedDistanceField();
    sdf->Fill([&surfaceSet](const Vector2D& x) {
        return surfaceSet.SignedDistance(x);
    });

    Array2<double> output(32, 150);
    ForEachIndex(output.Size(), [&output, &sdf, &dx](size_t i, size_t j) {
        output(i, j) = SmearedHeavisideSDF((*sdf)(i, j) / dx);
    });

    // Collider Setting
    auto sphere = std::make_shared<Sphere2>(
        Vector2D(domain.MidPoint().x, 0.75 - std::cos(0.0)), 0.2);
    auto surface = std::make_shared<SurfaceToImplicit2>(sphere);
    auto collider = std::make_shared<RigidBodyCollider2>(surface);
    solver.SetCollider(collider);

    char fileName[256];
    snprintf(fileName, sizeof(fileName), "data.#grid2,0000.npy");
    SaveData(output.View(), fileName);

    for (Frame frame(0, 1.0 / 60.0); frame.index < 240; frame.Advance())
    {
        double t = frame.TimeInSeconds();
        sphere->center = Vector2D(domain.MidPoint().x, 0.75 - std::cos(t));
        collider->linearVelocity = Vector2D(0, std::sin(t));

        solver.Update(frame);

        ForEachIndex(output.Size(), [&output, &sdf, &dx](size_t i, size_t j) {
            output(i, j) = SmearedHeavisideSDF((*sdf)(i, j) / dx);
        });
        snprintf(fileName, sizeof(fileName), "data.#grid2,%04d.npy",
                 frame.index);
        SaveData(output.View(), fileName);
    }
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(LevelSetLiquidSolver2, DropVariational)
{
    LevelSetLiquidSolver2 solver;
    solver.SetPressureSolver(
        std::make_shared<GridFractionalSinglePhasePressureSolver2>());

    auto data = solver.GetGridSystemData();
    double dx = 1.0 / 32.0;
    data->Resize(Vector2UZ(32, 150), Vector2D(dx, dx), Vector2D());

    // Source Setting
    BoundingBox2D domain = data->GetBoundingBox();
    ImplicitSurfaceSet2 surfaceSet;
    surfaceSet.AddExplicitSurface(
        std::make_shared<Plane2>(Vector2D(0, 1), Vector2D(0, 0.5)));
    surfaceSet.AddExplicitSurface(
        std::make_shared<Sphere2>(domain.MidPoint(), 0.15));

    auto sdf = solver.GetSignedDistanceField();
    sdf->Fill([&surfaceSet](const Vector2D& x) {
        return surfaceSet.SignedDistance(x);
    });

    Array2<double> output(32, 150);
    ForEachIndex(output.Size(), [&output, &sdf, &dx](size_t i, size_t j) {
        output(i, j) = SmearedHeavisideSDF((*sdf)(i, j) / dx);
    });

    char fileName[256];
    snprintf(fileName, sizeof(fileName), "data.#grid2,0000.npy");
    SaveData(output.View(), fileName);

    for (Frame frame(0, 1.0 / 60.0); frame.index < 120; frame.Advance())
    {
        solver.Update(frame);

        ForEachIndex(output.Size(), [&output, &sdf, &dx](size_t i, size_t j) {
            output(i, j) = SmearedHeavisideSDF((*sdf)(i, j) / dx);
        });
        snprintf(fileName, sizeof(fileName), "data.#grid2,%04d.npy",
                 frame.index);
        SaveData(output.View(), fileName);
    }
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(LevelSetLiquidSolver2, DropWithColliderVariational)
{
    LevelSetLiquidSolver2 solver;
    solver.SetPressureSolver(
        std::make_shared<GridFractionalSinglePhasePressureSolver2>());

    auto data = solver.GetGridSystemData();
    double dx = 1.0 / 32.0;
    data->Resize(Vector2UZ(32, 150), Vector2D(dx, dx), Vector2D());

    // Source Setting
    BoundingBox2D domain = data->GetBoundingBox();
    ImplicitSurfaceSet2 surfaceSet;
    surfaceSet.AddExplicitSurface(
        std::make_shared<Plane2>(Vector2D(0, 1), Vector2D(0, 0.5)));

    auto sdf = solver.GetSignedDistanceField();
    sdf->Fill([&surfaceSet](const Vector2D& x) {
        return surfaceSet.SignedDistance(x);
    });

    Array2<double> output(32, 150);
    ForEachIndex(output.Size(), [&output, &sdf, &dx](size_t i, size_t j) {
        output(i, j) = SmearedHeavisideSDF((*sdf)(i, j) / dx);
    });

    // Collider Setting
    auto sphere = std::make_shared<Sphere2>(
        Vector2D(domain.MidPoint().x, 0.75 - std::cos(0.0)), 0.2);
    auto surface = std::make_shared<SurfaceToImplicit2>(sphere);
    auto collider = std::make_shared<RigidBodyCollider2>(surface);
    solver.SetCollider(collider);

    char fileName[256];
    snprintf(fileName, sizeof(fileName), "data.#grid2,0000.npy");
    SaveData(output.View(), fileName);

    for (Frame frame(0, 1.0 / 60.0); frame.index < 240; frame.Advance())
    {
        double t = frame.TimeInSeconds();
        sphere->center = Vector2D(domain.MidPoint().x, 0.75 - std::cos(t));
        collider->linearVelocity = Vector2D(0, std::sin(t));

        solver.Update(frame);

        ForEachIndex(output.Size(), [&output, &sdf, &dx](size_t i, size_t j) {
            output(i, j) = SmearedHeavisideSDF((*sdf)(i, j) / dx);
        });
        snprintf(fileName, sizeof(fileName), "data.#grid2,%04d.npy",
                 frame.index);
        SaveData(output.View(), fileName);
    }
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(LevelSetLiquidSolver2, ViscousDropVariational)
{
    LevelSetLiquidSolver2 solver;
    solver.SetViscosityCoefficient(1.0);
    solver.SetPressureSolver(
        std::make_shared<GridFractionalSinglePhasePressureSolver2>());

    auto data = solver.GetGridSystemData();
    double dx = 1.0 / 50.0;
    data->Resize(Vector2UZ(50, 100), Vector2D(dx, dx), Vector2D());

    // Source Setting
    BoundingBox2D domain = data->GetBoundingBox();
    ImplicitSurfaceSet2 surfaceSet;
    surfaceSet.AddExplicitSurface(
        std::make_shared<Plane2>(Vector2D(0, 1), Vector2D(0, 0.5)));
    surfaceSet.AddExplicitSurface(
        std::make_shared<Sphere2>(domain.MidPoint(), 0.15));

    auto sdf = solver.GetSignedDistanceField();
    sdf->Fill([&surfaceSet](const Vector2D& x) {
        return surfaceSet.SignedDistance(x);
    });

    Array2<double> output(50, 100);
    ForEachIndex(output.Size(), [&output, &sdf, &dx](size_t i, size_t j) {
        output(i, j) = SmearedHeavisideSDF((*sdf)(i, j) / dx);
    });

    char fileName[256];
    snprintf(fileName, sizeof(fileName), "data.#grid2,0000.npy");
    SaveData(output.View(), fileName);

    for (Frame frame(0, 1.0 / 60.0); frame.index < 120; frame.Advance())
    {
        solver.Update(frame);

        ForEachIndex(output.Size(), [&output, &sdf, &dx](size_t i, size_t j) {
            output(i, j) = SmearedHeavisideSDF((*sdf)(i, j) / dx);
        });
        snprintf(fileName, sizeof(fileName), "data.#grid2,%04d.npy",
                 frame.index);
        SaveData(output.View(), fileName);
    }
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(LevelSetLiquidSolver2, DropWithoutGlobalComp)
{
    LevelSetLiquidSolver2 solver;
    solver.SetIsGlobalCompensationEnabled(false);

    auto data = solver.GetGridSystemData();
    double dx = 1.0 / 32.0;
    data->Resize(Vector2UZ(32, 64), Vector2D(dx, dx), Vector2D());

    // Source Setting
    BoundingBox2D domain = data->GetBoundingBox();
    ImplicitSurfaceSet2 surfaceSet;
    surfaceSet.AddExplicitSurface(
        std::make_shared<Sphere2>(domain.MidPoint(), 0.15));

    auto sdf = solver.GetSignedDistanceField();
    sdf->Fill([&surfaceSet](const Vector2D& x) {
        return surfaceSet.SignedDistance(x);
    });

    Array2<double> output(32, 64);
    ForEachIndex(output.Size(), [&output, &sdf, &dx](size_t i, size_t j) {
        output(i, j) = 1.0 - SmearedHeavisideSDF((*sdf)(i, j) / dx);
    });

    char fileName[256];
    snprintf(fileName, sizeof(fileName), "data.#grid2,0000.npy");
    SaveData(output.View(), fileName);

    for (Frame frame(0, 1.0 / 60.0); frame.index < 120; frame.Advance())
    {
        solver.Update(frame);

        ForEachIndex(output.Size(), [&output, &sdf, &dx](size_t i, size_t j) {
            output(i, j) = 1.0 - SmearedHeavisideSDF((*sdf)(i, j) / dx);
        });
        snprintf(fileName, sizeof(fileName), "data.#grid2,%04d.npy",
                 frame.index);
        SaveData(output.View(), fileName);
    }
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(LevelSetLiquidSolver2, DropWithGlobalComp)
{
    LevelSetLiquidSolver2 solver;
    solver.SetIsGlobalCompensationEnabled(true);

    auto data = solver.GetGridSystemData();
    double dx = 1.0 / 32.0;
    data->Resize(Vector2UZ(32, 64), Vector2D(dx, dx), Vector2D());

    // Source Setting
    BoundingBox2D domain = data->GetBoundingBox();
    ImplicitSurfaceSet2 surfaceSet;
    surfaceSet.AddExplicitSurface(
        std::make_shared<Sphere2>(domain.MidPoint(), 0.15));

    auto sdf = solver.GetSignedDistanceField();
    sdf->Fill([&surfaceSet](const Vector2D& x) {
        return surfaceSet.SignedDistance(x);
    });

    Array2<double> output(32, 64);
    ForEachIndex(output.Size(), [&output, &sdf, &dx](size_t i, size_t j) {
        output(i, j) = 1.0 - SmearedHeavisideSDF((*sdf)(i, j) / dx);
    });

    char fileName[256];
    snprintf(fileName, sizeof(fileName), "data.#grid2,0000.npy");
    SaveData(output.View(), fileName);

    for (Frame frame(0, 1.0 / 60.0); frame.index < 120; frame.Advance())
    {
        solver.Update(frame);

        ForEachIndex(output.Size(), [&output, &sdf, &dx](size_t i, size_t j) {
            output(i, j) = 1.0 - SmearedHeavisideSDF((*sdf)(i, j) / dx);
        });
        snprintf(fileName, sizeof(fileName), "data.#grid2,%04d.npy",
                 frame.index);
        SaveData(output.View(), fileName);
    }
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(LevelSetLiquidSolver2, RisingFloor)
{
    // Build solver
    auto solver = LevelSetLiquidSolver2::Builder()
                      .WithResolution({ 5, 10 })
                      .WithDomainSizeX(1.0)
                      .MakeShared();
    solver->SetGravity({ 0, 0, 0 });

    // Build emitter
    auto box = Box2::Builder()
                   .WithLowerCorner({ 0.0, 0.0 })
                   .WithUpperCorner({ 1.0, 0.8 })
                   .MakeShared();

    auto emitter =
        VolumeGridEmitter2::Builder().WithSourceRegion(box).MakeShared();

    solver->SetEmitter(emitter);
    emitter->AddSignedDistanceTarget(solver->GetSignedDistanceField());

    // Build collider
    auto tank = Box2::Builder()
                    .WithLowerCorner({ -1, 0 })
                    .WithUpperCorner({ 2, 2 })
                    .WithIsNormalFlipped(true)
                    .MakeShared();

    auto collider =
        RigidBodyCollider2::Builder().WithSurface(tank).MakeShared();

    collider->SetOnBeginUpdateCallback([](Collider2* col, double t, double) {
        col->GetSurface()->transform.SetTranslation({ 0, t });
        static_cast<RigidBodyCollider2*>(col)->linearVelocity.x = 0.0;
        static_cast<RigidBodyCollider2*>(col)->linearVelocity.y = 1.0;
    });

    solver->SetCollider(collider);

    char fileName[256];
    Array2<double> output(5, 10);
    Array2<double> div(5, 10);
    auto data = solver->GetGridSystemData();
    auto sdf = solver->GetSignedDistanceField();

    for (Frame frame(0, 1 / 100.0); frame.index < 120; ++frame)
    {
        solver->Update(frame);

        ForEachIndex(output.Size(), [&output, &sdf](size_t i, size_t j) {
            output(i, j) = 1.0 - SmearedHeavisideSDF((*sdf)(i, j) * 5.0);
        });
        snprintf(fileName, sizeof(fileName), "output.#grid2,%04d.npy",
                 frame.index);
        SaveData(output.View(), fileName);
    }
}
CUBBYFLOW_END_TEST_F