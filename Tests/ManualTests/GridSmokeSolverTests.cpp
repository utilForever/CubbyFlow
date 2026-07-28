#include "gtest/gtest.h"

#include <Core/Emitter/VolumeGridEmitter2.hpp>
#include <Core/Emitter/VolumeGridEmitter3.hpp>
#include <Core/Geometry/Box.hpp>
#include <Core/Geometry/RigidBodyCollider.hpp>
#include <Core/Geometry/Sphere.hpp>
#include <Core/Solver/Advection/CubicSemiLagrangian3.hpp>
#include <Core/Solver/Grid/GridSinglePhasePressureSolver2.hpp>
#include <Core/Solver/Grid/GridSmokeSolver2.hpp>
#include <Core/Solver/Grid/GridSmokeSolver3.hpp>

#include <ManualTests.hpp>

using namespace CubbyFlow;

CUBBYFLOW_TESTS(GridSmokeSolver2);

CUBBYFLOW_BEGIN_TEST_F(GridSmokeSolver2, Rising)
{
    // Build solver
    auto solver = GridSmokeSolver2::Builder()
                      .WithResolution({ 32, 64 })
                      .WithGridSpacing(1.0 / 32.0)
                      .MakeShared();

    // Build emitter
    auto box = Box2::Builder()
                   .WithLowerCorner({ 0.3, 0.0 })
                   .WithUpperCorner({ 0.7, 0.4 })
                   .MakeShared();

    auto emitter =
        VolumeGridEmitter2::Builder().WithSourceRegion(box).MakeShared();

    solver->SetEmitter(emitter);
    emitter->AddStepFunctionTarget(solver->GetSmokeDensity(), 0.0, 1.0);
    emitter->AddStepFunctionTarget(solver->GetTemperature(), 0.0, 1.0);

    for (Frame frame; frame.index < 240; ++frame)
    {
        solver->Update(frame);

        SaveData(solver->GetSmokeDensity()->DataView(), frame.index);
    }
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(GridSmokeSolver2, RisingWithCollider)
{
    // Build solver
    auto solver = GridSmokeSolver2::Builder()
                      .WithResolution({ 32, 64 })
                      .WithGridSpacing(1.0 / 32.0)
                      .MakeShared();

    // Build emitter
    auto box = Box2::Builder()
                   .WithLowerCorner({ 0.3, 0.0 })
                   .WithUpperCorner({ 0.7, 0.4 })
                   .MakeShared();

    auto emitter =
        VolumeGridEmitter2::Builder().WithSourceRegion(box).MakeShared();

    solver->SetEmitter(emitter);
    emitter->AddStepFunctionTarget(solver->GetSmokeDensity(), 0.0, 1.0);
    emitter->AddStepFunctionTarget(solver->GetTemperature(), 0.0, 1.0);

    // Build collider
    auto sphere = Sphere2::Builder()
                      .WithCenter({ 0.5, 1.0 })
                      .WithRadius(0.1)
                      .MakeShared();

    auto collider =
        RigidBodyCollider2::Builder().WithSurface(sphere).MakeShared();

    solver->SetCollider(collider);

    for (Frame frame; frame.index < 240; ++frame)
    {
        solver->Update(frame);

        SaveData(solver->GetSmokeDensity()->DataView(), frame.index);
    }
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(GridSmokeSolver2, MovingEmitterWithCollider)
{
    // Build solver
    auto solver = GridSmokeSolver2::Builder()
                      .WithResolution({ 32, 64 })
                      .WithGridSpacing(1.0 / 32.0)
                      .MakeShared();

    // Build emitter
    auto box = Box2::Builder()
                   .WithLowerCorner({ 0.3, 0.0 })
                   .WithUpperCorner({ 0.7, 0.1 })
                   .MakeShared();

    auto emitter = VolumeGridEmitter2::Builder()
                       .WithSourceRegion(box)
                       .WithIsOneShot(false)
                       .MakeShared();

    solver->SetEmitter(emitter);
    emitter->AddStepFunctionTarget(solver->GetSmokeDensity(), 0.0, 1.0);
    emitter->AddStepFunctionTarget(solver->GetTemperature(), 0.0, 1.0);
    emitter->SetOnBeginUpdateCallback(
        [&box](GridEmitter2*, double t, double dt) {
            box->bound.lowerCorner.x = 0.1 * std::sin(PI_DOUBLE * t) + 0.3;
            box->bound.upperCorner.x = 0.1 * std::sin(PI_DOUBLE * t) + 0.7;
        });

    // Build collider
    auto sphere = Sphere2::Builder()
                      .WithCenter({ 0.5, 1.0 })
                      .WithRadius(0.1)
                      .MakeShared();

    auto collider =
        RigidBodyCollider2::Builder().WithSurface(sphere).MakeShared();

    solver->SetCollider(collider);

    for (Frame frame; frame.index < 240; ++frame)
    {
        solver->Update(frame);

        SaveData(solver->GetSmokeDensity()->DataView(), frame.index);
    }
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(GridSmokeSolver2, RisingWithColliderNonVariational)
{
    // Build solver
    auto solver = GridSmokeSolver2::Builder()
                      .WithResolution({ 32, 64 })
                      .WithGridSpacing(1.0 / 32.0)
                      .MakeShared();

    solver->SetPressureSolver(
        std::make_shared<GridSinglePhasePressureSolver2>());

    // Build emitter
    auto box = Box2::Builder()
                   .WithLowerCorner({ 0.3, 0.0 })
                   .WithUpperCorner({ 0.7, 0.4 })
                   .MakeShared();

    auto emitter =
        VolumeGridEmitter2::Builder().WithSourceRegion(box).MakeShared();

    solver->SetEmitter(emitter);
    emitter->AddStepFunctionTarget(solver->GetSmokeDensity(), 0.0, 1.0);
    emitter->AddStepFunctionTarget(solver->GetTemperature(), 0.0, 1.0);

    // Build collider
    auto sphere = Sphere2::Builder()
                      .WithCenter({ 0.5, 1.0 })
                      .WithRadius(0.1)
                      .MakeShared();

    auto collider =
        RigidBodyCollider2::Builder().WithSurface(sphere).MakeShared();

    solver->SetCollider(collider);

    for (Frame frame; frame.index < 240; ++frame)
    {
        solver->Update(frame);

        SaveData(solver->GetSmokeDensity()->DataView(), frame.index);
    }
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(GridSmokeSolver2, RisingWithColliderAndDiffusion)
{
    // Build solver
    auto solver = GridSmokeSolver2::Builder()
                      .WithResolution({ 32, 64 })
                      .WithGridSpacing(1.0 / 32.0)
                      .MakeShared();

    // Parameter Setting
    solver->SetViscosityCoefficient(0.01);
    solver->SetSmokeDiffusionCoefficient(0.01);
    solver->SetTemperatureDiffusionCoefficient(0.01);

    // Build emitter
    auto box = Box2::Builder()
                   .WithLowerCorner({ 0.3, 0.0 })
                   .WithUpperCorner({ 0.7, 0.4 })
                   .MakeShared();

    auto emitter =
        VolumeGridEmitter2::Builder().WithSourceRegion(box).MakeShared();

    solver->SetEmitter(emitter);
    emitter->AddStepFunctionTarget(solver->GetSmokeDensity(), 0.0, 1.0);
    emitter->AddStepFunctionTarget(solver->GetTemperature(), 0.0, 1.0);

    // Build collider
    auto sphere = Sphere2::Builder()
                      .WithCenter({ 0.5, 1.0 })
                      .WithRadius(0.1)
                      .MakeShared();

    auto collider =
        RigidBodyCollider2::Builder().WithSurface(sphere).MakeShared();

    solver->SetCollider(collider);

    for (Frame frame; frame.index < 240; ++frame)
    {
        solver->Update(frame);

        SaveData(solver->GetSmokeDensity()->DataView(), frame.index);
    }
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_TESTS(GridSmokeSolver3);

CUBBYFLOW_BEGIN_TEST_F(GridSmokeSolver3, Rising)
{
    size_t resolutionX = 50;

    // Build solver
    auto solver = GridSmokeSolver3::Builder()
                      .WithResolution(
                          { resolutionX, 6 * resolutionX / 5, resolutionX / 2 })
                      .WithDomainSizeX(1.0)
                      .MakeShared();

    solver->SetBuoyancyTemperatureFactor(2.0);

    // Build emitter
    auto box = Box3::Builder()
                   .WithLowerCorner({ 0.05, 0.1, 0.225 })
                   .WithUpperCorner({ 0.1, 0.15, 0.275 })
                   .MakeShared();

    auto emitter = VolumeGridEmitter3::Builder()
                       .WithSourceRegion(box)
                       .WithIsOneShot(false)
                       .MakeShared();

    solver->SetEmitter(emitter);
    emitter->AddStepFunctionTarget(solver->GetSmokeDensity(), 0, 1);
    emitter->AddStepFunctionTarget(solver->GetTemperature(), 0, 1);
    emitter->AddTarget(solver->GetVelocity(), [](double sdf, const Vector3D& pt,
                                                 const Vector3D& oldVal) {
        if (sdf < 0.05)
        {
            return Vector3D(0.5, oldVal.y, oldVal.z);
        }
        else
        {
            return Vector3D(oldVal);
        }
    });

    auto grids = solver->GetGridSystemData();
    Vector3UZ resolution = grids->Resolution();
    Array2<double> output(resolution.x, resolution.y);
    auto density = solver->GetSmokeDensity();
    char fileName[256];

    for (Frame frame(0, 1.0 / 60.0); frame.index < 240; ++frame)
    {
        solver->Update(frame);

        output.Fill(0.0);
        density->ForEachDataPointIndex(
            [&output, &density](size_t i, size_t j, size_t k) {
                output(i, j) += (*density)(i, j, k);
            });
        snprintf(fileName, sizeof(fileName), "data.#grid2,%04d.npy",
                 frame.index);
        SaveData(output.View(), fileName);
    }
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(GridSmokeSolver3, RisingWithCollider)
{
    size_t resolutionX = 50;

    // Build solver
    auto solver =
        GridSmokeSolver3::Builder()
            .WithResolution({ resolutionX, 2 * resolutionX, resolutionX })
            .WithDomainSizeX(1.0)
            .MakeShared();

    solver->SetAdvectionSolver(std::make_shared<CubicSemiLagrangian3>());

    auto grids = solver->GetGridSystemData();
    BoundingBox3D domain = grids->GetBoundingBox();

    // Build emitter
    auto box = Box3::Builder()
                   .WithLowerCorner({ 0.45, -1, 0.45 })
                   .WithUpperCorner({ 0.55, 0.05, 0.55 })
                   .MakeShared();

    auto emitter = VolumeGridEmitter3::Builder()
                       .WithSourceRegion(box)
                       .WithIsOneShot(false)
                       .MakeShared();

    solver->SetEmitter(emitter);
    emitter->AddStepFunctionTarget(solver->GetSmokeDensity(), 0, 1);
    emitter->AddStepFunctionTarget(solver->GetTemperature(), 0, 1);

    // Build collider
    auto sphere = Sphere3::Builder()
                      .WithCenter({ 0.5, 0.3, 0.5 })
                      .WithRadius(0.075 * domain.Width())
                      .MakeShared();

    auto collider =
        RigidBodyCollider3::Builder().WithSurface(sphere).MakeShared();

    solver->SetCollider(collider);

    Vector3UZ resolution = grids->Resolution();
    Array2<double> output(resolution.x, resolution.y);
    auto density = solver->GetSmokeDensity();
    char fileName[256];

    for (Frame frame(0, 1.0 / 60.0); frame.index < 240; ++frame)
    {
        solver->Update(frame);

        output.Fill(0.0);
        density->ForEachDataPointIndex(
            [&output, &density](size_t i, size_t j, size_t k) {
                output(i, j) += (*density)(i, j, k);
            });
        snprintf(fileName, sizeof(fileName), "data.#grid2,%04d.npy",
                 frame.index);
        SaveData(output.View(), fileName);
    }
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(GridSmokeSolver3, RisingWithColliderLinear)
{
    size_t resolutionX = 50;

    // Build solver
    auto solver =
        GridSmokeSolver3::Builder()
            .WithResolution({ resolutionX, 2 * resolutionX, resolutionX })
            .WithDomainSizeX(1.0)
            .MakeShared();

    solver->SetAdvectionSolver(std::make_shared<SemiLagrangian3>());

    auto grids = solver->GetGridSystemData();
    BoundingBox3D domain = grids->GetBoundingBox();

    // Build emitter
    auto box = Box3::Builder()
                   .WithLowerCorner({ 0.45, -1, 0.45 })
                   .WithUpperCorner({ 0.55, 0.05, 0.55 })
                   .MakeShared();

    auto emitter = VolumeGridEmitter3::Builder()
                       .WithSourceRegion(box)
                       .WithIsOneShot(false)
                       .MakeShared();

    solver->SetEmitter(emitter);
    emitter->AddStepFunctionTarget(solver->GetSmokeDensity(), 0, 1);
    emitter->AddStepFunctionTarget(solver->GetTemperature(), 0, 1);

    // Build collider
    auto sphere = Sphere3::Builder()
                      .WithCenter({ 0.5, 0.3, 0.5 })
                      .WithRadius(0.075 * domain.Width())
                      .MakeShared();

    auto collider =
        RigidBodyCollider3::Builder().WithSurface(sphere).MakeShared();

    solver->SetCollider(collider);

    Vector3UZ resolution = grids->Resolution();
    Array2<double> output(resolution.x, resolution.y);
    auto density = solver->GetSmokeDensity();
    char fileName[256];

    for (Frame frame(0, 1.0 / 60.0); frame.index < 240; ++frame)
    {
        solver->Update(frame);

        output.Fill(0.0);
        density->ForEachDataPointIndex(
            [&output, &density](size_t i, size_t j, size_t k) {
                output(i, j) += (*density)(i, j, k);
            });
        snprintf(fileName, sizeof(fileName), "data.#grid2,%04d.npy",
                 frame.index);
        SaveData(output.View(), fileName);
    }
}
CUBBYFLOW_END_TEST_F