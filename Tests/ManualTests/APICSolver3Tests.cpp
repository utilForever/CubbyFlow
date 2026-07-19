#include "gtest/gtest.h"

#include <ManualTests.hpp>

#include <Core/Emitter/ParticleEmitterSet3.hpp>
#include <Core/Emitter/VolumeParticleEmitter3.hpp>
#include <Core/Geometry/Box.hpp>
#include <Core/Geometry/Cylinder3.hpp>
#include <Core/Geometry/ImplicitSurfaceSet.hpp>
#include <Core/Geometry/Plane.hpp>
#include <Core/Geometry/RigidBodyCollider.hpp>
#include <Core/Geometry/Sphere.hpp>
#include <Core/PointGenerator/GridPointGenerator3.hpp>
#include <Core/Solver/Grid/GridSinglePhasePressureSolver3.hpp>
#include <Core/Solver/Hybrid/APIC/APICSolver3.hpp>
#include <Core/Solver/Hybrid/PIC/PICSolver3.hpp>

using namespace CubbyFlow;

CUBBYFLOW_TESTS(APICSolver3);

CUBBYFLOW_BEGIN_TEST_F(APICSolver3, WaterDrop)
{
    size_t resolutionX = 32;

    // Build solver
    auto solver =
        APICSolver3::Builder()
            .WithResolution({ resolutionX, 2 * resolutionX, resolutionX })
            .WithDomainSizeX(1.0)
            .MakeShared();

    auto grids = solver->GetGridSystemData();
    auto particles = solver->GetParticleSystemData();

    Vector3D gridSpacing = grids->GridSpacing();
    double dx = gridSpacing.x;
    BoundingBox3D domain = grids->GetBoundingBox();

    // Build emitter
    auto plane = Plane3::Builder()
                     .WithNormal({ 0, 1, 0 })
                     .WithPoint({ 0, 0.25 * domain.Height(), 0 })
                     .MakeShared();

    auto sphere = Sphere3::Builder()
                      .WithCenter(domain.MidPoint())
                      .WithRadius(0.15 * domain.Width())
                      .MakeShared();

    auto emitter1 = VolumeParticleEmitter3::Builder()
                        .WithSurface(plane)
                        .WithSpacing(0.5 * dx)
                        .WithMaxRegion(domain)
                        .WithIsOneShot(true)
                        .MakeShared();
    emitter1->SetPointGenerator(std::make_shared<GridPointGenerator3>());

    auto emitter2 = VolumeParticleEmitter3::Builder()
                        .WithSurface(sphere)
                        .WithSpacing(0.5 * dx)
                        .WithMaxRegion(domain)
                        .WithIsOneShot(true)
                        .MakeShared();
    emitter2->SetPointGenerator(std::make_shared<GridPointGenerator3>());

    auto emitterSet = ParticleEmitterSet3::Builder()
                          .WithEmitters({ emitter1, emitter2 })
                          .MakeShared();

    solver->SetParticleEmitter(emitterSet);

    for (Frame frame; frame.index < 120; ++frame)
    {
        solver->Update(frame);

        SaveParticleDataXY(solver->GetParticleSystemData(), frame.index);
    }
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(APICSolver3, DamBreakingWithCollider)
{
    size_t resolutionX = 50;

    // Build solver
    Vector3UZ resolution{ 3 * resolutionX, 2 * resolutionX,
                          (3 * resolutionX) / 2 };
    auto solver = APICSolver3::Builder()
                      .WithResolution(resolution)
                      .WithDomainSizeX(3.0)
                      .MakeShared();
    solver->SetUseCompressedLinearSystem(true);

    auto grids = solver->GetGridSystemData();
    double dx = grids->GridSpacing().x;
    BoundingBox3D domain = grids->GetBoundingBox();
    double lz = domain.Depth();

    // Build emitter
    auto box1 =
        Box3::Builder()
            .WithLowerCorner({ 0, 0, 0 })
            .WithUpperCorner({ 0.5 + 0.001, 0.75 + 0.001, 0.75 * lz + 0.001 })
            .MakeShared();

    auto box2 =
        Box3::Builder()
            .WithLowerCorner({ 2.5 - 0.001, 0, 0.25 * lz - 0.001 })
            .WithUpperCorner({ 3.5 + 0.001, 0.75 + 0.001, 1.5 * lz + 0.001 })
            .MakeShared();

    auto boxSet = ImplicitSurfaceSet3::Builder()
                      .WithExplicitSurfaces(Array1<Surface3Ptr>{ box1, box2 })
                      .MakeShared();

    auto emitter = VolumeParticleEmitter3::Builder()
                       .WithSurface(boxSet)
                       .WithMaxRegion(domain)
                       .WithSpacing(0.5 * dx)
                       .MakeShared();

    emitter->SetPointGenerator(std::make_shared<GridPointGenerator3>());
    solver->SetParticleEmitter(emitter);

    // Build collider
    auto cyl1 = Cylinder3::Builder()
                    .WithCenter({ 1, 0.375, 0.375 })
                    .WithRadius(0.1)
                    .WithHeight(0.75)
                    .MakeShared();

    auto cyl2 = Cylinder3::Builder()
                    .WithCenter({ 1.5, 0.375, 0.75 })
                    .WithRadius(0.1)
                    .WithHeight(0.75)
                    .MakeShared();

    auto cyl3 = Cylinder3::Builder()
                    .WithCenter({ 2, 0.375, 1.125 })
                    .WithRadius(0.1)
                    .WithHeight(0.75)
                    .MakeShared();

    auto cylSet =
        ImplicitSurfaceSet3::Builder()
            .WithExplicitSurfaces(Array1<Surface3Ptr>{ cyl1, cyl2, cyl3 })
            .MakeShared();

    auto collider =
        RigidBodyCollider3::Builder().WithSurface(cylSet).MakeShared();

    solver->SetCollider(collider);

    // Run simulation
    for (Frame frame; frame.index < 200; ++frame)
    {
        solver->Update(frame);

        SaveParticleDataXY(solver->GetParticleSystemData(), frame.index);
    }
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(APICSolver3, Spherical)
{
    // Build solver
    auto solver = APICSolver3::Builder()
                      .WithResolution({ 30, 30, 30 })
                      .WithDomainSizeX(1.0)
                      .MakeShared();
    solver->SetUseCompressedLinearSystem(true);

    // Build collider
    auto sphere = Sphere3::Builder()
                      .WithCenter({ 0.5, 0.5, 0.5 })
                      .WithRadius(0.4)
                      .WithIsNormalFlipped(true)
                      .MakeShared();

    auto collider =
        RigidBodyCollider3::Builder().WithSurface(sphere).MakeShared();

    solver->SetCollider(collider);

    // Manually emit particles
    size_t resX = solver->GetResolution().x;
    std::mt19937 rng;
    std::uniform_real_distribution<> dist(0, 1);

    for (size_t i = 0; i < 8 * resX * resX * resX; ++i)
    {
        Vector3D pt{ dist(rng), dist(rng), dist(rng) };

        if ((pt - sphere->center).Length() < sphere->radius && pt.x > 0.5)
        {
            solver->GetParticleSystemData()->AddParticle(pt);
        }
    }

    for (Frame frame(0, 0.01); frame.index < 240; ++frame)
    {
        solver->Update(frame);

        SaveParticleDataXY(solver->GetParticleSystemData(), frame.index);
    }
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(APICSolver3, SphericalNonVariational)
{
    // Build solver
    auto solver = APICSolver3::Builder()
                      .WithResolution({ 30, 30, 30 })
                      .WithDomainSizeX(1.0)
                      .MakeShared();
    solver->SetUseCompressedLinearSystem(true);

    solver->SetPressureSolver(
        std::make_shared<GridSinglePhasePressureSolver3>());

    // Build collider
    auto sphere = Sphere3::Builder()
                      .WithCenter({ 0.5, 0.5, 0.5 })
                      .WithRadius(0.4)
                      .WithIsNormalFlipped(true)
                      .MakeShared();

    auto collider =
        RigidBodyCollider3::Builder().WithSurface(sphere).MakeShared();

    solver->SetCollider(collider);

    // Manually emit particles
    size_t resX = solver->GetResolution().x;
    std::mt19937 rng;
    std::uniform_real_distribution<> dist(0, 1);

    for (size_t i = 0; i < 8 * resX * resX * resX; ++i)
    {
        Vector3D pt{ dist(rng), dist(rng), dist(rng) };

        if ((pt - sphere->center).Length() < sphere->radius && pt.x > 0.5)
        {
            solver->GetParticleSystemData()->AddParticle(pt);
        }
    }

    for (Frame frame(0, 0.01); frame.index < 240; ++frame)
    {
        solver->Update(frame);

        SaveParticleDataXY(solver->GetParticleSystemData(), frame.index);
    }
}
CUBBYFLOW_END_TEST_F
