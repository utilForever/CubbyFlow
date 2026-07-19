#include "gtest/gtest.h"

#include <ManualTests.hpp>

#include <Core/Emitter/VolumeParticleEmitter2.hpp>
#include <Core/Geometry/Box.hpp>
#include <Core/Geometry/RigidBodyCollider.hpp>
#include <Core/Geometry/Sphere.hpp>
#include <Core/Solver/Hybrid/APIC/APICSolver2.hpp>
#include <Core/Solver/Hybrid/PIC/PICSolver2.hpp>

using namespace CubbyFlow;

CUBBYFLOW_TESTS(APICSolver2);

CUBBYFLOW_BEGIN_TEST_F(APICSolver2, SteadyState)
{
    // Build solver
    auto solver = APICSolver2::Builder()
                      .WithResolution({ 32, 32 })
                      .WithDomainSizeX(1.0)
                      .MakeShared();

    // Build emitter
    auto box = Box2::Builder()
                   .WithLowerCorner({ 0.0, 0.0 })
                   .WithUpperCorner({ 1.0, 0.5 })
                   .MakeShared();

    auto emitter = VolumeParticleEmitter2::Builder()
                       .WithSurface(box)
                       .WithSpacing(1.0 / 64.0)
                       .WithIsOneShot(true)
                       .MakeShared();

    solver->SetParticleEmitter(emitter);

    for (Frame frame; frame.index < 120; ++frame)
    {
        solver->Update(frame);

        SaveParticleDataXY(solver->GetParticleSystemData(), frame.index);
    }
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(APICSolver2, Rotation)
{
    // Build solver
    auto solver = APICSolver2::Builder()
                      .WithResolution({ 10, 10 })
                      .WithDomainSizeX(1.0)
                      .MakeShared();

    solver->SetGravity({ 0, 0 });

    // Build emitter
    auto box = Sphere2::Builder()
                   .WithCenter({ 0.5, 0.5 })
                   .WithRadius(0.4)
                   .MakeShared();

    auto emitter = VolumeParticleEmitter2::Builder()
                       .WithSurface(box)
                       .WithSpacing(1.0 / 20.0)
                       .WithIsOneShot(true)
                       .MakeShared();

    solver->SetParticleEmitter(emitter);

    Array1<double> r;

    for (Frame frame; frame.index < 360; ++frame)
    {
        auto x = solver->GetParticleSystemData()->Positions();
        auto v = solver->GetParticleSystemData()->Velocities();
        r.Resize(x.Size());

        for (size_t i = 0; i < x.Length(); ++i)
        {
            r[i] = (x[i] - Vector2D(0.5, 0.5)).Length();
        }

        solver->Update(frame);

        if (frame.index == 0)
        {
            x = solver->GetParticleSystemData()->Positions();
            v = solver->GetParticleSystemData()->Velocities();

            for (size_t i = 0; i < x.Length(); ++i)
            {
                Vector2D rp = x[i] - Vector2D(0.5, 0.5);
                v[i].x = rp.y;
                v[i].y = -rp.x;
            }
        }
        else
        {
            for (size_t i = 0; i < x.Length(); ++i)
            {
                Vector2D rp = x[i] - Vector2D(0.5, 0.5);

                if (rp.LengthSquared() > 0.0)
                {
                    double scale = r[i] / rp.Length();
                    x[i] = scale * rp + Vector2D(0.5, 0.5);
                }
            }
        }

        SaveParticleDataXY(solver->GetParticleSystemData(), frame.index);
    }
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(APICSolver2, DamBreaking)
{
    // Build solver
    auto solver = APICSolver2::Builder()
                      .WithResolution({ 100, 100 })
                      .WithDomainSizeX(1.0)
                      .MakeShared();

    // Build emitter
    auto box = Box2::Builder()
                   .WithLowerCorner({ 0.0, 0.0 })
                   .WithUpperCorner({ 0.2, 0.8 })
                   .MakeShared();

    auto emitter = VolumeParticleEmitter2::Builder()
                       .WithSurface(box)
                       .WithSpacing(0.005)
                       .WithIsOneShot(true)
                       .MakeShared();

    solver->SetParticleEmitter(emitter);

    for (Frame frame; frame.index < 240; ++frame)
    {
        solver->Update(frame);

        SaveParticleDataXY(solver->GetParticleSystemData(), frame.index);
    }
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(APICSolver2, LeftWall)
{
    // Build solver
    auto solver = APICSolver2::Builder()
                      .WithResolution({ 32, 32 })
                      .WithDomainSizeX(1.0)
                      .MakeShared();

    // Build emitter
    auto box = Box2::Builder()
                   .WithLowerCorner({ 0.0, 0.0 })
                   .WithUpperCorner({ 0.01, 0.5 })
                   .MakeShared();

    auto emitter = VolumeParticleEmitter2::Builder()
                       .WithSurface(box)
                       .WithSpacing(1.0 / 64.0)
                       .WithIsOneShot(true)
                       .MakeShared();

    solver->SetParticleEmitter(emitter);

    for (Frame frame; frame.index < 100; ++frame)
    {
        solver->Update(frame);

        SaveParticleDataXY(solver->GetParticleSystemData(), frame.index);
    }
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(APICSolver2, RightWall)
{
    // Build solver
    auto solver = APICSolver2::Builder()
                      .WithResolution({ 32, 32 })
                      .WithDomainSizeX(1.0)
                      .MakeShared();

    // Build emitter
    auto box = Box2::Builder()
                   .WithLowerCorner({ 1.0 - 1.0 / 64.0, 0.0 })
                   .WithUpperCorner({ 1.0, 0.5 })
                   .MakeShared();

    auto emitter = VolumeParticleEmitter2::Builder()
                       .WithSurface(box)
                       .WithSpacing(1.0 / 64.0)
                       .WithIsOneShot(true)
                       .MakeShared();

    solver->SetParticleEmitter(emitter);

    for (Frame frame; frame.index < 10; ++frame)
    {
        solver->Update(frame);

        SaveParticleDataXY(solver->GetParticleSystemData(), frame.index);
    }
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(APICSolver2, LeftWallPic)
{
    // Build solver
    auto solver = PICSolver2::Builder()
                      .WithResolution({ 32, 32 })
                      .WithDomainSizeX(1.0)
                      .MakeShared();

    // Build emitter
    auto box = Box2::Builder()
                   .WithLowerCorner({ 0.0, 0.0 })
                   .WithUpperCorner({ 0.01, 0.5 })
                   .MakeShared();

    auto emitter = VolumeParticleEmitter2::Builder()
                       .WithSurface(box)
                       .WithSpacing(1.0 / 64.0)
                       .WithIsOneShot(true)
                       .MakeShared();

    solver->SetParticleEmitter(emitter);

    for (Frame frame; frame.index < 100; ++frame)
    {
        solver->Update(frame);

        SaveParticleDataXY(solver->GetParticleSystemData(), frame.index);
    }
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(APICSolver2, DamBreakingWithCollider)
{
    // Build solver
    auto solver = APICSolver2::Builder()
                      .WithResolution({ 100, 100 })
                      .WithDomainSizeX(1.0)
                      .MakeShared();

    // Build emitter
    auto box = Box2::Builder()
                   .WithLowerCorner({ 0.0, 0.0 })
                   .WithUpperCorner({ 0.2, 0.8 })
                   .MakeShared();

    auto emitter = VolumeParticleEmitter2::Builder()
                       .WithSurface(box)
                       .WithSpacing(0.005)
                       .WithIsOneShot(true)
                       .MakeShared();

    solver->SetParticleEmitter(emitter);

    // Build collider
    auto sphere = Sphere2::Builder()
                      .WithCenter({ 0.5, 0.0 })
                      .WithRadius(0.15)
                      .MakeShared();

    auto collider =
        RigidBodyCollider2::Builder().WithSurface(sphere).MakeShared();

    solver->SetCollider(collider);

    for (Frame frame(0, 1.0 / 60.0); frame.index < 240; ++frame)
    {
        solver->Update(frame);

        SaveParticleDataXY(solver->GetParticleSystemData(), frame.index);
    }
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(APICSolver2, Circular)
{
    // Build solver
    auto solver = APICSolver2::Builder()
                      .WithResolution({ 40, 40 })
                      .WithDomainSizeX(1.0)
                      .MakeShared();

    // Build collider
    auto sphere = Sphere2::Builder()
                      .WithCenter({ 0.5, 0.5 })
                      .WithRadius(0.4)
                      .WithIsNormalFlipped(true)
                      .MakeShared();

    auto collider =
        RigidBodyCollider2::Builder().WithSurface(sphere).MakeShared();

    solver->SetCollider(collider);

    // Manually emit particles
    size_t resX = solver->GetResolution().x;
    std::mt19937 rng;
    std::uniform_real_distribution<> dist(0, 1);

    for (size_t i = 0; i < 4 * resX * resX; ++i)
    {
        Vector2D pt{ dist(rng), dist(rng) };

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

CUBBYFLOW_BEGIN_TEST_F(APICSolver2, CircularWithFriction)
{
    // Build solver
    auto solver = APICSolver2::Builder()
                      .WithResolution({ 40, 40 })
                      .WithDomainSizeX(1.0)
                      .MakeShared();

    // Build collider
    auto sphere = Sphere2::Builder()
                      .WithCenter({ 0.5, 0.5 })
                      .WithRadius(0.4)
                      .WithIsNormalFlipped(true)
                      .MakeShared();

    auto collider =
        RigidBodyCollider2::Builder().WithSurface(sphere).MakeShared();

    // Sticky boundary
    collider->SetFrictionCoefficient(100.0);

    solver->SetCollider(collider);

    // Manually emit particles
    size_t resX = solver->GetResolution().x;
    std::mt19937 rng;
    std::uniform_real_distribution<> dist(0, 1);

    for (size_t i = 0; i < 4 * resX * resX; ++i)
    {
        Vector2D pt{ dist(rng), dist(rng) };

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
