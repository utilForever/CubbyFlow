#include "UnitTestsUtils.hpp"
#include "gtest/gtest.h"

#include <Core/Animation/Frame.hpp>
#include <Core/Emitter/VolumeParticleEmitter2.hpp>
#include <Core/Geometry/Sphere.hpp>
#include <Core/Geometry/SurfaceToImplicit.hpp>

using namespace CubbyFlow;

TEST(VolumeParticleEmitter2, Constructors)
{
    auto sphere = std::make_shared<SurfaceToImplicit2>(
        std::make_shared<Sphere2>(Vector2D(1.0, 2.0), 3.0));

    BoundingBox2D region({ 0.0, 0.0 }, { 3.0, 3.0 });

    VolumeParticleEmitter2 emitter(sphere, region, 0.1, { -1.0, 0.5 },
                                   { 0.0, 0.0 }, 0.0, 30, 0.01, false, true);

    EXPECT_EQ(0.01, emitter.GetJitter());
    EXPECT_FALSE(emitter.GetIsOneShot());
    EXPECT_TRUE(emitter.GetAllowOverlapping());
    EXPECT_EQ(30u, emitter.GetMaxNumberOfParticles());
    EXPECT_EQ(0.1, emitter.GetSpacing());
    EXPECT_EQ(-1.0, emitter.GetInitialVelocity().x);
    EXPECT_EQ(0.5, emitter.GetInitialVelocity().y);
}

TEST(VolumeParticleEmitter2, Emit)
{
    auto sphere = std::make_shared<SurfaceToImplicit2>(
        std::make_shared<Sphere2>(Vector2D(1.0, 2.0), 3.0));

    BoundingBox2D box({ 0.0, 0.0 }, { 3.0, 3.0 });

    VolumeParticleEmitter2 emitter(sphere, box, 0.35, { -1.0, 0.5 },
                                   { 3.0, 4.0 }, 5.0, 30, 0.0, false, false);

    auto particles = std::make_shared<ParticleSystemData2>();
    emitter.SetTarget(particles);

    Frame frame(0, 1.0);
    emitter.Update(frame.TimeInSeconds(), frame.timeIntervalInSeconds);

    auto pos = particles->Positions();
    auto vel = particles->Velocities();

    EXPECT_EQ(30u, particles->NumberOfParticles());
    for (size_t i = 0; i < particles->NumberOfParticles(); ++i)
    {
        EXPECT_GE(3.0, (pos[i] - Vector2D(1.0, 2.0)).Length());
        EXPECT_TRUE(box.Contains(pos[i]));

        Vector2D r = pos[i];
        Vector2D w = 5.0 * Vector2D(-r.y, r.x);
        Vector2D res = Vector2D(2.0, 4.5) + w;
        EXPECT_VECTOR2_NEAR(res, vel[i], 1e-9);
    }

    ++frame;
    emitter.SetMaxNumberOfParticles(60);
    emitter.Update(frame.TimeInSeconds(), frame.timeIntervalInSeconds);

    EXPECT_EQ(41u, particles->NumberOfParticles());

    pos = particles->Positions();
    for (size_t i = 0; i < particles->NumberOfParticles(); ++i)
    {
        pos[i] += Vector2D(2.0, 1.5);
    }

    ++frame;
    emitter.Update(frame.TimeInSeconds(), frame.timeIntervalInSeconds);
    EXPECT_LT(41u, particles->NumberOfParticles());
}

TEST(VolumeParticleEmitter2, Builder)
{
    auto sphere = std::make_shared<Sphere2>(Vector2D(1.0, 2.0), 3.0);

    VolumeParticleEmitter2 emitter =
        VolumeParticleEmitter2::GetBuilder()
            .WithSurface(sphere)
            .WithMaxRegion(BoundingBox2D({ 0.0, 0.0 }, { 3.0, 3.0 }))
            .WithSpacing(0.1)
            .WithInitialVelocity({ -1.0, 0.5 })
            .WithMaxNumberOfParticles(30)
            .WithJitter(0.01)
            .WithIsOneShot(false)
            .WithAllowOverlapping(true)
            .Build();

    EXPECT_EQ(0.01, emitter.GetJitter());
    EXPECT_FALSE(emitter.GetIsOneShot());
    EXPECT_TRUE(emitter.GetAllowOverlapping());
    EXPECT_EQ(30u, emitter.GetMaxNumberOfParticles());
    EXPECT_EQ(0.1, emitter.GetSpacing());
    EXPECT_EQ(-1.0, emitter.GetInitialVelocity().x);
    EXPECT_EQ(0.5, emitter.GetInitialVelocity().y);
}