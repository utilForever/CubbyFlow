#include "UnitTestsUtils.hpp"
#include "gtest/gtest.h"

#include <Core/Animation/Frame.hpp>
#include <Core/Emitter/VolumeParticleEmitter3.hpp>
#include <Core/Geometry/Sphere.hpp>
#include <Core/Geometry/SurfaceToImplicit.hpp>

using namespace CubbyFlow;

TEST(VolumeParticleEmitter3, Constructors)
{
    auto sphere = std::make_shared<SurfaceToImplicit3>(
        std::make_shared<Sphere3>(Vector3D(1.0, 2.0, 4.0), 3.0));

    BoundingBox3D region({ 0.0, 0.0, 0.0 }, { 3.0, 3.0, 3.0 });

    VolumeParticleEmitter3 emitter(sphere, region, 0.1, { -1.0, 0.5, 2.5 },
                                   { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 30,
                                   0.01, false, true);

    EXPECT_EQ(0.01, emitter.GetJitter());
    EXPECT_FALSE(emitter.GetIsOneShot());
    EXPECT_TRUE(emitter.GetAllowOverlapping());
    EXPECT_EQ(30u, emitter.GetMaxNumberOfParticles());
    EXPECT_EQ(0.1, emitter.GetSpacing());
    EXPECT_EQ(-1.0, emitter.GetInitialVelocity().x);
    EXPECT_EQ(0.5, emitter.GetInitialVelocity().y);
    EXPECT_EQ(2.5, emitter.GetInitialVelocity().z);
}

TEST(VolumeParticleEmitter3, Emit)
{
    auto sphere = std::make_shared<SurfaceToImplicit3>(
        std::make_shared<Sphere3>(Vector3D(1.0, 2.0, 4.0), 3.0));

    BoundingBox3D box({ 0.0, 0.0, 0.0 }, { 3.0, 3.0, 3.0 });

    VolumeParticleEmitter3 emitter(sphere, box, 0.5, { -1.0, 0.5, 2.5 },
                                   { 3.0, 4.0, 5.0 }, { 0.0, 0.0, 5.0 }, 30,
                                   0.0, false, false);

    auto particles = std::make_shared<ParticleSystemData3>();
    emitter.SetTarget(particles);

    Frame frame(0, 1.0);
    emitter.Update(frame.TimeInSeconds(), frame.timeIntervalInSeconds);

    auto pos = particles->Positions();
    auto vel = particles->Velocities();

    EXPECT_EQ(30u, particles->NumberOfParticles());
    for (size_t i = 0; i < particles->NumberOfParticles(); ++i)
    {
        EXPECT_GE(3.0, (pos[i] - Vector3D(1.0, 2.0, 4.0)).Length());
        EXPECT_TRUE(box.Contains(pos[i]));

        Vector3D r = pos[i];
        Vector3D w = 5.0 * Vector3D(-r.y, r.x, 0.0);
        Vector3D res = Vector3D(2.0, 4.5, 7.5) + w;
        EXPECT_VECTOR3_NEAR(res, vel[i], 1e-9);
    }

    ++frame;
    emitter.SetMaxNumberOfParticles(80);
    emitter.Update(frame.TimeInSeconds(), frame.timeIntervalInSeconds);

    EXPECT_EQ(79u, particles->NumberOfParticles());

    pos = particles->Positions();
    for (size_t i = 0; i < particles->NumberOfParticles(); ++i)
    {
        pos[i] += Vector3D(2.0, 1.5, 5.0);
    }

    ++frame;
    emitter.Update(frame.TimeInSeconds(), frame.timeIntervalInSeconds);
    EXPECT_LT(79u, particles->NumberOfParticles());
}

TEST(VolumeParticleEmitter3, Builder)
{
    auto sphere = std::make_shared<Sphere3>(Vector3D(1.0, 2.0, 4.0), 3.0);

    auto builder =
        VolumeParticleEmitter3::GetBuilder()
            .WithSurface(sphere)
            .WithMaxRegion(BoundingBox3D({ 0.0, 0.0, 0.0 }, { 3.0, 3.0, 3.0 }))
            .WithSpacing(0.1)
            .WithInitialVelocity({ -1.0, 0.5, 2.5 })
            .WithMaxNumberOfParticles(30)
            .WithJitter(0.01)
            .WithIsOneShot(false)
            .WithAllowOverlapping(true)
            .WithRandomSeed(42);
    VolumeParticleEmitter3 emitter = builder.Build();
    auto sharedEmitter = builder.MakeShared();

    EXPECT_EQ(0.01, emitter.GetJitter());
    EXPECT_FALSE(emitter.GetIsOneShot());
    EXPECT_TRUE(emitter.GetAllowOverlapping());
    EXPECT_EQ(30u, emitter.GetMaxNumberOfParticles());
    EXPECT_EQ(0.1, emitter.GetSpacing());
    EXPECT_EQ(-1.0, emitter.GetInitialVelocity().x);
    EXPECT_EQ(0.5, emitter.GetInitialVelocity().y);
    EXPECT_EQ(2.5, emitter.GetInitialVelocity().z);

    auto particles = std::make_shared<ParticleSystemData3>();
    auto sharedParticles = std::make_shared<ParticleSystemData3>();
    emitter.SetTarget(particles);
    sharedEmitter->SetTarget(sharedParticles);
    emitter.Update(0.0, 1.0);
    sharedEmitter->Update(0.0, 1.0);

    ASSERT_EQ(particles->NumberOfParticles(),
              sharedParticles->NumberOfParticles());
    for (size_t i = 0; i < particles->NumberOfParticles(); ++i)
    {
        EXPECT_VECTOR3_EQ(particles->Positions()[i],
                          sharedParticles->Positions()[i]);
    }
}
