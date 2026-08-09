#include "gtest/gtest.h"

#include <Core/Geometry/ImplicitSurfaceSet.hpp>
#include <Core/Geometry/Plane.hpp>
#include <Core/Geometry/RigidBodyCollider.hpp>

using namespace CubbyFlow;

TEST(RigidBodyCollider3, ResolveCollision)
{
    // Exact surface with zero radius and closing velocity
    {
        RigidBodyCollider3 collider(
            std::make_shared<Plane3>(Vector3D{ 0.0, 1.0, 0.0 }, Vector3D{}));
        Vector3D position{ 1.0, 0.0, 2.0 };
        Vector3D velocity{ 1.0, -1.0, 3.0 };

        collider.ResolveCollision(0.0, 0.0, &position, &velocity);

        EXPECT_EQ(position, Vector3D(1.0, 0.0, 2.0));
        EXPECT_EQ(velocity, Vector3D(1.0, 0.0, 3.0));
    }

    // 1. No penetration
    {
        RigidBodyCollider3 collider(
            std::make_shared<Plane3>(Vector3D(0, 1, 0), Vector3D(0, 0, 0)));

        Vector3D newPosition(1, 0.1, 0);
        Vector3D newVelocity(1, 0, 0);
        double radius = 0.05;
        double restitutionCoefficient = 0.5;

        collider.ResolveCollision(radius, restitutionCoefficient, &newPosition,
                                  &newVelocity);

        EXPECT_DOUBLE_EQ(1.0, newPosition.x);
        EXPECT_DOUBLE_EQ(0.1, newPosition.y);
        EXPECT_DOUBLE_EQ(0.0, newPosition.z);
        EXPECT_DOUBLE_EQ(1.0, newVelocity.x);
        EXPECT_DOUBLE_EQ(0.0, newVelocity.y);
        EXPECT_DOUBLE_EQ(0.0, newVelocity.z);
    }

    // 2. Penetration within radius
    {
        RigidBodyCollider3 collider(
            std::make_shared<Plane3>(Vector3D(0, 1, 0), Vector3D(0, 0, 0)));

        Vector3D newPosition(1, 0.1, 0);
        Vector3D newVelocity(1, 0, 0);
        double radius = 0.2;
        double restitutionCoefficient = 0.5;

        collider.ResolveCollision(radius, restitutionCoefficient, &newPosition,
                                  &newVelocity);

        EXPECT_DOUBLE_EQ(1.0, newPosition.x);
        EXPECT_DOUBLE_EQ(0.2, newPosition.y);
        EXPECT_DOUBLE_EQ(0.0, newPosition.z);
    }

    // 3. Sitting
    {
        RigidBodyCollider3 collider(
            std::make_shared<Plane3>(Vector3D(0, 1, 0), Vector3D(0, 0, 0)));

        Vector3D newPosition(1, 0.1, 0);
        Vector3D newVelocity(1, 0, 0);
        double radius = 0.1;
        double restitutionCoefficient = 0.5;

        collider.ResolveCollision(radius, restitutionCoefficient, &newPosition,
                                  &newVelocity);

        EXPECT_DOUBLE_EQ(1.0, newPosition.x);
        EXPECT_DOUBLE_EQ(0.1, newPosition.y);
        EXPECT_DOUBLE_EQ(0.0, newPosition.z);
        EXPECT_DOUBLE_EQ(1.0, newVelocity.x);
        EXPECT_DOUBLE_EQ(0.0, newVelocity.y);
        EXPECT_DOUBLE_EQ(0.0, newVelocity.z);
    }

    // 4. Bounce-back
    {
        RigidBodyCollider3 collider(
            std::make_shared<Plane3>(Vector3D(0, 1, 0), Vector3D(0, 0, 0)));

        Vector3D newPosition(1, -1, 0);
        Vector3D newVelocity(1, -1, 0);
        double radius = 0.1;
        double restitutionCoefficient = 0.5;

        collider.ResolveCollision(radius, restitutionCoefficient, &newPosition,
                                  &newVelocity);

        EXPECT_DOUBLE_EQ(1.0, newPosition.x);
        EXPECT_DOUBLE_EQ(0.1, newPosition.y);
        EXPECT_DOUBLE_EQ(0.0, newPosition.z);
        EXPECT_DOUBLE_EQ(1.0, newVelocity.x);
        EXPECT_DOUBLE_EQ(restitutionCoefficient, newVelocity.y);
        EXPECT_DOUBLE_EQ(0.0, newVelocity.z);
    }

    // 4. Friction
    {
        RigidBodyCollider3 collider(
            std::make_shared<Plane3>(Vector3D(0, 1, 0), Vector3D(0, 0, 0)));

        Vector3D newPosition(1, -1, 0);
        Vector3D newVelocity(1, -1, 0);
        double radius = 0.1;
        double restitutionCoefficient = 0.5;

        collider.SetFrictionCoefficient(0.1);

        collider.ResolveCollision(radius, restitutionCoefficient, &newPosition,
                                  &newVelocity);

        EXPECT_DOUBLE_EQ(1.0, newPosition.x);
        EXPECT_DOUBLE_EQ(0.1, newPosition.y);
        EXPECT_DOUBLE_EQ(0.0, newPosition.z);
        EXPECT_GT(1.0, newVelocity.x);
        EXPECT_DOUBLE_EQ(restitutionCoefficient, newVelocity.y);
        EXPECT_DOUBLE_EQ(0.0, newVelocity.z);
    }
}

TEST(RigidBodyCollider3, VelocityAt)
{
    RigidBodyCollider3 collider(
        std::make_shared<Plane3>(Vector3D(0, 1, 0), Vector3D(0, 0, 0)));

    collider.linearVelocity = { 1, 3, -2 };
    collider.angularVelocity.value = { 0, 0, 4 };
    collider.GetSurface()->transform.SetTranslation({ -1, -2, 2 });
    collider.GetSurface()->transform.SetOrientation(
        QuaternionD({ 1, 0, 0 }, 0.1));

    Vector3D result = collider.VelocityAt({ 5, 7, 8 });
    EXPECT_DOUBLE_EQ(-35.0, result.x);
    EXPECT_DOUBLE_EQ(27.0, result.y);
    EXPECT_DOUBLE_EQ(-2.0, result.z);
}

TEST(RigidBodyCollider3, Empty)
{
    RigidBodyCollider3 collider{ ImplicitSurfaceSet3::Builder().MakeShared() };

    Vector3D newPosition{ 1, 0.1, 0 };
    Vector3D newVelocity{ 1, 0, 0 };
    const double radius = 0.05;
    const double restitutionCoefficient = 0.5;

    collider.ResolveCollision(radius, restitutionCoefficient, &newPosition,
                              &newVelocity);

    EXPECT_DOUBLE_EQ(1.0, newPosition.x);
    EXPECT_DOUBLE_EQ(0.1, newPosition.y);
    EXPECT_DOUBLE_EQ(0.0, newPosition.z);
    EXPECT_DOUBLE_EQ(1.0, newVelocity.x);
    EXPECT_DOUBLE_EQ(0.0, newVelocity.y);
    EXPECT_DOUBLE_EQ(0.0, newVelocity.z);
}
