#include "gtest/gtest.h"

#include <Core/Geometry/ImplicitSurfaceSet.hpp>
#include <Core/Geometry/Plane.hpp>
#include <Core/Geometry/RigidBodyCollider.hpp>

using namespace CubbyFlow;

TEST(RigidBodyCollider2, ResolveCollision)
{
    // Exact surface with zero radius and closing velocity
    {
        RigidBodyCollider2 collider(
            std::make_shared<Plane2>(Vector2D{ 0.0, 1.0 }, Vector2D{}));
        Vector2D position{ 1.0, 0.0 };
        Vector2D velocity{ 1.0, -1.0 };

        collider.ResolveCollision(0.0, 0.0, &position, &velocity);

        EXPECT_EQ(position, Vector2D(1.0, 0.0));
        EXPECT_EQ(velocity, Vector2D(1.0, 0.0));
    }

    // 1. No penetration
    {
        RigidBodyCollider2 collider(
            std::make_shared<Plane2>(Vector2D(0, 1), Vector2D(0, 0)));

        Vector2D newPosition(1, 0.1);
        Vector2D newVelocity(1, 0);
        double radius = 0.05;
        double restitutionCoefficient = 0.5;

        collider.ResolveCollision(radius, restitutionCoefficient, &newPosition,
                                  &newVelocity);

        EXPECT_DOUBLE_EQ(1.0, newPosition.x);
        EXPECT_DOUBLE_EQ(0.1, newPosition.y);
        EXPECT_DOUBLE_EQ(1.0, newVelocity.x);
        EXPECT_DOUBLE_EQ(0.0, newVelocity.y);
    }

    // 2. Penetration within radius
    {
        RigidBodyCollider2 collider(
            std::make_shared<Plane2>(Vector2D(0, 1), Vector2D(0, 0)));

        Vector2D newPosition(1, 0.1);
        Vector2D newVelocity(1, 0);
        double radius = 0.2;
        double restitutionCoefficient = 0.5;

        collider.ResolveCollision(radius, restitutionCoefficient, &newPosition,
                                  &newVelocity);

        EXPECT_DOUBLE_EQ(1.0, newPosition.x);
        EXPECT_DOUBLE_EQ(0.2, newPosition.y);
    }

    // 3. Sitting
    {
        RigidBodyCollider2 collider(
            std::make_shared<Plane2>(Vector2D(0, 1), Vector2D(0, 0)));

        Vector2D newPosition(1, 0.1);
        Vector2D newVelocity(1, 0);
        double radius = 0.1;
        double restitutionCoefficient = 0.5;

        collider.ResolveCollision(radius, restitutionCoefficient, &newPosition,
                                  &newVelocity);

        EXPECT_DOUBLE_EQ(1.0, newPosition.x);
        EXPECT_DOUBLE_EQ(0.1, newPosition.y);
        EXPECT_DOUBLE_EQ(1.0, newVelocity.x);
        EXPECT_DOUBLE_EQ(0.0, newVelocity.y);
    }

    // 4. Bounce-back
    {
        RigidBodyCollider2 collider(
            std::make_shared<Plane2>(Vector2D(0, 1), Vector2D(0, 0)));

        Vector2D newPosition(1, -1);
        Vector2D newVelocity(1, -1);
        double radius = 0.1;
        double restitutionCoefficient = 0.5;

        collider.ResolveCollision(radius, restitutionCoefficient, &newPosition,
                                  &newVelocity);

        EXPECT_DOUBLE_EQ(1.0, newPosition.x);
        EXPECT_DOUBLE_EQ(0.1, newPosition.y);
        EXPECT_DOUBLE_EQ(1.0, newVelocity.x);
        EXPECT_DOUBLE_EQ(restitutionCoefficient, newVelocity.y);
    }

    // 4. Friction
    {
        RigidBodyCollider2 collider(
            std::make_shared<Plane2>(Vector2D(0, 1), Vector2D(0, 0)));

        Vector2D newPosition(1, -1);
        Vector2D newVelocity(1, -1);
        double radius = 0.1;
        double restitutionCoefficient = 0.5;

        collider.SetFrictionCoefficient(0.1);

        collider.ResolveCollision(radius, restitutionCoefficient, &newPosition,
                                  &newVelocity);

        EXPECT_DOUBLE_EQ(1.0, newPosition.x);
        EXPECT_DOUBLE_EQ(0.1, newPosition.y);
        EXPECT_GT(1.0, newVelocity.x);
        EXPECT_DOUBLE_EQ(restitutionCoefficient, newVelocity.y);
    }
}

TEST(RigidBodyCollider2, VelocityAt)
{
    RigidBodyCollider2 collider(
        std::make_shared<Plane2>(Vector2D(0, 1), Vector2D(0, 0)));

    collider.GetSurface()->transform.SetTranslation({ -1, -2 });
    collider.GetSurface()->transform.SetOrientation(0.1);
    collider.linearVelocity = { 1, 3 };
    collider.angularVelocity.value = 4.0;

    Vector2D result = collider.VelocityAt({ 5, 7 });
    EXPECT_DOUBLE_EQ(-35.0, result.x);
    EXPECT_DOUBLE_EQ(27.0, result.y);
}

TEST(RigidBodyCollider2, Empty)
{
    RigidBodyCollider2 collider{ ImplicitSurfaceSet2::Builder().MakeShared() };

    Vector2D newPosition{ 1, 0.1 };
    Vector2D newVelocity{ 1, 0 };
    const double radius = 0.05;
    const double restitutionCoefficient = 0.5;

    collider.ResolveCollision(radius, restitutionCoefficient, &newPosition,
                              &newVelocity);

    EXPECT_DOUBLE_EQ(1.0, newPosition.x);
    EXPECT_DOUBLE_EQ(0.1, newPosition.y);
    EXPECT_DOUBLE_EQ(1.0, newVelocity.x);
    EXPECT_DOUBLE_EQ(0.0, newVelocity.y);
}
