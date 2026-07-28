#include "UnitTestsUtils.hpp"
#include "gtest/gtest.h"

#include <Core/Geometry/CustomImplicitSurface.hpp>
#include <Core/Geometry/Sphere.hpp>
#include <Core/Geometry/SurfaceToImplicit.hpp>

using namespace CubbyFlow;

TEST(CustomImplicitSurface2, SignedDistance)
{
    CustomImplicitSurface2 cis(
        [](const Vector2D& pt) {
            return (pt - Vector2D(0.5, 0.5)).Length() - 0.25;
        },
        BoundingBox2D({ 0, 0 }, { 1, 1 }), 1e-3);

    EXPECT_DOUBLE_EQ(0.25, cis.SignedDistance({ 1, 0.5 }));
    EXPECT_DOUBLE_EQ(-0.25, cis.SignedDistance({ 0.5, 0.5 }));
    EXPECT_DOUBLE_EQ(0.0, cis.SignedDistance({ 0.5, 0.75 }));
}

TEST(CustomImplicitSurface2, ClosestPoint)
{
    auto sphere = Sphere2::Builder()
                      .WithCenter({ 0.5, 0.45 })
                      .WithRadius(0.3)
                      .MakeShared();
    SurfaceToImplicit2 refSurf(sphere);
    CustomImplicitSurface2 cis1(
        [&refSurf](const Vector2D& pt) { return refSurf.SignedDistance(pt); },
        BoundingBox2D({ 0, 0 }, { 1, 1 }), 1e-3);

    for (size_t i = 0; i < GetNumberOfSamplePoints2(); ++i)
    {
        auto sample = GetSamplePoints2()[i];
        if ((sample - sphere->center).Length() > 0.01)
        {
            auto refAns = refSurf.ClosestPoint(sample);
            auto actAns = cis1.ClosestPoint(sample);

            EXPECT_VECTOR2_NEAR(refAns, actAns, 1e-3);
        }
    }
}

TEST(CustomImplicitSurface2, ClosestNormal)
{
    auto sphere = Sphere2::Builder()
                      .WithCenter({ 0.5, 0.45 })
                      .WithRadius(0.3)
                      .MakeShared();
    SurfaceToImplicit2 refSurf(sphere);
    CustomImplicitSurface2 cis1(
        [&refSurf](const Vector2D& pt) { return refSurf.SignedDistance(pt); },
        BoundingBox2D({ 0, 0 }, { 1, 1 }), 1e-3);

    for (size_t i = 0; i < GetNumberOfSamplePoints2(); ++i)
    {
        auto sample = GetSamplePoints2()[i];
        auto refAns = refSurf.ClosestNormal(sample);
        auto actAns = cis1.ClosestNormal(sample);

        EXPECT_VECTOR2_NEAR(refAns, actAns, 1e-3);
    }
}

TEST(CustomImplicitSurface2, Intersects)
{
    auto sphere = Sphere2::Builder()
                      .WithCenter({ 0.5, 0.45 })
                      .WithRadius(0.3)
                      .MakeShared();
    SurfaceToImplicit2 refSurf(sphere);
    CustomImplicitSurface2 cis1(
        [&refSurf](const Vector2D& pt) { return refSurf.SignedDistance(pt); },
        BoundingBox2D({ 0, 0 }, { 1, 1 }), 1e-3);

    for (size_t i = 0; i < GetNumberOfSamplePoints2(); ++i)
    {
        auto x = GetSamplePoints2()[i];
        auto d = GetSampleDirs2()[i];
        bool refAns = refSurf.Intersects(Ray2D(x, d));
        bool actAns = cis1.Intersects(Ray2D(x, d));

        EXPECT_EQ(refAns, actAns);
    }
}

TEST(CustomImplicitSurface2, ClosestIntersection)
{
    const auto sphere = Sphere2::Builder{}
                            .WithCenter({ 0.5, 0.45 })
                            .WithRadius(0.3)
                            .MakeShared();
    SurfaceToImplicit2 refSurf{ sphere };
    const CustomImplicitSurface2 cis1{
        [&refSurf](const Vector2D& pt) { return refSurf.SignedDistance(pt); },
        BoundingBox2D{ { 0, 0 }, { 1, 1 } }, 1e-3, 1e-3
    };

    for (size_t i = 0; i < GetNumberOfSamplePoints2(); ++i)
    {
        auto x = GetSamplePoints2()[i];
        auto d = GetSampleDirs2()[i];
        auto refAns = refSurf.ClosestIntersection(Ray2D{ x, d });
        auto actAns = cis1.ClosestIntersection(Ray2D{ x, d });

        EXPECT_EQ(refAns.isIntersecting, actAns.isIntersecting);
        EXPECT_NEAR(refAns.distance, actAns.distance, 1e-5);
        EXPECT_VECTOR2_NEAR(refAns.point, actAns.point, 1e-5);
        EXPECT_VECTOR2_NEAR(refAns.normal, actAns.normal, 1e-5);
    }
}