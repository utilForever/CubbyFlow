#include "UnitTestsUtils.hpp"
#include "gtest/gtest.h"

#include <Core/Geometry/CustomImplicitSurface.hpp>
#include <Core/Geometry/Sphere.hpp>
#include <Core/Geometry/SurfaceToImplicit.hpp>

using namespace CubbyFlow;

TEST(CustomImplicitSurface3, SignedDistance)
{
    CustomImplicitSurface3 cis(
        [](const Vector3D& pt) {
            return (pt - Vector3D(0.5, 0.5, 0.5)).Length() - 0.25;
        },
        BoundingBox3D({ 0, 0, 0 }, { 1, 1, 1 }), 1e-3);

    EXPECT_DOUBLE_EQ(0.25, cis.SignedDistance({ 1, 0.5, 0.5 }));
    EXPECT_DOUBLE_EQ(-0.25, cis.SignedDistance({ 0.5, 0.5, 0.5 }));
    EXPECT_DOUBLE_EQ(0.0, cis.SignedDistance({ 0.5, 0.75, 0.5 }));
}

TEST(CustomImplicitSurface3, ClosestPoint)
{
    auto sphere = Sphere3::Builder()
                      .WithCenter({ 0.5, 0.45, 0.55 })
                      .WithRadius(0.3)
                      .MakeShared();
    SurfaceToImplicit3 refSurf(sphere);
    CustomImplicitSurface3 cis1(
        [&refSurf](const Vector3D& pt) { return refSurf.SignedDistance(pt); },
        BoundingBox3D({ 0, 0, 0 }, { 1, 1, 1 }), 1e-3);

    for (size_t i = 0; i < GetNumberOfSamplePoints3(); ++i)
    {
        auto sample = GetSamplePoints3()[i];
        if ((sample - sphere->center).Length() > 0.01)
        {
            auto refAns = refSurf.ClosestPoint(sample);
            auto actAns = cis1.ClosestPoint(sample);

            EXPECT_VECTOR3_NEAR(refAns, actAns, 1e-3);
        }
    }
}

TEST(CustomImplicitSurface3, ClosestNormal)
{
    auto sphere = Sphere3::Builder()
                      .WithCenter({ 0.5, 0.45, 0.55 })
                      .WithRadius(0.3)
                      .MakeShared();
    SurfaceToImplicit3 refSurf(sphere);
    CustomImplicitSurface3 cis1(
        [&refSurf](const Vector3D& pt) { return refSurf.SignedDistance(pt); },
        BoundingBox3D({ 0, 0, 0 }, { 1, 1, 1 }), 1e-3);

    for (size_t i = 0; i < GetNumberOfSamplePoints3(); ++i)
    {
        auto sample = GetSamplePoints3()[i];
        auto refAns = refSurf.ClosestNormal(sample);
        auto actAns = cis1.ClosestNormal(sample);

        EXPECT_VECTOR3_NEAR(refAns, actAns, 1e-3);
    }
}

TEST(CustomImplicitSurface3, Intersects)
{
    auto sphere = Sphere3::Builder()
                      .WithCenter({ 0.5, 0.45, 0.55 })
                      .WithRadius(0.3)
                      .MakeShared();
    SurfaceToImplicit3 refSurf(sphere);
    CustomImplicitSurface3 cis1(
        [&refSurf](const Vector3D& pt) { return refSurf.SignedDistance(pt); },
        BoundingBox3D({ 0, 0, 0 }, { 1, 1, 1 }), 1e-3);

    for (size_t i = 0; i < GetNumberOfSamplePoints3(); ++i)
    {
        auto x = GetSamplePoints3()[i];
        auto d = GetSampleDirs3()[i];
        bool refAns = refSurf.Intersects(Ray3D(x, d));
        bool actAns = cis1.Intersects(Ray3D(x, d));

        EXPECT_EQ(refAns, actAns);
    }
}

TEST(CustomImplicitSurface3, ClosestIntersection)
{
    auto sphere = Sphere3::Builder{}
                      .WithCenter({ 0.5, 0.45, 0.55 })
                      .WithRadius(0.3)
                      .MakeShared();
    SurfaceToImplicit3 refSurf{ sphere };
    CustomImplicitSurface3 cis1{
        [&refSurf](const Vector3D& pt) { return refSurf.SignedDistance(pt); },
        BoundingBox3D{ { 0, 0, 0 }, { 1, 1, 1 } }, 1e-3, 1e-3
    };

    for (size_t i = 0; i < GetNumberOfSamplePoints3(); ++i)
    {
        auto x = GetSamplePoints3()[i];
        auto d = GetSampleDirs3()[i];
        auto refAns = refSurf.ClosestIntersection(Ray3D(x, d));
        auto actAns = cis1.ClosestIntersection(Ray3D(x, d));

        EXPECT_EQ(refAns.isIntersecting, actAns.isIntersecting);
        EXPECT_NEAR(refAns.distance, actAns.distance, 1e-5);
        EXPECT_VECTOR3_NEAR(refAns.point, actAns.point, 1e-5);
        EXPECT_VECTOR3_NEAR(refAns.normal, actAns.normal, 1e-5);
    }
}