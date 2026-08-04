#include "UnitTestsUtils.hpp"
#include "gtest/gtest.h"

#include <Core/Geometry/Plane.hpp>
#include <Core/Geometry/Sphere.hpp>
#include <Core/Geometry/SurfaceSet.hpp>

using namespace CubbyFlow;

TEST(SurfaceSet2, Constructors)
{
    SurfaceSet2 sset1;
    EXPECT_EQ(0u, sset1.NumberOfSurfaces());

    auto sph1 =
        Sphere2::Builder().WithRadius(1.0).WithCenter({ 0, 0 }).MakeShared();
    auto sph2 =
        Sphere2::Builder().WithRadius(0.5).WithCenter({ 0, 3 }).MakeShared();
    auto sph3 =
        Sphere2::Builder().WithRadius(0.25).WithCenter({ -2, 0 }).MakeShared();
    SurfaceSet2 sset2(Array1<Surface2Ptr>{ sph1, sph2, sph3 }, Transform2{},
                      false);
    EXPECT_EQ(3u, sset2.NumberOfSurfaces());
    EXPECT_EQ(sph1->radius,
              std::dynamic_pointer_cast<Sphere2>(sset2.SurfaceAt(0))->radius);
    EXPECT_EQ(sph2->radius,
              std::dynamic_pointer_cast<Sphere2>(sset2.SurfaceAt(1))->radius);
    EXPECT_EQ(sph3->radius,
              std::dynamic_pointer_cast<Sphere2>(sset2.SurfaceAt(2))->radius);
    EXPECT_EQ(Vector2D(), sset2.transform.GetTranslation());
    EXPECT_EQ(0.0, sset2.transform.GetOrientation().GetRotation());

    SurfaceSet2 sset3(Array1<Surface2Ptr>{ sph1, sph2, sph3 },
                      Transform2(Vector2D(1, 2), 0.5), false);
    EXPECT_EQ(Vector2D(1, 2), sset3.transform.GetTranslation());
    EXPECT_EQ(0.5, sset3.transform.GetOrientation().GetRotation());

    SurfaceSet2 sset4(sset3);
    EXPECT_EQ(3u, sset4.NumberOfSurfaces());
    EXPECT_EQ(sph1->radius,
              std::dynamic_pointer_cast<Sphere2>(sset4.SurfaceAt(0))->radius);
    EXPECT_EQ(sph2->radius,
              std::dynamic_pointer_cast<Sphere2>(sset4.SurfaceAt(1))->radius);
    EXPECT_EQ(sph3->radius,
              std::dynamic_pointer_cast<Sphere2>(sset4.SurfaceAt(2))->radius);
    EXPECT_EQ(Vector2D(1, 2), sset4.transform.GetTranslation());
    EXPECT_EQ(0.5, sset4.transform.GetOrientation().GetRotation());

    sset3.isNormalFlipped = true;
    SurfaceSet2 sset5;
    sset5 = sset3;
    EXPECT_EQ(Vector2D(1, 2), sset5.transform.GetTranslation());
    EXPECT_EQ(0.5, sset5.transform.GetOrientation().GetRotation());
    EXPECT_TRUE(sset5.isNormalFlipped);
}

TEST(SurfaceSet2, AddSurface)
{
    SurfaceSet2 sset1;
    EXPECT_EQ(0u, sset1.NumberOfSurfaces());

    auto sph1 =
        Sphere2::Builder().WithRadius(1.0).WithCenter({ 0, 0 }).MakeShared();
    auto sph2 =
        Sphere2::Builder().WithRadius(0.5).WithCenter({ 0, 3 }).MakeShared();
    auto sph3 =
        Sphere2::Builder().WithRadius(0.25).WithCenter({ -2, 0 }).MakeShared();

    sset1.AddSurface(sph1);
    sset1.AddSurface(sph2);
    sset1.AddSurface(sph3);

    EXPECT_EQ(3u, sset1.NumberOfSurfaces());
    EXPECT_EQ(sph1->radius,
              std::dynamic_pointer_cast<Sphere2>(sset1.SurfaceAt(0))->radius);
    EXPECT_EQ(sph2->radius,
              std::dynamic_pointer_cast<Sphere2>(sset1.SurfaceAt(1))->radius);
    EXPECT_EQ(sph3->radius,
              std::dynamic_pointer_cast<Sphere2>(sset1.SurfaceAt(2))->radius);
    EXPECT_EQ(Vector2D(), sset1.transform.GetTranslation());
    EXPECT_EQ(0.0, sset1.transform.GetOrientation().GetRotation());
}

TEST(SurfaceSet2, ClosestPoint)
{
    SurfaceSet2 sset1;

    // Test empty set
    Vector2D emptyPoint = sset1.ClosestPoint({ 1.0, 2.0 });
    EXPECT_DOUBLE_EQ(std::numeric_limits<double>::max(), emptyPoint.x);
    EXPECT_DOUBLE_EQ(std::numeric_limits<double>::max(), emptyPoint.y);

    size_t numSamples = GetNumberOfSamplePoints2();

    // Use first half of the samples as the centers of the spheres
    for (size_t i = 0; i < numSamples / 2; ++i)
    {
        auto sph = Sphere2::Builder()
                       .WithRadius(0.01)
                       .WithCenter(GetSamplePoints2()[i])
                       .MakeShared();
        sset1.AddSurface(sph);
    }

    const auto bruteForceSearch = [&numSamples, &sset1](const Vector2D& pt) {
        double minDist2 = std::numeric_limits<double>::max();
        Vector2D result;

        for (size_t i = 0; i < numSamples / 2; ++i)
        {
            auto localResult = sset1.SurfaceAt(i)->ClosestPoint(pt);
            double localDist2 = pt.DistanceSquaredTo(localResult);

            if (localDist2 < minDist2)
            {
                minDist2 = localDist2;
                result = localResult;
            }
        }

        return result;
    };

    // Use second half of the samples as the query points
    for (size_t i = numSamples / 2; i < numSamples; ++i)
    {
        auto actual = sset1.ClosestPoint(GetSamplePoints2()[i]);
        auto expected = bruteForceSearch(GetSamplePoints2()[i]);
        EXPECT_VECTOR2_EQ(expected, actual);
    }

    // Now With GetTranslation instead of center
    SurfaceSet2 sset2;
    for (size_t i = 0; i < numSamples / 2; ++i)
    {
        auto sph = Sphere2::Builder()
                       .WithRadius(0.01)
                       .WithCenter({ 0, 0, 0 })
                       .WithTranslation(GetSamplePoints2()[i])
                       .MakeShared();
        sset2.AddSurface(sph);
    }
    for (size_t i = numSamples / 2; i < numSamples; ++i)
    {
        auto actual = sset2.ClosestPoint(GetSamplePoints2()[i]);
        auto expected = bruteForceSearch(GetSamplePoints2()[i]);
        EXPECT_VECTOR2_EQ(expected, actual);
    }
}

TEST(SurfaceSet2, ClosestNormal)
{
    SurfaceSet2 sset1;

    // Test empty set
    Vector2D emptyNormal = sset1.ClosestNormal({ 1.0, 2.0 });
    // No expected value -- just see if it doesn't crash
    (void)emptyNormal;

    size_t numSamples = GetNumberOfSamplePoints2();

    // Use first half of the samples as the centers of the spheres
    for (size_t i = 0; i < numSamples / 2; ++i)
    {
        auto sph = Sphere2::Builder()
                       .WithRadius(0.01)
                       .WithCenter(GetSamplePoints2()[i])
                       .MakeShared();
        sset1.AddSurface(sph);
    }

    const auto bruteForceSearch = [&numSamples, &sset1](const Vector2D& pt) {
        double minDist2 = std::numeric_limits<double>::max();
        Vector2D result;

        for (size_t i = 0; i < numSamples / 2; ++i)
        {
            auto localResult = sset1.SurfaceAt(i)->ClosestNormal(pt);
            auto ClosestPt = sset1.SurfaceAt(i)->ClosestPoint(pt);
            double localDist2 = pt.DistanceSquaredTo(ClosestPt);

            if (localDist2 < minDist2)
            {
                minDist2 = localDist2;
                result = localResult;
            }
        }

        return result;
    };

    // Use second half of the samples as the query points
    for (size_t i = numSamples / 2; i < numSamples; ++i)
    {
        auto actual = sset1.ClosestNormal(GetSamplePoints2()[i]);
        auto expected = bruteForceSearch(GetSamplePoints2()[i]);
        EXPECT_VECTOR2_EQ(expected, actual);
    }

    // Now With GetTranslation instead of center
    SurfaceSet2 sset2;
    for (size_t i = 0; i < numSamples / 2; ++i)
    {
        auto sph = Sphere2::Builder()
                       .WithRadius(0.01)
                       .WithCenter({ 0, 0, 0 })
                       .WithTranslation(GetSamplePoints2()[i])
                       .MakeShared();
        sset2.AddSurface(sph);
    }
    for (size_t i = numSamples / 2; i < numSamples; ++i)
    {
        auto actual = sset2.ClosestNormal(GetSamplePoints2()[i]);
        auto expected = bruteForceSearch(GetSamplePoints2()[i]);
        EXPECT_VECTOR2_EQ(expected, actual);
    }
}

TEST(SurfaceSet2, ClosestDistance)
{
    SurfaceSet2 sset1;

    size_t numSamples = GetNumberOfSamplePoints2();

    // Use first half of the samples as the centers of the spheres
    for (size_t i = 0; i < numSamples / 2; ++i)
    {
        auto sph = Sphere2::Builder()
                       .WithRadius(0.01)
                       .WithCenter(GetSamplePoints2()[i])
                       .MakeShared();
        sset1.AddSurface(sph);
    }

    const auto bruteForceSearch = [&numSamples, &sset1](const Vector2D& pt) {
        double minDist = std::numeric_limits<double>::max();

        for (size_t i = 0; i < numSamples / 2; ++i)
        {
            double localDist = sset1.SurfaceAt(i)->ClosestDistance(pt);
            if (localDist < minDist)
            {
                minDist = localDist;
            }
        }

        return minDist;
    };

    // Use second half of the samples as the query points
    for (size_t i = numSamples / 2; i < numSamples; ++i)
    {
        auto actual = sset1.ClosestDistance(GetSamplePoints2()[i]);
        auto expected = bruteForceSearch(GetSamplePoints2()[i]);
        EXPECT_DOUBLE_EQ(expected, actual);
    }

    // Now With GetTranslation instead of center
    SurfaceSet2 sset2;
    for (size_t i = 0; i < numSamples / 2; ++i)
    {
        auto sph = Sphere2::Builder()
                       .WithRadius(0.01)
                       .WithCenter({ 0, 0, 0 })
                       .WithTranslation(GetSamplePoints2()[i])
                       .MakeShared();
        sset2.AddSurface(sph);
    }
    for (size_t i = numSamples / 2; i < numSamples; ++i)
    {
        auto actual = sset2.ClosestDistance(GetSamplePoints2()[i]);
        auto expected = bruteForceSearch(GetSamplePoints2()[i]);
        EXPECT_DOUBLE_EQ(expected, actual);
    }
}

TEST(SurfaceSet2, Intersects)
{
    SurfaceSet2 sset1;

    size_t numSamples = GetNumberOfSamplePoints2();

    // Use first half of the samples as the centers of the spheres
    for (size_t i = 0; i < numSamples / 2; ++i)
    {
        auto sph = Sphere2::Builder()
                       .WithRadius(0.01)
                       .WithCenter(GetSamplePoints2()[i])
                       .MakeShared();
        sset1.AddSurface(sph);
    }

    const auto bruteForceTest = [&numSamples, &sset1](const Ray2D& ray) {
        for (size_t i = 0; i < numSamples / 2; ++i)
        {
            if (sset1.SurfaceAt(i)->Intersects(ray))
            {
                return true;
            }
        }

        return false;
    };

    // Use second half of the samples as the query points
    for (size_t i = numSamples / 2; i < numSamples; ++i)
    {
        Ray2D ray(GetSamplePoints2()[i], GetSampleDirs2()[i]);
        bool actual = sset1.Intersects(ray);
        bool expected = bruteForceTest(ray);
        EXPECT_EQ(expected, actual);
    }

    // Now With GetTranslation instead of center
    SurfaceSet2 sset2;
    for (size_t i = 0; i < numSamples / 2; ++i)
    {
        auto sph = Sphere2::Builder()
                       .WithRadius(0.01)
                       .WithCenter({ 0, 0, 0 })
                       .WithTranslation(GetSamplePoints2()[i])
                       .MakeShared();
        sset2.AddSurface(sph);
    }
    for (size_t i = numSamples / 2; i < numSamples; ++i)
    {
        Ray2D ray(GetSamplePoints2()[i], GetSampleDirs2()[i]);
        bool actual = sset2.Intersects(ray);
        bool expected = bruteForceTest(ray);
        EXPECT_EQ(expected, actual);
    }
}

TEST(SurfaceSet2, ClosestIntersection)
{
    SurfaceSet2 sset1;

    size_t numSamples = GetNumberOfSamplePoints2();

    // Use first half of the samples as the centers of the spheres
    for (size_t i = 0; i < numSamples / 2; ++i)
    {
        auto sph = Sphere2::Builder()
                       .WithRadius(0.01)
                       .WithCenter(GetSamplePoints2()[i])
                       .MakeShared();
        sset1.AddSurface(sph);
    }

    const auto bruteForceTest = [&numSamples, &sset1](const Ray2D& ray) {
        SurfaceRayIntersection2 result{};

        for (size_t i = 0; i < numSamples / 2; ++i)
        {
            auto localResult = sset1.SurfaceAt(i)->ClosestIntersection(ray);
            if (localResult.distance < result.distance)
            {
                result = localResult;
            }
        }

        return result;
    };

    // Use second half of the samples as the query points
    for (size_t i = numSamples / 2; i < numSamples; ++i)
    {
        Ray2D ray(GetSamplePoints2()[i], GetSampleDirs2()[i]);
        auto actual = sset1.ClosestIntersection(ray);
        auto expected = bruteForceTest(ray);
        EXPECT_DOUBLE_EQ(expected.distance, actual.distance);
        EXPECT_VECTOR2_EQ(expected.point, actual.point);
        EXPECT_VECTOR2_EQ(expected.normal, actual.normal);
        EXPECT_EQ(expected.isIntersecting, actual.isIntersecting);
    }

    // Now With GetTranslation instead of center
    SurfaceSet2 sset2;
    for (size_t i = 0; i < numSamples / 2; ++i)
    {
        auto sph = Sphere2::Builder()
                       .WithRadius(0.01)
                       .WithCenter({ 0, 0, 0 })
                       .WithTranslation(GetSamplePoints2()[i])
                       .MakeShared();
        sset2.AddSurface(sph);
    }
    for (size_t i = numSamples / 2; i < numSamples; ++i)
    {
        Ray2D ray(GetSamplePoints2()[i], GetSampleDirs2()[i]);
        auto actual = sset2.ClosestIntersection(ray);
        auto expected = bruteForceTest(ray);
        EXPECT_DOUBLE_EQ(expected.distance, actual.distance);
        EXPECT_VECTOR2_EQ(expected.point, actual.point);
        EXPECT_VECTOR2_EQ(expected.normal, actual.normal);
        EXPECT_EQ(expected.isIntersecting, actual.isIntersecting);
    }
}

TEST(SurfaceSet2, BoundingBox)
{
    SurfaceSet2 sset1;

    size_t numSamples = GetNumberOfSamplePoints2();

    // Use first half of the samples as the centers of the spheres
    BoundingBox2D answer;
    for (size_t i = 0; i < numSamples / 2; ++i)
    {
        auto sph = Sphere2::Builder()
                       .WithRadius(0.01)
                       .WithCenter(GetSamplePoints2()[i])
                       .MakeShared();
        sset1.AddSurface(sph);

        answer.Merge(sph->GetBoundingBox());
    }

    EXPECT_BOUNDING_BOX2_NEAR(answer, sset1.GetBoundingBox(), 1e-9);

    // Now With GetTranslation instead of center
    SurfaceSet2 sset2;
    BoundingBox2D debug;
    for (size_t i = 0; i < numSamples / 2; ++i)
    {
        auto sph = Sphere2::Builder()
                       .WithRadius(0.01)
                       .WithCenter({ 0, 0, 0 })
                       .WithTranslation(GetSamplePoints2()[i])
                       .MakeShared();
        sset2.AddSurface(sph);

        debug.Merge(sph->GetBoundingBox());
    }

    EXPECT_BOUNDING_BOX2_NEAR(answer, debug, 1e-9);
    EXPECT_BOUNDING_BOX2_NEAR(answer, sset2.GetBoundingBox(), 1e-9);
}

TEST(SurfaceSet2, MixedBoundTypes)
{
    const BoundingBox2D domain{ Vector2D{}, Vector2D{ 1, 2 } };

    const auto plane = Plane2::Builder()
                           .WithNormal({ 0, 1 })
                           .WithPoint({ 0.0, 0.25 * domain.Height() })
                           .MakeShared();

    const auto sphere = Sphere2::Builder()
                            .WithCenter(domain.MidPoint())
                            .WithRadius(0.15 * domain.Width())
                            .MakeShared();

    const auto surfaceSet{ SurfaceSet2::Builder()
                               .WithSurfaces(
                                   Array1<Surface2Ptr>{ plane, sphere })
                               .MakeShared() };

    EXPECT_FALSE(surfaceSet->IsBounded());

    const auto cp = surfaceSet->ClosestPoint(Vector2D{ 0.5, 0.4 });
    const Vector2D answer{ 0.5, 0.5 };

    EXPECT_VECTOR2_NEAR(answer, cp, 1e-9);
}

TEST(SurfaceSet2, IsValidGeometry)
{
    const auto surfaceSet{ SurfaceSet2::Builder().MakeShared() };

    EXPECT_FALSE(surfaceSet->IsValidGeometry());

    const BoundingBox2D domain{ Vector2D{}, Vector2D{ 1, 2 } };

    const auto plane = Plane2::Builder()
                           .WithNormal({ 0, 1 })
                           .WithPoint({ 0, 0.25 * domain.Height() })
                           .MakeShared();

    const auto sphere = Sphere2::Builder()
                            .WithCenter(domain.MidPoint())
                            .WithRadius(0.15 * domain.Width())
                            .MakeShared();

    auto surfaceSet2 = SurfaceSet2::Builder()
                           .WithSurfaces(Array1<Surface2Ptr>{ plane, sphere })
                           .MakeShared();

    EXPECT_TRUE(surfaceSet2->IsValidGeometry());

    surfaceSet2->AddSurface(surfaceSet);

    EXPECT_FALSE(surfaceSet2->IsValidGeometry());
}

TEST(SurfaceSet2, IsInside)
{
    const BoundingBox2D domain(Vector2D{}, Vector2D{ 1, 2 });
    const Vector2D offset{ 1, 2 };

    const auto plane = Plane2::Builder{}
                           .WithNormal({ 0, 1 })
                           .WithPoint({ 0, 0.25 * domain.Height() })
                           .MakeShared();

    const auto sphere = Sphere2::Builder{}
                            .WithCenter(domain.MidPoint())
                            .WithRadius(0.15 * domain.Width())
                            .MakeShared();

    const auto surfaceSet =
        SurfaceSet2::Builder{}
            .WithSurfaces(Array1<Surface2Ptr>{ plane, sphere })
            .WithTransform(Transform2{ offset, 0.0 })
            .MakeShared();

    EXPECT_TRUE(surfaceSet->IsInside(Vector2D{ 0.5, 0.25 } + offset));
    EXPECT_TRUE(surfaceSet->IsInside(Vector2D{ 0.5, 1.0 } + offset));
    EXPECT_FALSE(surfaceSet->IsInside(Vector2D{ 0.5, 1.5 } + offset));
}

TEST(SurfaceSet2, UpdateQueryEngine)
{
    auto sphere = Sphere2::Builder{}
                      .WithCenter({ -1.0, 1.0 })
                      .WithRadius(0.5)
                      .MakeShared();

    auto surfaceSet = SurfaceSet2::Builder{}
                          .WithSurfaces(Array1<Surface2Ptr>{ sphere })
                          .WithTransform(Transform2{ { 1.0, 2.0 }, 0.0 })
                          .MakeShared();

    const auto bbox1 = surfaceSet->GetBoundingBox();
    EXPECT_BOUNDING_BOX2_EQ(BoundingBox2D({ -0.5, 2.5 }, { 0.5, 3.5 }), bbox1);

    surfaceSet->transform = Transform2{ { 3.0, -4.0 }, 0.0 };
    surfaceSet->UpdateQueryEngine();
    const auto bbox2 = surfaceSet->GetBoundingBox();
    EXPECT_BOUNDING_BOX2_EQ(BoundingBox2D({ 1.5, -3.5 }, { 2.5, -2.5 }), bbox2);

    sphere->transform = Transform2{ { -6.0, 9.0 }, 0.0 };
    surfaceSet->UpdateQueryEngine();
    const auto bbox3 = surfaceSet->GetBoundingBox();
    EXPECT_BOUNDING_BOX2_EQ(BoundingBox2D({ -4.5, 5.5 }, { -3.5, 6.5 }), bbox3);

    // Plane is unbounded. Total bbox should ignore it.
    auto plane = Plane2::Builder{}.WithNormal({ 1.0, 0.0 }).MakeShared();
    surfaceSet->AddSurface(plane);
    surfaceSet->UpdateQueryEngine();
    auto bbox4 = surfaceSet->GetBoundingBox();
    EXPECT_BOUNDING_BOX2_EQ(BoundingBox2D({ -4.5, 5.5 }, { -3.5, 6.5 }), bbox4);
}
