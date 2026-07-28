#include "UnitTestsUtils.hpp"
#include "gtest/gtest.h"

#include <Core/Geometry/Plane.hpp>
#include <Core/Geometry/Sphere.hpp>
#include <Core/Geometry/SurfaceSet.hpp>

using namespace CubbyFlow;

TEST(SurfaceSet3, Constructors)
{
    SurfaceSet3 sset1;
    EXPECT_EQ(0u, sset1.NumberOfSurfaces());

    auto sph1 =
        Sphere3::Builder().WithRadius(1.0).WithCenter({ 0, 0, 0 }).MakeShared();
    auto sph2 =
        Sphere3::Builder().WithRadius(0.5).WithCenter({ 0, 3, 2 }).MakeShared();
    auto sph3 = Sphere3::Builder()
                    .WithRadius(0.25)
                    .WithCenter({ -2, 0, 0 })
                    .MakeShared();
    SurfaceSet3 sset2(Array1<Surface3Ptr>{ sph1, sph2, sph3 }, Transform3{},
                      false);
    EXPECT_EQ(3u, sset2.NumberOfSurfaces());
    EXPECT_EQ(sph1->radius,
              std::dynamic_pointer_cast<Sphere3>(sset2.SurfaceAt(0))->radius);
    EXPECT_EQ(sph2->radius,
              std::dynamic_pointer_cast<Sphere3>(sset2.SurfaceAt(1))->radius);
    EXPECT_EQ(sph3->radius,
              std::dynamic_pointer_cast<Sphere3>(sset2.SurfaceAt(2))->radius);
    EXPECT_EQ(Vector3D(), sset2.transform.GetTranslation());
    EXPECT_EQ(QuaternionD(), sset2.transform.GetOrientation().GetRotation());

    SurfaceSet3 sset3(
        Array1<Surface3Ptr>{ sph1, sph2, sph3 },
        Transform3(Vector3D(1, 2, 3), QuaternionD({ 1, 0, 0 }, 0.5)), false);
    EXPECT_EQ(Vector3D(1, 2, 3), sset3.transform.GetTranslation());
    EXPECT_EQ(QuaternionD({ 1, 0, 0 }, 0.5),
              sset3.transform.GetOrientation().GetRotation());

    SurfaceSet3 sset4(sset3);
    EXPECT_EQ(3u, sset4.NumberOfSurfaces());
    EXPECT_EQ(sph1->radius,
              std::dynamic_pointer_cast<Sphere3>(sset4.SurfaceAt(0))->radius);
    EXPECT_EQ(sph2->radius,
              std::dynamic_pointer_cast<Sphere3>(sset4.SurfaceAt(1))->radius);
    EXPECT_EQ(sph3->radius,
              std::dynamic_pointer_cast<Sphere3>(sset4.SurfaceAt(2))->radius);
    EXPECT_EQ(Vector3D(1, 2, 3), sset4.transform.GetTranslation());
    EXPECT_EQ(QuaternionD({ 1, 0, 0 }, 0.5),
              sset4.transform.GetOrientation().GetRotation());
}

TEST(SurfaceSet3, AddSurface)
{
    SurfaceSet3 sset1;
    EXPECT_EQ(0u, sset1.NumberOfSurfaces());

    auto sph1 =
        Sphere3::Builder().WithRadius(1.0).WithCenter({ 0, 0, 0 }).MakeShared();
    auto sph2 =
        Sphere3::Builder().WithRadius(0.5).WithCenter({ 0, 3, 2 }).MakeShared();
    auto sph3 = Sphere3::Builder()
                    .WithRadius(0.25)
                    .WithCenter({ -2, 0, 0 })
                    .MakeShared();

    sset1.AddSurface(sph1);
    sset1.AddSurface(sph2);
    sset1.AddSurface(sph3);

    EXPECT_EQ(3u, sset1.NumberOfSurfaces());
    EXPECT_EQ(sph1->radius,
              std::dynamic_pointer_cast<Sphere3>(sset1.SurfaceAt(0))->radius);
    EXPECT_EQ(sph2->radius,
              std::dynamic_pointer_cast<Sphere3>(sset1.SurfaceAt(1))->radius);
    EXPECT_EQ(sph3->radius,
              std::dynamic_pointer_cast<Sphere3>(sset1.SurfaceAt(2))->radius);
    EXPECT_EQ(Vector3D(), sset1.transform.GetTranslation());
    EXPECT_EQ(QuaternionD(), sset1.transform.GetOrientation().GetRotation());
}

TEST(SurfaceSet3, ClosestPoint)
{
    SurfaceSet3 sset1;

    // Test empty set
    Vector3D emptyPoint = sset1.ClosestPoint({ 1.0, 2.0, 3.0 });
    EXPECT_DOUBLE_EQ(std::numeric_limits<double>::max(), emptyPoint.x);
    EXPECT_DOUBLE_EQ(std::numeric_limits<double>::max(), emptyPoint.y);
    EXPECT_DOUBLE_EQ(std::numeric_limits<double>::max(), emptyPoint.z);

    size_t numSamples = GetNumberOfSamplePoints3();

    // Use first half of the samples as the centers of the spheres
    for (size_t i = 0; i < numSamples / 2; ++i)
    {
        auto sph = Sphere3::Builder()
                       .WithRadius(0.01)
                       .WithCenter(GetSamplePoints3()[i])
                       .MakeShared();
        sset1.AddSurface(sph);
    }

    const auto bruteForceSearch = [&numSamples, &sset1](const Vector3D& pt) {
        double minDist2 = std::numeric_limits<double>::max();
        Vector3D result;

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
        auto actual = sset1.ClosestPoint(GetSamplePoints3()[i]);
        auto expected = bruteForceSearch(GetSamplePoints3()[i]);
        EXPECT_VECTOR3_EQ(expected, actual);
    }

    // Now With GetTranslation instead of center
    SurfaceSet3 sset2;
    for (size_t i = 0; i < numSamples / 2; ++i)
    {
        auto sph = Sphere3::Builder()
                       .WithRadius(0.01)
                       .WithCenter({ 0, 0, 0 })
                       .WithTranslation(GetSamplePoints3()[i])
                       .MakeShared();
        sset2.AddSurface(sph);
    }
    for (size_t i = numSamples / 2; i < numSamples; ++i)
    {
        auto actual = sset2.ClosestPoint(GetSamplePoints3()[i]);
        auto expected = bruteForceSearch(GetSamplePoints3()[i]);
        EXPECT_VECTOR3_EQ(expected, actual);
    }
}

TEST(SurfaceSet3, ClosestNormal)
{
    SurfaceSet3 sset1;

    // Test empty set
    Vector3D emptyNormal = sset1.ClosestNormal({ 1.0, 2.0, 3.0 });
    // No excepted value -- just see if it doesn't crash
    (void)emptyNormal;

    size_t numSamples = GetNumberOfSamplePoints3();

    // Use first half of the samples as the centers of the spheres
    for (size_t i = 0; i < numSamples / 2; ++i)
    {
        auto sph = Sphere3::Builder()
                       .WithRadius(0.01)
                       .WithCenter(GetSamplePoints3()[i])
                       .MakeShared();
        sset1.AddSurface(sph);
    }

    const auto bruteForceSearch = [&numSamples, &sset1](const Vector3D& pt) {
        double minDist2 = std::numeric_limits<double>::max();
        Vector3D result;

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
        auto actual = sset1.ClosestNormal(GetSamplePoints3()[i]);
        auto expected = bruteForceSearch(GetSamplePoints3()[i]);
        EXPECT_VECTOR3_EQ(expected, actual);
    }

    // Now With GetTranslation instead of center
    SurfaceSet3 sset2;
    for (size_t i = 0; i < numSamples / 2; ++i)
    {
        auto sph = Sphere3::Builder()
                       .WithRadius(0.01)
                       .WithCenter({ 0, 0, 0 })
                       .WithTranslation(GetSamplePoints3()[i])
                       .MakeShared();
        sset2.AddSurface(sph);
    }
    for (size_t i = numSamples / 2; i < numSamples; ++i)
    {
        auto actual = sset2.ClosestNormal(GetSamplePoints3()[i]);
        auto expected = bruteForceSearch(GetSamplePoints3()[i]);
        EXPECT_VECTOR3_EQ(expected, actual);
    }
}

TEST(SurfaceSet3, ClosestDistance)
{
    SurfaceSet3 sset1;

    size_t numSamples = GetNumberOfSamplePoints3();

    // Use first half of the samples as the centers of the spheres
    for (size_t i = 0; i < numSamples / 2; ++i)
    {
        auto sph = Sphere3::Builder()
                       .WithRadius(0.01)
                       .WithCenter(GetSamplePoints3()[i])
                       .MakeShared();
        sset1.AddSurface(sph);
    }

    const auto bruteForceSearch = [&numSamples, &sset1](const Vector3D& pt) {
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
        auto actual = sset1.ClosestDistance(GetSamplePoints3()[i]);
        auto expected = bruteForceSearch(GetSamplePoints3()[i]);
        EXPECT_DOUBLE_EQ(expected, actual);
    }

    // Now With GetTranslation instead of center
    SurfaceSet3 sset2;
    for (size_t i = 0; i < numSamples / 2; ++i)
    {
        auto sph = Sphere3::Builder()
                       .WithRadius(0.01)
                       .WithCenter({ 0, 0, 0 })
                       .WithTranslation(GetSamplePoints3()[i])
                       .MakeShared();
        sset2.AddSurface(sph);
    }
    for (size_t i = numSamples / 2; i < numSamples; ++i)
    {
        auto actual = sset2.ClosestDistance(GetSamplePoints3()[i]);
        auto expected = bruteForceSearch(GetSamplePoints3()[i]);
        EXPECT_DOUBLE_EQ(expected, actual);
    }
}

TEST(SurfaceSet3, Intersects)
{
    SurfaceSet3 sset1;

    size_t numSamples = GetNumberOfSamplePoints3();

    // Use first half of the samples as the centers of the spheres
    for (size_t i = 0; i < numSamples / 2; ++i)
    {
        auto sph = Sphere3::Builder()
                       .WithRadius(0.01)
                       .WithCenter(GetSamplePoints3()[i])
                       .MakeShared();
        sset1.AddSurface(sph);
    }

    const auto bruteForceTest = [&numSamples, &sset1](const Ray3D& ray) {
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
        Ray3D ray(GetSamplePoints3()[i], GetSampleDirs3()[i]);
        bool actual = sset1.Intersects(ray);
        bool expected = bruteForceTest(ray);
        EXPECT_EQ(expected, actual);
    }

    // Now With GetTranslation instead of center
    SurfaceSet3 sset2;
    for (size_t i = 0; i < numSamples / 2; ++i)
    {
        auto sph = Sphere3::Builder()
                       .WithRadius(0.01)
                       .WithCenter({ 0, 0, 0 })
                       .WithTranslation(GetSamplePoints3()[i])
                       .MakeShared();
        sset2.AddSurface(sph);
    }
    for (size_t i = numSamples / 2; i < numSamples; ++i)
    {
        Ray3D ray(GetSamplePoints3()[i], GetSampleDirs3()[i]);
        bool actual = sset2.Intersects(ray);
        bool expected = bruteForceTest(ray);
        EXPECT_EQ(expected, actual);
    }
}

TEST(SurfaceSet3, ClosestIntersection)
{
    SurfaceSet3 sset1;

    size_t numSamples = GetNumberOfSamplePoints3();

    // Use first half of the samples as the centers of the spheres
    for (size_t i = 0; i < numSamples / 2; ++i)
    {
        auto sph = Sphere3::Builder()
                       .WithRadius(0.01)
                       .WithCenter(GetSamplePoints3()[i])
                       .MakeShared();
        sset1.AddSurface(sph);
    }

    const auto bruteForceTest = [&numSamples, &sset1](const Ray3D& ray) {
        SurfaceRayIntersection3 result{};

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
        Ray3D ray(GetSamplePoints3()[i], GetSampleDirs3()[i]);
        auto actual = sset1.ClosestIntersection(ray);
        auto expected = bruteForceTest(ray);
        EXPECT_DOUBLE_EQ(expected.distance, actual.distance);
        EXPECT_VECTOR3_EQ(expected.point, actual.point);
        EXPECT_VECTOR3_EQ(expected.normal, actual.normal);
        EXPECT_EQ(expected.isIntersecting, actual.isIntersecting);
    }

    // Now With GetTranslation instead of center
    SurfaceSet3 sset2;
    for (size_t i = 0; i < numSamples / 2; ++i)
    {
        auto sph = Sphere3::Builder()
                       .WithRadius(0.01)
                       .WithCenter({ 0, 0, 0 })
                       .WithTranslation(GetSamplePoints3()[i])
                       .MakeShared();
        sset2.AddSurface(sph);
    }
    for (size_t i = numSamples / 2; i < numSamples; ++i)
    {
        Ray3D ray(GetSamplePoints3()[i], GetSampleDirs3()[i]);
        auto actual = sset2.ClosestIntersection(ray);
        auto expected = bruteForceTest(ray);
        EXPECT_DOUBLE_EQ(expected.distance, actual.distance);
        EXPECT_VECTOR3_EQ(expected.point, actual.point);
        EXPECT_VECTOR3_EQ(expected.normal, actual.normal);
        EXPECT_EQ(expected.isIntersecting, actual.isIntersecting);
    }
}

TEST(SurfaceSet3, BoundingBox)
{
    SurfaceSet3 sset1;

    size_t numSamples = GetNumberOfSamplePoints3();

    // Use first half of the samples as the centers of the spheres
    BoundingBox3D answer;
    for (size_t i = 0; i < numSamples / 2; ++i)
    {
        auto sph = Sphere3::Builder()
                       .WithRadius(0.01)
                       .WithCenter(GetSamplePoints3()[i])
                       .MakeShared();
        sset1.AddSurface(sph);

        answer.Merge(sph->GetBoundingBox());
    }

    EXPECT_BOUNDING_BOX3_NEAR(answer, sset1.GetBoundingBox(), 1e-9);

    // Now With GetTranslation instead of center
    SurfaceSet3 sset2;
    BoundingBox3D debug;
    for (size_t i = 0; i < numSamples / 2; ++i)
    {
        auto sph = Sphere3::Builder()
                       .WithRadius(0.01)
                       .WithCenter({ 0, 0, 0 })
                       .WithTranslation(GetSamplePoints3()[i])
                       .MakeShared();
        sset2.AddSurface(sph);

        debug.Merge(sph->GetBoundingBox());
    }

    EXPECT_BOUNDING_BOX3_NEAR(answer, debug, 1e-9);
    EXPECT_BOUNDING_BOX3_NEAR(answer, sset2.GetBoundingBox(), 1e-9);
}

TEST(SurfaceSet3, MixedBoundTypes)
{
    const BoundingBox3D domain{ Vector3D{}, Vector3D{ 1, 2, 1 } };

    const auto plane = Plane3::Builder()
                           .WithNormal({ 0, 1, 0 })
                           .WithPoint({ 0, 0.25 * domain.Height(), 0 })
                           .MakeShared();

    const auto sphere = Sphere3::Builder()
                            .WithCenter(domain.MidPoint())
                            .WithRadius(0.15 * domain.Width())
                            .MakeShared();

    const auto surfaceSet =
        SurfaceSet3::Builder()
            .WithSurfaces(Array1<Surface3Ptr>{ plane, sphere })
            .MakeShared();

    EXPECT_FALSE(surfaceSet->IsBounded());

    const auto cp = surfaceSet->ClosestPoint(Vector3D{ 0.5, 0.4, 0.5 });
    const Vector3D answer{ 0.5, 0.5, 0.5 };

    EXPECT_VECTOR3_NEAR(answer, cp, 1e-9);
}

TEST(SurfaceSet3, IsValidGeometry)
{
    const auto surfaceSet{ SurfaceSet3::Builder().MakeShared() };

    EXPECT_FALSE(surfaceSet->IsValidGeometry());

    const BoundingBox3D domain{ Vector3D{}, Vector3D{ 1, 2, 1 } };

    const auto plane = Plane3::Builder()
                           .WithNormal({ 0, 1, 0 })
                           .WithPoint({ 0, 0.25 * domain.Height(), 0 })
                           .MakeShared();

    const auto sphere = Sphere3::Builder()
                            .WithCenter(domain.MidPoint())
                            .WithRadius(0.15 * domain.Width())
                            .MakeShared();

    auto surfaceSet2 = SurfaceSet3::Builder()
                           .WithSurfaces(Array1<Surface3Ptr>{ plane, sphere })
                           .MakeShared();

    EXPECT_TRUE(surfaceSet2->IsValidGeometry());

    surfaceSet2->AddSurface(surfaceSet);

    EXPECT_FALSE(surfaceSet2->IsValidGeometry());
}

TEST(SurfaceSet3, IsInside)
{
    const BoundingBox3D domain(Vector3D{}, Vector3D{ 1, 2, 1 });
    const Vector3D offset{ 1, 2, 3 };

    const auto plane = Plane3::Builder{}
                           .WithNormal({ 0, 1, 0 })
                           .WithPoint({ 0, 0.25 * domain.Height(), 0 })
                           .MakeShared();

    const auto sphere = Sphere3::Builder{}
                            .WithCenter(domain.MidPoint())
                            .WithRadius(0.15 * domain.Width())
                            .MakeShared();

    const auto surfaceSet =
        SurfaceSet3::Builder{}
            .WithSurfaces(Array1<Surface3Ptr>{ plane, sphere })
            .WithTransform(Transform3{ offset, QuaternionD{} })
            .MakeShared();

    EXPECT_TRUE(surfaceSet->IsInside(Vector3D{ 0.5, 0.25, 0.5 } + offset));
    EXPECT_TRUE(surfaceSet->IsInside(Vector3D{ 0.5, 1.0, 0.5 } + offset));
    EXPECT_FALSE(surfaceSet->IsInside(Vector3D{ 0.5, 1.5, 0.5 } + offset));
}

TEST(SurfaceSet3, UpdateQueryEngine)
{
    auto sphere = Sphere3::Builder{}
                      .WithCenter({ -1.0, 1.0, 2.0 })
                      .WithRadius(0.5)
                      .MakeShared();

    auto surfaceSet =
        SurfaceSet3::Builder{}
            .WithSurfaces(Array1<Surface3Ptr>{ sphere })
            .WithTransform(Transform3{ { 1.0, 2.0, -1.0 }, QuaternionD{} })
            .MakeShared();

    const auto bbox1 = surfaceSet->GetBoundingBox();
    EXPECT_BOUNDING_BOX2_EQ(
        BoundingBox3D({ -0.5, 2.5, 0.5 }, { 0.5, 3.5, 1.5 }), bbox1);

    surfaceSet->transform = Transform3{ { 3.0, -4.0, 7.0 }, QuaternionD{} };
    surfaceSet->UpdateQueryEngine();
    const auto bbox2 = surfaceSet->GetBoundingBox();
    EXPECT_BOUNDING_BOX2_EQ(
        BoundingBox3D({ 1.5, -3.5, 4.5 }, { 2.5, -2.5, 5.5 }), bbox2);

    sphere->transform = Transform3{ { -6.0, 9.0, 2.0 }, QuaternionD{} };
    surfaceSet->UpdateQueryEngine();
    const auto bbox3 = surfaceSet->GetBoundingBox();
    EXPECT_BOUNDING_BOX2_EQ(
        BoundingBox3D({ -4.5, 5.5, 10.5 }, { -3.5, 6.5, 11.5 }), bbox3);

    // Plane is unbounded. Total bbox should ignore it.
    auto plane = Plane3::Builder{}.WithNormal({ 1.0, 0.0, 0.0 }).MakeShared();
    surfaceSet->AddSurface(plane);
    surfaceSet->UpdateQueryEngine();
    auto bbox4 = surfaceSet->GetBoundingBox();
    EXPECT_BOUNDING_BOX2_EQ(
        BoundingBox3D({ -4.5, 5.5, 10.5 }, { -3.5, 6.5, 11.5 }), bbox4);
}