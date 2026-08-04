#include "UnitTestsUtils.hpp"
#include "gtest/gtest.h"

#include <Core/Geometry/TriangleMesh3.hpp>

using namespace CubbyFlow;

TEST(TriangleMesh3, Constructors)
{
    TriangleMesh3 mesh1;
    EXPECT_EQ(0u, mesh1.NumberOfPoints());
    EXPECT_EQ(0u, mesh1.NumberOfNormals());
    EXPECT_EQ(0u, mesh1.NumberOfUVs());
    EXPECT_EQ(0u, mesh1.NumberOfTriangles());

    TriangleMesh3 mesh2(
        Transform3(Vector3D(1, 2, 3), QuaternionD({ 1, 0, 0 }, 0.5)), true);
    mesh1 = mesh2;
    EXPECT_EQ(Vector3D(1, 2, 3), mesh1.transform.GetTranslation());
    EXPECT_EQ(QuaternionD({ 1, 0, 0 }, 0.5),
              mesh1.transform.GetOrientation().GetRotation());
    EXPECT_TRUE(mesh1.isNormalFlipped);
}

TEST(TriangleMesh3, ReadObj)
{
    std::string objStr = GetCubeTriMesh3x3x3Obj();
    std::istringstream objStream(objStr);

    TriangleMesh3 mesh;
    [[maybe_unused]] bool isLoaded = mesh.ReadObj(&objStream);

    EXPECT_EQ(56u, mesh.NumberOfPoints());
    EXPECT_EQ(96u, mesh.NumberOfNormals());
    EXPECT_EQ(76u, mesh.NumberOfUVs());
    EXPECT_EQ(108u, mesh.NumberOfTriangles());
}

TEST(TriangleMesh3, ClosestPoint)
{
    std::string objStr = GetCubeTriMesh3x3x3Obj();
    std::istringstream objStream(objStr);

    TriangleMesh3 mesh;
    [[maybe_unused]] bool isLoaded = mesh.ReadObj(&objStream);

    const auto bruteForceSearch = [&mesh](const Vector3D& pt) {
        double minDist2 = std::numeric_limits<double>::max();
        Vector3D result;

        for (size_t i = 0; i < mesh.NumberOfTriangles(); ++i)
        {
            Triangle3 tri = mesh.Triangle(i);
            auto localResult = tri.ClosestPoint(pt);
            double localDist2 = pt.DistanceSquaredTo(localResult);

            if (localDist2 < minDist2)
            {
                minDist2 = localDist2;
                result = localResult;
            }
        }

        return result;
    };

    size_t numSamples = GetNumberOfSamplePoints3();
    for (size_t i = 0; i < numSamples; ++i)
    {
        auto actual = mesh.ClosestPoint(GetSamplePoints3()[i]);
        auto expected = bruteForceSearch(GetSamplePoints3()[i]);
        EXPECT_VECTOR3_EQ(expected, actual);
    }
}

TEST(TriangleMesh3, ClosestNormal)
{
    std::string objStr = GetSphereTriMesh5x5Obj();
    std::istringstream objStream(objStr);

    TriangleMesh3 mesh;
    [[maybe_unused]] bool isLoaded = mesh.ReadObj(&objStream);

    const auto bruteForceSearch = [&mesh](const Vector3D& pt) {
        double minDist2 = std::numeric_limits<double>::max();
        Vector3D result;

        for (size_t i = 0; i < mesh.NumberOfTriangles(); ++i)
        {
            Triangle3 tri = mesh.Triangle(i);
            auto localResult = tri.ClosestNormal(pt);
            auto closestPt = tri.ClosestPoint(pt);
            double localDist2 = pt.DistanceSquaredTo(closestPt);

            if (localDist2 < minDist2)
            {
                minDist2 = localDist2;
                result = localResult;
            }
        }

        return result;
    };

    size_t numSamples = GetNumberOfSamplePoints3();
    for (size_t i = 0; i < numSamples; ++i)
    {
        auto actual = mesh.ClosestNormal(GetSamplePoints3()[i]);
        auto expected = bruteForceSearch(GetSamplePoints3()[i]);
        EXPECT_VECTOR3_NEAR(expected, actual, 1e-9);
    }
}

TEST(TriangleMesh3, ClosestDistance)
{
    std::string objStr = GetCubeTriMesh3x3x3Obj();
    std::istringstream objStream(objStr);

    TriangleMesh3 mesh;
    [[maybe_unused]] bool isLoaded = mesh.ReadObj(&objStream);

    const auto bruteForceSearch = [&mesh](const Vector3D& pt) {
        double minDist = std::numeric_limits<double>::max();

        for (size_t i = 0; i < mesh.NumberOfTriangles(); ++i)
        {
            Triangle3 tri = mesh.Triangle(i);
            auto localResult = tri.ClosestDistance(pt);

            if (localResult < minDist)
            {
                minDist = localResult;
            }
        }

        return minDist;
    };

    size_t numSamples = GetNumberOfSamplePoints3();
    for (size_t i = 0; i < numSamples; ++i)
    {
        auto actual = mesh.ClosestDistance(GetSamplePoints3()[i]);
        auto expected = bruteForceSearch(GetSamplePoints3()[i]);
        EXPECT_DOUBLE_EQ(expected, actual);
    }
}

TEST(TriangleMesh3, Intersects)
{
    std::string objStr = GetCubeTriMesh3x3x3Obj();
    std::istringstream objStream(objStr);

    TriangleMesh3 mesh;
    [[maybe_unused]] bool isLoaded = mesh.ReadObj(&objStream);

    size_t numSamples = GetNumberOfSamplePoints3();

    const auto bruteForceTest = [&mesh](const Ray3D& ray) {
        for (size_t i = 0; i < mesh.NumberOfTriangles(); ++i)
        {
            Triangle3 tri = mesh.Triangle(i);
            if (tri.Intersects(ray))
            {
                return true;
            }
        }

        return false;
    };

    for (size_t i = 0; i < numSamples; ++i)
    {
        Ray3D ray(GetSamplePoints3()[i], GetSampleDirs3()[i]);
        bool actual = mesh.Intersects(ray);
        bool expected = bruteForceTest(ray);
        EXPECT_EQ(expected, actual);
    }
}

TEST(TriangleMesh3, ClosestIntersection)
{
    std::string objStr = GetCubeTriMesh3x3x3Obj();
    std::istringstream objStream(objStr);

    TriangleMesh3 mesh;
    [[maybe_unused]] bool isLoaded = mesh.ReadObj(&objStream);

    size_t numSamples = GetNumberOfSamplePoints3();

    const auto bruteForceTest = [&mesh](const Ray3D& ray) {
        SurfaceRayIntersection3 result{};

        for (size_t i = 0; i < mesh.NumberOfTriangles(); ++i)
        {
            Triangle3 tri = mesh.Triangle(i);
            auto localResult = tri.ClosestIntersection(ray);

            if (localResult.distance < result.distance)
            {
                result = localResult;
            }
        }

        return result;
    };

    for (size_t i = 0; i < numSamples; ++i)
    {
        Ray3D ray(GetSamplePoints3()[i], GetSampleDirs3()[i]);
        auto actual = mesh.ClosestIntersection(ray);
        auto expected = bruteForceTest(ray);
        EXPECT_DOUBLE_EQ(expected.distance, actual.distance);
        EXPECT_VECTOR3_EQ(expected.point, actual.point);
        EXPECT_VECTOR3_EQ(expected.normal, actual.normal);
        EXPECT_EQ(expected.isIntersecting, actual.isIntersecting);
    }
}

TEST(TriangleMesh3, IsInside)
{
    const std::string objStr = GetCubeTriMesh3x3x3Obj();
    std::istringstream objStream(objStr);

    TriangleMesh3 mesh;
    [[maybe_unused]] bool isLoaded = mesh.ReadObj(&objStream);

    const size_t numSamples = GetNumberOfSamplePoints3();

    for (size_t i = 0; i < numSamples; ++i)
    {
        Vector3D p = GetSamplePoints3()[i];
        auto actual = mesh.IsInside(p);
        auto expected = mesh.GetBoundingBox().Contains(p);
        EXPECT_EQ(expected, actual);
    }
}

TEST(TriangleMesh3, BoundingBox)
{
    std::string objStr = GetCubeTriMesh3x3x3Obj();
    std::istringstream objStream(objStr);

    TriangleMesh3 mesh;
    [[maybe_unused]] bool isLoaded = mesh.ReadObj(&objStream);

    EXPECT_BOUNDING_BOX3_EQ(
        BoundingBox3D({ -0.5, -0.5, -0.5 }, { 0.5, 0.5, 0.5 }),
        mesh.GetBoundingBox());
}

TEST(TriangleMesh3, Builder)
{
    TriangleMesh3::PointArray points = { Vector3D(1, 2, 3), Vector3D(4, 5, 6),
                                         Vector3D(7, 8, 9),
                                         Vector3D(10, 11, 12) };

    TriangleMesh3::NormalArray normals = { Vector3D(10, 11, 12),
                                           Vector3D(7, 8, 9), Vector3D(4, 5, 6),
                                           Vector3D(1, 2, 3) };

    TriangleMesh3::UVArray uvs = { Vector2D(13, 14), Vector2D(15, 16) };

    TriangleMesh3::IndexArray pointIndices = { Vector3UZ(0, 1, 2),
                                               Vector3UZ(0, 1, 3) };

    TriangleMesh3::IndexArray normalIndices = { Vector3UZ(1, 2, 3),
                                                Vector3UZ(2, 1, 0) };

    TriangleMesh3::IndexArray uvIndices = { Vector3UZ(1, 0, 2),
                                            Vector3UZ(3, 1, 0) };

    TriangleMesh3 mesh = TriangleMesh3::GetBuilder()
                             .WithPoints(points)
                             .WithNormals(normals)
                             .WithUVs(uvs)
                             .WithPointIndices(pointIndices)
                             .WithNormalIndices(normalIndices)
                             .WithUVIndices(uvIndices)
                             .Build();

    EXPECT_EQ(4u, mesh.NumberOfPoints());
    EXPECT_EQ(4u, mesh.NumberOfNormals());
    EXPECT_EQ(2u, mesh.NumberOfUVs());
    EXPECT_EQ(2u, mesh.NumberOfTriangles());

    for (size_t i = 0; i < mesh.NumberOfPoints(); ++i)
    {
        EXPECT_EQ(points[i], mesh.Point(i));
    }

    for (size_t i = 0; i < mesh.NumberOfNormals(); ++i)
    {
        EXPECT_EQ(normals[i], mesh.Normal(i));
    }

    for (size_t i = 0; i < mesh.NumberOfUVs(); ++i)
    {
        EXPECT_EQ(uvs[i], mesh.UV(i));
    }

    for (size_t i = 0; i < mesh.NumberOfTriangles(); ++i)
    {
        EXPECT_EQ(pointIndices[i], mesh.PointIndex(i));
        EXPECT_EQ(normalIndices[i], mesh.NormalIndex(i));
        EXPECT_EQ(uvIndices[i], mesh.UVIndex(i));
    }
}
