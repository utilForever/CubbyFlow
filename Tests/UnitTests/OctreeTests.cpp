#include "UnitTestsUtils.hpp"
#include "gtest/gtest.h"

#include <Core/Geometry/Octree.hpp>

using namespace CubbyFlow;

TEST(Octree, Constructors)
{
    Octree<Vector3D> octree;
    EXPECT_EQ(octree.begin(), octree.end());
}

TEST(Octree, Clear)
{
    Octree<Vector3D> octree;

    octree.Build(
        { Vector3D(0.2, 0.7, 0.3) }, BoundingBox3D({ 0, 0, 0 }, { 1, 1, 1 }),
        [](const Vector3D& point, const BoundingBox3D& box) {
            return box.Contains(point);
        },
        3);
    octree.Clear();

    EXPECT_EQ(octree.begin(), octree.end());
    EXPECT_EQ(0u, octree.GetNumberOfNodes());
}

TEST(Octree, Nearest)
{
    Octree<Vector3D> octree;

    auto overlapsFunc = [](const Vector3D& pt, const BoundingBox3D& bbox) {
        return bbox.Contains(pt);
    };

    auto distanceFunc = [](const Vector3D& a, const Vector3D& b) {
        return a.DistanceTo(b);
    };

    // Single point
    octree.Build({ Vector3D(0.2, 0.7, 0.3) },
                 BoundingBox3D({ 0, 0, 0 }, { 0.9, 0.8, 1 }), overlapsFunc, 3);

    EXPECT_EQ(3u, octree.GetMaxDepth());
    EXPECT_VECTOR3_EQ(Vector3D(0, 0, 0), octree.GetBoundingBox().lowerCorner);
    EXPECT_VECTOR3_EQ(Vector3D(1, 1, 1), octree.GetBoundingBox().upperCorner);
    EXPECT_EQ(17u, octree.GetNumberOfNodes());

    size_t child = octree.GetChildIndex(0, 2);
    EXPECT_EQ(3u, child);

    child = octree.GetChildIndex(child, 0);
    EXPECT_EQ(9u, child);

    size_t theNonEmptyLeafNode = child + 4;
    for (size_t i = 0; i < 17; ++i)
    {
        if (i == theNonEmptyLeafNode)
        {
            EXPECT_EQ(1u, octree.GetItemsAtNode(i).size());
        }
        else
        {
            EXPECT_EQ(0u, octree.GetItemsAtNode(i).size());
        }
    }

    // Many points
    size_t numSamples = GetNumberOfSamplePoints3();
    std::vector<Vector3D> points(GetSamplePoints3(),
                                 GetSamplePoints3() + numSamples);

    octree.Build(points, BoundingBox3D({ 0, 0, 0 }, { 1, 1, 1 }), overlapsFunc,
                 5);

    Vector3D testPt(0.5, 0.5, 0.5);
    auto nearest = octree.Nearest(testPt, distanceFunc);
    ptrdiff_t answerIdx = 0;
    double bestDist = testPt.DistanceTo(points[answerIdx]);

    for (size_t i = 1; i < numSamples; ++i)
    {
        double dist = testPt.DistanceTo(GetSamplePoints3()[i]);
        if (dist < bestDist)
        {
            bestDist = dist;
            answerIdx = i;
        }
    }

    EXPECT_EQ(answerIdx, nearest.item - &octree.GetItem(0));
}

TEST(Octree, BBoxIntersects)
{
    Octree<Vector3D> octree;

    auto overlapsFunc = [](const Vector3D& pt, const BoundingBox3D& bbox) {
        return bbox.Contains(pt);
    };

    size_t numSamples = GetNumberOfSamplePoints3();
    std::vector<Vector3D> points(GetSamplePoints3(),
                                 GetSamplePoints3() + numSamples);

    octree.Build(points, BoundingBox3D({ 0, 0, 0 }, { 1, 1, 1 }), overlapsFunc,
                 5);

    BoundingBox3D testBox({ 0.25, 0.15, 0.3 }, { 0.5, 0.6, 0.4 });
    bool hasOverlaps = false;
    for (size_t i = 0; i < numSamples; ++i)
    {
        hasOverlaps |= overlapsFunc(GetSamplePoints3()[i], testBox);
    }

    EXPECT_EQ(hasOverlaps, octree.Intersects(testBox, overlapsFunc));

    BoundingBox3D testBox2({ 0.3, 0.2, 0.1 }, { 0.6, 0.5, 0.4 });
    hasOverlaps = false;
    for (size_t i = 0; i < numSamples; ++i)
    {
        hasOverlaps |= overlapsFunc(GetSamplePoints3()[i], testBox2);
    }

    EXPECT_EQ(hasOverlaps, octree.Intersects(testBox2, overlapsFunc));
}

TEST(Octree, ForEachOverlappingItems)
{
    Octree<Vector3D> octree;

    auto overlapsFunc = [](const Vector3D& pt, const BoundingBox3D& bbox) {
        return bbox.Contains(pt);
    };

    size_t numSamples = GetNumberOfSamplePoints3();
    std::vector<Vector3D> points(GetSamplePoints3(),
                                 GetSamplePoints3() + numSamples);

    octree.Build(points, BoundingBox3D({ 0, 0, 0 }, { 1, 1, 1 }), overlapsFunc,
                 5);

    BoundingBox3D testBox({ 0.3, 0.2, 0.1 }, { 0.6, 0.5, 0.4 });
    size_t numOverlaps = 0;
    for (size_t i = 0; i < numSamples; ++i)
    {
        numOverlaps += overlapsFunc(GetSamplePoints3()[i], testBox);
    }

    size_t measured = 0;
    octree.ForEachIntersectingItem(
        testBox, overlapsFunc,
        [&overlapsFunc, &testBox, &measured](const Vector3D& pt) {
            EXPECT_TRUE(overlapsFunc(pt, testBox));
            ++measured;
        });

    EXPECT_EQ(numOverlaps, measured);
}

TEST(Octree, RayIntersects)
{
    Octree<BoundingBox3D> octree;

    auto overlapsFunc = [](const BoundingBox3D& a, const BoundingBox3D& bbox) {
        return bbox.Overlaps(a);
    };

    auto intersectsFunc = [](const BoundingBox3D& a, const Ray3D& ray) {
        return a.Intersects(ray);
    };

    size_t numSamples = GetNumberOfSamplePoints3();
    std::vector<BoundingBox3D> items(numSamples / 2);
    size_t i = 0;

    std::ranges::generate(items, [&i]() {
        auto c = GetSamplePoints3()[i++];
        BoundingBox3D box(c, c);

        box.Expand(0.1);

        return box;
    });

    octree.Build(items, BoundingBox3D({ 0, 0, 0 }, { 1, 1, 1 }), overlapsFunc,
                 5);

    for (i = 0; i < numSamples / 2; ++i)
    {
        Ray3D ray(GetSamplePoints3()[i + numSamples / 2],
                  GetSampleDirs3()[i + numSamples / 2]);

        // ad-hoc search
        bool ansInts = false;
        for (size_t j = 0; j < numSamples / 2; ++j)
        {
            if (intersectsFunc(items[j], ray))
            {
                ansInts = true;
                break;
            }
        }

        // octree search
        bool octInts = octree.Intersects(ray, intersectsFunc);

        EXPECT_EQ(ansInts, octInts);
    }
}

TEST(Octree, ClosestIntersection)
{
    Octree<BoundingBox3D> octree;

    auto overlapsFunc = [](const BoundingBox3D& a, const BoundingBox3D& bbox) {
        return bbox.Overlaps(a);
    };

    auto intersectsFunc = [](const BoundingBox3D& a, const Ray3D& ray) {
        auto bboxResult = a.ClosestIntersection(ray);
        if (bboxResult.isIntersecting)
        {
            return bboxResult.near;
        }
        else
        {
            return std::numeric_limits<double>::max();
        }
    };

    size_t numSamples = GetNumberOfSamplePoints3();
    std::vector<BoundingBox3D> items(numSamples / 2);
    size_t i = 0;

    std::ranges::generate(items, [&i]() {
        auto c = GetSamplePoints3()[i++];
        BoundingBox3D box(c, c);

        box.Expand(0.1);

        return box;
    });

    octree.Build(items, BoundingBox3D({ 0, 0, 0 }, { 1, 1, 1 }), overlapsFunc,
                 5);

    for (i = 0; i < numSamples / 2; ++i)
    {
        Ray3D ray(GetSamplePoints3()[i + numSamples / 2],
                  GetSampleDirs3()[i + numSamples / 2]);

        // ad-hoc search
        ClosestIntersectionQueryResult3<BoundingBox3D> ansInts;
        for (size_t j = 0; j < numSamples / 2; ++j)
        {
            double dist = intersectsFunc(items[j], ray);
            if (dist < ansInts.distance)
            {
                ansInts.distance = dist;
                ansInts.item = &octree.GetItem(j);
            }
        }

        // octree search
        auto octInts = octree.ClosestIntersection(ray, intersectsFunc);

        EXPECT_DOUBLE_EQ(ansInts.distance, octInts.distance);
        EXPECT_EQ(ansInts.item, octInts.item);
    }
}
