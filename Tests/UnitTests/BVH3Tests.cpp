#include "UnitTestsUtils.hpp"
#include "gtest/gtest.h"

#include <Core/Geometry/BVH.hpp>

using namespace CubbyFlow;

TEST(BVH3, Constructors)
{
    BVH3<Vector3D> bvh;
    EXPECT_EQ(bvh.begin(), bvh.end());
}

TEST(BVH3, BasicGetters)
{
    BVH3<Vector3D> bvh;

    Array1<Vector3D> points{ Vector3D{ 0, 0, 0 }, Vector3D{ 1, 1, 1 } };
    Array1<BoundingBox3D> bounds{ points.Length() };
    size_t i = 0;
    BoundingBox3D rootBounds;

    std::generate(bounds.begin(), bounds.end(), [&points, &i, &rootBounds]() {
        const auto c = points[i++];
        BoundingBox3D box{ c, c };

        box.Expand(0.1);
        rootBounds.Merge(box);

        return box;
    });

    bvh.Build(points, bounds);

    EXPECT_EQ(2u, bvh.NumberOfItems());
    EXPECT_VECTOR3_EQ(points[0], bvh.Item(0));
    EXPECT_VECTOR3_EQ(points[1], bvh.Item(1));
    EXPECT_EQ(3u, bvh.NumberOfNodes());
    EXPECT_EQ(1u, bvh.Children(0).first);
    EXPECT_EQ(2u, bvh.Children(0).second);
    EXPECT_FALSE(bvh.IsLeaf(0));
    EXPECT_TRUE(bvh.IsLeaf(1));
    EXPECT_TRUE(bvh.IsLeaf(2));
    EXPECT_BOUNDING_BOX3_EQ(rootBounds, bvh.NodeBound(0));
    EXPECT_BOUNDING_BOX3_EQ(bounds[0], bvh.NodeBound(1));
    EXPECT_BOUNDING_BOX3_EQ(bounds[1], bvh.NodeBound(2));
    EXPECT_EQ(bvh.end(), bvh.ItemOfNode(0));
    EXPECT_EQ(bvh.begin(), bvh.ItemOfNode(1));
    EXPECT_EQ(bvh.begin() + 1, bvh.ItemOfNode(2));
}

TEST(BVH3, Nearest)
{
    BVH3<Vector3D> bvh;

    auto distanceFunc = [](const Vector3D& a, const Vector3D& b) {
        return a.DistanceTo(b);
    };

    size_t numSamples = GetNumberOfSamplePoints3();
    Array1<Vector3D> points;
    size_t i = 0;

    const Vector3D* samplePoints = GetSamplePoints3();
    for (size_t j = 0; j < numSamples; ++j)
    {
        points.Append(samplePoints[j]);
    }

    Array1<BoundingBox3D> bounds(points.Length());
    std::generate(bounds.begin(), bounds.end(), [&points, &i]() {
        auto c = points[i++];
        BoundingBox3D box(c, c);

        box.Expand(0.1);

        return box;
    });

    bvh.Build(points, bounds);

    Vector3D testPt(0.5, 0.5, 0.5);
    auto nearest = bvh.Nearest(testPt, distanceFunc);
    ptrdiff_t answerIdx = 0;
    double bestDist = testPt.DistanceTo(points[answerIdx]);

    for (i = 1; i < numSamples; ++i)
    {
        double dist = testPt.DistanceTo(GetSamplePoints3()[i]);
        if (dist < bestDist)
        {
            bestDist = dist;
            answerIdx = i;
        }
    }

    EXPECT_EQ(answerIdx, nearest.item - &bvh.Item(0));
}

TEST(BVH3, BBoxIntersects)
{
    BVH3<Vector3D> bvh;

    auto overlapsFunc = [](const Vector3D& pt, const BoundingBox3D& bbox) {
        BoundingBox3D box(pt, pt);

        box.Expand(0.1);

        return bbox.Overlaps(box);
    };

    size_t numSamples = GetNumberOfSamplePoints3();
    Array1<Vector3D> points;
    size_t i = 0;

    const Vector3D* samplePoints = GetSamplePoints3();
    for (size_t j = 0; j < numSamples; ++j)
    {
        points.Append(samplePoints[j]);
    }

    Array1<BoundingBox3D> bounds(points.Length());
    std::generate(bounds.begin(), bounds.end(), [&points, &i]() {
        auto c = points[i++];
        BoundingBox3D box(c, c);

        box.Expand(0.1);

        return box;
    });

    bvh.Build(points, bounds);

    BoundingBox3D testBox({ 0.25, 0.15, 0.3 }, { 0.5, 0.6, 0.4 });
    bool hasOverlaps = false;

    for (i = 0; i < numSamples; ++i)
    {
        hasOverlaps |= overlapsFunc(GetSamplePoints3()[i], testBox);
    }

    EXPECT_EQ(hasOverlaps, bvh.Intersects(testBox, overlapsFunc));

    BoundingBox3D testBox2({ 0.3, 0.2, 0.1 }, { 0.6, 0.5, 0.4 });
    hasOverlaps = false;

    for (i = 0; i < numSamples; ++i)
    {
        hasOverlaps |= overlapsFunc(GetSamplePoints3()[i], testBox2);
    }

    EXPECT_EQ(hasOverlaps, bvh.Intersects(testBox2, overlapsFunc));
}

TEST(BVH3, RayIntersects)
{
    BVH3<BoundingBox3D> bvh;

    auto intersectsFunc = [](const BoundingBox3D& a, const Ray3D& ray) {
        return a.Intersects(ray);
    };

    size_t numSamples = GetNumberOfSamplePoints3();
    Array1<BoundingBox3D> items(numSamples / 2);
    size_t i = 0;

    std::generate(items.begin(), items.end(), [&i]() {
        auto c = GetSamplePoints3()[i++];
        BoundingBox3D box(c, c);

        box.Expand(0.1);

        return box;
    });

    bvh.Build(items, items);

    for (i = 0; i < numSamples / 2; ++i)
    {
        Ray3D ray(GetSampleDirs3()[i + numSamples / 2],
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

        // bvh search
        bool octInts = bvh.Intersects(ray, intersectsFunc);

        EXPECT_EQ(ansInts, octInts);
    }
}

TEST(BVH3, ClosestIntersection)
{
    BVH3<BoundingBox3D> bvh;

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
    Array1<BoundingBox3D> items(numSamples / 2);
    size_t i = 0;

    std::generate(items.begin(), items.end(), [&i]() {
        auto c = GetSamplePoints3()[i++];
        BoundingBox3D box(c, c);

        box.Expand(0.1);

        return box;
    });

    bvh.Build(items, items);

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
                ansInts.item = &bvh.Item(j);
            }
        }

        // bvh search
        auto bvhInts = bvh.ClosestIntersection(ray, intersectsFunc);

        EXPECT_DOUBLE_EQ(ansInts.distance, bvhInts.distance);
        EXPECT_EQ(ansInts.item, bvhInts.item);
    }
}

TEST(BVH3, ForEachOverlappingItems)
{
    BVH3<Vector3D> bvh;

    auto overlapsFunc = [](const Vector3D& pt, const BoundingBox3D& bbox) {
        return bbox.Contains(pt);
    };

    size_t numSamples = GetNumberOfSamplePoints3();
    Array1<Vector3D> points;
    size_t i = 0;

    const Vector3D* samplePoints = GetSamplePoints3();
    for (size_t j = 0; j < numSamples; ++j)
    {
        points.Append(samplePoints[j]);
    }

    Array1<BoundingBox3D> bounds(points.Length());
    std::generate(bounds.begin(), bounds.end(), [&points, &i]() {
        auto c = points[i++];
        BoundingBox3D box(c, c);

        box.Expand(0.1);

        return box;
    });

    bvh.Build(points, bounds);

    BoundingBox3D testBox({ 0.3, 0.2, 0.1 }, { 0.6, 0.5, 0.4 });
    size_t numOverlaps = 0;

    for (i = 0; i < numSamples; ++i)
    {
        numOverlaps += overlapsFunc(GetSamplePoints3()[i], testBox);
    }

    size_t measured = 0;
    bvh.ForEachIntersectingItem(
        testBox, overlapsFunc,
        [&overlapsFunc, &testBox, &measured](const Vector3D& pt) {
            EXPECT_TRUE(overlapsFunc(pt, testBox));
            ++measured;
        });

    EXPECT_EQ(numOverlaps, measured);
}