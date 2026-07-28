#include "UnitTestsUtils.hpp"
#include "gtest/gtest.h"

#include <Core/Geometry/Quadtree.hpp>

using namespace CubbyFlow;

TEST(Quadtree, Constructors)
{
    Quadtree<Vector2D> quadtree;
    EXPECT_EQ(quadtree.begin(), quadtree.end());
}

TEST(Quadtree, Nearest)
{
    Quadtree<Vector2D> quadtree;

    auto overlapsFunc = [](const Vector2D& pt, const BoundingBox2D& bbox) {
        return bbox.Contains(pt);
    };

    auto distanceFunc = [](const Vector2D& a, const Vector2D& b) {
        return a.DistanceTo(b);
    };

    // Single point
    quadtree.Build({ Vector2D(0.2, 0.7) },
                   BoundingBox2D({ 0, 0 }, { 0.9, 1.0 }), overlapsFunc, 3);

    EXPECT_EQ(3u, quadtree.GetMaxDepth());
    EXPECT_VECTOR2_EQ(Vector2D(0, 0), quadtree.GetBoundingBox().lowerCorner);
    EXPECT_VECTOR2_EQ(Vector2D(1, 1), quadtree.GetBoundingBox().upperCorner);
    EXPECT_EQ(9u, quadtree.GetNumberOfNodes());

    size_t child = quadtree.GetChildIndex(0, 2);
    EXPECT_EQ(3u, child);

    child = quadtree.GetChildIndex(child, 0);
    EXPECT_EQ(5u, child);

    size_t theNonEmptyLeafNode = child + 0;
    for (size_t i = 0; i < 9; ++i)
    {
        if (i == theNonEmptyLeafNode)
        {
            EXPECT_EQ(1u, quadtree.GetItemsAtNode(i).size());
        }
        else
        {
            EXPECT_EQ(0u, quadtree.GetItemsAtNode(i).size());
        }
    }

    // Many points
    size_t numSamples = GetNumberOfSamplePoints2();
    std::vector<Vector2D> points(GetSamplePoints2(),
                                 GetSamplePoints2() + numSamples);

    quadtree.Build(points, BoundingBox2D({ 0, 0 }, { 1, 1 }), overlapsFunc, 5);

    Vector2D testPt(0.5, 0.5);
    auto nearest = quadtree.Nearest(testPt, distanceFunc);
    ptrdiff_t answerIdx = 0;
    double bestDist = testPt.DistanceTo(points[answerIdx]);

    for (size_t i = 1; i < numSamples; ++i)
    {
        double dist = testPt.DistanceTo(GetSamplePoints2()[i]);
        if (dist < bestDist)
        {
            bestDist = dist;
            answerIdx = i;
        }
    }

    EXPECT_EQ(answerIdx, nearest.item - &quadtree.GetItem(0));
}

TEST(Quadtree, BBoxIntersects)
{
    Quadtree<Vector2D> quadtree;

    auto overlapsFunc = [](const Vector2D& pt, const BoundingBox2D& bbox) {
        return bbox.Contains(pt);
    };

    size_t numSamples = GetNumberOfSamplePoints2();
    std::vector<Vector2D> points(GetSamplePoints2(),
                                 GetSamplePoints2() + numSamples);

    quadtree.Build(points, BoundingBox2D({ 0, 0 }, { 1, 1 }), overlapsFunc, 5);

    BoundingBox2D testBox({ 0.25, 0.15 }, { 0.5, 0.6 });
    bool hasOverlaps = false;
    for (size_t i = 0; i < numSamples; ++i)
    {
        hasOverlaps |= overlapsFunc(GetSamplePoints2()[i], testBox);
    }

    EXPECT_EQ(hasOverlaps, quadtree.Intersects(testBox, overlapsFunc));

    BoundingBox2D testBox2({ 0.2, 0.2 }, { 0.6, 0.5 });
    hasOverlaps = false;
    for (size_t i = 0; i < numSamples; ++i)
    {
        hasOverlaps |= overlapsFunc(GetSamplePoints2()[i], testBox2);
    }

    EXPECT_EQ(hasOverlaps, quadtree.Intersects(testBox2, overlapsFunc));
}

TEST(Quadtree, ForEachOverlappingItems)
{
    Quadtree<Vector2D> quadtree;

    auto overlapsFunc = [](const Vector2D& pt, const BoundingBox2D& bbox) {
        return bbox.Contains(pt);
    };

    size_t numSamples = GetNumberOfSamplePoints2();
    std::vector<Vector2D> points(GetSamplePoints2(),
                                 GetSamplePoints2() + numSamples);

    quadtree.Build(points, BoundingBox2D({ 0, 0 }, { 1, 1 }), overlapsFunc, 5);

    BoundingBox2D testBox({ 0.2, 0.2 }, { 0.6, 0.5 });
    size_t numOverlaps = 0;
    for (size_t i = 0; i < numSamples; ++i)
    {
        numOverlaps += overlapsFunc(GetSamplePoints2()[i], testBox);
    }

    size_t measured = 0;
    quadtree.ForEachIntersectingItem(
        testBox, overlapsFunc,
        [&overlapsFunc, &testBox, &measured](const Vector2D& pt) {
            EXPECT_TRUE(overlapsFunc(pt, testBox));
            ++measured;
        });

    EXPECT_EQ(numOverlaps, measured);
}

TEST(Quadtree, RayIntersects)
{
    Quadtree<BoundingBox2D> quadtree;

    auto overlapsFunc = [](const BoundingBox2D& a, const BoundingBox2D& bbox) {
        return bbox.Overlaps(a);
    };

    auto intersectsFunc = [](const BoundingBox2D& a, const Ray2D& ray) {
        return a.Intersects(ray);
    };

    size_t numSamples = GetNumberOfSamplePoints2();
    std::vector<BoundingBox2D> items(numSamples / 2);
    size_t i = 0;

    std::generate(items.begin(), items.end(), [&i]() {
        auto c = GetSamplePoints2()[i++];
        BoundingBox2D box(c, c);

        box.Expand(0.1);

        return box;
    });

    quadtree.Build(items, BoundingBox2D({ 0, 0 }, { 1, 1 }), overlapsFunc, 5);

    for (i = 0; i < numSamples / 2; ++i)
    {
        Ray2D ray(GetSamplePoints2()[i + numSamples / 2],
                  GetSampleDirs2()[i + numSamples / 2]);

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

        // quadtree search
        bool quadInts = quadtree.Intersects(ray, intersectsFunc);

        EXPECT_EQ(ansInts, quadInts);
    }
}

TEST(Quadtree, ClosestIntersection)
{
    Quadtree<BoundingBox2D> quadtree;

    auto overlapsFunc = [](const BoundingBox2D& a, const BoundingBox2D& bbox) {
        return bbox.Overlaps(a);
    };

    auto intersectsFunc = [](const BoundingBox2D& a, const Ray2D& ray) {
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

    size_t numSamples = GetNumberOfSamplePoints2();
    std::vector<BoundingBox2D> items(numSamples / 2);
    size_t i = 0;

    std::generate(items.begin(), items.end(), [&i]() {
        auto c = GetSamplePoints2()[i++];
        BoundingBox2D box(c, c);

        box.Expand(0.1);

        return box;
    });

    quadtree.Build(items, BoundingBox2D({ 0, 0 }, { 1, 1 }), overlapsFunc, 5);

    for (i = 0; i < numSamples / 2; ++i)
    {
        Ray2D ray(GetSamplePoints2()[i + numSamples / 2],
                  GetSampleDirs2()[i + numSamples / 2]);

        // ad-hoc search
        ClosestIntersectionQueryResult2<BoundingBox2D> ansInts;
        for (size_t j = 0; j < numSamples / 2; ++j)
        {
            double dist = intersectsFunc(items[j], ray);
            if (dist < ansInts.distance)
            {
                ansInts.distance = dist;
                ansInts.item = &quadtree.GetItem(j);
            }
        }

        // quadtree search
        auto quadInts = quadtree.ClosestIntersection(ray, intersectsFunc);

        EXPECT_DOUBLE_EQ(ansInts.distance, quadInts.distance);
        EXPECT_EQ(ansInts.item, quadInts.item);
    }
}