#include "UnitTestsUtils.hpp"
#include "gtest/gtest.h"

#include <Core/QueryEngine/ListQueryEngine.hpp>

using namespace CubbyFlow;

TEST(ListQueryEngine2, BoxIntersection)
{
    size_t numSamples = GetNumberOfSamplePoints2();
    Array1<Vector2D> points(numSamples);
    std::copy_n(GetSamplePoints2(), numSamples, points.begin());

    ListQueryEngine2<Vector2D> engine;
    engine.Add(points);

    auto testFunc = [](const Vector2D& pt, const BoundingBox2D& bbox) {
        return bbox.Contains(pt);
    };

    BoundingBox2D testBox({ 0.25, 0.2 }, { 0.5, 0.4 });
    size_t numIntersections = 0;
    for (size_t i = 0; i < numSamples; ++i)
    {
        numIntersections += testFunc(GetSamplePoints2()[i], testBox);
    }
    bool hasIntersection = numIntersections > 0;

    EXPECT_EQ(hasIntersection, engine.Intersects(testBox, testFunc));

    BoundingBox2D testBox2({ 0.2, 0.2 }, { 0.6, 0.5 });
    numIntersections = 0;
    for (size_t i = 0; i < numSamples; ++i)
    {
        numIntersections += testFunc(GetSamplePoints2()[i], testBox2);
    }
    hasIntersection = numIntersections > 0;

    EXPECT_EQ(hasIntersection, engine.Intersects(testBox2, testFunc));

    size_t measured = 0;
    engine.ForEachIntersectingItem(
        testBox2, testFunc,
        [&testFunc, &testBox2, &measured](const Vector2D& pt) {
            EXPECT_TRUE(testFunc(pt, testBox2));
            ++measured;
        });

    EXPECT_EQ(numIntersections, measured);
}

TEST(ListQueryEngine2, RayIntersection)
{
    ListQueryEngine2<BoundingBox2D> engine;

    auto intersectsFunc = [](const BoundingBox2D& a, const Ray2D& ray) {
        return a.Intersects(ray);
    };

    size_t numSamples = GetNumberOfSamplePoints2();
    Array1<BoundingBox2D> items(numSamples / 2);
    size_t i = 0;

    std::generate(items.begin(), items.end(), [&i]() {
        auto c = GetSamplePoints2()[i++];
        BoundingBox2D box(c, c);

        box.Expand(0.1);

        return box;
    });

    engine.Add(items);

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

        // engine search
        bool engInts = engine.Intersects(ray, intersectsFunc);

        EXPECT_EQ(ansInts, engInts);
    }
}

TEST(ListQueryEngine2, ClosestIntersection)
{
    ListQueryEngine2<BoundingBox2D> engine;

    auto intersectsFunc = [](const BoundingBox2D& a, const Ray2D& ray) {
        auto bboxResult = a.ClosestIntersection(ray);
        return bboxResult.near;
    };

    size_t numSamples = GetNumberOfSamplePoints2();
    Array1<BoundingBox2D> items(numSamples / 2);
    size_t i = 0;

    std::generate(items.begin(), items.end(), [&i]() {
        auto c = GetSamplePoints2()[i++];
        BoundingBox2D box(c, c);

        box.Expand(0.1);

        return box;
    });

    engine.Add(items);

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
                ansInts.item = &items[j];
            }
        }

        // engine search
        auto engInts = engine.ClosestIntersection(ray, intersectsFunc);

        if (ansInts.item != nullptr && engInts.item != nullptr)
        {
            EXPECT_VECTOR2_EQ(ansInts.item->lowerCorner,
                              engInts.item->lowerCorner);
            EXPECT_VECTOR2_EQ(ansInts.item->upperCorner,
                              engInts.item->upperCorner);
        }
        else
        {
            EXPECT_EQ(nullptr, ansInts.item);
            EXPECT_EQ(nullptr, engInts.item);
        }

        EXPECT_DOUBLE_EQ(ansInts.distance, engInts.distance);
    }
}

TEST(ListQueryEngine2, Nearest)
{
    ListQueryEngine2<Vector2D> engine;

    auto distanceFunc = [](const Vector2D& a, const Vector2D& b) {
        return a.DistanceTo(b);
    };

    size_t numSamples = GetNumberOfSamplePoints2();
    Array1<Vector2D> points(numSamples);
    std::copy_n(GetSamplePoints2(), numSamples, points.begin());

    engine.Add(points);

    Vector2D testPt(0.5, 0.5);
    auto closest = engine.Nearest(testPt, distanceFunc);

    Vector2D answer = GetSamplePoints2()[0];
    double bestDist = testPt.DistanceTo(answer);
    for (size_t i = 1; i < numSamples; ++i)
    {
        double dist = testPt.DistanceTo(GetSamplePoints2()[i]);
        if (dist < bestDist)
        {
            bestDist = dist;
            answer = GetSamplePoints2()[i];
        }
    }

    EXPECT_VECTOR2_EQ(answer, (*(closest.item)));
}