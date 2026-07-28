#include "gtest/gtest.h"

#include <Core/Searcher/PointHashGridSearcher.hpp>
#include <Core/Utils/PointHashGridUtils.hpp>

using namespace CubbyFlow;

TEST(PointHashGridSearcher2, ForEachNearByPoint)
{
    Array1<Vector2D> points = { Vector2D(1, 1), Vector2D(3, 4),
                                Vector2D(-1, 2) };

    PointHashGridSearcher2 searcher(Vector2UZ{ 4, 4 }, std::sqrt(10));
    searcher.Build(points);

    searcher.ForEachNearbyPoint(Vector2D{ 0, 0 }, std::sqrt(15.0),
                                [&points](size_t i, const Vector2D& pt) {
                                    EXPECT_TRUE(i == 0 || i == 2);
                                    if (i == 0)
                                    {
                                        EXPECT_EQ(points[0], pt);
                                    }
                                    else if (i == 2)
                                    {
                                        EXPECT_EQ(points[2], pt);
                                    }
                                });
}

TEST(PointHashGridSearcher2, ForEachNearbyPointEmpty)
{
    const Array1<Vector2D> points;

    PointHashGridSearcher2 searcher{ Vector2UZ{ 4, 4 }, 2.0 * std::sqrt(10) };
    searcher.Build(points);

    searcher.ForEachNearbyPoint(
        Vector2D{ 0, 0 }, std::sqrt(10.0),
        [](size_t, const Vector2D&) { FAIL() << "Unexpected nearby point"; });
}

TEST(PointHashGridSearcher2, HasEachNearByPoint)
{
    const Array1<Vector2D> points = { Vector2D(1, 1), Vector2D(3, 4),
                                      Vector2D(4, 5) };

    PointHashGridSearcher2 searcher(Vector2UZ{ 4, 4 }, std::sqrt(10));
    searcher.Build(points);

    const bool result = searcher.HasNearbyPoint(Vector2D{}, std::sqrt(15.0));

    EXPECT_TRUE(result);
}

TEST(PointHashGridSearcher2, Build)
{
    const Array1<Vector2D> points = { Vector2D(3, 4), Vector2D(1, 5),
                                      Vector2D(-3, 0) };

    PointHashGridSearcher2 searcher(Vector2UZ{ 4, 4 }, std::sqrt(10));
    searcher.Build(points);

    EXPECT_EQ(Vector2Z(0, 1), PointHashGridUtils2::GetBucketIndex(
                                  Vector2D{ 3, 4 }, std::sqrt(10)));
    EXPECT_EQ(Vector2Z(0, 1), PointHashGridUtils2::GetBucketIndex(
                                  Vector2D{ 1, 5 }, std::sqrt(10)));
    EXPECT_EQ(Vector2Z(-1, 0), PointHashGridUtils2::GetBucketIndex(
                                   Vector2D{ -3, 0 }, std::sqrt(10)));

    EXPECT_EQ(4, PointHashGridUtils2::GetHashKeyFromBucketIndex(
                     Vector2Z{ 0, 1 }, Vector2Z{ 4, 4 }));
    EXPECT_EQ(8, PointHashGridUtils2::GetHashKeyFromBucketIndex(
                     Vector2Z{ 0, 2 }, Vector2Z{ 4, 4 }));
    EXPECT_EQ(3, PointHashGridUtils2::GetHashKeyFromBucketIndex(
                     Vector2Z{ -1, 0 }, Vector2Z{ 4, 4 }));
}
