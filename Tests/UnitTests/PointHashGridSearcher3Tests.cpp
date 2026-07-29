#include "gtest/gtest.h"

#include <Core/Searcher/PointHashGridSearcher.hpp>
#include <Core/Utils/PointHashGridUtils.hpp>

using namespace CubbyFlow;

TEST(PointHashGridSearcher3, ForEachNearByPoint)
{
    Array1<Vector3D> points = { Vector3D(1, 1, 4), Vector3D(3, 4, 5),
                                Vector3D(-1, 2, 4) };

    PointHashGridSearcher3 searcher(Vector3UZ{ 4, 4, 4 }, std::sqrt(10));
    searcher.Build(points);

    searcher.ForEachNearbyPoint(Vector3D{ 0, 0, 0 }, std::sqrt(25.0),
                                [&points](size_t i, const Vector3D& pt) {
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

TEST(PointHashGridSearcher3, ForEachNearbyPointEmpty)
{
    const Array1<Vector3D> points;

    PointHashGridSearcher3 searcher(Vector3UZ{ 4, 4, 4 }, 2.0 * std::sqrt(10));
    searcher.Build(points);

    searcher.ForEachNearbyPoint(
        Vector3D{ 0, 0, 0 }, std::sqrt(10.0),
        [](size_t, const Vector3D&) { FAIL() << "Unexpected nearby point"; });
}

TEST(PointHashGridSearcher3, HasEachNearByPoint)
{
    const Array1<Vector3D> points = { Vector3D{ 1, 1, 1 },
                                      Vector3D{ 3, 444, 1 },
                                      Vector3D{ 4, 15, 111 } };

    PointHashGridSearcher3 searcher(Vector3UZ(4, 4, 4), std::sqrt(10));
    searcher.Build(points);

    const bool result = searcher.HasNearbyPoint(Vector3D{}, std::sqrt(15.0));

    EXPECT_TRUE(result);
}

TEST(PointHashGridSearcher3, Build)
{
    Array1<Vector3D> points = { Vector3D{ 3, 4, 111 }, Vector3D{ 111, 5, 1 },
                                Vector3D{ -311, 1123, 0 } };

    PointHashGridSearcher3 searcher(Vector3UZ{ 4, 4, 4 }, std::sqrt(9));
    searcher.Build(points);

    EXPECT_EQ(Vector3Z(1, 1, 37),
              PointHashGridUtils3::GetBucketIndex(points[0], std::sqrt(9)));
    EXPECT_EQ(Vector3Z(37, 1, 0),
              PointHashGridUtils3::GetBucketIndex(points[1], std::sqrt(9)));
    EXPECT_EQ(Vector3Z(-104, 374, 0),
              PointHashGridUtils3::GetBucketIndex(points[2], std::sqrt(9)));

    EXPECT_EQ(21, PointHashGridUtils3::GetHashKeyFromBucketIndex(
                      Vector3Z{ 1, 1, 37 }, Vector3Z{ 4, 4, 4 }));
    EXPECT_EQ(5, PointHashGridUtils3::GetHashKeyFromBucketIndex(
                     Vector3Z{ 37, 1, 0 }, Vector3Z{ 4, 4, 4 }));
    EXPECT_EQ(8, PointHashGridUtils3::GetHashKeyFromBucketIndex(
                     Vector3Z{ -104, 374, 0 }, Vector3Z{ 4, 4, 4 }));
}
