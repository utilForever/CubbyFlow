#include "gtest/gtest.h"

#include <Core/Searcher/PointKdTreeSearcher.hpp>

using namespace CubbyFlow;

TEST(PointKdTreeSearcher3, ForEachNearbyPoint)
{
    Array1<Vector3D> points = { Vector3D(0, 1, 3), Vector3D(2, 5, 4),
                                Vector3D(-1, 3, 0) };

    PointKdTreeSearcher3 searcher;
    searcher.Build(points);

    int cnt = 0;
    searcher.ForEachNearbyPoint(Vector3D(0, 0, 0), std::sqrt(10.0),
                                [&points, &cnt](size_t i, const Vector3D& pt) {
                                    EXPECT_TRUE(i == 0 || i == 2);

                                    if (i == 0)
                                    {
                                        EXPECT_EQ(points[0], pt);
                                    }
                                    else if (i == 2)
                                    {
                                        EXPECT_EQ(points[2], pt);
                                    }

                                    ++cnt;
                                });

    EXPECT_EQ(2, cnt);
}

TEST(PointKdTreeSearcher3, ForEachNearbyPointEmpty)
{
    Array1<Vector3D> points;

    PointKdTreeSearcher3 searcher;
    searcher.Build(points);

    searcher.ForEachNearbyPoint(Vector3D(0, 0, 0), std::sqrt(10.0),
                                [](size_t, const Vector3D&) {
                                    // Do nothing
                                });
}

TEST(PointKdTreeSearcher3, CopyConstructor)
{
    Array1<Vector3D> points = { Vector3D(0, 1, 3), Vector3D(2, 5, 4),
                                Vector3D(-1, 3, 0) };

    PointKdTreeSearcher3 searcher;
    searcher.Build(points);

    const PointKdTreeSearcher3 searcher2(searcher);
    int cnt = 0;
    searcher2.ForEachNearbyPoint(Vector3D(0, 0, 0), std::sqrt(10.0),
                                 [&points, &cnt](size_t i, const Vector3D& pt) {
                                     EXPECT_TRUE(i == 0 || i == 2);

                                     if (i == 0)
                                     {
                                         EXPECT_EQ(points[0], pt);
                                     }
                                     else if (i == 2)
                                     {
                                         EXPECT_EQ(points[2], pt);
                                     }

                                     ++cnt;
                                 });

    EXPECT_EQ(2, cnt);
}

TEST(PointKdTreeSearcher3, Serialize)
{
    Array1<Vector3D> points = { Vector3D(0, 1, 3), Vector3D(2, 5, 4),
                                Vector3D(-1, 3, 0) };

    PointKdTreeSearcher3 searcher;
    searcher.Build(points);

    std::vector<uint8_t> buffer;
    searcher.Serialize(&buffer);

    PointKdTreeSearcher3 searcher2;
    searcher2.Deserialize(buffer);

    int cnt = 0;
    searcher2.ForEachNearbyPoint(Vector3D(0, 0, 0), std::sqrt(10.0),
                                 [&points, &cnt](size_t i, const Vector3D& pt) {
                                     EXPECT_TRUE(i == 0 || i == 2);

                                     if (i == 0)
                                     {
                                         EXPECT_EQ(points[0], pt);
                                     }
                                     else if (i == 2)
                                     {
                                         EXPECT_EQ(points[2], pt);
                                     }

                                     ++cnt;
                                 });

    EXPECT_EQ(2, cnt);
}