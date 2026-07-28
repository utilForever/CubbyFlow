#include "gtest/gtest.h"

#include <Core/PointGenerator/BccLatticePointGenerator.hpp>
#include <Core/Searcher/PointHashGridSearcher.hpp>
#include <Core/Searcher/PointParallelHashGridSearcher.hpp>
#include <Core/Utils/PointHashGridUtils.hpp>

using namespace CubbyFlow;

TEST(PointParallelHashGridSearcher3, ForEachNearByPoint)
{
    Array1<Vector3D> points = { Vector3D{ 1, 1, 1 }, Vector3D{ 3, 411, 5 },
                                Vector3D{ -1, 2, -3 } };

    PointParallelHashGridSearcher3 searcher{ Vector3UZ{ 4, 4, 4 },
                                             std::sqrt(10) };
    searcher.Build(points);

    searcher.ForEachNearbyPoint(Vector3D{ 0, 0, 0 }, std::sqrt(15.0),
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

TEST(PointParallelHashGridSearcher3, HasEachNearByPoint)
{
    const Array1<Vector3D> points = { Vector3D{ 1, 142, 1 },
                                      Vector3D{ 3, 4123, 13 },
                                      Vector3D{ 4, 1, 25 } };

    PointParallelHashGridSearcher3 searcher{ Vector3UZ{ 4, 4, 4 },
                                             std::sqrt(10) };
    searcher.Build(points);

    const bool result = searcher.HasNearbyPoint(Vector3D{}, std::sqrt(15.0));

    EXPECT_FALSE(result);
}

TEST(PointParallelHashGridSearcher3, Build)
{
    Array1<Vector3D> points;
    BccLatticePointGenerator pointsGenerator;
    BoundingBox3D bbox{ Vector3D{ 0, 0, 0 }, Vector3D{ 1, 1, 1 } };
    double spacing = 0.1;

    pointsGenerator.Generate(bbox, spacing, &points);

    PointHashGridSearcher3 pointSearcher{ Vector3UZ{ 4, 4, 4 }, 0.18 };
    pointSearcher.Build(points, std::numeric_limits<double>::max());

    Array3<size_t> grid(4, 4, 4);

    ForEachIndex(
        grid.Size(), [&pointSearcher, &grid](size_t i, size_t j, size_t k) {
            const size_t key = PointHashGridUtils3::GetHashKeyFromBucketIndex(
                Vector3Z{ static_cast<ssize_t>(i), static_cast<ssize_t>(j),
                          static_cast<ssize_t>(k) },
                Vector3Z{ 4, 4, 4 });
            const size_t value = pointSearcher.Buckets()[key].Length();
            grid(i, j, k) = value;
        });

    PointParallelHashGridSearcher3 parallelSearcher{ Vector3UZ{ 4, 4, 4 },
                                                     0.18 };
    parallelSearcher.Build(points, std::numeric_limits<double>::max());

    ForEachIndex(
        grid.Size(), [&parallelSearcher, &grid](size_t i, size_t j, size_t k) {
            const size_t key = PointHashGridUtils3::GetHashKeyFromBucketIndex(
                Vector3Z{ static_cast<ssize_t>(i), static_cast<ssize_t>(j),
                          static_cast<ssize_t>(k) },
                Vector3Z{ 4, 4, 4 });
            const size_t start = parallelSearcher.StartIndexTable()[key];
            const size_t end = parallelSearcher.EndIndexTable()[key];
            const size_t value = end - start;
            EXPECT_EQ(grid(i, j, k), value);
        });
}

TEST(PointParallelHashGridSearcher3, Serialization)
{
    Array1<Vector3D> points = { Vector3D(0, 1, 3), Vector3D(2, 5, 4),
                                Vector3D(-1, 3, 0) };

    PointParallelHashGridSearcher3 searcher(Vector3UZ{ 4, 4, 4 },
                                            std::sqrt(10));
    searcher.Build(points);

    std::vector<uint8_t> buffer;
    searcher.Serialize(&buffer);

    PointParallelHashGridSearcher3 searcher2(Vector3UZ{ 1, 1, 1 }, 1.0);
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