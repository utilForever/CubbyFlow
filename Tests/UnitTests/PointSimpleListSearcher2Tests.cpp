#include "gtest/gtest.h"

#include <Core/Searcher/PointSimpleListSearcher.hpp>

#include <utility>

using namespace CubbyFlow;

TEST(PointSimpleListSearcher2, ForEachNearByPoint)
{
    Array1<Vector2D> points = { Vector2D(1, 1), Vector2D(3, 4),
                                Vector2D(-1, 2) };

    PointSimpleListSearcher2 searcher;
    searcher.Build(points);

    searcher.ForEachNearbyPoint(Vector2D(0, 0), std::sqrt(15.0),
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

TEST(PointSimpleListSearcher2, CopyMoveSerializationAndBuilder)
{
    const Array1<Vector2D> points = { Vector2D(1, 1), Vector2D(3, 4),
                                      Vector2D(-1, 2) };

    PointSimpleListSearcher2 searcher;
    searcher.Build(points);

    PointSimpleListSearcher2 copied(searcher);
    PointSimpleListSearcher2 copyAssigned;
    copyAssigned = copied;

    PointSimpleListSearcher2 moved(std::move(copied));
    PointSimpleListSearcher2 moveAssigned;
    moveAssigned = std::move(moved);

    PointSimpleListSearcher2 set;
    set.Set(copyAssigned);
    const auto cloned = set.Clone();

    EXPECT_TRUE(cloned->HasNearbyPoint(Vector2D(1, 1), 0.0));
    EXPECT_FALSE(cloned->HasNearbyPoint(Vector2D(10, 10), 1.0));

    std::vector<uint8_t> buffer;
    moveAssigned.Serialize(&buffer);
    ASSERT_FALSE(buffer.empty());

    PointSimpleListSearcher2 restored;
    restored.Deserialize(buffer);
    EXPECT_TRUE(restored.HasNearbyPoint(Vector2D(-1, 2), 0.0));

    EXPECT_EQ("PointSimpleListSearcher2",
              PointSimpleListSearcher2::GetBuilder().Build().TypeName());
    EXPECT_EQ("PointSimpleListSearcher2",
              PointSimpleListSearcher2::GetBuilder().MakeShared()->TypeName());
    EXPECT_EQ("PointSimpleListSearcher2", PointSimpleListSearcher2::GetBuilder()
                                              .BuildPointNeighborSearcher()
                                              ->TypeName());
}
