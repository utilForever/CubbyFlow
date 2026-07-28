#include "gtest/gtest.h"

#include <Core/Searcher/PointSimpleListSearcher.hpp>

#include <utility>

using namespace CubbyFlow;

TEST(PointSimpleListSearcher3, ForEachNearByPoint)
{
    Array1<Vector3D> points = { Vector3D(1, 1, 3), Vector3D(3, 4, 1),
                                Vector3D(-1, 2, 1) };

    PointSimpleListSearcher3 searcher;
    searcher.Build(points);

    searcher.ForEachNearbyPoint(Vector3D(0, 0, 1), std::sqrt(15.0),
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

TEST(PointSimpleListSearcher3, CopyMoveSerializationAndBuilder)
{
    const Array1<Vector3D> points = { Vector3D(1, 1, 3), Vector3D(3, 4, 1),
                                      Vector3D(-1, 2, 1) };

    PointSimpleListSearcher3 searcher;
    searcher.Build(points);

    PointSimpleListSearcher3 copied(searcher);
    PointSimpleListSearcher3 copyAssigned;
    copyAssigned = copied;

    PointSimpleListSearcher3 moved(std::move(copied));
    PointSimpleListSearcher3 moveAssigned;
    moveAssigned = std::move(moved);

    PointSimpleListSearcher3 set;
    set.Set(copyAssigned);
    const auto cloned = set.Clone();

    EXPECT_TRUE(cloned->HasNearbyPoint(Vector3D(1, 1, 3), 0.0));
    EXPECT_FALSE(cloned->HasNearbyPoint(Vector3D(10, 10, 10), 1.0));

    std::vector<uint8_t> buffer;
    moveAssigned.Serialize(&buffer);
    ASSERT_FALSE(buffer.empty());

    PointSimpleListSearcher3 restored;
    restored.Deserialize(buffer);
    EXPECT_TRUE(restored.HasNearbyPoint(Vector3D(-1, 2, 1), 0.0));

    EXPECT_EQ("PointSimpleListSearcher3",
              PointSimpleListSearcher3::GetBuilder().Build().TypeName());
    EXPECT_EQ("PointSimpleListSearcher3",
              PointSimpleListSearcher3::GetBuilder().MakeShared()->TypeName());
    EXPECT_EQ("PointSimpleListSearcher3", PointSimpleListSearcher3::GetBuilder()
                                              .BuildPointNeighborSearcher()
                                              ->TypeName());
}
