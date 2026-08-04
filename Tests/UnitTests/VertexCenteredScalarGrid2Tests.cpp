#include "gtest/gtest.h"

#include <Core/Grid/VertexCenteredScalarGrid.hpp>

#include <utility>

using namespace CubbyFlow;

TEST(VertexCenteredScalarGrid2, Constructors)
{
    // Default constructors
    VertexCenteredScalarGrid2 grid1;
    EXPECT_EQ(0u, grid1.Resolution().x);
    EXPECT_EQ(0u, grid1.Resolution().y);
    EXPECT_DOUBLE_EQ(1.0, grid1.GridSpacing().x);
    EXPECT_DOUBLE_EQ(1.0, grid1.GridSpacing().y);
    EXPECT_DOUBLE_EQ(0.0, grid1.Origin().x);
    EXPECT_DOUBLE_EQ(0.0, grid1.Origin().y);
    EXPECT_EQ(0u, grid1.DataSize().x);
    EXPECT_EQ(0u, grid1.DataSize().y);
    EXPECT_DOUBLE_EQ(0.0, grid1.DataOrigin().x);
    EXPECT_DOUBLE_EQ(0.0, grid1.DataOrigin().y);

    // Constructor with params
    VertexCenteredScalarGrid2 grid2({ 5, 4 }, { 1.0, 2.0 }, { 3.0, 4.0 }, 5.0);
    EXPECT_EQ(5u, grid2.Resolution().x);
    EXPECT_EQ(4u, grid2.Resolution().y);
    EXPECT_DOUBLE_EQ(1.0, grid2.GridSpacing().x);
    EXPECT_DOUBLE_EQ(2.0, grid2.GridSpacing().y);
    EXPECT_DOUBLE_EQ(3.0, grid2.Origin().x);
    EXPECT_DOUBLE_EQ(4.0, grid2.Origin().y);
    EXPECT_EQ(6u, grid2.DataSize().x);
    EXPECT_EQ(5u, grid2.DataSize().y);
    EXPECT_DOUBLE_EQ(3.0, grid2.DataOrigin().x);
    EXPECT_DOUBLE_EQ(4.0, grid2.DataOrigin().y);
    grid2.ForEachDataPointIndex(
        [&grid2](size_t i, size_t j) { EXPECT_DOUBLE_EQ(5.0, grid2(i, j)); });

    // Copy constructor
    VertexCenteredScalarGrid2 grid3(grid2);
    EXPECT_EQ(5u, grid3.Resolution().x);
    EXPECT_EQ(4u, grid3.Resolution().y);
    EXPECT_DOUBLE_EQ(1.0, grid3.GridSpacing().x);
    EXPECT_DOUBLE_EQ(2.0, grid3.GridSpacing().y);
    EXPECT_DOUBLE_EQ(3.0, grid3.Origin().x);
    EXPECT_DOUBLE_EQ(4.0, grid3.Origin().y);
    EXPECT_EQ(6u, grid3.DataSize().x);
    EXPECT_EQ(5u, grid3.DataSize().y);
    EXPECT_DOUBLE_EQ(3.0, grid3.DataOrigin().x);
    EXPECT_DOUBLE_EQ(4.0, grid3.DataOrigin().y);
    grid3.ForEachDataPointIndex(
        [&grid3](size_t i, size_t j) { EXPECT_DOUBLE_EQ(5.0, grid3(i, j)); });
}

TEST(VertexCenteredScalarGrid2, MoveSemantics)
{
    VertexCenteredScalarGrid2 source({ 2, 2 }, { 1.0, 2.0 }, { 3.0, 4.0 }, 5.0);
    VertexCenteredScalarGrid2 moved(std::move(source));

    EXPECT_EQ(Vector2UZ(2, 2), moved.Resolution());
    EXPECT_DOUBLE_EQ(5.0, moved(0, 0));
    EXPECT_DOUBLE_EQ(5.0, moved.Sample(moved.DataOrigin()));

    VertexCenteredScalarGrid2 assigned;
    assigned = std::move(moved);

    EXPECT_EQ(Vector2UZ(2, 2), assigned.Resolution());
    EXPECT_DOUBLE_EQ(5.0, assigned(0, 0));
    EXPECT_DOUBLE_EQ(5.0, assigned.Sample(assigned.DataOrigin()));
}

TEST(VertexCenteredScalarGrid2, Swap)
{
    VertexCenteredScalarGrid2 grid1({ 5, 4 }, { 1.0, 2.0 }, { 3.0, 4.0 }, 5.0);
    VertexCenteredScalarGrid2 grid2({ 3, 8 }, { 2.0, 3.0 }, { 1.0, 5.0 }, 4.0);
    grid1.Swap(&grid2);

    EXPECT_EQ(3u, grid1.Resolution().x);
    EXPECT_EQ(8u, grid1.Resolution().y);
    EXPECT_DOUBLE_EQ(2.0, grid1.GridSpacing().x);
    EXPECT_DOUBLE_EQ(3.0, grid1.GridSpacing().y);
    EXPECT_DOUBLE_EQ(1.0, grid1.Origin().x);
    EXPECT_DOUBLE_EQ(5.0, grid1.Origin().y);
    EXPECT_EQ(4u, grid1.DataSize().x);
    EXPECT_EQ(9u, grid1.DataSize().y);
    EXPECT_DOUBLE_EQ(1.0, grid1.DataOrigin().x);
    EXPECT_DOUBLE_EQ(5.0, grid1.DataOrigin().y);
    grid1.ForEachDataPointIndex(
        [&grid1](size_t i, size_t j) { EXPECT_DOUBLE_EQ(4.0, grid1(i, j)); });

    EXPECT_EQ(5u, grid2.Resolution().x);
    EXPECT_EQ(4u, grid2.Resolution().y);
    EXPECT_DOUBLE_EQ(1.0, grid2.GridSpacing().x);
    EXPECT_DOUBLE_EQ(2.0, grid2.GridSpacing().y);
    EXPECT_DOUBLE_EQ(3.0, grid2.Origin().x);
    EXPECT_DOUBLE_EQ(4.0, grid2.Origin().y);
    EXPECT_EQ(6u, grid2.DataSize().x);
    EXPECT_EQ(5u, grid2.DataSize().y);
    EXPECT_DOUBLE_EQ(3.0, grid2.DataOrigin().x);
    EXPECT_DOUBLE_EQ(4.0, grid2.DataOrigin().y);
    grid2.ForEachDataPointIndex(
        [&grid2](size_t i, size_t j) { EXPECT_DOUBLE_EQ(5.0, grid2(i, j)); });
}

TEST(VertexCenteredScalarGrid2, Set)
{
    VertexCenteredScalarGrid2 grid1({ 5, 4 }, { 1.0, 2.0 }, { 3.0, 4.0 }, 5.0);
    VertexCenteredScalarGrid2 grid2({ 3, 8 }, { 2.0, 3.0 }, { 1.0, 5.0 }, 4.0);
    grid1.Set(grid2);

    EXPECT_EQ(3u, grid1.Resolution().x);
    EXPECT_EQ(8u, grid1.Resolution().y);
    EXPECT_DOUBLE_EQ(2.0, grid1.GridSpacing().x);
    EXPECT_DOUBLE_EQ(3.0, grid1.GridSpacing().y);
    EXPECT_DOUBLE_EQ(1.0, grid1.Origin().x);
    EXPECT_DOUBLE_EQ(5.0, grid1.Origin().y);
    EXPECT_EQ(4u, grid1.DataSize().x);
    EXPECT_EQ(9u, grid1.DataSize().y);
    EXPECT_DOUBLE_EQ(1.0, grid1.DataOrigin().x);
    EXPECT_DOUBLE_EQ(5.0, grid1.DataOrigin().y);
    grid1.ForEachDataPointIndex(
        [&grid1](size_t i, size_t j) { EXPECT_DOUBLE_EQ(4.0, grid1(i, j)); });
}

TEST(VertexCenteredScalarGrid2, AssignmentOperator)
{
    VertexCenteredScalarGrid2 grid1;
    VertexCenteredScalarGrid2 grid2({ 3, 8 }, { 2.0, 3.0 }, { 1.0, 5.0 }, 4.0);
    grid1 = grid2;

    EXPECT_EQ(3u, grid1.Resolution().x);
    EXPECT_EQ(8u, grid1.Resolution().y);
    EXPECT_DOUBLE_EQ(2.0, grid1.GridSpacing().x);
    EXPECT_DOUBLE_EQ(3.0, grid1.GridSpacing().y);
    EXPECT_DOUBLE_EQ(1.0, grid1.Origin().x);
    EXPECT_DOUBLE_EQ(5.0, grid1.Origin().y);
    EXPECT_EQ(4u, grid1.DataSize().x);
    EXPECT_EQ(9u, grid1.DataSize().y);
    EXPECT_DOUBLE_EQ(1.0, grid1.DataOrigin().x);
    EXPECT_DOUBLE_EQ(5.0, grid1.DataOrigin().y);
    grid1.ForEachDataPointIndex(
        [&grid1](size_t i, size_t j) { EXPECT_DOUBLE_EQ(4.0, grid1(i, j)); });
}

TEST(VertexCenteredScalarGrid2, Clone)
{
    VertexCenteredScalarGrid2 grid2({ 3, 8 }, { 2.0, 3.0 }, { 1.0, 5.0 }, 4.0);
    auto grid1 = grid2.Clone();

    EXPECT_EQ(3u, grid1->Resolution().x);
    EXPECT_EQ(8u, grid1->Resolution().y);
    EXPECT_DOUBLE_EQ(2.0, grid1->GridSpacing().x);
    EXPECT_DOUBLE_EQ(3.0, grid1->GridSpacing().y);
    EXPECT_DOUBLE_EQ(1.0, grid1->Origin().x);
    EXPECT_DOUBLE_EQ(5.0, grid1->Origin().y);
    EXPECT_EQ(4u, grid1->DataSize().x);
    EXPECT_EQ(9u, grid1->DataSize().y);
    EXPECT_DOUBLE_EQ(1.0, grid1->DataOrigin().x);
    EXPECT_DOUBLE_EQ(5.0, grid1->DataOrigin().y);
    grid1->ForEachDataPointIndex([&grid1](size_t i, size_t j) {
        EXPECT_DOUBLE_EQ(4.0, (*grid1)(i, j));
    });
}

TEST(VertexCenteredScalarGrid2, Builder)
{
    {
        auto grid1 = VertexCenteredScalarGrid2::GetBuilder().Build(
            Vector2UZ(3, 8), Vector2D(2.0, 3.0), Vector2D(1.0, 5.0), 4.0);

        auto grid2 =
            std::dynamic_pointer_cast<VertexCenteredScalarGrid2>(grid1);
        EXPECT_TRUE(grid2 != nullptr);

        EXPECT_EQ(3u, grid1->Resolution().x);
        EXPECT_EQ(8u, grid1->Resolution().y);
        EXPECT_DOUBLE_EQ(2.0, grid1->GridSpacing().x);
        EXPECT_DOUBLE_EQ(3.0, grid1->GridSpacing().y);
        EXPECT_DOUBLE_EQ(1.0, grid1->Origin().x);
        EXPECT_DOUBLE_EQ(5.0, grid1->Origin().y);
        EXPECT_EQ(4u, grid1->DataSize().x);
        EXPECT_EQ(9u, grid1->DataSize().y);
        EXPECT_DOUBLE_EQ(1.0, grid1->DataOrigin().x);
        EXPECT_DOUBLE_EQ(5.0, grid1->DataOrigin().y);
        grid1->ForEachDataPointIndex([&grid1](size_t i, size_t j) {
            EXPECT_DOUBLE_EQ(4.0, (*grid1)(i, j));
        });
    }

    {
        auto grid1 = VertexCenteredScalarGrid2::GetBuilder()
                         .WithResolution({ 3, 8 })
                         .WithGridSpacing({ 2, 3 })
                         .WithOrigin({ 1, 5 })
                         .WithInitialValue(4)
                         .Build();

        EXPECT_EQ(3u, grid1.Resolution().x);
        EXPECT_EQ(8u, grid1.Resolution().y);
        EXPECT_DOUBLE_EQ(2.0, grid1.GridSpacing().x);
        EXPECT_DOUBLE_EQ(3.0, grid1.GridSpacing().y);
        EXPECT_DOUBLE_EQ(1.0, grid1.Origin().x);
        EXPECT_DOUBLE_EQ(5.0, grid1.Origin().y);
        EXPECT_EQ(4u, grid1.DataSize().x);
        EXPECT_EQ(9u, grid1.DataSize().y);
        EXPECT_DOUBLE_EQ(1.0, grid1.DataOrigin().x);
        EXPECT_DOUBLE_EQ(5.0, grid1.DataOrigin().y);
        grid1.ForEachDataPointIndex([&grid1](size_t i, size_t j) {
            EXPECT_DOUBLE_EQ(4.0, grid1(i, j));
        });
    }
}

TEST(VertexCenteredScalarGrid2, Fill)
{
    VertexCenteredScalarGrid2 grid({ 5, 4 }, { 1.0, 1.0 }, { 0.0, 0.0 }, 0.0);
    grid.Fill(42.0);

    for (size_t j = 0; j < grid.DataSize().y; ++j)
    {
        for (size_t i = 0; i < grid.DataSize().x; ++i)
        {
            EXPECT_DOUBLE_EQ(42.0, grid(i, j));
        }
    }

    auto func = [](const Vector2D& x) {
        if (x.x < 3.0)
        {
            return 2.0;
        }
        else
        {
            return 5.0;
        }
    };
    grid.Fill(func);

    for (size_t j = 0; j < 4; ++j)
    {
        for (size_t i = 0; i < 5; ++i)
        {
            if (i < 3)
            {
                EXPECT_DOUBLE_EQ(2.0, grid(i, j));
            }
            else
            {
                EXPECT_DOUBLE_EQ(5.0, grid(i, j));
            }
        }
    }
}

TEST(VertexCenteredScalarGrid2, Serialization)
{
    VertexCenteredScalarGrid2 grid1({ 5, 4 }, { 1.0, 2.0 }, { -5.0, 3.0 });
    grid1.Fill([](const Vector2D& pt) { return pt.x + pt.y; });

    // Serialize to in-memory stream
    std::vector<uint8_t> buffer1;
    grid1.Serialize(&buffer1);

    // Deserialize to non-zero array
    VertexCenteredScalarGrid2 grid2({ 1, 2 }, { 0.5, 1.0 }, { 0.5, 2.0 });
    grid2.Deserialize(buffer1);
    EXPECT_EQ(5u, grid2.Resolution().x);
    EXPECT_EQ(4u, grid2.Resolution().y);
    EXPECT_DOUBLE_EQ(-5.0, grid2.Origin().x);
    EXPECT_DOUBLE_EQ(3.0, grid2.Origin().y);
    EXPECT_DOUBLE_EQ(1.0, grid2.GridSpacing().x);
    EXPECT_DOUBLE_EQ(2.0, grid2.GridSpacing().y);
    EXPECT_DOUBLE_EQ(-5.0, grid2.GetBoundingBox().lowerCorner.x);
    EXPECT_DOUBLE_EQ(3.0, grid2.GetBoundingBox().lowerCorner.y);
    EXPECT_DOUBLE_EQ(0.0, grid2.GetBoundingBox().upperCorner.x);
    EXPECT_DOUBLE_EQ(11.0, grid2.GetBoundingBox().upperCorner.y);

    grid1.ForEachDataPointIndex([&grid1, &grid2](size_t i, size_t j) {
        EXPECT_DOUBLE_EQ(grid1(i, j), grid2(i, j));
    });

    // Serialize zero-sized array
    VertexCenteredScalarGrid2 grid3;
    std::vector<uint8_t> buffer2;
    grid3.Serialize(&buffer2);

    // Deserialize to non-zero array
    grid2.Deserialize(buffer2);
    EXPECT_EQ(0u, grid2.Resolution().x);
    EXPECT_EQ(0u, grid2.Resolution().y);
    EXPECT_DOUBLE_EQ(0.0, grid2.Origin().x);
    EXPECT_DOUBLE_EQ(0.0, grid2.Origin().y);
    EXPECT_DOUBLE_EQ(1.0, grid2.GridSpacing().x);
    EXPECT_DOUBLE_EQ(1.0, grid2.GridSpacing().y);
}
