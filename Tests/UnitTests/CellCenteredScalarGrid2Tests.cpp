#include "gtest/gtest.h"

#include <Core/Grid/CellCenteredScalarGrid.hpp>

#include <utility>

using namespace CubbyFlow;

TEST(CellCenteredScalarGrid2, Constructors)
{
    // Default constructors
    CellCenteredScalarGrid2 grid1;
    EXPECT_EQ(0u, grid1.Resolution().x);
    EXPECT_EQ(0u, grid1.Resolution().y);
    EXPECT_DOUBLE_EQ(1.0, grid1.GridSpacing().x);
    EXPECT_DOUBLE_EQ(1.0, grid1.GridSpacing().y);
    EXPECT_DOUBLE_EQ(0.0, grid1.Origin().x);
    EXPECT_DOUBLE_EQ(0.0, grid1.Origin().y);
    EXPECT_EQ(0u, grid1.DataSize().x);
    EXPECT_EQ(0u, grid1.DataSize().y);
    EXPECT_DOUBLE_EQ(0.5, grid1.DataOrigin().x);
    EXPECT_DOUBLE_EQ(0.5, grid1.DataOrigin().y);

    // Constructor with params
    CellCenteredScalarGrid2 grid2({ 5, 4 }, { 1.0, 2.0 }, { 3.0, 4.0 }, 5.0);
    EXPECT_EQ(5u, grid2.Resolution().x);
    EXPECT_EQ(4u, grid2.Resolution().y);
    EXPECT_DOUBLE_EQ(1.0, grid2.GridSpacing().x);
    EXPECT_DOUBLE_EQ(2.0, grid2.GridSpacing().y);
    EXPECT_DOUBLE_EQ(3.0, grid2.Origin().x);
    EXPECT_DOUBLE_EQ(4.0, grid2.Origin().y);
    EXPECT_EQ(5u, grid2.DataSize().x);
    EXPECT_EQ(4u, grid2.DataSize().y);
    EXPECT_DOUBLE_EQ(3.5, grid2.DataOrigin().x);
    EXPECT_DOUBLE_EQ(5.0, grid2.DataOrigin().y);
    grid2.ForEachDataPointIndex(
        [&grid2](size_t i, size_t j) { EXPECT_DOUBLE_EQ(5.0, grid2(i, j)); });

    // Copy constructor
    CellCenteredScalarGrid2 grid3(grid2);
    EXPECT_EQ(5u, grid3.Resolution().x);
    EXPECT_EQ(4u, grid3.Resolution().y);
    EXPECT_DOUBLE_EQ(1.0, grid3.GridSpacing().x);
    EXPECT_DOUBLE_EQ(2.0, grid3.GridSpacing().y);
    EXPECT_DOUBLE_EQ(3.0, grid3.Origin().x);
    EXPECT_DOUBLE_EQ(4.0, grid3.Origin().y);
    EXPECT_EQ(5u, grid3.DataSize().x);
    EXPECT_EQ(4u, grid3.DataSize().y);
    EXPECT_DOUBLE_EQ(3.5, grid3.DataOrigin().x);
    EXPECT_DOUBLE_EQ(5.0, grid3.DataOrigin().y);
    grid3.ForEachDataPointIndex(
        [&grid3](size_t i, size_t j) { EXPECT_DOUBLE_EQ(5.0, grid3(i, j)); });
}

TEST(CellCenteredScalarGrid2, MoveSemantics)
{
    CellCenteredScalarGrid2 source({ 2, 2 }, { 1.0, 2.0 }, { 3.0, 4.0 }, 5.0);
    CellCenteredScalarGrid2 moved(std::move(source));

    EXPECT_EQ(Vector2UZ(2, 2), moved.Resolution());
    EXPECT_DOUBLE_EQ(5.0, moved(0, 0));
    EXPECT_DOUBLE_EQ(5.0, moved.Sample(moved.DataOrigin()));

    CellCenteredScalarGrid2 assigned;
    assigned = std::move(moved);

    EXPECT_EQ(Vector2UZ(2, 2), assigned.Resolution());
    EXPECT_DOUBLE_EQ(5.0, assigned(0, 0));
    EXPECT_DOUBLE_EQ(5.0, assigned.Sample(assigned.DataOrigin()));
}

TEST(CellCenteredScalarGrid2, Swap)
{
    CellCenteredScalarGrid2 grid1({ 5, 4 }, { 1.0, 2.0 }, { 3.0, 4.0 }, 5.0);
    CellCenteredScalarGrid2 grid2({ 3, 8 }, { 2.0, 3.0 }, { 1.0, 5.0 }, 4.0);
    grid1.Swap(&grid2);

    EXPECT_EQ(3u, grid1.Resolution().x);
    EXPECT_EQ(8u, grid1.Resolution().y);
    EXPECT_DOUBLE_EQ(2.0, grid1.GridSpacing().x);
    EXPECT_DOUBLE_EQ(3.0, grid1.GridSpacing().y);
    EXPECT_DOUBLE_EQ(1.0, grid1.Origin().x);
    EXPECT_DOUBLE_EQ(5.0, grid1.Origin().y);
    EXPECT_EQ(3u, grid1.DataSize().x);
    EXPECT_EQ(8u, grid1.DataSize().y);
    EXPECT_DOUBLE_EQ(2.0, grid1.DataOrigin().x);
    EXPECT_DOUBLE_EQ(6.5, grid1.DataOrigin().y);
    grid1.ForEachDataPointIndex(
        [&grid1](size_t i, size_t j) { EXPECT_DOUBLE_EQ(4.0, grid1(i, j)); });

    EXPECT_EQ(5u, grid2.Resolution().x);
    EXPECT_EQ(4u, grid2.Resolution().y);
    EXPECT_DOUBLE_EQ(1.0, grid2.GridSpacing().x);
    EXPECT_DOUBLE_EQ(2.0, grid2.GridSpacing().y);
    EXPECT_DOUBLE_EQ(3.0, grid2.Origin().x);
    EXPECT_DOUBLE_EQ(4.0, grid2.Origin().y);
    EXPECT_EQ(5u, grid2.DataSize().x);
    EXPECT_EQ(4u, grid2.DataSize().y);
    EXPECT_DOUBLE_EQ(3.5, grid2.DataOrigin().x);
    EXPECT_DOUBLE_EQ(5.0, grid2.DataOrigin().y);
    grid2.ForEachDataPointIndex(
        [&grid2](size_t i, size_t j) { EXPECT_DOUBLE_EQ(5.0, grid2(i, j)); });
}

TEST(CellCenteredScalarGrid2, Set)
{
    CellCenteredScalarGrid2 grid1({ 5, 4 }, { 1.0, 2.0 }, { 3.0, 4.0 }, 5.0);
    CellCenteredScalarGrid2 grid2({ 3, 8 }, { 2.0, 3.0 }, { 1.0, 5.0 }, 4.0);
    grid1.Set(grid2);

    EXPECT_EQ(3u, grid1.Resolution().x);
    EXPECT_EQ(8u, grid1.Resolution().y);
    EXPECT_DOUBLE_EQ(2.0, grid1.GridSpacing().x);
    EXPECT_DOUBLE_EQ(3.0, grid1.GridSpacing().y);
    EXPECT_DOUBLE_EQ(1.0, grid1.Origin().x);
    EXPECT_DOUBLE_EQ(5.0, grid1.Origin().y);
    EXPECT_EQ(3u, grid1.DataSize().x);
    EXPECT_EQ(8u, grid1.DataSize().y);
    EXPECT_DOUBLE_EQ(2.0, grid1.DataOrigin().x);
    EXPECT_DOUBLE_EQ(6.5, grid1.DataOrigin().y);
    grid1.ForEachDataPointIndex(
        [&grid1](size_t i, size_t j) { EXPECT_DOUBLE_EQ(4.0, grid1(i, j)); });
}

TEST(CellCenteredScalarGrid2, AssignmentOperator)
{
    CellCenteredScalarGrid2 grid1;
    CellCenteredScalarGrid2 grid2({ 3, 8 }, { 2.0, 3.0 }, { 1.0, 5.0 }, 4.0);
    grid1 = grid2;

    EXPECT_EQ(3u, grid1.Resolution().x);
    EXPECT_EQ(8u, grid1.Resolution().y);
    EXPECT_DOUBLE_EQ(2.0, grid1.GridSpacing().x);
    EXPECT_DOUBLE_EQ(3.0, grid1.GridSpacing().y);
    EXPECT_DOUBLE_EQ(1.0, grid1.Origin().x);
    EXPECT_DOUBLE_EQ(5.0, grid1.Origin().y);
    EXPECT_EQ(3u, grid1.DataSize().x);
    EXPECT_EQ(8u, grid1.DataSize().y);
    EXPECT_DOUBLE_EQ(2.0, grid1.DataOrigin().x);
    EXPECT_DOUBLE_EQ(6.5, grid1.DataOrigin().y);
    grid1.ForEachDataPointIndex(
        [&grid1](size_t i, size_t j) { EXPECT_DOUBLE_EQ(4.0, grid1(i, j)); });
}

TEST(CellCenteredScalarGrid2, Clone)
{
    CellCenteredScalarGrid2 grid2({ 3, 8 }, { 2.0, 3.0 }, { 1.0, 5.0 }, 4.0);
    auto grid1 = grid2.Clone();

    EXPECT_EQ(3u, grid1->Resolution().x);
    EXPECT_EQ(8u, grid1->Resolution().y);
    EXPECT_DOUBLE_EQ(2.0, grid1->GridSpacing().x);
    EXPECT_DOUBLE_EQ(3.0, grid1->GridSpacing().y);
    EXPECT_DOUBLE_EQ(1.0, grid1->Origin().x);
    EXPECT_DOUBLE_EQ(5.0, grid1->Origin().y);
    EXPECT_EQ(3u, grid1->DataSize().x);
    EXPECT_EQ(8u, grid1->DataSize().y);
    EXPECT_DOUBLE_EQ(2.0, grid1->DataOrigin().x);
    EXPECT_DOUBLE_EQ(6.5, grid1->DataOrigin().y);
    grid1->ForEachDataPointIndex([&grid1](size_t i, size_t j) {
        EXPECT_DOUBLE_EQ(4.0, (*grid1)(i, j));
    });
}

TEST(CellCenteredScalarGrid2, Builder)
{
    {
        auto grid1 = CellCenteredScalarGrid2::GetBuilder().Build(
            Vector2UZ(3, 8), Vector2D(2.0, 3.0), Vector2D(1.0, 5.0), 4.0);

        auto grid2 = std::dynamic_pointer_cast<CellCenteredScalarGrid2>(grid1);
        EXPECT_TRUE(grid2 != nullptr);

        EXPECT_EQ(3u, grid1->Resolution().x);
        EXPECT_EQ(8u, grid1->Resolution().y);
        EXPECT_DOUBLE_EQ(2.0, grid1->GridSpacing().x);
        EXPECT_DOUBLE_EQ(3.0, grid1->GridSpacing().y);
        EXPECT_DOUBLE_EQ(1.0, grid1->Origin().x);
        EXPECT_DOUBLE_EQ(5.0, grid1->Origin().y);
        EXPECT_EQ(3u, grid1->DataSize().x);
        EXPECT_EQ(8u, grid1->DataSize().y);
        EXPECT_DOUBLE_EQ(2.0, grid1->DataOrigin().x);
        EXPECT_DOUBLE_EQ(6.5, grid1->DataOrigin().y);
        grid1->ForEachDataPointIndex([&grid1](size_t i, size_t j) {
            EXPECT_DOUBLE_EQ(4.0, (*grid1)(i, j));
        });
    }

    {
        auto grid1 = CellCenteredScalarGrid2::GetBuilder()
                         .WithResolution({ 3, 8 })
                         .WithGridSpacing({ 2, 3 })
                         .WithOrigin({ 1, 5 })
                         .WithInitialValue(4.0)
                         .Build();

        EXPECT_EQ(3u, grid1.Resolution().x);
        EXPECT_EQ(8u, grid1.Resolution().y);
        EXPECT_DOUBLE_EQ(2.0, grid1.GridSpacing().x);
        EXPECT_DOUBLE_EQ(3.0, grid1.GridSpacing().y);
        EXPECT_DOUBLE_EQ(1.0, grid1.Origin().x);
        EXPECT_DOUBLE_EQ(5.0, grid1.Origin().y);
        EXPECT_EQ(3u, grid1.DataSize().x);
        EXPECT_EQ(8u, grid1.DataSize().y);
        EXPECT_DOUBLE_EQ(2.0, grid1.DataOrigin().x);
        EXPECT_DOUBLE_EQ(6.5, grid1.DataOrigin().y);
        grid1.ForEachDataPointIndex([&grid1](size_t i, size_t j) {
            EXPECT_DOUBLE_EQ(4.0, grid1(i, j));
        });
    }
}

TEST(CellCenteredScalarGrid2, Fill)
{
    CellCenteredScalarGrid2 grid({ 5, 4 }, { 1.0, 1.0 }, { 0.0, 0.0 }, 0.0);
    grid.Fill(42.0);

    for (size_t j = 0; j < grid.DataSize().y; ++j)
    {
        for (size_t i = 0; i < grid.DataSize().x; ++i)
        {
            EXPECT_DOUBLE_EQ(42.0, grid(i, j));
        }
    }

    auto func = [](const Vector2D& x) { return x.Sum(); };
    grid.Fill(func);

    for (size_t j = 0; j < 4; ++j)
    {
        for (size_t i = 0; i < 5; ++i)
        {
            EXPECT_DOUBLE_EQ(static_cast<double>(i + j) + 1.0, grid(i, j));
        }
    }
}

TEST(CellCenteredScalarGrid2, GradientAtAtDataPoint)
{
    CellCenteredScalarGrid2 grid({ 5, 8 }, { 2.0, 3.0 });

    grid.Fill(1.0);

    for (size_t j = 0; j < grid.Resolution().y; ++j)
    {
        for (size_t i = 0; i < grid.Resolution().x; ++i)
        {
            Vector2D grad = grid.GradientAtDataPoint(i, j);
            EXPECT_DOUBLE_EQ(0.0, grad.x);
            EXPECT_DOUBLE_EQ(0.0, grad.y);
        }
    }

    grid.Fill([](const Vector2D& x) { return x.x + 2.0 * x.y; });

    for (size_t j = 1; j < grid.Resolution().y - 1; ++j)
    {
        for (size_t i = 1; i < grid.Resolution().x - 1; ++i)
        {
            Vector2D grad = grid.GradientAtDataPoint(i, j);
            EXPECT_NEAR(1.0, grad.x, 1e-6);
            EXPECT_NEAR(2.0, grad.y, 1e-6);
        }
    }
}

TEST(CellCenteredScalarGrid2, LaplacianAtAtDataPoint)
{
    CellCenteredScalarGrid2 grid({ 5, 8 }, { 2.0, 3.0 });

    grid.Fill(1.0);

    for (size_t j = 0; j < grid.Resolution().y; ++j)
    {
        for (size_t i = 0; i < grid.Resolution().x; ++i)
        {
            EXPECT_DOUBLE_EQ(0.0, grid.LaplacianAtDataPoint(i, j));
        }
    }

    grid.Fill(
        [](const Vector2D& x) { return Square(x.x) + 2.0 * Square(x.y); });

    for (size_t j = 1; j < grid.Resolution().y - 1; ++j)
    {
        for (size_t i = 1; i < grid.Resolution().x - 1; ++i)
        {
            EXPECT_NEAR(6.0, grid.LaplacianAtDataPoint(i, j), 1e-6);
        }
    }
}

TEST(CellCenteredScalarGrid2, Serialization)
{
    CellCenteredScalarGrid2 grid1({ 5, 4 }, { 1.0, 2.0 }, { -5.0, 3.0 });
    grid1.Fill([](const Vector2D& pt) { return pt.x + pt.y; });

    // Serialize to in-memory stream
    std::vector<uint8_t> buffer1;
    grid1.Serialize(&buffer1);

    // Deserialize to non-zero array
    CellCenteredScalarGrid2 grid2({ 1, 2 }, { 0.5, 1.0 }, { 0.5, 2.0 });
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
    CellCenteredScalarGrid2 grid3;
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
