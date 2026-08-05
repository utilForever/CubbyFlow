#include "gtest/gtest.h"

#include <Core/Grid/CellCenteredVectorGrid.hpp>

using namespace CubbyFlow;

TEST(CellCenteredVectorGrid2, Constructors)
{
    // Default constructors
    CellCenteredVectorGrid2 grid1;
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
    CellCenteredVectorGrid2 grid2({ 5, 4 }, { 1.0, 2.0 }, { 3.0, 4.0 },
                                  { 5.0, 6.0 });
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
    grid2.ForEachDataPointIndex([&grid2](size_t i, size_t j) {
        EXPECT_DOUBLE_EQ(5.0, grid2(i, j).x);
        EXPECT_DOUBLE_EQ(6.0, grid2(i, j).y);
    });

    // Copy constructor
    CellCenteredVectorGrid2 grid3(grid2);
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
    grid3.ForEachDataPointIndex([&grid3](size_t i, size_t j) {
        EXPECT_DOUBLE_EQ(5.0, grid3(i, j).x);
        EXPECT_DOUBLE_EQ(6.0, grid3(i, j).y);
    });
}

TEST(CellCenteredVectorGrid2, CopySemantics)
{
    CellCenteredVectorGrid2 source({ 2, 2 }, { 1.0, 2.0 }, { 3.0, 4.0 },
                                   { 5.0, 6.0 });
    CellCenteredVectorGrid2 copied(source);
    CellCenteredVectorGrid2 assigned;
    assigned = source;

    source.Fill({ 7.0, 8.0 });

    EXPECT_EQ(Vector2D(5.0, 6.0), copied.Sample(copied.DataOrigin()));
    EXPECT_EQ(Vector2D(5.0, 6.0), assigned.Sample(assigned.DataOrigin()));
}

TEST(CellCenteredVectorGrid2, Swap)
{
    CellCenteredVectorGrid2 grid1({ 5, 4 }, { 1.0, 2.0 }, { 3.0, 4.0 },
                                  { 5.0, 6.0 });
    CellCenteredVectorGrid2 grid2({ 3, 8 }, { 2.0, 3.0 }, { 1.0, 5.0 },
                                  { 4.0, 7.0 });
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
    grid1.ForEachDataPointIndex([&grid1](size_t i, size_t j) {
        EXPECT_DOUBLE_EQ(4.0, grid1(i, j).x);
        EXPECT_DOUBLE_EQ(7.0, grid1(i, j).y);
    });

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
    grid2.ForEachDataPointIndex([&grid2](size_t i, size_t j) {
        EXPECT_DOUBLE_EQ(5.0, grid2(i, j).x);
        EXPECT_DOUBLE_EQ(6.0, grid2(i, j).y);
    });
}

TEST(CellCenteredVectorGrid2, Set)
{
    CellCenteredVectorGrid2 grid1({ 5, 4 }, { 1.0, 2.0 }, { 3.0, 4.0 },
                                  { 5.0, 6.0 });
    CellCenteredVectorGrid2 grid2({ 3, 8 }, { 2.0, 3.0 }, { 1.0, 5.0 },
                                  { 4.0, 7.0 });
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
    grid1.ForEachDataPointIndex([&grid1](size_t i, size_t j) {
        EXPECT_DOUBLE_EQ(4.0, grid1(i, j).x);
        EXPECT_DOUBLE_EQ(7.0, grid1(i, j).y);
    });
}

TEST(CellCenteredVectorGrid2, AssignmentOperator)
{
    CellCenteredVectorGrid2 grid1;
    CellCenteredVectorGrid2 grid2({ 3, 8 }, { 2.0, 3.0 }, { 1.0, 5.0 },
                                  { 4.0, 7.0 });
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
    grid1.ForEachDataPointIndex([&grid1](size_t i, size_t j) {
        EXPECT_DOUBLE_EQ(4.0, grid1(i, j).x);
        EXPECT_DOUBLE_EQ(7.0, grid1(i, j).y);
    });
}

TEST(CellCenteredVectorGrid2, Clone)
{
    CellCenteredVectorGrid2 grid2({ 3, 8 }, { 2.0, 3.0 }, { 1.0, 5.0 },
                                  { 4.0, 7.0 });
    auto grid1 = grid2.Clone();

    auto grid3 = std::dynamic_pointer_cast<CellCenteredVectorGrid2>(grid1);
    EXPECT_TRUE(grid3 != nullptr);

    EXPECT_EQ(3u, grid1->Resolution().x);
    EXPECT_EQ(8u, grid1->Resolution().y);
    EXPECT_DOUBLE_EQ(2.0, grid1->GridSpacing().x);
    EXPECT_DOUBLE_EQ(3.0, grid1->GridSpacing().y);
    EXPECT_DOUBLE_EQ(1.0, grid1->Origin().x);
    EXPECT_DOUBLE_EQ(5.0, grid1->Origin().y);
    EXPECT_EQ(3u, grid3->DataSize().x);
    EXPECT_EQ(8u, grid3->DataSize().y);
    EXPECT_DOUBLE_EQ(2.0, grid3->DataOrigin().x);
    EXPECT_DOUBLE_EQ(6.5, grid3->DataOrigin().y);
    grid3->ForEachDataPointIndex([&grid3](size_t i, size_t j) {
        EXPECT_DOUBLE_EQ(4.0, (*grid3)(i, j).x);
        EXPECT_DOUBLE_EQ(7.0, (*grid3)(i, j).y);
    });
}

TEST(CellCenteredVectorGrid2, Builder)
{
    {
        auto grid1 = CellCenteredVectorGrid2::GetBuilder().Build(
            Vector2UZ(3, 8), Vector2D(2.0, 3.0), Vector2D(1.0, 5.0),
            { 4.0, 7.0 });

        auto grid2 = std::dynamic_pointer_cast<CellCenteredVectorGrid2>(grid1);
        EXPECT_TRUE(grid2 != nullptr);

        EXPECT_EQ(3u, grid1->Resolution().x);
        EXPECT_EQ(8u, grid1->Resolution().y);
        EXPECT_DOUBLE_EQ(2.0, grid1->GridSpacing().x);
        EXPECT_DOUBLE_EQ(3.0, grid1->GridSpacing().y);
        EXPECT_DOUBLE_EQ(1.0, grid1->Origin().x);
        EXPECT_DOUBLE_EQ(5.0, grid1->Origin().y);
        EXPECT_EQ(3u, grid2->DataSize().x);
        EXPECT_EQ(8u, grid2->DataSize().y);
        EXPECT_DOUBLE_EQ(2.0, grid2->DataOrigin().x);
        EXPECT_DOUBLE_EQ(6.5, grid2->DataOrigin().y);
        grid2->ForEachDataPointIndex([&grid2](size_t i, size_t j) {
            EXPECT_DOUBLE_EQ(4.0, (*grid2)(i, j).x);
            EXPECT_DOUBLE_EQ(7.0, (*grid2)(i, j).y);
        });
    }

    {
        auto grid1 = CellCenteredVectorGrid2::GetBuilder()
                         .WithResolution({ 3, 8 })
                         .WithGridSpacing({ 2, 3 })
                         .WithOrigin({ 1, 5 })
                         .WithInitialValue({ 4, 7 })
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
            EXPECT_DOUBLE_EQ(4.0, grid1(i, j).x);
            EXPECT_DOUBLE_EQ(7.0, grid1(i, j).y);
        });
    }
}

TEST(CellCenteredVectorGrid2, Fill)
{
    CellCenteredVectorGrid2 grid({ 5, 4 }, { 1.0, 1.0 }, { 0.0, 0.0 },
                                 { 0.0, 0.0 });
    grid.Fill(Vector2D(42.0, 27.0));

    for (size_t j = 0; j < grid.DataSize().y; ++j)
    {
        for (size_t i = 0; i < grid.DataSize().x; ++i)
        {
            EXPECT_DOUBLE_EQ(42.0, grid(i, j).x);
            EXPECT_DOUBLE_EQ(27.0, grid(i, j).y);
        }
    }

    auto func = [](const Vector2D& x) {
        if (x.x < 3.0)
        {
            return Vector2D(2.0, 3.0);
        }
        else
        {
            return Vector2D(5.0, 7.0);
        }
    };
    grid.Fill(func);

    for (size_t j = 0; j < grid.DataSize().y; ++j)
    {
        for (size_t i = 0; i < grid.DataSize().x; ++i)
        {
            if (i < 3)
            {
                EXPECT_DOUBLE_EQ(2.0, grid(i, j).x);
                EXPECT_DOUBLE_EQ(3.0, grid(i, j).y);
            }
            else
            {
                EXPECT_DOUBLE_EQ(5.0, grid(i, j).x);
                EXPECT_DOUBLE_EQ(7.0, grid(i, j).y);
            }
        }
    }
}

TEST(CellCenteredVectorGrid2, DivergenceAtDataPoint)
{
    CellCenteredVectorGrid2 grid({ 5, 8 }, { 2.0, 3.0 });

    grid.Fill(Vector2D(1.0, -2.0));

    for (size_t j = 0; j < grid.Resolution().y; ++j)
    {
        for (size_t i = 0; i < grid.Resolution().x; ++i)
        {
            EXPECT_DOUBLE_EQ(0.0, grid.DivergenceAtDataPoint(i, j));
        }
    }

    grid.Fill([](const Vector2D& x) { return x; });

    for (size_t j = 1; j < grid.Resolution().y - 1; ++j)
    {
        for (size_t i = 1; i < grid.Resolution().x - 1; ++i)
        {
            EXPECT_NEAR(2.0, grid.DivergenceAtDataPoint(i, j), 1e-6);
        }
    }
}

TEST(CellCenteredVectorGrid2, CurlAtAtDataPoint)
{
    CellCenteredVectorGrid2 grid({ 5, 8 }, { 2.0, 3.0 });

    grid.Fill(Vector2D(1.0, -2.0));

    for (size_t j = 0; j < grid.Resolution().y; ++j)
    {
        for (size_t i = 0; i < grid.Resolution().x; ++i)
        {
            EXPECT_DOUBLE_EQ(0.0, grid.CurlAtDataPoint(i, j));
        }
    }

    grid.Fill([](const Vector2D& x) { return Vector2D(-x.y, x.x); });

    for (size_t j = 1; j < grid.Resolution().y - 1; ++j)
    {
        for (size_t i = 1; i < grid.Resolution().x - 1; ++i)
        {
            EXPECT_NEAR(2.0, grid.CurlAtDataPoint(i, j), 1e-6);
        }
    }
}

TEST(CellCenteredVectorGrid2, Serialization)
{
    CellCenteredVectorGrid2 grid1({ 5, 4 }, { 1.0, 2.0 }, { -5.0, 3.0 });
    grid1.Fill([](const Vector2D& pt) { return Vector2D(pt.x, pt.y); });

    // Serialize to in-memory stream
    std::vector<uint8_t> buffer1;
    grid1.Serialize(&buffer1);

    // Deserialize to non-zero array
    CellCenteredVectorGrid2 grid2({ 1, 2 }, { 0.5, 1.0 }, { 0.5, 2.0 });
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
        EXPECT_DOUBLE_EQ(grid1(i, j).x, grid2(i, j).x);
        EXPECT_DOUBLE_EQ(grid1(i, j).y, grid2(i, j).y);
    });

    // Serialize zero-sized array
    CellCenteredVectorGrid2 grid3;
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
