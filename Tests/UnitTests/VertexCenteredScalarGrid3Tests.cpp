#include "gtest/gtest.h"

#include <Core/Grid/VertexCenteredScalarGrid.hpp>

using namespace CubbyFlow;

TEST(VertexCenteredScalarGrid3, Constructors)
{
    // Default constructors
    VertexCenteredScalarGrid3 grid1;
    EXPECT_EQ(0u, grid1.Resolution().x);
    EXPECT_EQ(0u, grid1.Resolution().y);
    EXPECT_EQ(0u, grid1.Resolution().z);
    EXPECT_DOUBLE_EQ(1.0, grid1.GridSpacing().x);
    EXPECT_DOUBLE_EQ(1.0, grid1.GridSpacing().y);
    EXPECT_DOUBLE_EQ(1.0, grid1.GridSpacing().z);
    EXPECT_DOUBLE_EQ(0.0, grid1.Origin().x);
    EXPECT_DOUBLE_EQ(0.0, grid1.Origin().y);
    EXPECT_DOUBLE_EQ(0.0, grid1.Origin().z);
    EXPECT_EQ(0u, grid1.DataSize().x);
    EXPECT_EQ(0u, grid1.DataSize().y);
    EXPECT_EQ(0u, grid1.DataSize().z);
    EXPECT_DOUBLE_EQ(0.0, grid1.DataOrigin().x);
    EXPECT_DOUBLE_EQ(0.0, grid1.DataOrigin().y);
    EXPECT_DOUBLE_EQ(0.0, grid1.DataOrigin().z);

    // Constructor with params
    VertexCenteredScalarGrid3 grid2({ 5, 4, 3 }, { 1.0, 2.0, 3.0 },
                                    { 4.0, 5.0, 6.0 }, 7.0);
    EXPECT_EQ(5u, grid2.Resolution().x);
    EXPECT_EQ(4u, grid2.Resolution().y);
    EXPECT_EQ(3u, grid2.Resolution().z);
    EXPECT_DOUBLE_EQ(1.0, grid2.GridSpacing().x);
    EXPECT_DOUBLE_EQ(2.0, grid2.GridSpacing().y);
    EXPECT_DOUBLE_EQ(3.0, grid2.GridSpacing().z);
    EXPECT_DOUBLE_EQ(4.0, grid2.Origin().x);
    EXPECT_DOUBLE_EQ(5.0, grid2.Origin().y);
    EXPECT_DOUBLE_EQ(6.0, grid2.Origin().z);
    EXPECT_EQ(6u, grid2.DataSize().x);
    EXPECT_EQ(5u, grid2.DataSize().y);
    EXPECT_EQ(4u, grid2.DataSize().z);
    EXPECT_DOUBLE_EQ(4.0, grid2.DataOrigin().x);
    EXPECT_DOUBLE_EQ(5.0, grid2.DataOrigin().y);
    EXPECT_DOUBLE_EQ(6.0, grid2.DataOrigin().z);
    grid2.ForEachDataPointIndex([&grid2](size_t i, size_t j, size_t k) {
        EXPECT_DOUBLE_EQ(7.0, grid2(i, j, k));
    });

    // Copy constructor
    VertexCenteredScalarGrid3 grid3(grid2);
    EXPECT_EQ(5u, grid3.Resolution().x);
    EXPECT_EQ(4u, grid3.Resolution().y);
    EXPECT_EQ(3u, grid3.Resolution().z);
    EXPECT_DOUBLE_EQ(1.0, grid3.GridSpacing().x);
    EXPECT_DOUBLE_EQ(2.0, grid3.GridSpacing().y);
    EXPECT_DOUBLE_EQ(3.0, grid3.GridSpacing().z);
    EXPECT_DOUBLE_EQ(4.0, grid3.Origin().x);
    EXPECT_DOUBLE_EQ(5.0, grid3.Origin().y);
    EXPECT_DOUBLE_EQ(6.0, grid3.Origin().z);
    EXPECT_EQ(6u, grid3.DataSize().x);
    EXPECT_EQ(5u, grid3.DataSize().y);
    EXPECT_EQ(4u, grid3.DataSize().z);
    EXPECT_DOUBLE_EQ(4.0, grid3.DataOrigin().x);
    EXPECT_DOUBLE_EQ(5.0, grid3.DataOrigin().y);
    EXPECT_DOUBLE_EQ(6.0, grid3.DataOrigin().z);
    grid3.ForEachDataPointIndex([&grid3](size_t i, size_t j, size_t k) {
        EXPECT_DOUBLE_EQ(7.0, grid3(i, j, k));
    });
}

TEST(VertexCenteredScalarGrid3, Swap)
{
    VertexCenteredScalarGrid3 grid1({ 5, 4, 3 }, { 1.0, 2.0, 3.0 },
                                    { 4.0, 5.0, 6.0 }, 7.0);
    VertexCenteredScalarGrid3 grid2({ 3, 8, 5 }, { 2.0, 3.0, 1.0 },
                                    { 5.0, 4.0, 7.0 }, 8.0);
    grid1.Swap(&grid2);

    EXPECT_EQ(3u, grid1.Resolution().x);
    EXPECT_EQ(8u, grid1.Resolution().y);
    EXPECT_EQ(5u, grid1.Resolution().z);
    EXPECT_DOUBLE_EQ(2.0, grid1.GridSpacing().x);
    EXPECT_DOUBLE_EQ(3.0, grid1.GridSpacing().y);
    EXPECT_DOUBLE_EQ(1.0, grid1.GridSpacing().z);
    EXPECT_DOUBLE_EQ(5.0, grid1.Origin().x);
    EXPECT_DOUBLE_EQ(4.0, grid1.Origin().y);
    EXPECT_DOUBLE_EQ(7.0, grid1.Origin().z);
    EXPECT_EQ(4u, grid1.DataSize().x);
    EXPECT_EQ(9u, grid1.DataSize().y);
    EXPECT_EQ(6u, grid1.DataSize().z);
    EXPECT_DOUBLE_EQ(5.0, grid1.DataOrigin().x);
    EXPECT_DOUBLE_EQ(4.0, grid1.DataOrigin().y);
    EXPECT_DOUBLE_EQ(7.0, grid1.DataOrigin().z);
    grid1.ForEachDataPointIndex([&grid1](size_t i, size_t j, size_t k) {
        EXPECT_DOUBLE_EQ(8.0, grid1(i, j, k));
    });

    EXPECT_EQ(5u, grid2.Resolution().x);
    EXPECT_EQ(4u, grid2.Resolution().y);
    EXPECT_EQ(3u, grid2.Resolution().z);
    EXPECT_DOUBLE_EQ(1.0, grid2.GridSpacing().x);
    EXPECT_DOUBLE_EQ(2.0, grid2.GridSpacing().y);
    EXPECT_DOUBLE_EQ(3.0, grid2.GridSpacing().z);
    EXPECT_DOUBLE_EQ(4.0, grid2.Origin().x);
    EXPECT_DOUBLE_EQ(5.0, grid2.Origin().y);
    EXPECT_DOUBLE_EQ(6.0, grid2.Origin().z);
    EXPECT_EQ(6u, grid2.DataSize().x);
    EXPECT_EQ(5u, grid2.DataSize().y);
    EXPECT_EQ(4u, grid2.DataSize().z);
    EXPECT_DOUBLE_EQ(4.0, grid2.DataOrigin().x);
    EXPECT_DOUBLE_EQ(5.0, grid2.DataOrigin().y);
    EXPECT_DOUBLE_EQ(6.0, grid2.DataOrigin().z);
    grid2.ForEachDataPointIndex([&grid2](size_t i, size_t j, size_t k) {
        EXPECT_DOUBLE_EQ(7.0, grid2(i, j, k));
    });
}

TEST(VertexCenteredScalarGrid3, Set)
{
    VertexCenteredScalarGrid3 grid1({ 5, 4, 3 }, { 1.0, 2.0, 3.0 },
                                    { 4.0, 5.0, 6.0 }, 7.0);
    VertexCenteredScalarGrid3 grid2({ 3, 8, 5 }, { 2.0, 3.0, 1.0 },
                                    { 5.0, 4.0, 7.0 }, 8.0);
    grid1.Set(grid2);

    EXPECT_EQ(3u, grid1.Resolution().x);
    EXPECT_EQ(8u, grid1.Resolution().y);
    EXPECT_EQ(5u, grid1.Resolution().z);
    EXPECT_DOUBLE_EQ(2.0, grid1.GridSpacing().x);
    EXPECT_DOUBLE_EQ(3.0, grid1.GridSpacing().y);
    EXPECT_DOUBLE_EQ(1.0, grid1.GridSpacing().z);
    EXPECT_DOUBLE_EQ(5.0, grid1.Origin().x);
    EXPECT_DOUBLE_EQ(4.0, grid1.Origin().y);
    EXPECT_DOUBLE_EQ(7.0, grid1.Origin().z);
    EXPECT_EQ(4u, grid1.DataSize().x);
    EXPECT_EQ(9u, grid1.DataSize().y);
    EXPECT_EQ(6u, grid1.DataSize().z);
    EXPECT_DOUBLE_EQ(5.0, grid1.DataOrigin().x);
    EXPECT_DOUBLE_EQ(4.0, grid1.DataOrigin().y);
    EXPECT_DOUBLE_EQ(7.0, grid1.DataOrigin().z);
    grid1.ForEachDataPointIndex([&grid1](size_t i, size_t j, size_t k) {
        EXPECT_DOUBLE_EQ(8.0, grid1(i, j, k));
    });
}

TEST(VertexCenteredScalarGrid3, AssignmentOperator)
{
    VertexCenteredScalarGrid3 grid1;
    VertexCenteredScalarGrid3 grid2({ 3, 8, 5 }, { 2.0, 3.0, 1.0 },
                                    { 5.0, 4.0, 7.0 }, 8.0);
    grid1 = grid2;

    EXPECT_EQ(3u, grid1.Resolution().x);
    EXPECT_EQ(8u, grid1.Resolution().y);
    EXPECT_EQ(5u, grid1.Resolution().z);
    EXPECT_DOUBLE_EQ(2.0, grid1.GridSpacing().x);
    EXPECT_DOUBLE_EQ(3.0, grid1.GridSpacing().y);
    EXPECT_DOUBLE_EQ(1.0, grid1.GridSpacing().z);
    EXPECT_DOUBLE_EQ(5.0, grid1.Origin().x);
    EXPECT_DOUBLE_EQ(4.0, grid1.Origin().y);
    EXPECT_DOUBLE_EQ(7.0, grid1.Origin().z);
    EXPECT_EQ(4u, grid1.DataSize().x);
    EXPECT_EQ(9u, grid1.DataSize().y);
    EXPECT_EQ(6u, grid1.DataSize().z);
    EXPECT_DOUBLE_EQ(5.0, grid1.DataOrigin().x);
    EXPECT_DOUBLE_EQ(4.0, grid1.DataOrigin().y);
    EXPECT_DOUBLE_EQ(7.0, grid1.DataOrigin().z);
    grid1.ForEachDataPointIndex([&grid1](size_t i, size_t j, size_t k) {
        EXPECT_DOUBLE_EQ(8.0, grid1(i, j, k));
    });
}

TEST(VertexCenteredScalarGrid3, Clone)
{
    VertexCenteredScalarGrid3 grid2({ 3, 8, 5 }, { 2.0, 3.0, 1.0 },
                                    { 5.0, 4.0, 7.0 }, 8.0);
    auto grid1 = grid2.Clone();

    EXPECT_EQ(3u, grid1->Resolution().x);
    EXPECT_EQ(8u, grid1->Resolution().y);
    EXPECT_EQ(5u, grid1->Resolution().z);
    EXPECT_DOUBLE_EQ(2.0, grid1->GridSpacing().x);
    EXPECT_DOUBLE_EQ(3.0, grid1->GridSpacing().y);
    EXPECT_DOUBLE_EQ(1.0, grid1->GridSpacing().z);
    EXPECT_DOUBLE_EQ(5.0, grid1->Origin().x);
    EXPECT_DOUBLE_EQ(4.0, grid1->Origin().y);
    EXPECT_DOUBLE_EQ(7.0, grid1->Origin().z);
    EXPECT_EQ(4u, grid1->DataSize().x);
    EXPECT_EQ(9u, grid1->DataSize().y);
    EXPECT_EQ(6u, grid1->DataSize().z);
    EXPECT_DOUBLE_EQ(5.0, grid1->DataOrigin().x);
    EXPECT_DOUBLE_EQ(4.0, grid1->DataOrigin().y);
    EXPECT_DOUBLE_EQ(7.0, grid1->DataOrigin().z);
    grid1->ForEachDataPointIndex([&grid1](size_t i, size_t j, size_t k) {
        EXPECT_DOUBLE_EQ(8.0, (*grid1)(i, j, k));
    });
}

TEST(VertexCenteredScalarGrid3, Builder)
{
    {
        auto grid1 = VertexCenteredScalarGrid3::GetBuilder().Build(
            { 3, 8, 5 }, { 2.0, 3.0, 1.0 }, { 5.0, 4.0, 7.0 }, 8.0);

        auto grid2 =
            std::dynamic_pointer_cast<VertexCenteredScalarGrid3>(grid1);
        EXPECT_TRUE(grid2 != nullptr);

        EXPECT_EQ(3u, grid1->Resolution().x);
        EXPECT_EQ(8u, grid1->Resolution().y);
        EXPECT_EQ(5u, grid1->Resolution().z);
        EXPECT_DOUBLE_EQ(2.0, grid1->GridSpacing().x);
        EXPECT_DOUBLE_EQ(3.0, grid1->GridSpacing().y);
        EXPECT_DOUBLE_EQ(1.0, grid1->GridSpacing().z);
        EXPECT_DOUBLE_EQ(5.0, grid1->Origin().x);
        EXPECT_DOUBLE_EQ(4.0, grid1->Origin().y);
        EXPECT_DOUBLE_EQ(7.0, grid1->Origin().z);
        EXPECT_EQ(4u, grid1->DataSize().x);
        EXPECT_EQ(9u, grid1->DataSize().y);
        EXPECT_EQ(6u, grid1->DataSize().z);
        EXPECT_DOUBLE_EQ(5.0, grid1->DataOrigin().x);
        EXPECT_DOUBLE_EQ(4.0, grid1->DataOrigin().y);
        EXPECT_DOUBLE_EQ(7.0, grid1->DataOrigin().z);
        grid1->ForEachDataPointIndex([&grid1](size_t i, size_t j, size_t k) {
            EXPECT_DOUBLE_EQ(8.0, (*grid1)(i, j, k));
        });
    }

    {
        auto grid1 = VertexCenteredScalarGrid3::GetBuilder()
                         .WithResolution({ 3, 8, 5 })
                         .WithGridSpacing({ 2, 3, 1 })
                         .WithOrigin({ 5, 4, 7 })
                         .WithInitialValue(8.0)
                         .Build();

        EXPECT_EQ(3u, grid1.Resolution().x);
        EXPECT_EQ(8u, grid1.Resolution().y);
        EXPECT_EQ(5u, grid1.Resolution().z);
        EXPECT_DOUBLE_EQ(2.0, grid1.GridSpacing().x);
        EXPECT_DOUBLE_EQ(3.0, grid1.GridSpacing().y);
        EXPECT_DOUBLE_EQ(1.0, grid1.GridSpacing().z);
        EXPECT_DOUBLE_EQ(5.0, grid1.Origin().x);
        EXPECT_DOUBLE_EQ(4.0, grid1.Origin().y);
        EXPECT_DOUBLE_EQ(7.0, grid1.Origin().z);
        EXPECT_EQ(4u, grid1.DataSize().x);
        EXPECT_EQ(9u, grid1.DataSize().y);
        EXPECT_EQ(6u, grid1.DataSize().z);
        EXPECT_DOUBLE_EQ(5.0, grid1.DataOrigin().x);
        EXPECT_DOUBLE_EQ(4.0, grid1.DataOrigin().y);
        EXPECT_DOUBLE_EQ(7.0, grid1.DataOrigin().z);
        grid1.ForEachDataPointIndex([&grid1](size_t i, size_t j, size_t k) {
            EXPECT_DOUBLE_EQ(8.0, grid1(i, j, k));
        });
    }
}

TEST(VertexCenteredScalarGrid3, Fill)
{
    VertexCenteredScalarGrid3 grid({ 5, 4, 6 }, { 1.0, 1.0, 1.0 },
                                   { 0.0, 0.0, 0.0 }, 0.0);
    grid.Fill(42.0);

    for (size_t k = 0; k < grid.DataSize().z; ++k)
    {
        for (size_t j = 0; j < grid.DataSize().y; ++j)
        {
            for (size_t i = 0; i < grid.DataSize().x; ++i)
            {
                EXPECT_DOUBLE_EQ(42.0, grid(i, j, k));
            }
        }
    }

    auto func = [](const Vector3D& x) {
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

    for (size_t k = 0; k < grid.DataSize().z; ++k)
    {
        for (size_t j = 0; j < grid.DataSize().y; ++j)
        {
            for (size_t i = 0; i < grid.DataSize().x; ++i)
            {
                if (i < 3)
                {
                    EXPECT_DOUBLE_EQ(2.0, grid(i, j, k));
                }
                else
                {
                    EXPECT_DOUBLE_EQ(5.0, grid(i, j, k));
                }
            }
        }
    }
}

TEST(VertexCenteredScalarGrid3, Serialization)
{
    VertexCenteredScalarGrid3 grid1({ 5, 4, 3 }, { 1.0, 2.0, 3.0 },
                                    { -5.0, 3.0, 1.0 });
    grid1.Fill([](const Vector3D& pt) { return pt.x + pt.y + pt.z; });

    // Serialize to in-memory stream
    std::vector<uint8_t> buffer1;
    grid1.Serialize(&buffer1);

    // Deserialize to non-zero array
    VertexCenteredScalarGrid3 grid2({ 1, 2, 4 }, { 0.5, 1.0, 2.0 },
                                    { 0.5, 2.0, -3.0 });
    grid2.Deserialize(buffer1);
    EXPECT_EQ(5u, grid2.Resolution().x);
    EXPECT_EQ(4u, grid2.Resolution().y);
    EXPECT_EQ(3u, grid2.Resolution().z);
    EXPECT_DOUBLE_EQ(1.0, grid2.GridSpacing().x);
    EXPECT_DOUBLE_EQ(2.0, grid2.GridSpacing().y);
    EXPECT_DOUBLE_EQ(3.0, grid2.GridSpacing().z);
    EXPECT_DOUBLE_EQ(-5.0, grid2.Origin().x);
    EXPECT_DOUBLE_EQ(3.0, grid2.Origin().y);
    EXPECT_DOUBLE_EQ(1.0, grid2.Origin().z);
    EXPECT_DOUBLE_EQ(-5.0, grid2.GetBoundingBox().lowerCorner.x);
    EXPECT_DOUBLE_EQ(3.0, grid2.GetBoundingBox().lowerCorner.y);
    EXPECT_DOUBLE_EQ(1.0, grid2.GetBoundingBox().lowerCorner.z);
    EXPECT_DOUBLE_EQ(0.0, grid2.GetBoundingBox().upperCorner.x);
    EXPECT_DOUBLE_EQ(11.0, grid2.GetBoundingBox().upperCorner.y);
    EXPECT_DOUBLE_EQ(10.0, grid2.GetBoundingBox().upperCorner.z);

    grid1.ForEachDataPointIndex([&grid1, &grid2](size_t i, size_t j, size_t k) {
        EXPECT_DOUBLE_EQ(grid1(i, j, k), grid2(i, j, k));
    });

    // Serialize zero-sized array
    VertexCenteredScalarGrid3 grid3;
    std::vector<uint8_t> buffer2;
    grid3.Serialize(&buffer2);

    // Deserialize to non-zero array
    grid2.Deserialize(buffer2);
    EXPECT_EQ(0u, grid2.Resolution().x);
    EXPECT_EQ(0u, grid2.Resolution().y);
    EXPECT_EQ(0u, grid2.Resolution().z);
    EXPECT_DOUBLE_EQ(0.0, grid2.Origin().x);
    EXPECT_DOUBLE_EQ(0.0, grid2.Origin().y);
    EXPECT_DOUBLE_EQ(0.0, grid2.Origin().z);
    EXPECT_DOUBLE_EQ(1.0, grid2.GridSpacing().x);
    EXPECT_DOUBLE_EQ(1.0, grid2.GridSpacing().y);
    EXPECT_DOUBLE_EQ(1.0, grid2.GridSpacing().z);
}