#include "gtest/gtest.h"

#include <Core/Grid/CellCenteredVectorGrid.hpp>

using namespace CubbyFlow;

TEST(CellCenteredVectorGrid3, Constructors)
{
    // Default constructors
    CellCenteredVectorGrid3 grid1;
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
    EXPECT_DOUBLE_EQ(0.5, grid1.DataOrigin().x);
    EXPECT_DOUBLE_EQ(0.5, grid1.DataOrigin().y);
    EXPECT_DOUBLE_EQ(0.5, grid1.DataOrigin().z);

    // Constructor with params
    CellCenteredVectorGrid3 grid2({ 5, 4, 3 }, { 1.0, 2.0, 3.0 },
                                  { 4.0, 5.0, 6.0 }, { 7.0, 8.0, 9.0 });
    EXPECT_EQ(5u, grid2.Resolution().x);
    EXPECT_EQ(4u, grid2.Resolution().y);
    EXPECT_EQ(3u, grid2.Resolution().z);
    EXPECT_DOUBLE_EQ(1.0, grid2.GridSpacing().x);
    EXPECT_DOUBLE_EQ(2.0, grid2.GridSpacing().y);
    EXPECT_DOUBLE_EQ(3.0, grid2.GridSpacing().z);
    EXPECT_DOUBLE_EQ(4.0, grid2.Origin().x);
    EXPECT_DOUBLE_EQ(5.0, grid2.Origin().y);
    EXPECT_DOUBLE_EQ(6.0, grid2.Origin().z);
    EXPECT_EQ(5u, grid2.DataSize().x);
    EXPECT_EQ(4u, grid2.DataSize().y);
    EXPECT_EQ(3u, grid2.DataSize().z);
    EXPECT_DOUBLE_EQ(4.5, grid2.DataOrigin().x);
    EXPECT_DOUBLE_EQ(6.0, grid2.DataOrigin().y);
    EXPECT_DOUBLE_EQ(7.5, grid2.DataOrigin().z);
    grid2.ForEachDataPointIndex([&grid2](size_t i, size_t j, size_t k) {
        EXPECT_DOUBLE_EQ(7.0, grid2(i, j, k).x);
        EXPECT_DOUBLE_EQ(8.0, grid2(i, j, k).y);
        EXPECT_DOUBLE_EQ(9.0, grid2(i, j, k).z);
    });

    // Copy constructor
    CellCenteredVectorGrid3 grid3(grid2);
    EXPECT_EQ(5u, grid3.Resolution().x);
    EXPECT_EQ(4u, grid3.Resolution().y);
    EXPECT_EQ(3u, grid3.Resolution().z);
    EXPECT_DOUBLE_EQ(1.0, grid3.GridSpacing().x);
    EXPECT_DOUBLE_EQ(2.0, grid3.GridSpacing().y);
    EXPECT_DOUBLE_EQ(3.0, grid3.GridSpacing().z);
    EXPECT_DOUBLE_EQ(4.0, grid3.Origin().x);
    EXPECT_DOUBLE_EQ(5.0, grid3.Origin().y);
    EXPECT_DOUBLE_EQ(6.0, grid3.Origin().z);
    EXPECT_EQ(5u, grid3.DataSize().x);
    EXPECT_EQ(4u, grid3.DataSize().y);
    EXPECT_EQ(3u, grid3.DataSize().z);
    EXPECT_DOUBLE_EQ(4.5, grid3.DataOrigin().x);
    EXPECT_DOUBLE_EQ(6.0, grid3.DataOrigin().y);
    EXPECT_DOUBLE_EQ(7.5, grid3.DataOrigin().z);
    grid3.ForEachDataPointIndex([&grid3](size_t i, size_t j, size_t k) {
        EXPECT_DOUBLE_EQ(7.0, grid3(i, j, k).x);
        EXPECT_DOUBLE_EQ(8.0, grid3(i, j, k).y);
        EXPECT_DOUBLE_EQ(9.0, grid3(i, j, k).z);
    });
}

TEST(CellCenteredVectorGrid3, Swap)
{
    CellCenteredVectorGrid3 grid1({ 5, 4, 3 }, { 1.0, 2.0, 3.0 },
                                  { 4.0, 5.0, 6.0 }, { 7.0, 8.0, 9.0 });
    CellCenteredVectorGrid3 grid2({ 3, 8, 5 }, { 2.0, 3.0, 1.0 },
                                  { 5.0, 4.0, 7.0 }, { 8.0, 1.0, 3.0 });
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
    EXPECT_EQ(3u, grid1.DataSize().x);
    EXPECT_EQ(8u, grid1.DataSize().y);
    EXPECT_EQ(5u, grid1.DataSize().z);
    EXPECT_DOUBLE_EQ(6.0, grid1.DataOrigin().x);
    EXPECT_DOUBLE_EQ(5.5, grid1.DataOrigin().y);
    EXPECT_DOUBLE_EQ(7.5, grid1.DataOrigin().z);
    grid1.ForEachDataPointIndex([&grid1](size_t i, size_t j, size_t k) {
        EXPECT_DOUBLE_EQ(8.0, grid1(i, j, k).x);
        EXPECT_DOUBLE_EQ(1.0, grid1(i, j, k).y);
        EXPECT_DOUBLE_EQ(3.0, grid1(i, j, k).z);
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
    EXPECT_EQ(5u, grid2.DataSize().x);
    EXPECT_EQ(4u, grid2.DataSize().y);
    EXPECT_EQ(3u, grid2.DataSize().z);
    EXPECT_DOUBLE_EQ(4.5, grid2.DataOrigin().x);
    EXPECT_DOUBLE_EQ(6.0, grid2.DataOrigin().y);
    EXPECT_DOUBLE_EQ(7.5, grid2.DataOrigin().z);
    grid2.ForEachDataPointIndex([&grid2](size_t i, size_t j, size_t k) {
        EXPECT_DOUBLE_EQ(7.0, grid2(i, j, k).x);
        EXPECT_DOUBLE_EQ(8.0, grid2(i, j, k).y);
        EXPECT_DOUBLE_EQ(9.0, grid2(i, j, k).z);
    });
}

TEST(CellCenteredVectorGrid3, Set)
{
    CellCenteredVectorGrid3 grid1({ 5, 4, 3 }, { 1.0, 2.0, 3.0 },
                                  { 4.0, 5.0, 6.0 }, { 7.0, 8.0, 9.0 });
    CellCenteredVectorGrid3 grid2({ 3, 8, 5 }, { 2.0, 3.0, 1.0 },
                                  { 5.0, 4.0, 7.0 }, { 8.0, 1.0, 3.0 });
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
    EXPECT_EQ(3u, grid1.DataSize().x);
    EXPECT_EQ(8u, grid1.DataSize().y);
    EXPECT_EQ(5u, grid1.DataSize().z);
    EXPECT_DOUBLE_EQ(6.0, grid1.DataOrigin().x);
    EXPECT_DOUBLE_EQ(5.5, grid1.DataOrigin().y);
    EXPECT_DOUBLE_EQ(7.5, grid1.DataOrigin().z);
    grid1.ForEachDataPointIndex([&grid1](size_t i, size_t j, size_t k) {
        EXPECT_DOUBLE_EQ(8.0, grid1(i, j, k).x);
        EXPECT_DOUBLE_EQ(1.0, grid1(i, j, k).y);
        EXPECT_DOUBLE_EQ(3.0, grid1(i, j, k).z);
    });
}

TEST(CellCenteredVectorGrid3, AssignmentOperator)
{
    CellCenteredVectorGrid3 grid1;
    CellCenteredVectorGrid3 grid2({ 3, 8, 5 }, { 2.0, 3.0, 1.0 },
                                  { 5.0, 4.0, 7.0 }, { 8.0, 1.0, 3.0 });
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
    EXPECT_EQ(3u, grid1.DataSize().x);
    EXPECT_EQ(8u, grid1.DataSize().y);
    EXPECT_EQ(5u, grid1.DataSize().z);
    EXPECT_DOUBLE_EQ(6.0, grid1.DataOrigin().x);
    EXPECT_DOUBLE_EQ(5.5, grid1.DataOrigin().y);
    EXPECT_DOUBLE_EQ(7.5, grid1.DataOrigin().z);
    grid1.ForEachDataPointIndex([&grid1](size_t i, size_t j, size_t k) {
        EXPECT_DOUBLE_EQ(8.0, grid1(i, j, k).x);
        EXPECT_DOUBLE_EQ(1.0, grid1(i, j, k).y);
        EXPECT_DOUBLE_EQ(3.0, grid1(i, j, k).z);
    });
}

TEST(CellCenteredVectorGrid3, Clone)
{
    CellCenteredVectorGrid3 grid2({ 3, 8, 5 }, { 2.0, 3.0, 1.0 },
                                  { 5.0, 4.0, 7.0 }, { 8.0, 1.0, 3.0 });
    auto grid1 = grid2.Clone();

    auto grid3 = std::dynamic_pointer_cast<CellCenteredVectorGrid3>(grid1);
    EXPECT_TRUE(grid3 != nullptr);

    EXPECT_EQ(3u, grid1->Resolution().x);
    EXPECT_EQ(8u, grid1->Resolution().y);
    EXPECT_EQ(5u, grid1->Resolution().z);
    EXPECT_DOUBLE_EQ(2.0, grid1->GridSpacing().x);
    EXPECT_DOUBLE_EQ(3.0, grid1->GridSpacing().y);
    EXPECT_DOUBLE_EQ(1.0, grid1->GridSpacing().z);
    EXPECT_DOUBLE_EQ(5.0, grid1->Origin().x);
    EXPECT_DOUBLE_EQ(4.0, grid1->Origin().y);
    EXPECT_DOUBLE_EQ(7.0, grid1->Origin().z);
    EXPECT_EQ(3u, grid3->DataSize().x);
    EXPECT_EQ(8u, grid3->DataSize().y);
    EXPECT_EQ(5u, grid3->DataSize().z);
    EXPECT_DOUBLE_EQ(6.0, grid3->DataOrigin().x);
    EXPECT_DOUBLE_EQ(5.5, grid3->DataOrigin().y);
    EXPECT_DOUBLE_EQ(7.5, grid3->DataOrigin().z);
    grid3->ForEachDataPointIndex([&grid3](size_t i, size_t j, size_t k) {
        EXPECT_DOUBLE_EQ(8.0, (*grid3)(i, j, k).x);
        EXPECT_DOUBLE_EQ(1.0, (*grid3)(i, j, k).y);
        EXPECT_DOUBLE_EQ(3.0, (*grid3)(i, j, k).z);
    });
}

TEST(CellCenteredVectorGrid3, Builder)
{
    {
        auto grid1 = CellCenteredVectorGrid3::GetBuilder().Build(
            { 3, 8, 5 }, { 2.0, 3.0, 1.0 }, { 5.0, 4.0, 7.0 },
            { 8.0, 1.0, 3.0 });

        auto grid2 = std::dynamic_pointer_cast<CellCenteredVectorGrid3>(grid1);
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
        EXPECT_EQ(3u, grid2->DataSize().x);
        EXPECT_EQ(8u, grid2->DataSize().y);
        EXPECT_EQ(5u, grid2->DataSize().z);
        EXPECT_DOUBLE_EQ(6.0, grid2->DataOrigin().x);
        EXPECT_DOUBLE_EQ(5.5, grid2->DataOrigin().y);
        EXPECT_DOUBLE_EQ(7.5, grid2->DataOrigin().z);
        grid2->ForEachDataPointIndex([&grid2](size_t i, size_t j, size_t k) {
            EXPECT_DOUBLE_EQ(8.0, (*grid2)(i, j, k).x);
            EXPECT_DOUBLE_EQ(1.0, (*grid2)(i, j, k).y);
            EXPECT_DOUBLE_EQ(3.0, (*grid2)(i, j, k).z);
        });
    }

    {
        auto grid1 = CellCenteredVectorGrid3::GetBuilder()
                         .WithResolution({ 3, 8, 5 })
                         .WithGridSpacing({ 2.0, 3.0, 1.0 })
                         .WithOrigin({ 5.0, 4.0, 7.0 })
                         .WithInitialValue({ 8.0, 1.0, 3.0 })
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
        EXPECT_EQ(3u, grid1.DataSize().x);
        EXPECT_EQ(8u, grid1.DataSize().y);
        EXPECT_EQ(5u, grid1.DataSize().z);
        EXPECT_DOUBLE_EQ(6.0, grid1.DataOrigin().x);
        EXPECT_DOUBLE_EQ(5.5, grid1.DataOrigin().y);
        EXPECT_DOUBLE_EQ(7.5, grid1.DataOrigin().z);
        grid1.ForEachDataPointIndex([&grid1](size_t i, size_t j, size_t k) {
            EXPECT_DOUBLE_EQ(8.0, grid1(i, j, k).x);
            EXPECT_DOUBLE_EQ(1.0, grid1(i, j, k).y);
            EXPECT_DOUBLE_EQ(3.0, grid1(i, j, k).z);
        });
    }
}

TEST(CellCenteredVectorGrid3, Fill)
{
    CellCenteredVectorGrid3 grid({ 5, 4, 6 }, { 1.0, 1.0, 1.0 },
                                 { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 });
    grid.Fill(Vector3D(42.0, 27.0, 31.0));

    for (size_t k = 0; k < grid.DataSize().z; ++k)
    {
        for (size_t j = 0; j < grid.DataSize().y; ++j)
        {
            for (size_t i = 0; i < grid.DataSize().x; ++i)
            {
                EXPECT_DOUBLE_EQ(42.0, grid(i, j, k).x);
                EXPECT_DOUBLE_EQ(27.0, grid(i, j, k).y);
                EXPECT_DOUBLE_EQ(31.0, grid(i, j, k).z);
            }
        }
    }

    auto func = [](const Vector3D& x) {
        if (x.x < 3.0)
        {
            return Vector3D(2.0, 3.0, 1.0);
        }
        else
        {
            return Vector3D(5.0, 7.0, 9.0);
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
                    EXPECT_DOUBLE_EQ(2.0, grid(i, j, k).x);
                    EXPECT_DOUBLE_EQ(3.0, grid(i, j, k).y);
                    EXPECT_DOUBLE_EQ(1.0, grid(i, j, k).z);
                }
                else
                {
                    EXPECT_DOUBLE_EQ(5.0, grid(i, j, k).x);
                    EXPECT_DOUBLE_EQ(7.0, grid(i, j, k).y);
                    EXPECT_DOUBLE_EQ(9.0, grid(i, j, k).z);
                }
            }
        }
    }
}

TEST(CellCenteredVectorGrid3, DivergenceAtDataPoint)
{
    CellCenteredVectorGrid3 grid({ 5, 8, 6 });

    grid.Fill(Vector3D(1.0, -2.0, 3.0));

    for (size_t k = 0; k < grid.Resolution().z; ++k)
    {
        for (size_t j = 0; j < grid.Resolution().y; ++j)
        {
            for (size_t i = 0; i < grid.Resolution().x; ++i)
            {
                EXPECT_DOUBLE_EQ(0.0, grid.DivergenceAtDataPoint(i, j, k));
            }
        }
    }

    grid.Fill([](const Vector3D& x) { return x; });

    for (size_t k = 1; k < grid.Resolution().z - 1; ++k)
    {
        for (size_t j = 1; j < grid.Resolution().y - 1; ++j)
        {
            for (size_t i = 1; i < grid.Resolution().x - 1; ++i)
            {
                EXPECT_DOUBLE_EQ(3.0, grid.DivergenceAtDataPoint(i, j, k));
            }
        }
    }
}

TEST(CellCenteredVectorGrid3, CurlAtDataPoint)
{
    CellCenteredVectorGrid3 grid({ 5, 8, 6 }, { 2.0, 3.0, 1.5 });

    grid.Fill(Vector3D(1.0, -2.0, 3.0));

    for (size_t k = 0; k < grid.Resolution().z; ++k)
    {
        for (size_t j = 0; j < grid.Resolution().y; ++j)
        {
            for (size_t i = 0; i < grid.Resolution().x; ++i)
            {
                Vector3D curl = grid.CurlAtDataPoint(i, j, k);
                EXPECT_DOUBLE_EQ(0.0, curl.x);
                EXPECT_DOUBLE_EQ(0.0, curl.y);
                EXPECT_DOUBLE_EQ(0.0, curl.z);
            }
        }
    }

    grid.Fill([](const Vector3D& x) { return Vector3D(x.y, x.z, x.x); });

    for (size_t k = 1; k < grid.Resolution().z - 1; ++k)
    {
        for (size_t j = 1; j < grid.Resolution().y - 1; ++j)
        {
            for (size_t i = 1; i < grid.Resolution().x - 1; ++i)
            {
                Vector3D curl = grid.CurlAtDataPoint(i, j, k);
                EXPECT_DOUBLE_EQ(-1.0, curl.x);
                EXPECT_DOUBLE_EQ(-1.0, curl.y);
                EXPECT_DOUBLE_EQ(-1.0, curl.z);
            }
        }
    }
}

TEST(CellCenteredVectorGrid3, Serialization)
{
    CellCenteredVectorGrid3 grid1({ 5, 4, 3 }, { 1.0, 2.0, 3.0 },
                                  { -5.0, 3.0, 1.0 });
    grid1.Fill([](const Vector3D& pt) { return Vector3D(pt.x, pt.y, pt.z); });

    // Serialize to in-memory stream
    std::vector<uint8_t> buffer1;
    grid1.Serialize(&buffer1);

    // Deserialize to non-zero array
    CellCenteredVectorGrid3 grid2({ 1, 2, 4 }, { 0.5, 1.0, 2.0 },
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
        EXPECT_EQ(grid1(i, j, k), grid2(i, j, k));
    });

    // Serialize zero-sized array
    CellCenteredVectorGrid3 grid3;
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