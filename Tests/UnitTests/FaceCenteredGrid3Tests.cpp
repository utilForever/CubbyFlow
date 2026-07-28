#include "gtest/gtest.h"

#include <Core/Grid/FaceCenteredGrid.hpp>

using namespace CubbyFlow;

TEST(FaceCenteredGrid3, Constructors)
{
    // Default constructors
    FaceCenteredGrid3 grid1;
    EXPECT_EQ(0u, grid1.Resolution().x);
    EXPECT_EQ(0u, grid1.Resolution().y);
    EXPECT_EQ(0u, grid1.Resolution().z);
    EXPECT_DOUBLE_EQ(1.0, grid1.GridSpacing().x);
    EXPECT_DOUBLE_EQ(1.0, grid1.GridSpacing().y);
    EXPECT_DOUBLE_EQ(1.0, grid1.GridSpacing().z);
    EXPECT_DOUBLE_EQ(0.0, grid1.Origin().x);
    EXPECT_DOUBLE_EQ(0.0, grid1.Origin().y);
    EXPECT_DOUBLE_EQ(0.0, grid1.Origin().z);
    EXPECT_EQ(0u, grid1.USize().x);
    EXPECT_EQ(0u, grid1.USize().y);
    EXPECT_EQ(0u, grid1.USize().z);
    EXPECT_EQ(0u, grid1.VSize().x);
    EXPECT_EQ(0u, grid1.VSize().y);
    EXPECT_EQ(0u, grid1.VSize().z);
    EXPECT_EQ(0u, grid1.WSize().x);
    EXPECT_EQ(0u, grid1.WSize().y);
    EXPECT_EQ(0u, grid1.WSize().z);
    EXPECT_DOUBLE_EQ(0.0, grid1.UOrigin().x);
    EXPECT_DOUBLE_EQ(0.5, grid1.UOrigin().y);
    EXPECT_DOUBLE_EQ(0.5, grid1.UOrigin().z);
    EXPECT_DOUBLE_EQ(0.5, grid1.VOrigin().x);
    EXPECT_DOUBLE_EQ(0.0, grid1.VOrigin().y);
    EXPECT_DOUBLE_EQ(0.5, grid1.VOrigin().z);
    EXPECT_DOUBLE_EQ(0.5, grid1.WOrigin().x);
    EXPECT_DOUBLE_EQ(0.5, grid1.WOrigin().y);
    EXPECT_DOUBLE_EQ(0.0, grid1.WOrigin().z);

    // Constructor with params
    FaceCenteredGrid3 grid2({ 5, 4, 3 }, { 1.0, 2.0, 3.0 }, { 4.0, 5.0, 6.0 },
                            { 7.0, 8.0, 9.0 });
    EXPECT_EQ(5u, grid2.Resolution().x);
    EXPECT_EQ(4u, grid2.Resolution().y);
    EXPECT_EQ(3u, grid2.Resolution().z);
    EXPECT_DOUBLE_EQ(1.0, grid2.GridSpacing().x);
    EXPECT_DOUBLE_EQ(2.0, grid2.GridSpacing().y);
    EXPECT_DOUBLE_EQ(3.0, grid2.GridSpacing().z);
    EXPECT_DOUBLE_EQ(4.0, grid2.Origin().x);
    EXPECT_DOUBLE_EQ(5.0, grid2.Origin().y);
    EXPECT_DOUBLE_EQ(6.0, grid2.Origin().z);
    EXPECT_EQ(6u, grid2.USize().x);
    EXPECT_EQ(4u, grid2.USize().y);
    EXPECT_EQ(3u, grid2.USize().z);
    EXPECT_EQ(5u, grid2.VSize().x);
    EXPECT_EQ(5u, grid2.VSize().y);
    EXPECT_EQ(3u, grid2.VSize().z);
    EXPECT_EQ(5u, grid2.WSize().x);
    EXPECT_EQ(4u, grid2.WSize().y);
    EXPECT_EQ(4u, grid2.WSize().z);
    EXPECT_DOUBLE_EQ(4.0, grid2.UOrigin().x);
    EXPECT_DOUBLE_EQ(6.0, grid2.UOrigin().y);
    EXPECT_DOUBLE_EQ(7.5, grid2.UOrigin().z);
    EXPECT_DOUBLE_EQ(4.5, grid2.VOrigin().x);
    EXPECT_DOUBLE_EQ(5.0, grid2.VOrigin().y);
    EXPECT_DOUBLE_EQ(7.5, grid2.VOrigin().z);
    EXPECT_DOUBLE_EQ(4.5, grid2.WOrigin().x);
    EXPECT_DOUBLE_EQ(6.0, grid2.WOrigin().y);
    EXPECT_DOUBLE_EQ(6.0, grid2.WOrigin().z);
    grid2.ForEachUIndex([&grid2](const Vector3UZ& idx) {
        EXPECT_DOUBLE_EQ(7.0, grid2.U(idx));
    });
    grid2.ForEachVIndex([&grid2](const Vector3UZ& idx) {
        EXPECT_DOUBLE_EQ(8.0, grid2.V(idx));
    });
    grid2.ForEachWIndex([&grid2](const Vector3UZ& idx) {
        EXPECT_DOUBLE_EQ(9.0, grid2.W(idx));
    });

    // Copy constructor
    FaceCenteredGrid3 grid3(grid2);
    EXPECT_EQ(5u, grid3.Resolution().x);
    EXPECT_EQ(4u, grid3.Resolution().y);
    EXPECT_EQ(3u, grid3.Resolution().z);
    EXPECT_DOUBLE_EQ(1.0, grid3.GridSpacing().x);
    EXPECT_DOUBLE_EQ(2.0, grid3.GridSpacing().y);
    EXPECT_DOUBLE_EQ(3.0, grid3.GridSpacing().z);
    EXPECT_DOUBLE_EQ(4.0, grid3.Origin().x);
    EXPECT_DOUBLE_EQ(5.0, grid3.Origin().y);
    EXPECT_DOUBLE_EQ(6.0, grid3.Origin().z);
    EXPECT_EQ(6u, grid3.USize().x);
    EXPECT_EQ(4u, grid3.USize().y);
    EXPECT_EQ(3u, grid3.USize().z);
    EXPECT_EQ(5u, grid3.VSize().x);
    EXPECT_EQ(5u, grid3.VSize().y);
    EXPECT_EQ(3u, grid3.VSize().z);
    EXPECT_EQ(5u, grid3.WSize().x);
    EXPECT_EQ(4u, grid3.WSize().y);
    EXPECT_EQ(4u, grid3.WSize().z);
    EXPECT_DOUBLE_EQ(4.0, grid3.UOrigin().x);
    EXPECT_DOUBLE_EQ(6.0, grid3.UOrigin().y);
    EXPECT_DOUBLE_EQ(7.5, grid3.UOrigin().z);
    EXPECT_DOUBLE_EQ(4.5, grid3.VOrigin().x);
    EXPECT_DOUBLE_EQ(5.0, grid3.VOrigin().y);
    EXPECT_DOUBLE_EQ(7.5, grid3.VOrigin().z);
    EXPECT_DOUBLE_EQ(4.5, grid3.WOrigin().x);
    EXPECT_DOUBLE_EQ(6.0, grid3.WOrigin().y);
    EXPECT_DOUBLE_EQ(6.0, grid3.WOrigin().z);
    grid3.ForEachUIndex([&grid3](const Vector3UZ& idx) {
        EXPECT_DOUBLE_EQ(7.0, grid3.U(idx));
    });
    grid3.ForEachVIndex([&grid3](const Vector3UZ& idx) {
        EXPECT_DOUBLE_EQ(8.0, grid3.V(idx));
    });
    grid3.ForEachWIndex([&grid3](const Vector3UZ& idx) {
        EXPECT_DOUBLE_EQ(9.0, grid3.W(idx));
    });
}

TEST(FaceCenteredGrid3, Fill)
{
    FaceCenteredGrid3 grid({ 5, 4, 6 }, { 1.0, 1.0, 1.0 }, { 0.0, 0.0, 0.0 },
                           { 0.0, 0.0, 0.0 });
    grid.Fill(Vector3D(42.0, 27.0, 31.0));

    for (size_t k = 0; k < grid.USize().z; ++k)
    {
        for (size_t j = 0; j < grid.USize().y; ++j)
        {
            for (size_t i = 0; i < grid.USize().x; ++i)
            {
                EXPECT_DOUBLE_EQ(42.0, grid.U(i, j, k));
            }
        }
    }

    for (size_t k = 0; k < grid.VSize().z; ++k)
    {
        for (size_t j = 0; j < grid.VSize().y; ++j)
        {
            for (size_t i = 0; i < grid.VSize().x; ++i)
            {
                EXPECT_DOUBLE_EQ(27.0, grid.V(i, j, k));
            }
        }
    }

    for (size_t k = 0; k < grid.WSize().z; ++k)
    {
        for (size_t j = 0; j < grid.WSize().y; ++j)
        {
            for (size_t i = 0; i < grid.WSize().x; ++i)
            {
                EXPECT_DOUBLE_EQ(31.0, grid.W(i, j, k));
            }
        }
    }

    auto func = [](const Vector3D& x) { return x; };
    grid.Fill(func);

    for (size_t k = 0; k < grid.USize().z; ++k)
    {
        for (size_t j = 0; j < grid.USize().y; ++j)
        {
            for (size_t i = 0; i < grid.USize().x; ++i)
            {
                EXPECT_DOUBLE_EQ(static_cast<double>(i), grid.U(i, j, k));
            }
        }
    }

    for (size_t k = 0; k < grid.VSize().z; ++k)
    {
        for (size_t j = 0; j < grid.VSize().y; ++j)
        {
            for (size_t i = 0; i < grid.VSize().x; ++i)
            {
                EXPECT_DOUBLE_EQ(static_cast<double>(j), grid.V(i, j, k));
            }
        }
    }

    for (size_t k = 0; k < grid.WSize().z; ++k)
    {
        for (size_t j = 0; j < grid.WSize().y; ++j)
        {
            for (size_t i = 0; i < grid.WSize().x; ++i)
            {
                EXPECT_DOUBLE_EQ(static_cast<double>(k), grid.W(i, j, k));
            }
        }
    }
}

TEST(FaceCenteredGrid3, DivergenceAtCellCenter)
{
    FaceCenteredGrid3 grid({ 5, 8, 6 });

    grid.Fill(Vector3D(1.0, -2.0, 3.0));

    for (size_t k = 0; k < grid.Resolution().z; ++k)
    {
        for (size_t j = 0; j < grid.Resolution().y; ++j)
        {
            for (size_t i = 0; i < grid.Resolution().x; ++i)
            {
                EXPECT_DOUBLE_EQ(0.0, grid.DivergenceAtCellCenter(i, j, k));
            }
        }
    }

    grid.Fill([](const Vector3D& x) { return x; });

    for (size_t k = 0; k < grid.Resolution().z; ++k)
    {
        for (size_t j = 0; j < grid.Resolution().y; ++j)
        {
            for (size_t i = 0; i < grid.Resolution().x; ++i)
            {
                EXPECT_DOUBLE_EQ(3.0, grid.DivergenceAtCellCenter(i, j, k));
            }
        }
    }
}

TEST(FaceCenteredGrid3, CurlAtCellCenter)
{
    FaceCenteredGrid3 grid({ 5, 8, 6 }, { 2.0, 3.0, 1.5 });

    grid.Fill(Vector3D(1.0, -2.0, 3.0));

    for (size_t k = 0; k < grid.Resolution().z; ++k)
    {
        for (size_t j = 0; j < grid.Resolution().y; ++j)
        {
            for (size_t i = 0; i < grid.Resolution().x; ++i)
            {
                Vector3D curl = grid.CurlAtCellCenter(i, j, k);
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
                Vector3D curl = grid.CurlAtCellCenter(i, j, k);
                EXPECT_DOUBLE_EQ(-1.0, curl.x);
                EXPECT_DOUBLE_EQ(-1.0, curl.y);
                EXPECT_DOUBLE_EQ(-1.0, curl.z);
            }
        }
    }
}

TEST(FaceCenteredGrid3, ValueAtCellCenter)
{
    FaceCenteredGrid3 grid({ 5, 8, 6 }, { 2.0, 3.0, 1.5 });
    grid.Fill([](const Vector3D& x) {
        return Vector3D(3.0 * x.y + 1.0, 5.0 * x.z + 7.0, -1.0 * x.x - 9.0);
    });

    GridDataPositionFunc<3> pos = grid.CellCenterPosition();
    grid.ForEachCellIndex([&grid, &pos](size_t i, size_t j, size_t k) {
        Vector3D val = grid.ValueAtCellCenter(i, j, k);
        Vector3D x = pos(i, j, k);
        Vector3D expected =
            Vector3D(3.0 * x.y + 1.0, 5.0 * x.z + 7.0, -1.0 * x.x - 9.0);
        EXPECT_NEAR(expected.x, val.x, 1e-6);
        EXPECT_NEAR(expected.y, val.y, 1e-6);
        EXPECT_NEAR(expected.z, val.z, 1e-6);
    });
}

TEST(FaceCenteredGrid3, Sample)
{
    FaceCenteredGrid3 grid({ 5, 8, 6 }, { 2.0, 3.0, 1.5 });
    grid.Fill([](const Vector3D& x) {
        return Vector3D(3.0 * x.y + 1.0, 5.0 * x.z + 7.0, -1.0 * x.x - 9.0);
    });

    GridDataPositionFunc<3> pos = grid.CellCenterPosition();
    grid.ForEachCellIndex([&pos, &grid](size_t i, size_t j, size_t k) {
        Vector3D x = pos(i, j, k);
        Vector3D val = grid.Sample(x);
        Vector3D expected =
            Vector3D(3.0 * x.y + 1.0, 5.0 * x.z + 7.0, -1.0 * x.x - 9.0);
        EXPECT_NEAR(expected.x, val.x, 1e-6);
        EXPECT_NEAR(expected.y, val.y, 1e-6);
        EXPECT_NEAR(expected.z, val.z, 1e-6);
    });
}

TEST(FaceCenteredGrid3, Builder)
{
    {
        auto builder = FaceCenteredGrid3::Builder();

        auto grid =
            builder.Build(Vector3UZ(5, 2, 7), Vector3D(2.0, 4.0, 1.5),
                          Vector3D(-1.0, 2.0, 7.0), Vector3D(3.0, 5.0, -2.0));
        EXPECT_EQ(Vector3UZ(5, 2, 7), grid->Resolution());
        EXPECT_EQ(Vector3D(2.0, 4.0, 1.5), grid->GridSpacing());
        EXPECT_EQ(Vector3D(-1.0, 2.0, 7.0), grid->Origin());

        auto faceCenteredGrid =
            std::dynamic_pointer_cast<FaceCenteredGrid3>(grid);
        faceCenteredGrid->ForEachUIndex(
            [&faceCenteredGrid](const Vector3UZ& idx) {
                EXPECT_DOUBLE_EQ(3.0, faceCenteredGrid->U(idx));
            });
        faceCenteredGrid->ForEachVIndex(
            [&faceCenteredGrid](const Vector3UZ& idx) {
                EXPECT_DOUBLE_EQ(5.0, faceCenteredGrid->V(idx));
            });
        faceCenteredGrid->ForEachWIndex(
            [&faceCenteredGrid](const Vector3UZ& idx) {
                EXPECT_DOUBLE_EQ(-2.0, faceCenteredGrid->W(idx));
            });
    }

    {
        auto grid = FaceCenteredGrid3::Builder()
                        .WithResolution({ 5, 2, 7 })
                        .WithGridSpacing({ 2.0, 4.0, 1.5 })
                        .WithOrigin({ -1.0, 2.0, 7.0 })
                        .WithInitialValue({ 3.0, 5.0, -2.0 })
                        .Build();
        EXPECT_EQ(Vector3UZ(5, 2, 7), grid.Resolution());
        EXPECT_EQ(Vector3D(2.0, 4.0, 1.5), grid.GridSpacing());
        EXPECT_EQ(Vector3D(-1.0, 2.0, 7.0), grid.Origin());

        grid.ForEachUIndex([&grid](const Vector3UZ& idx) {
            EXPECT_DOUBLE_EQ(3.0, grid.U(idx));
        });
        grid.ForEachVIndex([&grid](const Vector3UZ& idx) {
            EXPECT_DOUBLE_EQ(5.0, grid.V(idx));
        });
        grid.ForEachWIndex([&grid](const Vector3UZ& idx) {
            EXPECT_DOUBLE_EQ(-2.0, grid.W(idx));
        });
    }
}

TEST(FaceCenteredGrid3, Serialization)
{
    FaceCenteredGrid3 grid1({ 5, 4, 3 }, { 1.0, 2.0, 3.0 }, { -5.0, 3.0, 1.0 });
    grid1.Fill([](const Vector3D& pt) { return Vector3D(pt.x, pt.y, pt.z); });

    // Serialize to in-memory stream
    std::vector<uint8_t> buffer1;
    grid1.Serialize(&buffer1);

    // Deserialize to non-zero array
    FaceCenteredGrid3 grid2({ 1, 2, 4 }, { 0.5, 1.0, 2.0 }, { 0.5, 2.0, -3.0 });
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

    grid1.ForEachUIndex([&grid1, &grid2](const Vector3UZ& idx) {
        EXPECT_DOUBLE_EQ(grid1.U(idx), grid2.U(idx));
    });
    grid1.ForEachVIndex([&grid1, &grid2](const Vector3UZ& idx) {
        EXPECT_DOUBLE_EQ(grid1.V(idx), grid2.V(idx));
    });
    grid1.ForEachWIndex([&grid1, &grid2](const Vector3UZ& idx) {
        EXPECT_DOUBLE_EQ(grid1.W(idx), grid2.W(idx));
    });

    // Serialize zero-sized array
    FaceCenteredGrid3 grid3;
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