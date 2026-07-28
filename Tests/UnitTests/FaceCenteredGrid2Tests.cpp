#include "gtest/gtest.h"

#include <Core/Grid/FaceCenteredGrid.hpp>

using namespace CubbyFlow;

TEST(FaceCenteredGrid2, Constructors)
{
    // Default constructors
    FaceCenteredGrid2 grid1;
    EXPECT_EQ(0u, grid1.Resolution().x);
    EXPECT_EQ(0u, grid1.Resolution().y);
    EXPECT_DOUBLE_EQ(1.0, grid1.GridSpacing().x);
    EXPECT_DOUBLE_EQ(1.0, grid1.GridSpacing().y);
    EXPECT_DOUBLE_EQ(0.0, grid1.Origin().x);
    EXPECT_DOUBLE_EQ(0.0, grid1.Origin().y);
    EXPECT_EQ(0u, grid1.USize().x);
    EXPECT_EQ(0u, grid1.USize().y);
    EXPECT_EQ(0u, grid1.VSize().x);
    EXPECT_EQ(0u, grid1.VSize().y);
    EXPECT_DOUBLE_EQ(0.0, grid1.UOrigin().x);
    EXPECT_DOUBLE_EQ(0.5, grid1.UOrigin().y);
    EXPECT_DOUBLE_EQ(0.5, grid1.VOrigin().x);
    EXPECT_DOUBLE_EQ(0.0, grid1.VOrigin().y);

    // Constructor with params
    FaceCenteredGrid2 grid2({ 5, 4 }, { 1.0, 2.0 }, { 3.0, 4.0 }, { 5.0, 6.0 });
    EXPECT_EQ(5u, grid2.Resolution().x);
    EXPECT_EQ(4u, grid2.Resolution().y);
    EXPECT_DOUBLE_EQ(1.0, grid2.GridSpacing().x);
    EXPECT_DOUBLE_EQ(2.0, grid2.GridSpacing().y);
    EXPECT_DOUBLE_EQ(3.0, grid2.Origin().x);
    EXPECT_DOUBLE_EQ(4.0, grid2.Origin().y);
    EXPECT_EQ(6u, grid2.USize().x);
    EXPECT_EQ(4u, grid2.USize().y);
    EXPECT_EQ(5u, grid2.VSize().x);
    EXPECT_EQ(5u, grid2.VSize().y);
    EXPECT_DOUBLE_EQ(3.0, grid2.UOrigin().x);
    EXPECT_DOUBLE_EQ(5.0, grid2.UOrigin().y);
    EXPECT_DOUBLE_EQ(3.5, grid2.VOrigin().x);
    EXPECT_DOUBLE_EQ(4.0, grid2.VOrigin().y);
    grid2.ForEachUIndex([&grid2](const Vector2UZ& idx) {
        EXPECT_DOUBLE_EQ(5.0, grid2.U(idx));
    });
    grid2.ForEachVIndex([&grid2](const Vector2UZ& idx) {
        EXPECT_DOUBLE_EQ(6.0, grid2.V(idx));
    });

    // Copy constructor
    FaceCenteredGrid2 grid3(grid2);
    EXPECT_EQ(5u, grid3.Resolution().x);
    EXPECT_EQ(4u, grid3.Resolution().y);
    EXPECT_DOUBLE_EQ(1.0, grid3.GridSpacing().x);
    EXPECT_DOUBLE_EQ(2.0, grid3.GridSpacing().y);
    EXPECT_DOUBLE_EQ(3.0, grid3.Origin().x);
    EXPECT_DOUBLE_EQ(4.0, grid3.Origin().y);
    EXPECT_EQ(6u, grid3.USize().x);
    EXPECT_EQ(4u, grid3.USize().y);
    EXPECT_EQ(5u, grid3.VSize().x);
    EXPECT_EQ(5u, grid3.VSize().y);
    EXPECT_DOUBLE_EQ(3.0, grid3.UOrigin().x);
    EXPECT_DOUBLE_EQ(5.0, grid3.UOrigin().y);
    EXPECT_DOUBLE_EQ(3.5, grid3.VOrigin().x);
    EXPECT_DOUBLE_EQ(4.0, grid3.VOrigin().y);
    grid3.ForEachUIndex([&grid3](const Vector2UZ& idx) {
        EXPECT_DOUBLE_EQ(5.0, grid3.U(idx));
    });
    grid3.ForEachVIndex([&grid3](const Vector2UZ& idx) {
        EXPECT_DOUBLE_EQ(6.0, grid3.V(idx));
    });
}

TEST(FaceCenteredGrid2, Fill)
{
    FaceCenteredGrid2 grid({ 5, 4 }, { 1.0, 1.0 }, { 0.0, 0.0 }, { 0.0, 0.0 });
    grid.Fill(Vector2D(42.0, 27.0));

    for (size_t j = 0; j < grid.USize().y; ++j)
    {
        for (size_t i = 0; i < grid.USize().x; ++i)
        {
            EXPECT_DOUBLE_EQ(42.0, grid.U(i, j));
        }
    }

    for (size_t j = 0; j < grid.VSize().y; ++j)
    {
        for (size_t i = 0; i < grid.VSize().x; ++i)
        {
            EXPECT_DOUBLE_EQ(27.0, grid.V(i, j));
        }
    }

    auto func = [](const Vector2D& x) { return x; };
    grid.Fill(func);

    for (size_t j = 0; j < grid.USize().y; ++j)
    {
        for (size_t i = 0; i < grid.USize().x; ++i)
        {
            EXPECT_DOUBLE_EQ(static_cast<double>(i), grid.U(i, j));
        }
    }

    for (size_t j = 0; j < grid.VSize().y; ++j)
    {
        for (size_t i = 0; i < grid.VSize().x; ++i)
        {
            EXPECT_DOUBLE_EQ(static_cast<double>(j), grid.V(i, j));
        }
    }
}

TEST(FaceCenteredGrid2, DivergenceAtCellCenter)
{
    FaceCenteredGrid2 grid({ 5, 8 }, { 2.0, 3.0 });

    grid.Fill(Vector2D(1.0, -2.0));

    for (size_t j = 0; j < grid.Resolution().y; ++j)
    {
        for (size_t i = 0; i < grid.Resolution().x; ++i)
        {
            EXPECT_DOUBLE_EQ(0.0, grid.DivergenceAtCellCenter(i, j));
        }
    }

    grid.Fill([](const Vector2D& x) { return x; });

    for (size_t j = 0; j < grid.Resolution().y; ++j)
    {
        for (size_t i = 0; i < grid.Resolution().x; ++i)
        {
            EXPECT_NEAR(2.0, grid.DivergenceAtCellCenter(i, j), 1e-6);
        }
    }
}

TEST(FaceCenteredGrid2, CurlAtCellCenter)
{
    FaceCenteredGrid2 grid({ 5, 8 }, { 2.0, 3.0 });

    grid.Fill(Vector2D(1.0, -2.0));

    for (size_t j = 0; j < grid.Resolution().y; ++j)
    {
        for (size_t i = 0; i < grid.Resolution().x; ++i)
        {
            EXPECT_DOUBLE_EQ(0.0, grid.CurlAtCellCenter(i, j));
        }
    }

    grid.Fill([](const Vector2D& x) { return Vector2D(-x.y, x.x); });

    for (size_t j = 1; j < grid.Resolution().y - 1; ++j)
    {
        for (size_t i = 1; i < grid.Resolution().x - 1; ++i)
        {
            EXPECT_NEAR(2.0, grid.CurlAtCellCenter(i, j), 1e-6);
        }
    }
}

TEST(FaceCenteredGrid2, ValueAtCellCenter)
{
    FaceCenteredGrid2 grid({ 5, 8 }, { 2.0, 3.0 });
    grid.Fill([](const Vector2D& x) {
        return Vector2D(3.0 * x.y + 1.0, 5.0 * x.x + 7.0);
    });

    auto pos = grid.CellCenterPosition();
    grid.ForEachCellIndex([&grid, &pos](const Vector2UZ& idx) {
        Vector2D val = grid.ValueAtCellCenter(idx);
        Vector2D x = pos(idx);
        Vector2D expected = Vector2D(3.0 * x.y + 1.0, 5.0 * x.x + 7.0);
        EXPECT_NEAR(expected.x, val.x, 1e-6);
        EXPECT_NEAR(expected.y, val.y, 1e-6);
    });
}

TEST(FaceCenteredGrid2, Sample)
{
    FaceCenteredGrid2 grid({ 5, 8 }, { 2.0, 3.0 });
    grid.Fill([](const Vector2D& x) {
        return Vector2D(3.0 * x.y + 1.0, 5.0 * x.x + 7.0);
    });

    auto pos = grid.CellCenterPosition();
    grid.ForEachCellIndex([&pos, &grid](const Vector2UZ& idx) {
        Vector2D x = pos(idx);
        Vector2D val = grid.Sample(x);
        Vector2D expected = Vector2D(3.0 * x.y + 1.0, 5.0 * x.x + 7.0);
        EXPECT_NEAR(expected.x, val.x, 1e-6);
        EXPECT_NEAR(expected.y, val.y, 1e-6);
    });
}

TEST(FaceCenteredGrid2, Builder)
{
    {
        auto builder = FaceCenteredGrid2::GetBuilder();

        auto grid = builder.Build(Vector2UZ(5, 2), Vector2D(2.0, 4.0),
                                  Vector2D(-1.0, 2.0), Vector2D(3.0, 5.0));
        EXPECT_EQ(Vector2UZ(5, 2), grid->Resolution());
        EXPECT_EQ(Vector2D(2.0, 4.0), grid->GridSpacing());
        EXPECT_EQ(Vector2D(-1.0, 2.0), grid->Origin());

        auto faceCenteredGrid =
            std::dynamic_pointer_cast<FaceCenteredGrid2>(grid);
        EXPECT_TRUE(faceCenteredGrid != nullptr);

        faceCenteredGrid->ForEachUIndex(
            [&faceCenteredGrid](const Vector2UZ& idx) {
                EXPECT_DOUBLE_EQ(3.0, faceCenteredGrid->U(idx));
            });
        faceCenteredGrid->ForEachVIndex(
            [&faceCenteredGrid](const Vector2UZ& idx) {
                EXPECT_DOUBLE_EQ(5.0, faceCenteredGrid->V(idx));
            });
    }

    {
        auto grid = FaceCenteredGrid2::Builder()
                        .WithResolution({ 5, 2 })
                        .WithGridSpacing({ 2, 4 })
                        .WithOrigin({ -1, 2 })
                        .WithInitialValue({ 3, 5 })
                        .Build();

        EXPECT_EQ(Vector2UZ(5, 2), grid.Resolution());
        EXPECT_EQ(Vector2D(2.0, 4.0), grid.GridSpacing());
        EXPECT_EQ(Vector2D(-1.0, 2.0), grid.Origin());

        grid.ForEachUIndex([&grid](const Vector2UZ& idx) {
            EXPECT_DOUBLE_EQ(3.0, grid.U(idx));
        });
        grid.ForEachVIndex([&grid](const Vector2UZ& idx) {
            EXPECT_DOUBLE_EQ(5.0, grid.V(idx));
        });
    }
}

TEST(FaceCenteredGrid2, Serialization)
{
    FaceCenteredGrid2 grid1({ 5, 4 }, { 1.0, 2.0 }, { -5.0, 3.0 });
    grid1.Fill([](const Vector2D& pt) { return Vector2D(pt.x, pt.y); });

    // Serialize to in-memory stream
    std::vector<uint8_t> buffer1;
    grid1.Serialize(&buffer1);

    // Deserialize to non-zero array
    FaceCenteredGrid2 grid2({ 1, 2 }, { 0.5, 1.0 }, { 0.5, 2.0 });
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
    EXPECT_EQ(6u, grid2.USize().x);
    EXPECT_EQ(4u, grid2.USize().y);
    EXPECT_EQ(5u, grid2.VSize().x);
    EXPECT_EQ(5u, grid2.VSize().y);
    EXPECT_DOUBLE_EQ(-5.0, grid2.UOrigin().x);
    EXPECT_DOUBLE_EQ(4.0, grid2.UOrigin().y);
    EXPECT_DOUBLE_EQ(-4.5, grid2.VOrigin().x);
    EXPECT_DOUBLE_EQ(3.0, grid2.VOrigin().y);

    grid1.ForEachUIndex([&grid1, &grid2](const Vector2UZ& idx) {
        EXPECT_DOUBLE_EQ(grid1.U(idx), grid2.U(idx));
    });

    grid1.ForEachVIndex([&grid1, &grid2](const Vector2UZ& idx) {
        EXPECT_DOUBLE_EQ(grid1.V(idx), grid2.V(idx));
    });

    // Serialize zero-sized array
    FaceCenteredGrid2 grid3;
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
    EXPECT_EQ(0u, grid2.USize().x);
    EXPECT_EQ(0u, grid2.USize().y);
    EXPECT_EQ(0u, grid2.VSize().x);
    EXPECT_EQ(0u, grid2.VSize().y);
    EXPECT_DOUBLE_EQ(0.0, grid2.UOrigin().x);
    EXPECT_DOUBLE_EQ(0.5, grid2.UOrigin().y);
    EXPECT_DOUBLE_EQ(0.5, grid2.VOrigin().x);
    EXPECT_DOUBLE_EQ(0.0, grid2.VOrigin().y);
}