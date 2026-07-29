#include "gtest/gtest.h"

#include <Core/Grid/CellCenteredScalarGrid.hpp>
#include <Core/Grid/CellCenteredVectorGrid.hpp>
#include <Core/Grid/GridSystemData.hpp>
#include <Core/Grid/VertexCenteredScalarGrid.hpp>
#include <Core/Grid/VertexCenteredVectorGrid.hpp>

using namespace CubbyFlow;

TEST(GridSystemData2, Constructors)
{
    GridSystemData2 grids1;
    EXPECT_EQ(0u, grids1.Resolution().x);
    EXPECT_EQ(0u, grids1.Resolution().y);
    EXPECT_EQ(1.0, grids1.GridSpacing().x);
    EXPECT_EQ(1.0, grids1.GridSpacing().y);
    EXPECT_EQ(0.0, grids1.Origin().x);
    EXPECT_EQ(0.0, grids1.Origin().y);

    GridSystemData2 grids2({ 32, 64 }, { 1.0, 2.0 }, { -5.0, 4.5 });

    EXPECT_EQ(32u, grids2.Resolution().x);
    EXPECT_EQ(64u, grids2.Resolution().y);
    EXPECT_EQ(1.0, grids2.GridSpacing().x);
    EXPECT_EQ(2.0, grids2.GridSpacing().y);
    EXPECT_EQ(-5.0, grids2.Origin().x);
    EXPECT_EQ(4.5, grids2.Origin().y);

    GridSystemData2 grids3(grids2);

    EXPECT_EQ(32u, grids3.Resolution().x);
    EXPECT_EQ(64u, grids3.Resolution().y);
    EXPECT_EQ(1.0, grids3.GridSpacing().x);
    EXPECT_EQ(2.0, grids3.GridSpacing().y);
    EXPECT_EQ(-5.0, grids3.Origin().x);
    EXPECT_EQ(4.5, grids3.Origin().y);

    EXPECT_TRUE(grids2.Velocity() != grids3.Velocity());
}

TEST(GridSystemData2, Serialize)
{
    std::vector<uint8_t> buffer;

    GridSystemData2 grids({ 32, 64 }, { 1.0, 2.0 }, { -5.0, 4.5 });

    size_t scalarIdx0 = grids.AddScalarData(
        std::make_shared<CellCenteredScalarGrid2::Builder>());
    size_t vectorIdx0 = grids.AddVectorData(
        std::make_shared<CellCenteredVectorGrid2::Builder>());
    size_t scalarIdx1 = grids.AddAdvectableScalarData(
        std::make_shared<VertexCenteredScalarGrid2::Builder>());
    size_t vectorIdx1 = grids.AddAdvectableVectorData(
        std::make_shared<VertexCenteredVectorGrid2::Builder>());

    auto scalar0 = grids.ScalarDataAt(scalarIdx0);
    auto vector0 = grids.VectorDataAt(vectorIdx0);
    auto scalar1 = grids.AdvectableScalarDataAt(scalarIdx1);
    auto vector1 = grids.AdvectableVectorDataAt(vectorIdx1);

    scalar0->Fill([](const Vector2D& pt) -> double { return pt.Length(); });

    vector0->Fill([](const Vector2D& pt) -> Vector2D { return pt; });

    scalar1->Fill([](const Vector2D& pt) -> double {
        return (pt - Vector2D(1, 2)).Length();
    });

    vector1->Fill(
        [](const Vector2D& pt) -> Vector2D { return pt - Vector2D(1, 2); });

    grids.Serialize(&buffer);

    GridSystemData2 grids2;
    grids2.Deserialize(buffer);

    EXPECT_EQ(32u, grids2.Resolution().x);
    EXPECT_EQ(64u, grids2.Resolution().y);
    EXPECT_EQ(1.0, grids2.GridSpacing().x);
    EXPECT_EQ(2.0, grids2.GridSpacing().y);
    EXPECT_EQ(-5.0, grids2.Origin().x);
    EXPECT_EQ(4.5, grids2.Origin().y);

    EXPECT_EQ(1u, grids2.NumberOfScalarData());
    EXPECT_EQ(1u, grids2.NumberOfVectorData());
    EXPECT_EQ(1u, grids2.NumberOfAdvectableScalarData());
    EXPECT_EQ(2u, grids2.NumberOfAdvectableVectorData());

    auto scalar0_2 = grids2.ScalarDataAt(scalarIdx0);
    EXPECT_TRUE(std::dynamic_pointer_cast<CellCenteredScalarGrid2>(scalar0_2) !=
                nullptr);
    EXPECT_EQ(scalar0->Resolution(), scalar0_2->Resolution());
    EXPECT_EQ(scalar0->GridSpacing(), scalar0_2->GridSpacing());
    EXPECT_EQ(scalar0->Origin(), scalar0_2->Origin());
    EXPECT_EQ(scalar0->DataSize(), scalar0_2->DataSize());
    EXPECT_EQ(scalar0->DataOrigin(), scalar0_2->DataOrigin());
    scalar0->ForEachDataPointIndex([&scalar0, &scalar0_2](size_t i, size_t j) {
        EXPECT_EQ((*scalar0)(i, j), (*scalar0_2)(i, j));
    });

    auto vector0_2 = grids2.VectorDataAt(vectorIdx0);
    auto cell_vector0 =
        std::dynamic_pointer_cast<CellCenteredVectorGrid2>(vector0);
    auto cell_vector0_2 =
        std::dynamic_pointer_cast<CellCenteredVectorGrid2>(vector0_2);
    EXPECT_TRUE(cell_vector0_2 != nullptr);
    EXPECT_EQ(vector0->Resolution(), vector0_2->Resolution());
    EXPECT_EQ(vector0->GridSpacing(), vector0_2->GridSpacing());
    EXPECT_EQ(vector0->Origin(), vector0_2->Origin());
    EXPECT_EQ(cell_vector0->DataSize(), cell_vector0_2->DataSize());
    EXPECT_EQ(cell_vector0->DataOrigin(), cell_vector0_2->DataOrigin());
    cell_vector0->ForEachDataPointIndex(
        [&cell_vector0, &cell_vector0_2](size_t i, size_t j) {
            EXPECT_EQ((*cell_vector0)(i, j), (*cell_vector0_2)(i, j));
        });

    auto scalar1_2 = grids2.AdvectableScalarDataAt(scalarIdx1);
    EXPECT_TRUE(std::dynamic_pointer_cast<VertexCenteredScalarGrid2>(
                    scalar1_2) != nullptr);
    EXPECT_EQ(scalar1->Resolution(), scalar1_2->Resolution());
    EXPECT_EQ(scalar1->GridSpacing(), scalar1_2->GridSpacing());
    EXPECT_EQ(scalar1->Origin(), scalar1_2->Origin());
    EXPECT_EQ(scalar1->DataSize(), scalar1_2->DataSize());
    EXPECT_EQ(scalar1->DataOrigin(), scalar1_2->DataOrigin());
    scalar1->ForEachDataPointIndex([&scalar1, &scalar1_2](size_t i, size_t j) {
        EXPECT_EQ((*scalar1)(i, j), (*scalar1_2)(i, j));
    });

    auto vector1_2 = grids2.AdvectableVectorDataAt(vectorIdx1);
    auto vert_vector1 =
        std::dynamic_pointer_cast<VertexCenteredVectorGrid2>(vector1);
    auto vert_vector1_2 =
        std::dynamic_pointer_cast<VertexCenteredVectorGrid2>(vector1_2);
    EXPECT_TRUE(vert_vector1_2 != nullptr);
    EXPECT_EQ(vector1->Resolution(), vector1_2->Resolution());
    EXPECT_EQ(vector1->GridSpacing(), vector1_2->GridSpacing());
    EXPECT_EQ(vector1->Origin(), vector1_2->Origin());
    EXPECT_EQ(vert_vector1->DataSize(), vert_vector1_2->DataSize());
    EXPECT_EQ(vert_vector1->DataOrigin(), vert_vector1_2->DataOrigin());
    vert_vector1->ForEachDataPointIndex(
        [&vert_vector1, &vert_vector1_2](size_t i, size_t j) {
            EXPECT_EQ((*vert_vector1)(i, j), (*vert_vector1_2)(i, j));
        });

    auto velocity = grids.Velocity();
    auto velocity2 = grids2.Velocity();
    EXPECT_EQ(velocity->Resolution(), velocity2->Resolution());
    EXPECT_EQ(velocity->GridSpacing(), velocity2->GridSpacing());
    EXPECT_EQ(velocity->Origin(), velocity2->Origin());
    EXPECT_EQ(velocity->USize(), velocity2->USize());
    EXPECT_EQ(velocity->VSize(), velocity2->VSize());
    EXPECT_EQ(velocity->UOrigin(), velocity2->UOrigin());
    EXPECT_EQ(velocity->VOrigin(), velocity2->VOrigin());
    velocity->ForEachUIndex([&velocity, &velocity2](const Vector2UZ& idx) {
        EXPECT_EQ(velocity->U(idx), velocity2->U(idx));
    });
    velocity->ForEachVIndex([&velocity, &velocity2](const Vector2UZ& idx) {
        EXPECT_EQ(velocity->V(idx), velocity2->V(idx));
    });
}