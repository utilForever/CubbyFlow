#include "../../Sources/Core/Flatbuffers/generated/GridSystemData3_generated.h"
#include "gtest/gtest.h"

#include <Core/Grid/CellCenteredScalarGrid.hpp>
#include <Core/Grid/CellCenteredVectorGrid.hpp>
#include <Core/Grid/FaceCenteredGrid.hpp>
#include <Core/Grid/GridSystemData.hpp>
#include <Core/Grid/VertexCenteredScalarGrid.hpp>
#include <Core/Grid/VertexCenteredVectorGrid.hpp>

#include <utility>

using namespace CubbyFlow;

namespace
{
std::vector<uint8_t> MakeGridSystemDataWithSecondVelocity()
{
    FaceCenteredGrid3 velocity0({ 2, 3, 4 }, {}, {}, { 1.0, 2.0, 3.0 });
    FaceCenteredGrid3 velocity1({ 2, 3, 4 }, {}, {}, { 4.0, 5.0, 6.0 });
    std::vector<uint8_t> velocityBuffer0;
    std::vector<uint8_t> velocityBuffer1;
    velocity0.Serialize(&velocityBuffer0);
    velocity1.Serialize(&velocityBuffer1);

    flatbuffers::FlatBufferBuilder builder(1024);
    std::vector<flatbuffers::Offset<fbs::VectorGridSerialized3>> velocities;
    velocities.push_back(fbs::CreateVectorGridSerialized3Direct(
        builder, velocity0.TypeName().c_str(), &velocityBuffer0));
    velocities.push_back(fbs::CreateVectorGridSerialized3Direct(
        builder, velocity1.TypeName().c_str(), &velocityBuffer1));

    std::vector<flatbuffers::Offset<fbs::ScalarGridSerialized3>> scalars;
    std::vector<flatbuffers::Offset<fbs::VectorGridSerialized3>> vectors;
    const auto emptyScalars = builder.CreateVector(scalars);
    const auto emptyVectors = builder.CreateVector(vectors);
    const auto advectableVectors = builder.CreateVector(velocities);
    const fbs::Vector3UZ resolution(2, 3, 4);
    const fbs::Vector3D gridSpacing(1.0, 1.0, 1.0);
    const fbs::Vector3D origin(0.0, 0.0, 0.0);
    const auto data = fbs::CreateGridSystemData3(
        builder, &resolution, &gridSpacing, &origin, 1, emptyScalars,
        emptyVectors, emptyScalars, advectableVectors);
    builder.Finish(data);

    return { builder.GetBufferPointer(),
             builder.GetBufferPointer() + builder.GetSize() };
}

void ExpectSecondVelocity(const GridSystemData3& grids)
{
    EXPECT_EQ(1u, grids.VelocityIndex());
    EXPECT_EQ(grids.AdvectableVectorDataAt(1), grids.Velocity());
    EXPECT_DOUBLE_EQ(4.0, grids.Velocity()->U(0, 0, 0));
}
}  // namespace

TEST(GridSystemData3, Constructors)
{
    GridSystemData3 grids1;
    EXPECT_EQ(0u, grids1.Resolution().x);
    EXPECT_EQ(0u, grids1.Resolution().y);
    EXPECT_EQ(0u, grids1.Resolution().z);
    EXPECT_EQ(1.0, grids1.GridSpacing().x);
    EXPECT_EQ(1.0, grids1.GridSpacing().y);
    EXPECT_EQ(1.0, grids1.GridSpacing().z);
    EXPECT_EQ(0.0, grids1.Origin().x);
    EXPECT_EQ(0.0, grids1.Origin().y);
    EXPECT_EQ(0.0, grids1.Origin().z);

    GridSystemData3 grids2({ 32, 64, 48 }, { 1.0, 2.0, 3.0 },
                           { -5.0, 4.5, 10.0 });
    EXPECT_EQ(32u, grids2.Resolution().x);
    EXPECT_EQ(64u, grids2.Resolution().y);
    EXPECT_EQ(48u, grids2.Resolution().z);
    EXPECT_EQ(1.0, grids2.GridSpacing().x);
    EXPECT_EQ(2.0, grids2.GridSpacing().y);
    EXPECT_EQ(3.0, grids2.GridSpacing().z);
    EXPECT_EQ(-5.0, grids2.Origin().x);
    EXPECT_EQ(4.5, grids2.Origin().y);
    EXPECT_EQ(10.0, grids2.Origin().z);

    GridSystemData3 grids3(grids2);

    EXPECT_EQ(32u, grids3.Resolution().x);
    EXPECT_EQ(64u, grids3.Resolution().y);
    EXPECT_EQ(48u, grids3.Resolution().z);
    EXPECT_EQ(1.0, grids3.GridSpacing().x);
    EXPECT_EQ(2.0, grids3.GridSpacing().y);
    EXPECT_EQ(3.0, grids3.GridSpacing().z);
    EXPECT_EQ(-5.0, grids3.Origin().x);
    EXPECT_EQ(4.5, grids3.Origin().y);
    EXPECT_EQ(10.0, grids3.Origin().z);

    EXPECT_TRUE(grids2.Velocity() != grids3.Velocity());
}

TEST(GridSystemData3, CopyAssignment)
{
    GridSystemData3 source({ 2, 3, 4 }, { 1.0, 1.0, 1.0 }, {});
    source.AddScalarData(std::make_shared<CellCenteredScalarGrid3::Builder>());
    source.AddVectorData(std::make_shared<CellCenteredVectorGrid3::Builder>());
    source.AddAdvectableScalarData(
        std::make_shared<VertexCenteredScalarGrid3::Builder>());
    source.AddAdvectableVectorData(
        std::make_shared<VertexCenteredVectorGrid3::Builder>());

    GridSystemData3 destination({ 1, 1, 1 }, { 1.0, 1.0, 1.0 }, {});
    destination.AddScalarData(
        std::make_shared<VertexCenteredScalarGrid3::Builder>());
    destination.AddVectorData(
        std::make_shared<VertexCenteredVectorGrid3::Builder>());
    destination.AddAdvectableScalarData(
        std::make_shared<CellCenteredScalarGrid3::Builder>());
    destination.AddAdvectableVectorData(
        std::make_shared<CellCenteredVectorGrid3::Builder>());

    destination = source;

    EXPECT_EQ(source.NumberOfScalarData(), destination.NumberOfScalarData());
    EXPECT_EQ(source.NumberOfVectorData(), destination.NumberOfVectorData());
    EXPECT_EQ(source.NumberOfAdvectableScalarData(),
              destination.NumberOfAdvectableScalarData());
    EXPECT_EQ(source.NumberOfAdvectableVectorData(),
              destination.NumberOfAdvectableVectorData());
    EXPECT_NE(source.Velocity(), destination.Velocity());
}

TEST(GridSystemData3, PreservesDeserializedVelocityIndex)
{
    GridSystemData3 source;
    source.Deserialize(MakeGridSystemDataWithSecondVelocity());
    ExpectSecondVelocity(source);

    GridSystemData3 copied(source);
    ExpectSecondVelocity(copied);

    GridSystemData3 assigned;
    assigned = source;
    ExpectSecondVelocity(assigned);

    GridSystemData3 moved(std::move(copied));
    ExpectSecondVelocity(moved);

    GridSystemData3 moveAssigned;
    moveAssigned = std::move(assigned);
    ExpectSecondVelocity(moveAssigned);
}

TEST(GridSystemData3, Serialize)
{
    std::vector<uint8_t> buffer;

    GridSystemData3 grids({ 32, 64, 48 }, { 1.0, 2.0, 3.0 },
                          { -5.0, 4.5, 10.0 });

    size_t scalarIdx0 = grids.AddScalarData(
        std::make_shared<CellCenteredScalarGrid3::Builder>());
    size_t vectorIdx0 = grids.AddVectorData(
        std::make_shared<CellCenteredVectorGrid3::Builder>());
    size_t scalarIdx1 = grids.AddAdvectableScalarData(
        std::make_shared<VertexCenteredScalarGrid3::Builder>());
    size_t vectorIdx1 = grids.AddAdvectableVectorData(
        std::make_shared<VertexCenteredVectorGrid3::Builder>());

    auto scalar0 = grids.ScalarDataAt(scalarIdx0);
    auto vector0 = grids.VectorDataAt(vectorIdx0);
    auto scalar1 = grids.AdvectableScalarDataAt(scalarIdx1);
    auto vector1 = grids.AdvectableVectorDataAt(vectorIdx1);

    scalar0->Fill([](const Vector3D& pt) -> double { return pt.Length(); });

    vector0->Fill([](const Vector3D& pt) -> Vector3D { return pt; });

    scalar1->Fill([](const Vector3D& pt) -> double {
        return (pt - Vector3D(1, 2, 3)).Length();
    });

    vector1->Fill(
        [](const Vector3D& pt) -> Vector3D { return pt - Vector3D(1, 2, 3); });

    grids.Serialize(&buffer);

    GridSystemData3 grids2;
    grids2.Deserialize(buffer);

    EXPECT_EQ(32u, grids2.Resolution().x);
    EXPECT_EQ(64u, grids2.Resolution().y);
    EXPECT_EQ(48u, grids2.Resolution().z);
    EXPECT_EQ(1.0, grids2.GridSpacing().x);
    EXPECT_EQ(2.0, grids2.GridSpacing().y);
    EXPECT_EQ(3.0, grids2.GridSpacing().z);
    EXPECT_EQ(-5.0, grids2.Origin().x);
    EXPECT_EQ(4.5, grids2.Origin().y);
    EXPECT_EQ(10.0, grids2.Origin().z);

    EXPECT_EQ(1u, grids2.NumberOfScalarData());
    EXPECT_EQ(1u, grids2.NumberOfVectorData());
    EXPECT_EQ(1u, grids2.NumberOfAdvectableScalarData());
    EXPECT_EQ(2u, grids2.NumberOfAdvectableVectorData());

    auto scalar0_2 = grids2.ScalarDataAt(scalarIdx0);
    EXPECT_TRUE(std::dynamic_pointer_cast<CellCenteredScalarGrid3>(scalar0_2) !=
                nullptr);
    EXPECT_EQ(scalar0->Resolution(), scalar0_2->Resolution());
    EXPECT_EQ(scalar0->GridSpacing(), scalar0_2->GridSpacing());
    EXPECT_EQ(scalar0->Origin(), scalar0_2->Origin());
    EXPECT_EQ(scalar0->DataSize(), scalar0_2->DataSize());
    EXPECT_EQ(scalar0->DataOrigin(), scalar0_2->DataOrigin());
    scalar0->ForEachDataPointIndex(
        [&scalar0, &scalar0_2](size_t i, size_t j, size_t k) {
            EXPECT_EQ((*scalar0)(i, j, k), (*scalar0_2)(i, j, k));
        });

    auto vector0_2 = grids2.VectorDataAt(vectorIdx0);
    auto cell_vector0 =
        std::dynamic_pointer_cast<CellCenteredVectorGrid3>(vector0);
    auto cell_vector0_2 =
        std::dynamic_pointer_cast<CellCenteredVectorGrid3>(vector0_2);
    EXPECT_TRUE(cell_vector0_2 != nullptr);
    EXPECT_EQ(vector0->Resolution(), vector0_2->Resolution());
    EXPECT_EQ(vector0->GridSpacing(), vector0_2->GridSpacing());
    EXPECT_EQ(vector0->Origin(), vector0_2->Origin());
    EXPECT_EQ(cell_vector0->DataSize(), cell_vector0_2->DataSize());
    EXPECT_EQ(cell_vector0->DataOrigin(), cell_vector0_2->DataOrigin());
    cell_vector0->ForEachDataPointIndex(
        [&cell_vector0, &cell_vector0_2](size_t i, size_t j, size_t k) {
            EXPECT_EQ((*cell_vector0)(i, j, k), (*cell_vector0_2)(i, j, k));
        });

    auto scalar1_2 = grids2.AdvectableScalarDataAt(scalarIdx1);
    EXPECT_TRUE(std::dynamic_pointer_cast<VertexCenteredScalarGrid3>(
                    scalar1_2) != nullptr);
    EXPECT_EQ(scalar1->Resolution(), scalar1_2->Resolution());
    EXPECT_EQ(scalar1->GridSpacing(), scalar1_2->GridSpacing());
    EXPECT_EQ(scalar1->Origin(), scalar1_2->Origin());
    EXPECT_EQ(scalar1->DataSize(), scalar1_2->DataSize());
    EXPECT_EQ(scalar1->DataOrigin(), scalar1_2->DataOrigin());
    scalar1->ForEachDataPointIndex(
        [&scalar1, &scalar1_2](size_t i, size_t j, size_t k) {
            EXPECT_EQ((*scalar1)(i, j, k), (*scalar1_2)(i, j, k));
        });

    auto vector1_2 = grids2.AdvectableVectorDataAt(vectorIdx1);
    auto vert_vector1 =
        std::dynamic_pointer_cast<VertexCenteredVectorGrid3>(vector1);
    auto vert_vector1_2 =
        std::dynamic_pointer_cast<VertexCenteredVectorGrid3>(vector1_2);
    EXPECT_TRUE(vert_vector1_2 != nullptr);
    EXPECT_EQ(vector1->Resolution(), vector1_2->Resolution());
    EXPECT_EQ(vector1->GridSpacing(), vector1_2->GridSpacing());
    EXPECT_EQ(vector1->Origin(), vector1_2->Origin());
    EXPECT_EQ(vert_vector1->DataSize(), vert_vector1_2->DataSize());
    EXPECT_EQ(vert_vector1->DataOrigin(), vert_vector1_2->DataOrigin());
    vert_vector1->ForEachDataPointIndex(
        [&vert_vector1, &vert_vector1_2](size_t i, size_t j, size_t k) {
            EXPECT_EQ((*vert_vector1)(i, j, k), (*vert_vector1_2)(i, j, k));
        });

    auto velocity = grids.Velocity();
    auto velocity2 = grids2.Velocity();
    EXPECT_EQ(velocity->Resolution(), velocity2->Resolution());
    EXPECT_EQ(velocity->GridSpacing(), velocity2->GridSpacing());
    EXPECT_EQ(velocity->Origin(), velocity2->Origin());
    EXPECT_EQ(velocity->USize(), velocity2->USize());
    EXPECT_EQ(velocity->VSize(), velocity2->VSize());
    EXPECT_EQ(velocity->WSize(), velocity2->WSize());
    EXPECT_EQ(velocity->UOrigin(), velocity2->UOrigin());
    EXPECT_EQ(velocity->VOrigin(), velocity2->VOrigin());
    EXPECT_EQ(velocity->WOrigin(), velocity2->WOrigin());
    velocity->ForEachUIndex([&velocity, &velocity2](const Vector3UZ& idx) {
        EXPECT_EQ(velocity->U(idx), velocity2->U(idx));
    });
    velocity->ForEachVIndex([&velocity, &velocity2](const Vector3UZ& idx) {
        EXPECT_EQ(velocity->V(idx), velocity2->V(idx));
    });
    velocity->ForEachWIndex([&velocity, &velocity2](const Vector3UZ& idx) {
        EXPECT_EQ(velocity->W(idx), velocity2->W(idx));
    });
}
