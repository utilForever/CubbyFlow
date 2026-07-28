#include "gtest/gtest.h"

#include <Core/Emitter/VolumeGridEmitter3.hpp>
#include <Core/Geometry/Sphere.hpp>
#include <Core/Grid/CellCenteredScalarGrid.hpp>
#include <Core/Grid/CellCenteredVectorGrid.hpp>
#include <Core/Utils/LevelSetUtils.hpp>

using namespace CubbyFlow;

TEST(VolumeGridEmitter3, Velocity)
{
    auto sphere = Sphere3::Builder()
                      .WithCenter({ 0.5, 0.75, 0.5 })
                      .WithRadius(0.15)
                      .MakeShared();

    auto emitter =
        VolumeGridEmitter3::Builder().WithSourceRegion(sphere).MakeShared();

    auto grid = CellCenteredVectorGrid3::Builder()
                    .WithResolution({ 16, 16, 16 })
                    .WithGridSpacing({ 1.0 / 16.0, 1.0 / 16.0, 1.0 / 16.0 })
                    .WithOrigin({ 0, 0, 0 })
                    .MakeShared();

    auto mapper = [](double sdf, const Vector3D& pt, const Vector3D& oldVal) {
        if (sdf < 0.0)
        {
            return Vector3D(pt.y, -pt.x, 3.5);
        }
        else
        {
            return Vector3D(oldVal);
        }
    };

    emitter->AddTarget(grid, mapper);

    emitter->Update(0.0, 0.01);

    GridDataPositionFunc<3> pos = grid->DataPosition();
    grid->ForEachDataPointIndex(
        [&pos, &emitter, &grid](size_t i, size_t j, size_t k) {
            Vector3D gx = pos(i, j, k);
            double sdf = emitter->GetSourceRegion()->SignedDistance(gx);

            if (IsInsideSDF(sdf))
            {
                Vector3D answer{ gx.y, -gx.x, 3.5 };
                Vector3D acttual = (*grid)(i, j, k);

                EXPECT_NEAR(answer.x, acttual.x, 1e-6);
                EXPECT_NEAR(answer.y, acttual.y, 1e-6);
            }
        });
}

TEST(VolumeGridEmitter3, SignedDistance)
{
    auto sphere = Sphere3::Builder()
                      .WithCenter({ 0.5, 0.75, 0.5 })
                      .WithRadius(0.15)
                      .MakeShared();

    auto emitter =
        VolumeGridEmitter3::Builder().WithSourceRegion(sphere).MakeShared();

    auto grid = CellCenteredScalarGrid3::Builder()
                    .WithResolution({ 16, 16, 16 })
                    .WithGridSpacing({ 1.0 / 16.0, 1.0 / 16.0, 1.0 / 16.0 })
                    .WithOrigin({ 0, 0, 0 })
                    .WithInitialValue(std::numeric_limits<double>::max())
                    .MakeShared();

    emitter->AddSignedDistanceTarget(grid);

    emitter->Update(0.0, 0.01);

    GridDataPositionFunc<3> pos = grid->DataPosition();
    grid->ForEachDataPointIndex(
        [&pos, &sphere, &grid](size_t i, size_t j, size_t k) {
            Vector3D gx = pos(i, j, k);
            double answer = (sphere->center - gx).Length() - 0.15;
            double acttual = (*grid)(i, j, k);

            EXPECT_NEAR(answer, acttual, 1e-6);
        });
}

TEST(VolumeGridEmitter3, StepFunction)
{
    auto sphere = Sphere3::Builder()
                      .WithCenter({ 0.5, 0.75, 0.5 })
                      .WithRadius(0.15)
                      .MakeShared();

    auto emitter =
        VolumeGridEmitter3::Builder().WithSourceRegion(sphere).MakeShared();

    auto grid = CellCenteredScalarGrid3::Builder()
                    .WithResolution({ 16, 16, 16 })
                    .WithGridSpacing({ 1.0 / 16.0, 1.0 / 16.0, 1.0 / 16.0 })
                    .WithOrigin({ 0, 0, 0 })
                    .MakeShared();

    emitter->AddStepFunctionTarget(grid, 3.0, 7.0);

    emitter->Update(0.0, 0.01);

    GridDataPositionFunc<3> pos = grid->DataPosition();
    grid->ForEachDataPointIndex(
        [&pos, &sphere, &grid](size_t i, size_t j, size_t k) {
            Vector3D gx = pos(i, j, k);
            double answer = (sphere->center - gx).Length() - 0.15;
            answer = 4.0 * (1.0 - SmearedHeavisideSDF(answer * 16.0)) + 3.0;
            double acttual = (*grid)(i, j, k);

            EXPECT_NEAR(answer, acttual, 1e-6);
        });
}