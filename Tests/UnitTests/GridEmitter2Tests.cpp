#include "gtest/gtest.h"

#include <Core/Emitter/VolumeGridEmitter2.hpp>
#include <Core/Geometry/Sphere.hpp>
#include <Core/Grid/CellCenteredScalarGrid.hpp>
#include <Core/Grid/CellCenteredVectorGrid.hpp>
#include <Core/Utils/LevelSetUtils.hpp>

using namespace CubbyFlow;

TEST(VolumeGridEmitter2, Velocity)
{
    auto sphere = Sphere2::Builder()
                      .WithCenter({ 0.5, 0.75 })
                      .WithRadius(0.15)
                      .MakeShared();

    auto emitter =
        VolumeGridEmitter2::Builder().WithSourceRegion(sphere).MakeShared();

    auto grid = CellCenteredVectorGrid2::Builder()
                    .WithResolution({ 16, 16 })
                    .WithGridSpacing({ 1.0 / 16.0, 1.0 / 16.0 })
                    .WithOrigin({ 0, 0 })
                    .MakeShared();

    auto mapper = [](double sdf, const Vector2D& pt, const Vector2D& oldVal) {
        if (sdf < 0.0)
        {
            return Vector2D(pt.y, -pt.x);
        }
        else
        {
            return Vector2D(oldVal);
        }
    };

    emitter->AddTarget(grid, mapper);

    emitter->Update(0.0, 0.01);

    GridDataPositionFunc<2> pos = grid->DataPosition();
    grid->ForEachDataPointIndex([&pos, &emitter, &grid](size_t i, size_t j) {
        Vector2D gx = pos(i, j);
        double sdf = emitter->GetSourceRegion()->SignedDistance(gx);

        if (IsInsideSDF(sdf))
        {
            Vector2D answer{ gx.y, -gx.x };
            Vector2D acttual = (*grid)(i, j);

            EXPECT_NEAR(answer.x, acttual.x, 1e-6);
            EXPECT_NEAR(answer.y, acttual.y, 1e-6);
        }
    });
}

TEST(VolumeGridEmitter2, SignedDistance)
{
    auto sphere = Sphere2::Builder()
                      .WithCenter({ 0.5, 0.75 })
                      .WithRadius(0.15)
                      .MakeShared();

    auto emitter =
        VolumeGridEmitter2::Builder().WithSourceRegion(sphere).MakeShared();

    auto grid = CellCenteredScalarGrid2::Builder()
                    .WithResolution({ 16, 16 })
                    .WithGridSpacing({ 1.0 / 16.0, 1.0 / 16.0 })
                    .WithOrigin({ 0, 0 })
                    .WithInitialValue(std::numeric_limits<double>::max())
                    .MakeShared();

    emitter->AddSignedDistanceTarget(grid);

    emitter->Update(0.0, 0.01);

    GridDataPositionFunc<2> pos = grid->DataPosition();
    grid->ForEachDataPointIndex([&pos, &sphere, &grid](size_t i, size_t j) {
        Vector2D gx = pos(i, j);
        double answer = (sphere->center - gx).Length() - 0.15;
        double acttual = (*grid)(i, j);

        EXPECT_NEAR(answer, acttual, 1e-6);
    });
}

TEST(VolumeGridEmitter2, StepFunction)
{
    auto sphere = Sphere2::Builder()
                      .WithCenter({ 0.5, 0.75 })
                      .WithRadius(0.15)
                      .MakeShared();

    auto emitter =
        VolumeGridEmitter2::Builder().WithSourceRegion(sphere).MakeShared();

    auto grid = CellCenteredScalarGrid2::Builder()
                    .WithResolution({ 16, 16 })
                    .WithGridSpacing({ 1.0 / 16.0, 1.0 / 16.0 })
                    .WithOrigin({ 0, 0 })
                    .MakeShared();

    emitter->AddStepFunctionTarget(grid, 3.0, 7.0);

    emitter->Update(0.0, 0.01);

    GridDataPositionFunc<2> pos = grid->DataPosition();
    grid->ForEachDataPointIndex([&pos, &sphere, &grid](size_t i, size_t j) {
        Vector2D gx = pos(i, j);
        double answer = (sphere->center - gx).Length() - 0.15;
        answer = 4.0 * (1.0 - SmearedHeavisideSDF(answer * 16.0)) + 3.0;
        double acttual = (*grid)(i, j);

        EXPECT_NEAR(answer, acttual, 1e-6);
    });
}