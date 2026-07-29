#include "gtest/gtest.h"

#include <ManualTests.hpp>

#include <Core/Geometry/BoundingBox.hpp>
#include <Core/Grid/CellCenteredScalarGrid.hpp>
#include <Core/Particle/SPHSystemData.hpp>
#include <Core/PointGenerator/BccLatticePointGenerator.hpp>
#include <Core/PointGenerator/TrianglePointGenerator.hpp>
#include <Core/Utils/Parallel.hpp>

#include <random>

using namespace CubbyFlow;

CUBBYFLOW_TESTS(SPHSystemData2);

CUBBYFLOW_BEGIN_TEST_F(SPHSystemData2, Interpolate)
{
    Array1<Vector2D> points;
    TrianglePointGenerator pointsGenerator;
    BoundingBox2D bbox(Vector2D(0, 0), Vector2D(1, 1));
    double spacing = 0.1;

    pointsGenerator.Generate(bbox, spacing, &points);

    SPHSystemData2 sphSystem;
    sphSystem.AddParticles(
        ConstArrayView1<Vector2D>(points.data(), points.Size()));
    sphSystem.SetTargetSpacing(spacing);
    sphSystem.BuildNeighborSearcher();
    sphSystem.BuildNeighborLists();
    sphSystem.UpdateDensities();

    Array1<double> data(points.Size(), 1.0);

    CellCenteredScalarGrid2 grid({ 512, 512 }, { 1.0 / 512, 1.0 / 512 });

    GridDataPositionFunc<2> gridPos = grid.DataPosition();
    ParallelFor(ZERO_SIZE, grid.DataSize().x, ZERO_SIZE, grid.DataSize().y,
                [&gridPos, &grid, &sphSystem, &data](size_t i, size_t j) {
                    Vector2D xy = gridPos(i, j);
                    Vector2D p(xy.x, xy.y);
                    grid(i, j) = sphSystem.Interpolate(p, data);
                });

    SaveData(grid.DataView(), "data_#grid2.npy");
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(SPHSystemData2, Gradient)
{
    Array1<Vector2D> points;
    TrianglePointGenerator pointsGenerator;
    BoundingBox2D bbox(Vector2D(0, 0), Vector2D(1, 1));
    double spacing = 0.1;

    pointsGenerator.Generate(bbox, spacing, &points);

    SPHSystemData2 sphSystem;
    sphSystem.AddParticles(
        ConstArrayView1<Vector2D>(points.data(), points.Size()));
    sphSystem.SetTargetSpacing(spacing);
    sphSystem.BuildNeighborSearcher();
    sphSystem.BuildNeighborLists();
    sphSystem.UpdateDensities();

    Array1<double> data(points.Size()), gradX(points.Size()),
        gradY(points.Size());
    std::mt19937 rng(0);
    std::uniform_real_distribution<> d(0.0, 1.0);

    for (size_t i = 0; i < data.Length(); ++i)
    {
        data[i] = d(rng);
    }

    for (size_t i = 0; i < data.Length(); ++i)
    {
        Vector2D g = sphSystem.GradientAt(i, data);
        gradX[i] = g.x;
        gradY[i] = g.y;
    }

    CellCenteredScalarGrid2 grid({ 64, 64 }, { 1.0 / 64, 1.0 / 64 });
    CellCenteredScalarGrid2 grid2({ 64, 64 }, { 1.0 / 64, 1.0 / 64 });

    GridDataPositionFunc<2> gridPos = grid.DataPosition();
    ParallelFor(ZERO_SIZE, grid.DataSize().x, ZERO_SIZE, grid.DataSize().y,
                [&gridPos, &grid, &sphSystem, &data](size_t i, size_t j) {
                    Vector2D xy = gridPos(i, j);
                    Vector2D p(xy.x, xy.y);
                    grid(i, j) = sphSystem.Interpolate(p, data);
                });

    SaveData(grid.DataView(), "data_#grid2.npy");

    ParallelFor(ZERO_SIZE, grid.DataSize().x, ZERO_SIZE, grid.DataSize().y,
                [&gridPos, &grid, &sphSystem, &gradX, &grid2, &gradY](
                    size_t i, size_t j) {
                    Vector2D xy = gridPos(i, j);
                    Vector2D p(xy.x, xy.y);
                    grid(i, j) = sphSystem.Interpolate(p, gradX);
                    grid2(i, j) = sphSystem.Interpolate(p, gradY);
                });

    SaveData(grid.DataView(), "gradient_#grid2,x.npy");
    SaveData(grid2.DataView(), "gradient_#grid2,y.npy");
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(SPHSystemData2, Laplacian)
{
    Array1<Vector2D> points;
    TrianglePointGenerator pointsGenerator;
    BoundingBox2D bbox(Vector2D(0, 0), Vector2D(1, 1));
    double spacing = 0.1;

    pointsGenerator.Generate(bbox, spacing, &points);

    SPHSystemData2 sphSystem;
    sphSystem.AddParticles(
        ConstArrayView1<Vector2D>(points.data(), points.Size()));
    sphSystem.SetTargetSpacing(spacing);
    sphSystem.BuildNeighborSearcher();
    sphSystem.BuildNeighborLists();
    sphSystem.UpdateDensities();

    Array1<double> data(points.Size()), laplacian(points.Size());
    std::mt19937 rng(0);
    std::uniform_real_distribution<> d(0.0, 1.0);

    for (size_t i = 0; i < data.Length(); ++i)
    {
        data[i] = d(rng);
    }

    for (size_t i = 0; i < data.Length(); ++i)
    {
        laplacian[i] = sphSystem.LaplacianAt(i, data);
    }

    CellCenteredScalarGrid2 grid({ 512, 512 }, { 1.0 / 512, 1.0 / 512 });

    GridDataPositionFunc<2> gridPos = grid.DataPosition();
    ParallelFor(ZERO_SIZE, grid.DataSize().x, ZERO_SIZE, grid.DataSize().y,
                [&gridPos, &grid, &sphSystem, &data](size_t i, size_t j) {
                    Vector2D xy = gridPos(i, j);
                    Vector2D p(xy.x, xy.y);
                    grid(i, j) = sphSystem.Interpolate(p, data);
                });

    SaveData(grid.DataView(), "data_#grid2.npy");

    ParallelFor(ZERO_SIZE, grid.DataSize().x, ZERO_SIZE, grid.DataSize().y,
                [&gridPos, &grid, &sphSystem, &laplacian](size_t i, size_t j) {
                    Vector2D xy = gridPos(i, j);
                    Vector2D p(xy.x, xy.y);
                    grid(i, j) = sphSystem.Interpolate(p, laplacian);
                });

    SaveData(grid.DataView(), "laplacian_#grid2.npy");
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_TESTS(SPHSystemData3);

CUBBYFLOW_BEGIN_TEST_F(SPHSystemData3, Interpolate)
{
    Array1<Vector3D> points;
    BccLatticePointGenerator pointsGenerator;
    BoundingBox3D bbox(Vector3D(0, 0, 0), Vector3D(1, 1, 1));
    double spacing = 0.1;

    pointsGenerator.Generate(bbox, spacing, &points);

    SPHSystemData3 sphSystem;
    sphSystem.AddParticles(
        ConstArrayView1<Vector3D>(points.data(), points.Size()));
    sphSystem.SetTargetSpacing(spacing);
    sphSystem.BuildNeighborSearcher();
    sphSystem.BuildNeighborLists();
    sphSystem.UpdateDensities();

    Array1<double> data(points.Size(), 1.0);

    CellCenteredScalarGrid2 grid({ 512, 512 }, { 1.0 / 512, 1.0 / 512 });

    GridDataPositionFunc<2> gridPos = grid.DataPosition();
    ParallelFor(ZERO_SIZE, grid.DataSize().x, ZERO_SIZE, grid.DataSize().y,
                [&gridPos, &grid, &sphSystem, &data](size_t i, size_t j) {
                    Vector2D xy = gridPos(i, j);
                    Vector3D p(xy.x, xy.y, 0.5);
                    grid(i, j) = sphSystem.Interpolate(p, data);
                });

    SaveData(grid.DataView(), "data_#grid2.npy");
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(SPHSystemData3, Gradient)
{
    Array1<Vector3D> points;
    BccLatticePointGenerator pointsGenerator;
    BoundingBox3D bbox(Vector3D(0, 0, 0), Vector3D(1, 1, 1));
    double spacing = 0.1;

    pointsGenerator.Generate(bbox, spacing, &points);

    SPHSystemData3 sphSystem;
    sphSystem.AddParticles(
        ConstArrayView1<Vector3D>(points.data(), points.Size()));
    sphSystem.SetTargetSpacing(spacing);
    sphSystem.BuildNeighborSearcher();
    sphSystem.BuildNeighborLists();
    sphSystem.UpdateDensities();

    Array1<double> data(points.Size());
    Array1<double> gradX(points.Size()), gradY(points.Size());
    std::mt19937 rng(0);
    std::uniform_real_distribution<> d(0.0, 1.0);

    for (size_t i = 0; i < data.Length(); ++i)
    {
        data[i] = d(rng);
    }

    for (size_t i = 0; i < data.Length(); ++i)
    {
        Vector3D g = sphSystem.GradientAt(i, data);
        gradX[i] = g.x;
        gradY[i] = g.y;
    }

    CellCenteredScalarGrid2 grid({ 64, 64 }, { 1.0 / 64, 1.0 / 64 });
    CellCenteredScalarGrid2 grid2({ 64, 64 }, { 1.0 / 64, 1.0 / 64 });

    GridDataPositionFunc<2> gridPos = grid.DataPosition();
    ParallelFor(ZERO_SIZE, grid.DataSize().x, ZERO_SIZE, grid.DataSize().y,
                [&gridPos, &grid, &sphSystem, &data](size_t i, size_t j) {
                    Vector2D xy = gridPos(i, j);
                    Vector3D p(xy.x, xy.y, 0.5);
                    grid(i, j) = sphSystem.Interpolate(p, data);
                });

    SaveData(grid.DataView(), "data_#grid2.npy");

    ParallelFor(ZERO_SIZE, grid.DataSize().x, ZERO_SIZE, grid.DataSize().y,
                [&gridPos, &grid, &sphSystem, &gradX, &grid2, &gradY](
                    size_t i, size_t j) {
                    Vector2D xy = gridPos(i, j);
                    Vector3D p(xy.x, xy.y, 0.5);
                    grid(i, j) = sphSystem.Interpolate(p, gradX);
                    grid2(i, j) = sphSystem.Interpolate(p, gradY);
                });

    SaveData(grid.DataView(), "gradient_#grid2,x.npy");
    SaveData(grid2.DataView(), "gradient_#grid2,y.npy");
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(SPHSystemData3, Laplacian)
{
    Array1<Vector3D> points;
    BccLatticePointGenerator pointsGenerator;
    BoundingBox3D bbox(Vector3D(0, 0, 0), Vector3D(1, 1, 1));
    double spacing = 0.1;

    pointsGenerator.Generate(bbox, spacing, &points);

    SPHSystemData3 sphSystem;
    sphSystem.AddParticles(
        ConstArrayView1<Vector3D>(points.data(), points.Size()));
    sphSystem.SetTargetSpacing(spacing);
    sphSystem.BuildNeighborSearcher();
    sphSystem.BuildNeighborLists();
    sphSystem.UpdateDensities();

    Array1<double> data(points.Size()), laplacian(points.Size());
    std::mt19937 rng(0);
    std::uniform_real_distribution<> d(0.0, 1.0);

    for (size_t i = 0; i < data.Length(); ++i)
    {
        data[i] = d(rng);
    }

    for (size_t i = 0; i < data.Length(); ++i)
    {
        laplacian[i] = sphSystem.LaplacianAt(i, data);
    }

    CellCenteredScalarGrid2 grid({ 512, 512 }, { 1.0 / 512, 1.0 / 512 });

    GridDataPositionFunc<2> gridPos = grid.DataPosition();
    ParallelFor(ZERO_SIZE, grid.DataSize().x, ZERO_SIZE, grid.DataSize().y,
                [&gridPos, &grid, &sphSystem, &data](size_t i, size_t j) {
                    Vector2D xy = gridPos(i, j);
                    Vector3D p(xy.x, xy.y, 0.5);
                    grid(i, j) = sphSystem.Interpolate(p, data);
                });

    SaveData(grid.DataView(), "data_#grid2.npy");

    ParallelFor(ZERO_SIZE, grid.DataSize().x, ZERO_SIZE, grid.DataSize().y,
                [&gridPos, &grid, &sphSystem, &laplacian](size_t i, size_t j) {
                    Vector2D xy = gridPos(i, j);
                    Vector3D p(xy.x, xy.y, 0.5);
                    grid(i, j) = sphSystem.Interpolate(p, laplacian);
                });

    SaveData(grid.DataView(), "laplacian_#grid2.npy");
}
CUBBYFLOW_END_TEST_F