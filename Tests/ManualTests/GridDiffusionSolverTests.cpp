#include "gtest/gtest.h"

#include <ManualTests.hpp>

#include <Core/Array/Array.hpp>
#include <Core/Field/CustomScalarField.hpp>
#include <Core/Grid/CellCenteredScalarGrid.hpp>
#include <Core/Solver/Grid/GridBackwardEulerDiffusionSolver2.hpp>
#include <Core/Solver/Grid/GridBackwardEulerDiffusionSolver3.hpp>
#include <Core/Solver/Grid/GridForwardEulerDiffusionSolver2.hpp>
#include <Core/Solver/Grid/GridForwardEulerDiffusionSolver3.hpp>

using namespace CubbyFlow;

CUBBYFLOW_TESTS(GridForwardEulerDiffusionSolver3);

CUBBYFLOW_BEGIN_TEST_F(GridForwardEulerDiffusionSolver3, Solve)
{
    Vector3UZ size(160, 120, 150);
    Vector3D gridSpacing(1.0 / size.x, 1.0 / size.x, 1.0 / size.x);

    CellCenteredScalarGrid3 src(size, gridSpacing);
    CellCenteredScalarGrid3 dst(size, gridSpacing);
    Array2<double> data(160, 120);

    src.Fill([&src](const Vector3D& x) {
        return (x.DistanceTo(src.GetBoundingBox().MidPoint()) < 0.2) ? 1.0
                                                                     : 0.0;
    });

    ForEachIndex(data.Size(), [&data, &src](size_t i, size_t j) {
        data(i, j) = src(i, j, 75);
    });

    double timeStep = 0.01;
    double diffusionCoeff = Square(gridSpacing.x) / timeStep / 12.0;

    GridForwardEulerDiffusionSolver3 diffusionSolver;

    diffusionSolver.Solve(src, diffusionCoeff, timeStep, &dst);
    dst.Swap(&src);

    SaveData(data.View(), "src_#grid2.npy");

    ForEachIndex(data.Size(), [&data, &src](size_t i, size_t j) {
        data(i, j) = src(i, j, 75);
    });

    SaveData(data.View(), "dst_#grid2.npy");
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(GridForwardEulerDiffusionSolver3, Unstable)
{
    Vector3UZ size(160, 120, 150);
    Vector3D gridSpacing(1.0 / size.x, 1.0 / size.x, 1.0 / size.x);

    CellCenteredScalarGrid3 src(size, gridSpacing);
    CellCenteredScalarGrid3 dst(size, gridSpacing);
    Array2<double> data(160, 120);

    src.Fill([&src](const Vector3D& x) {
        return (x.DistanceTo(src.GetBoundingBox().MidPoint()) < 0.2) ? 1.0
                                                                     : 0.0;
    });

    ForEachIndex(data.Size(), [&data, &src](size_t i, size_t j) {
        data(i, j) = src(i, j, 75);
    });

    double timeStep = 0.01;
    double diffusionCoeff = Square(gridSpacing.x) / timeStep / 12.0;

    GridForwardEulerDiffusionSolver3 diffusionSolver;

    diffusionSolver.Solve(src, 10.0 * diffusionCoeff, timeStep, &dst);
    dst.Swap(&src);

    SaveData(data.View(), "src_#grid2.npy");

    ForEachIndex(data.Size(), [&data, &src](size_t i, size_t j) {
        data(i, j) = src(i, j, 75);
    });

    SaveData(data.View(), "dst_#grid2.npy");
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_TESTS(GridBackwardEulerDiffusionSolver2);

CUBBYFLOW_BEGIN_TEST_F(GridBackwardEulerDiffusionSolver2, Solve)
{
    Vector2UZ size(160, 120);
    Vector2D gridSpacing(1.0 / size.x, 1.0 / size.x);

    CellCenteredScalarGrid2 src(size, gridSpacing);
    CellCenteredScalarGrid2 dst(size, gridSpacing);
    Array2<double> data(160, 120);

    src.Fill([&src](const Vector2D& x) {
        return (x.DistanceTo(src.GetBoundingBox().MidPoint()) < 0.2) ? 1.0
                                                                     : 0.0;
    });

    ForEachIndex(data.Size(),
                 [&data, &src](size_t i, size_t j) { data(i, j) = src(i, j); });

    double timeStep = 0.01;
    double diffusionCoeff = Square(gridSpacing.x) / timeStep / 12.0;

    GridBackwardEulerDiffusionSolver2 diffusionSolver;

    diffusionSolver.Solve(
        src, 100.0 * diffusionCoeff, timeStep, &dst,
        ConstantScalarField2(std::numeric_limits<double>::max()),
        CustomScalarField2([&src](const Vector2D& pt) {
            Vector2D md = src.GetBoundingBox().MidPoint();
            return pt.x - md.x;
        }));
    dst.Swap(&src);

    SaveData(data.View(), "src_#grid2.npy");

    ForEachIndex(data.Size(),
                 [&data, &src](size_t i, size_t j) { data(i, j) = src(i, j); });

    SaveData(data.View(), "dst_#grid2.npy");
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_TESTS(GridBackwardEulerDiffusionSolver3);

CUBBYFLOW_BEGIN_TEST_F(GridBackwardEulerDiffusionSolver3, Solve)
{
    Vector3UZ size(160, 120, 150);
    Vector3D gridSpacing(1.0 / size.x, 1.0 / size.x, 1.0 / size.x);

    CellCenteredScalarGrid3 src(size, gridSpacing);
    CellCenteredScalarGrid3 dst(size, gridSpacing);
    Array2<double> data(160, 120);

    src.Fill([&src](const Vector3D& x) {
        return (x.DistanceTo(src.GetBoundingBox().MidPoint()) < 0.2) ? 1.0
                                                                     : 0.0;
    });

    ForEachIndex(data.Size(), [&data, &src](size_t i, size_t j) {
        data(i, j) = src(i, j, 75);
    });

    double timeStep = 0.01;
    double diffusionCoeff = Square(gridSpacing.x) / timeStep / 12.0;

    GridBackwardEulerDiffusionSolver3 diffusionSolver;

    diffusionSolver.Solve(src, diffusionCoeff, timeStep, &dst);
    dst.Swap(&src);

    SaveData(data.View(), "src_#grid2.npy");

    ForEachIndex(data.Size(), [&data, &src](size_t i, size_t j) {
        data(i, j) = src(i, j, 75);
    });

    SaveData(data.View(), "dst_#grid2.npy");
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(GridBackwardEulerDiffusionSolver3, Stable)
{
    Vector3UZ size(160, 120, 150);
    Vector3D gridSpacing(1.0 / size.x, 1.0 / size.x, 1.0 / size.x);

    CellCenteredScalarGrid3 src(size, gridSpacing);
    CellCenteredScalarGrid3 dst(size, gridSpacing);
    Array2<double> data(160, 120);

    src.Fill([&src](const Vector3D& x) {
        return (x.DistanceTo(src.GetBoundingBox().MidPoint()) < 0.2) ? 1.0
                                                                     : 0.0;
    });

    ForEachIndex(data.Size(), [&data, &src](size_t i, size_t j) {
        data(i, j) = src(i, j, 75);
    });

    double timeStep = 0.01;
    double diffusionCoeff = Square(gridSpacing.x) / timeStep / 12.0;

    GridBackwardEulerDiffusionSolver3 diffusionSolver;

    diffusionSolver.Solve(src, 10.0 * diffusionCoeff, timeStep, &dst);
    dst.Swap(&src);

    SaveData(data.View(), "src_#grid2.npy");

    ForEachIndex(data.Size(), [&data, &src](size_t i, size_t j) {
        data(i, j) = src(i, j, 75);
    });

    SaveData(data.View(), "dst_#grid2.npy");
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(GridBackwardEulerDiffusionSolver3,
                       SolveWithBoundaryDirichlet)
{
    Vector3UZ size(80, 60, 75);
    Vector3D gridSpacing(1.0 / size.x, 1.0 / size.x, 1.0 / size.x);

    CellCenteredScalarGrid3 src(size, gridSpacing);
    CellCenteredScalarGrid3 dst(size, gridSpacing);

    Vector3D boundaryCenter = src.GetBoundingBox().MidPoint();
    CustomScalarField3 boundarySDF([&boundaryCenter](const Vector3D& x) {
        return boundaryCenter.x - x.x;
    });

    Array2<double> data(size.x, size.y);

    src.Fill([&src](const Vector3D& x) {
        return (x.DistanceTo(src.GetBoundingBox().MidPoint()) < 0.2) ? 1.0
                                                                     : 0.0;
    });

    ForEachIndex(data.Size(), [&data, &src, &size](size_t i, size_t j) {
        data(i, j) = src(i, j, size.z / 2);
    });

    double timeStep = 0.01;
    double diffusionCoeff = 100 * Square(gridSpacing.x) / timeStep / 12.0;

    GridBackwardEulerDiffusionSolver3 diffusionSolver(
        GridBackwardEulerDiffusionSolver3::BoundaryType::Dirichlet);

    diffusionSolver.Solve(src, diffusionCoeff, timeStep, &dst, boundarySDF);
    dst.Swap(&src);

    SaveData(data.View(), "src_#grid2.npy");

    ForEachIndex(data.Size(), [&data, &src, &size](size_t i, size_t j) {
        data(i, j) = src(i, j, size.z / 2);
    });

    SaveData(data.View(), "dst_#grid2.npy");
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(GridBackwardEulerDiffusionSolver3,
                       SolveWithBoundaryNeumann)
{
    Vector3UZ size(80, 60, 75);
    Vector3D gridSpacing(1.0 / size.x, 1.0 / size.x, 1.0 / size.x);

    CellCenteredScalarGrid3 src(size, gridSpacing);
    CellCenteredScalarGrid3 dst(size, gridSpacing);

    Vector3D boundaryCenter = src.GetBoundingBox().MidPoint();
    CustomScalarField3 boundarySDF([&boundaryCenter](const Vector3D& x) {
        return boundaryCenter.x - x.x;
    });

    Array2<double> data(size.x, size.y);

    src.Fill([&src](const Vector3D& x) {
        return (x.DistanceTo(src.GetBoundingBox().MidPoint()) < 0.2) ? 1.0
                                                                     : 0.0;
    });

    ForEachIndex(data.Size(), [&data, &src, &size](size_t i, size_t j) {
        data(i, j) = src(i, j, size.z / 2);
    });

    double timeStep = 0.01;
    double diffusionCoeff = 100 * Square(gridSpacing.x) / timeStep / 12.0;

    GridBackwardEulerDiffusionSolver3 diffusionSolver(
        GridBackwardEulerDiffusionSolver3::BoundaryType::Neumann);

    diffusionSolver.Solve(src, diffusionCoeff, timeStep, &dst, boundarySDF);
    dst.Swap(&src);

    SaveData(data.View(), "src_#grid2.npy");

    ForEachIndex(data.Size(), [&data, &src, &size](size_t i, size_t j) {
        data(i, j) = src(i, j, size.z / 2);
    });

    SaveData(data.View(), "dst_#grid2.npy");
}
CUBBYFLOW_END_TEST_F