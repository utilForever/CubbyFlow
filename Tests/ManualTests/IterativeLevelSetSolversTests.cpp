#include "gtest/gtest.h"

#include <Core/Field/CustomVectorField.hpp>
#include <Core/Grid/CellCenteredScalarGrid.hpp>
#include <Core/Solver/Advection/CubicSemiLagrangian2.hpp>
#include <Core/Solver/LevelSet/ENOLevelSetSolver2.hpp>
#include <Core/Solver/LevelSet/ENOLevelSetSolver3.hpp>
#include <Core/Solver/LevelSet/UpwindLevelSetSolver2.hpp>
#include <Core/Solver/LevelSet/UpwindLevelSetSolver3.hpp>

#include <ManualTests.hpp>

using namespace CubbyFlow;

CUBBYFLOW_TESTS(LevelSetSolver2);

CUBBYFLOW_BEGIN_TEST_F(LevelSetSolver2, Reinitialize)
{
    Vector2UZ size(256, 256);
    Vector2D gridSpacing(1.0 / size.x, 1.0 / size.x);

    CellCenteredScalarGrid2 data0(size, gridSpacing);
    CellCenteredScalarGrid2 data1(size, gridSpacing);

    data0.Fill([](const Vector2D& pt) {
        return pt.DistanceTo(Vector2D(0.5, 0.75)) - 0.15;
    });

    auto flowFunc = [](const Vector2D& pt) {
        Vector2D ret;
        ret.x = 2.0 * Square(std::sin(PI_DOUBLE * pt.x)) *
                std::sin(PI_DOUBLE * pt.y) * std::cos(PI_DOUBLE * pt.y);
        ret.y = -2.0 * Square(std::sin(PI_DOUBLE * pt.y)) *
                std::sin(PI_DOUBLE * pt.x) * std::cos(PI_DOUBLE * pt.x);
        return ret;
    };

    CustomVectorField2 flow(flowFunc);

    CellCenteredScalarGrid2 dataU(Vector2UZ(20, 20),
                                  Vector2D(1 / 20.0, 1 / 20.0));
    CellCenteredScalarGrid2 dataV(Vector2UZ(20, 20),
                                  Vector2D(1 / 20.0, 1 / 20.0));

    dataU.Fill([&flowFunc](const Vector2D& pt) { return flowFunc(pt).x; });
    dataV.Fill([&flowFunc](const Vector2D& pt) { return flowFunc(pt).y; });

    SaveData(dataU.DataView(), "flow_#grid2,x.npy");
    SaveData(dataV.DataView(), "flow_#grid2,y.npy");

    CubicSemiLagrangian2 advSolver;
    ENOLevelSetSolver2 lsSolver;

    SaveData(data0.DataView(), "data0_#grid2,iso.npy");

    for (int i = 0; i < 50; ++i)
    {
        advSolver.Advect(data0, flow, 0.02, &data1);
        lsSolver.Reinitialize(data1, 5.0 * gridSpacing.x, &data0);
    }

    SaveData(data0.DataView(), "data_#grid2,iso.npy");
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(LevelSetSolver2, NoReinitialize)
{
    Vector2UZ size(256, 256);
    Vector2D gridSpacing(1.0 / size.x, 1.0 / size.x);

    CellCenteredScalarGrid2 data0(size, gridSpacing);
    CellCenteredScalarGrid2 data1(size, gridSpacing);

    data0.Fill([](const Vector2D& pt) {
        return pt.DistanceTo(Vector2D(0.5, 0.75)) - 0.15;
    });

    CustomVectorField2 flow([](const Vector2D& pt) {
        Vector2D ret;
        ret.x = 2.0 * Square(std::sin(PI_DOUBLE * pt.x)) *
                std::sin(PI_DOUBLE * pt.y) * std::cos(PI_DOUBLE * pt.y);
        ret.y = -2.0 * Square(std::sin(PI_DOUBLE * pt.y)) *
                std::sin(PI_DOUBLE * pt.x) * std::cos(PI_DOUBLE * pt.x);
        return ret;
    });

    CubicSemiLagrangian2 solver;

    for (int i = 0; i < 50; ++i)
    {
        solver.Advect(data0, flow, 0.02, &data1);
        data0.Swap(&data1);
    }

    SaveData(data0.DataView(), "data_#grid2,iso.npy");
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_TESTS(UpwindLevelSetSolver2);

CUBBYFLOW_BEGIN_TEST_F(UpwindLevelSetSolver2, ReinitializeSmall)
{
    CellCenteredScalarGrid2 sdf({ 40, 30 }), temp({ 40, 30 });
    UpwindLevelSetSolver2 solver;

    // Starting from constant field
    sdf.Fill([](const Vector2D& x) { return 1.0; });

    SaveData(sdf.DataView(), "constant0_#grid2,iso.npy");

    solver.Reinitialize(sdf, 40.0, &temp);

    SaveData(temp.DataView(), "constant1_#grid2,iso.npy");

    // Starting from SDF field
    sdf.Fill([](const Vector2D& x) {
        return (x - Vector2D(20, 20)).Length() - 8.0;
    });

    SaveData(sdf.DataView(), "sdf0_#grid2,iso.npy");

    solver.Reinitialize(sdf, 40.0, &temp);

    SaveData(temp.DataView(), "sdf1_#grid2,iso.npy");

    // Starting from scaled SDF field
    sdf.Fill([](const Vector2D& x) {
        double r = (x - Vector2D(20, 20)).Length() - 8.0;
        return 2.0 * r;
    });

    SaveData(sdf.DataView(), "scaled0_#grid2,iso.npy");

    solver.Reinitialize(sdf, 40.0, &temp);

    SaveData(temp.DataView(), "scaled1_#grid2,iso.npy");

    // Starting from scaled SDF field
    sdf.Fill([](const Vector2D& x) {
        double r = (x - Vector2D(20, 20)).Length() - 8.0;
        return (r < 0.0) ? -0.5 : 0.5;
    });

    SaveData(sdf.DataView(), "unit_step0_#grid2,iso.npy");

    solver.Reinitialize(sdf, 40.0, &temp);

    SaveData(temp.DataView(), "unit_step1_#grid2,iso.npy");
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(UpwindLevelSetSolver2, Reinitialize)
{
    CellCenteredScalarGrid2 sdf({ 160, 120 }), temp({ 160, 120 });
    UpwindLevelSetSolver2 solver;

    // Starting from constant field
    sdf.Fill([](const Vector2D& x) { return 1.0; });

    SaveData(sdf.DataView(), "constant0_#grid2,iso.npy");

    solver.Reinitialize(sdf, 160.0, &temp);

    SaveData(temp.DataView(), "constant1_#grid2,iso.npy");

    // Starting from SDF field
    sdf.Fill([](const Vector2D& x) {
        return (x - Vector2D(80, 80)).Length() - 32.0;
    });

    SaveData(sdf.DataView(), "sdf0_#grid2,iso.npy");

    solver.Reinitialize(sdf, 160.0, &temp);

    SaveData(temp.DataView(), "sdf1_#grid2,iso.npy");

    // Starting from scaled SDF field
    sdf.Fill([](const Vector2D& x) {
        double r = (x - Vector2D(80, 80)).Length() - 32.0;
        return 2.0 * r;
    });

    SaveData(sdf.DataView(), "scaled0_#grid2,iso.npy");

    solver.Reinitialize(sdf, 160.0, &temp);

    SaveData(temp.DataView(), "scaled1_#grid2,iso.npy");

    // Starting from scaled SDF field
    sdf.Fill([](const Vector2D& x) {
        double r = (x - Vector2D(80, 80)).Length() - 32.0;
        return (r < 0.0) ? -0.5 : 0.5;
    });

    SaveData(sdf.DataView(), "unit_step0_#grid2,iso.npy");

    solver.Reinitialize(sdf, 160.0, &temp);

    SaveData(temp.DataView(), "unit_step1_#grid2,iso.npy");
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(UpwindLevelSetSolver2, Extrapolate)
{
    Vector2UZ size(160, 120);
    Vector2D gridSpacing(1.0 / size.x, 1.0 / size.x);
    double maxDistance = 20.0 * gridSpacing.x;

    UpwindLevelSetSolver2 solver;
    CellCenteredScalarGrid2 sdf(size, gridSpacing);
    CellCenteredScalarGrid2 input(size, gridSpacing);
    CellCenteredScalarGrid2 output(size, gridSpacing);

    sdf.Fill([](const Vector2D& x) {
        return (x - Vector2D(0.75, 0.5)).Length() - 0.3;
    });

    input.Fill([](const Vector2D& x) {
        if ((x - Vector2D(0.75, 0.5)).Length() <= 0.3)
        {
            double p = 10.0 * PI_DOUBLE;
            return 0.5 * 0.25 * std::sin(p * x.x) * std::sin(p * x.y);
        }
        else
        {
            return 0.0;
        }
    });

    solver.Extrapolate(input, sdf, maxDistance, &output);

    SaveData(sdf.DataView(), "sdf_#grid2,iso.npy");
    SaveData(input.DataView(), "input_#grid2.npy");
    SaveData(output.DataView(), "output_#grid2.npy");
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_TESTS(UpwindLevelSetSolver3);

CUBBYFLOW_BEGIN_TEST_F(UpwindLevelSetSolver3, ReinitializeSmall)
{
    CellCenteredScalarGrid3 sdf({ 40, 30, 50 }), temp({ 40, 30, 50 });

    sdf.Fill([](const Vector3D& x) {
        return (x - Vector3D(20, 20, 20)).Length() - 8.0;
    });

    UpwindLevelSetSolver3 solver;
    solver.Reinitialize(sdf, 5.0, &temp);

    Array2<double> sdf2(40, 30);
    Array2<double> temp2(40, 30);

    for (size_t j = 0; j < 30; ++j)
    {
        for (size_t i = 0; i < 40; ++i)
        {
            sdf2(i, j) = sdf(i, j, 10);
            temp2(i, j) = temp(i, j, 10);
        }
    }

    SaveData(sdf2.View(), "sdf_#grid2,iso.npy");
    SaveData(temp2.View(), "temp_#grid2,iso.npy");
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(UpwindLevelSetSolver3, ExtrapolateSmall)
{
    CellCenteredScalarGrid3 sdf({ 40, 30, 50 }), temp({ 40, 30, 50 });
    CellCenteredScalarGrid3 field({ 40, 30, 50 });

    sdf.Fill([](const Vector3D& x) {
        return (x - Vector3D(20, 20, 20)).Length() - 8.0;
    });

    field.Fill([](const Vector3D& x) {
        if ((x - Vector3D(20, 20, 20)).Length() <= 8.0)
        {
            return 0.5 * 0.25 * std::sin(x.x) * std::sin(x.y) * std::sin(x.z);
        }
        else
        {
            return 0.0;
        }
    });

    UpwindLevelSetSolver3 solver;
    solver.Extrapolate(field, sdf, 5.0, &temp);

    Array2<double> field2(40, 30);
    Array2<double> temp2(40, 30);

    for (size_t j = 0; j < 30; ++j)
    {
        for (size_t i = 0; i < 40; ++i)
        {
            field2(i, j) = field(i, j, 12);
            temp2(i, j) = temp(i, j, 12);
        }
    }

    SaveData(field2.View(), "field_#grid2.npy");
    SaveData(temp2.View(), "temp_#grid2.npy");
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_TESTS(ENOLevelSetSolver2);

CUBBYFLOW_BEGIN_TEST_F(ENOLevelSetSolver2, ReinitializeSmall)
{
    CellCenteredScalarGrid2 sdf({ 40, 30 }), temp({ 40, 30 });
    ENOLevelSetSolver2 solver;

    // Starting from constant field
    sdf.Fill([](const Vector2D& x) { return 1.0; });

    SaveData(sdf.DataView(), "constant0_#grid2,iso.npy");

    solver.Reinitialize(sdf, 40.0, &temp);

    SaveData(temp.DataView(), "constant1_#grid2,iso.npy");

    // Starting from SDF field
    sdf.Fill([](const Vector2D& x) {
        return (x - Vector2D(20, 20)).Length() - 8.0;
    });

    SaveData(sdf.DataView(), "sdf0_#grid2,iso.npy");

    solver.Reinitialize(sdf, 40.0, &temp);

    SaveData(temp.DataView(), "sdf1_#grid2,iso.npy");

    // Starting from scaled SDF field
    sdf.Fill([](const Vector2D& x) {
        double r = (x - Vector2D(20, 20)).Length() - 8.0;
        return 2.0 * r;
    });

    SaveData(sdf.DataView(), "scaled0_#grid2,iso.npy");

    solver.Reinitialize(sdf, 40.0, &temp);

    SaveData(temp.DataView(), "scaled1_#grid2,iso.npy");

    // Starting from scaled SDF field
    sdf.Fill([](const Vector2D& x) {
        double r = (x - Vector2D(20, 20)).Length() - 8.0;
        return (r < 0.0) ? -0.5 : 0.5;
    });

    SaveData(sdf.DataView(), "unit_step0_#grid2,iso.npy");

    solver.Reinitialize(sdf, 40.0, &temp);

    SaveData(temp.DataView(), "unit_step1_#grid2,iso.npy");
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(ENOLevelSetSolver2, Reinitialize)
{
    CellCenteredScalarGrid2 sdf({ 160, 120 }), temp({ 160, 120 });
    ENOLevelSetSolver2 solver;

    // Starting from constant field
    sdf.Fill([](const Vector2D& x) { return 1.0; });

    SaveData(sdf.DataView(), "constant0_#grid2,iso.npy");

    solver.Reinitialize(sdf, 160.0, &temp);

    SaveData(temp.DataView(), "constant1_#grid2,iso.npy");

    // Starting from SDF field
    sdf.Fill([](const Vector2D& x) {
        return (x - Vector2D(80, 80)).Length() - 32.0;
    });

    SaveData(sdf.DataView(), "sdf0_#grid2,iso.npy");

    solver.Reinitialize(sdf, 160.0, &temp);

    SaveData(temp.DataView(), "sdf1_#grid2,iso.npy");

    // Starting from scaled SDF field
    sdf.Fill([](const Vector2D& x) {
        double r = (x - Vector2D(80, 80)).Length() - 32.0;
        return 2.0 * r;
    });

    SaveData(sdf.DataView(), "scaled0_#grid2,iso.npy");

    solver.Reinitialize(sdf, 160.0, &temp);

    SaveData(temp.DataView(), "scaled1_#grid2,iso.npy");

    // Starting from scaled SDF field
    sdf.Fill([](const Vector2D& x) {
        double r = (x - Vector2D(80, 80)).Length() - 32.0;
        return (r < 0.0) ? -0.5 : 0.5;
    });

    SaveData(sdf.DataView(), "unit_step0_#grid2,iso.npy");

    solver.Reinitialize(sdf, 160.0, &temp);

    SaveData(temp.DataView(), "unit_step1_#grid2,iso.npy");
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(ENOLevelSetSolver2, Extrapolate)
{
    Vector2UZ size(160, 120);
    Vector2D gridSpacing(1.0 / size.x, 1.0 / size.x);
    double maxDistance = 20.0 * gridSpacing.x;

    ENOLevelSetSolver2 solver;
    CellCenteredScalarGrid2 sdf(size, gridSpacing);
    CellCenteredScalarGrid2 input(size, gridSpacing);
    CellCenteredScalarGrid2 output(size, gridSpacing);

    sdf.Fill([](const Vector2D& x) {
        return (x - Vector2D(0.75, 0.5)).Length() - 0.3;
    });

    input.Fill([](const Vector2D& x) {
        if ((x - Vector2D(0.75, 0.5)).Length() <= 0.3)
        {
            double p = 10.0 * PI_DOUBLE;
            return 0.5 * 0.25 * std::sin(p * x.x) * std::sin(p * x.y);
        }
        else
        {
            return 0.0;
        }
    });

    solver.Extrapolate(input, sdf, maxDistance, &output);

    SaveData(sdf.DataView(), "sdf_#grid2,iso.npy");
    SaveData(input.DataView(), "input_#grid2.npy");
    SaveData(output.DataView(), "output_#grid2.npy");
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_TESTS(ENOLevelSetSolver3);

CUBBYFLOW_BEGIN_TEST_F(ENOLevelSetSolver3, ReinitializeSmall)
{
    CellCenteredScalarGrid3 sdf({ 40, 30, 50 }), temp({ 40, 30, 50 });

    sdf.Fill([](const Vector3D& x) {
        return (x - Vector3D(20, 20, 20)).Length() - 8.0;
    });

    ENOLevelSetSolver3 solver;
    solver.Reinitialize(sdf, 5.0, &temp);

    Array2<double> sdf2(40, 30);
    Array2<double> temp2(40, 30);
    for (size_t j = 0; j < 30; ++j)
    {
        for (size_t i = 0; i < 40; ++i)
        {
            sdf2(i, j) = sdf(i, j, 10);
            temp2(i, j) = temp(i, j, 10);
        }
    }

    SaveData(sdf2.View(), "sdf_#grid2,iso.npy");
    SaveData(temp2.View(), "temp_#grid2,iso.npy");
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(ENOLevelSetSolver3, ExtrapolateSmall)
{
    CellCenteredScalarGrid3 sdf({ 40, 30, 50 }), temp({ 40, 30, 50 });
    CellCenteredScalarGrid3 field({ 40, 30, 50 });

    sdf.Fill([](const Vector3D& x) {
        return (x - Vector3D(20, 20, 20)).Length() - 8.0;
    });

    field.Fill([](const Vector3D& x) {
        if ((x - Vector3D(20, 20, 20)).Length() <= 8.0)
        {
            return 0.5 * 0.25 * std::sin(x.x) * std::sin(x.y) * std::sin(x.z);
        }
        else
        {
            return 0.0;
        }
    });

    ENOLevelSetSolver3 solver;
    solver.Extrapolate(field, sdf, 5.0, &temp);

    Array2<double> field2(40, 30);
    Array2<double> temp2(40, 30);
    for (size_t j = 0; j < 30; ++j)
    {
        for (size_t i = 0; i < 40; ++i)
        {
            field2(i, j) = field(i, j, 12);
            temp2(i, j) = temp(i, j, 12);
        }
    }

    SaveData(field2.View(), "field_#grid2.npy");
    SaveData(temp2.View(), "temp_#grid2.npy");
}
CUBBYFLOW_END_TEST_F