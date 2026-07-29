#include "gtest/gtest.h"

#include <ManualTests.hpp>

#include <Core/Geometry/RigidBodyCollider.hpp>
#include <Core/Geometry/Sphere.hpp>
#include <Core/Geometry/SurfaceToImplicit.hpp>
#include <Core/Solver/Grid/GridFluidSolver2.hpp>
#include <Core/Solver/Grid/GridFractionalSinglePhasePressureSolver2.hpp>
#include <Core/Solver/Grid/GridSinglePhasePressureSolver2.hpp>

using namespace CubbyFlow;

CUBBYFLOW_TESTS(GridFluidSolver2);

CUBBYFLOW_BEGIN_TEST_F(GridFluidSolver2, ApplyBoundaryConditionWithPressure)
{
    GridFluidSolver2 solver;
    solver.SetGravity(Vector2D(0, 0));
    solver.SetAdvectionSolver(nullptr);
    solver.SetDiffusionSolver(nullptr);

    auto ppe = std::make_shared<GridSinglePhasePressureSolver2>();
    solver.SetPressureSolver(ppe);

    GridSystemData2Ptr data = solver.GetGridSystemData();
    double dx = 1.0 / 32.0;
    data->Resize(Vector2UZ(64, 32), Vector2D(dx, dx), Vector2D());
    data->Velocity()->Fill(Vector2D(1.0, 0.0));

    BoundingBox2D domain = data->GetBoundingBox();

    SurfaceToImplicit2Ptr sphere = std::make_shared<SurfaceToImplicit2>(
        std::make_shared<Sphere2>(domain.MidPoint(), 0.25));
    RigidBodyCollider2Ptr collider =
        std::make_shared<RigidBodyCollider2>(sphere);
    solver.SetCollider(collider);

    Frame frame(0, 1.0 / 60.0);
    solver.Update(frame);

    Array2<double> dataU(64, 32);
    Array2<double> dataV(64, 32);
    Array2<double> div(64, 32);
    Array2<double> pressure(64, 32);

    ForEachIndex(dataU.Size(), [&data, &dataU, &dataV, &div, &pressure, &ppe](
                                   size_t i, size_t j) {
        Vector2D vel = data->Velocity()->ValueAtCellCenter(i, j);
        dataU(i, j) = vel.x;
        dataV(i, j) = vel.y;
        div(i, j) = data->Velocity()->DivergenceAtCellCenter(i, j);
        pressure(i, j) = ppe->GetPressure()(i, j);
    });

    SaveData(dataU.View(), "data_#grid2,x.npy");
    SaveData(dataV.View(), "data_#grid2,y.npy");
    SaveData(div.View(), "div_#grid2.npy");
    SaveData(pressure.View(), "pressure_#grid2.npy");
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(GridFluidSolver2,
                       ApplyBoundaryConditionWithVariationalPressure)
{
    GridFluidSolver2 solver;
    solver.SetGravity(Vector2D(0, 0));
    solver.SetAdvectionSolver(nullptr);
    solver.SetDiffusionSolver(nullptr);

    auto ppe = std::make_shared<GridFractionalSinglePhasePressureSolver2>();
    solver.SetPressureSolver(ppe);

    GridSystemData2Ptr data = solver.GetGridSystemData();
    double dx = 1.0 / 32.0;
    data->Resize(Vector2UZ(64, 32), Vector2D(dx, dx), Vector2D());
    data->Velocity()->Fill(Vector2D(1.0, 0.0));

    BoundingBox2D domain = data->GetBoundingBox();

    SurfaceToImplicit2Ptr sphere = std::make_shared<SurfaceToImplicit2>(
        std::make_shared<Sphere2>(domain.MidPoint(), 0.25));
    RigidBodyCollider2Ptr collider =
        std::make_shared<RigidBodyCollider2>(sphere);
    solver.SetCollider(collider);

    Frame frame(0, 1.0 / 60.0);
    solver.Update(frame);

    Array2<double> dataU(64, 32);
    Array2<double> dataV(64, 32);
    Array2<double> div(64, 32);
    Array2<double> pressure(64, 32);

    ForEachIndex(dataU.Size(), [&data, &dataU, &dataV, &div, &pressure, &ppe](
                                   size_t i, size_t j) {
        Vector2D vel = data->Velocity()->ValueAtCellCenter(i, j);
        dataU(i, j) = vel.x;
        dataV(i, j) = vel.y;
        div(i, j) = data->Velocity()->DivergenceAtCellCenter(i, j);
        pressure(i, j) = ppe->GetPressure()(i, j);
    });

    SaveData(dataU.View(), "data_#grid2,x.npy");
    SaveData(dataV.View(), "data_#grid2,y.npy");
    SaveData(div.View(), "div_#grid2.npy");
    SaveData(pressure.View(), "pressure_#grid2.npy");
}
CUBBYFLOW_END_TEST_F

CUBBYFLOW_BEGIN_TEST_F(GridFluidSolver2, ApplyBoundaryConditionWithPressureOpen)
{
    GridFluidSolver2 solver;
    solver.SetGravity(Vector2D(0, 0));
    solver.SetAdvectionSolver(nullptr);
    solver.SetDiffusionSolver(nullptr);

    // Open left and right
    solver.SetClosedDomainBoundaryFlag(DIRECTION_DOWN | DIRECTION_UP);

    GridSinglePhasePressureSolver2Ptr ppe =
        std::make_shared<GridSinglePhasePressureSolver2>();
    solver.SetPressureSolver(ppe);

    GridSystemData2Ptr data = solver.GetGridSystemData();
    double dx = 1.0 / 32.0;
    data->Resize(Vector2UZ(64, 32), Vector2D(dx, dx), Vector2D());
    data->Velocity()->Fill(Vector2D(1.0, 0.0));

    BoundingBox2D domain = data->GetBoundingBox();

    SurfaceToImplicit2Ptr sphere = std::make_shared<SurfaceToImplicit2>(
        std::make_shared<Sphere2>(domain.MidPoint(), 0.25));
    RigidBodyCollider2Ptr collider =
        std::make_shared<RigidBodyCollider2>(sphere);
    solver.SetCollider(collider);

    Frame frame(0, 1.0 / 60.0);
    solver.Update(frame);

    Array2<double> dataU(64, 32);
    Array2<double> dataV(64, 32);
    Array2<double> div(64, 32);
    Array2<double> pressure(64, 32);

    ForEachIndex(dataU.Size(), [&data, &dataU, &dataV, &div, &pressure, &ppe](
                                   size_t i, size_t j) {
        Vector2D vel = data->Velocity()->ValueAtCellCenter(i, j);
        dataU(i, j) = vel.x;
        dataV(i, j) = vel.y;
        div(i, j) = data->Velocity()->DivergenceAtCellCenter(i, j);
        pressure(i, j) = ppe->GetPressure()(i, j);
    });

    SaveData(dataU.View(), "data_#grid2,x.npy");
    SaveData(dataV.View(), "data_#grid2,y.npy");
    SaveData(div.View(), "div_#grid2.npy");
    SaveData(pressure.View(), "pressure_#grid2.npy");
}
CUBBYFLOW_END_TEST_F