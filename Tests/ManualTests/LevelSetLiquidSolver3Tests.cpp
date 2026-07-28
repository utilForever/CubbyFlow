#include "gtest/gtest.h"

#include <Core/Array/Array.hpp>
#include <Core/Geometry/ImplicitSurfaceSet.hpp>
#include <Core/Geometry/MarchingCubes.hpp>
#include <Core/Geometry/Plane.hpp>
#include <Core/Solver/LevelSet/LevelSetLiquidSolver3.hpp>
#include <Core/Utils/LevelSetUtils.hpp>

#include <ManualTests.hpp>

using namespace CubbyFlow;

namespace
{
void SaveTriangleMesh(const TriangleMesh3& mesh, const std::string& fileName)
{
    std::ofstream file(fileName.c_str());

    if (file)
    {
        mesh.WriteObj(&file);
        file.close();
    }
}

void TriangulateAndSave(const ScalarGrid3Ptr& sdf, const std::string& fileName)
{
    TriangleMesh3 mesh;
    int flag = DIRECTION_ALL & ~DIRECTION_DOWN;

    MarchingCubes(sdf->DataView(), sdf->GridSpacing(), sdf->DataOrigin(), &mesh,
                  0.0, flag);
    SaveTriangleMesh(mesh, fileName);
}
}  // namespace

CUBBYFLOW_TESTS(LevelSetLiquidSolver3);

CUBBYFLOW_BEGIN_TEST_F(LevelSetLiquidSolver3, SubtleSloshing)
{
    LevelSetLiquidSolver3 solver;

    auto data = solver.GetGridSystemData();
    double dx = 1.0 / 64.0;
    data->Resize({ 64, 64, 8 }, { dx, dx, dx }, Vector3D());

    // Source setting
    ImplicitSurfaceSet3 surfaceSet;
    surfaceSet.AddExplicitSurface(std::make_shared<Plane3>(
        Vector3D(0.02, 1, 0).Normalized(), Vector3D(0.0, 0.5, 0.0)));

    auto sdf = solver.GetSignedDistanceField();
    sdf->Fill([&surfaceSet](const Vector3D& x) {
        return surfaceSet.SignedDistance(x);
    });

    Array2<double> output(64, 64);
    auto sdfToBinary = [&output, &sdf, &dx](size_t i, size_t j) {
        output(i, j) = 1.0 - SmearedHeavisideSDF((*sdf)(i, j, 4) / dx);
    };
    ForEachIndex(output.Size(), sdfToBinary);

    char fileName[256];
    snprintf(fileName, sizeof(fileName), "data.#grid2,0000.npy");
    SaveData(output.View(), fileName);

    snprintf(fileName, sizeof(fileName), "data.#grid2,0000.obj");
    TriangulateAndSave(sdf, GetFullFilePath(fileName));

    for (Frame frame(0, 1.0 / 60.0); frame.index < 120; frame.Advance())
    {
        solver.Update(frame);

        ForEachIndex(output.Size(), sdfToBinary);
        snprintf(fileName, sizeof(fileName), "data.#grid2,%04d.npy",
                 frame.index);
        SaveData(output.View(), fileName);

        snprintf(fileName, sizeof(fileName), "data.#grid2,%04d.obj",
                 frame.index);
        TriangulateAndSave(sdf, GetFullFilePath(fileName));
    }
}
CUBBYFLOW_END_TEST_F