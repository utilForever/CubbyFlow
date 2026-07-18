// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <../ClaraUtils.hpp>

#include <Core/Array/Array.hpp>
#include <Core/Emitter/VolumeGridEmitter3.hpp>
#include <Core/Geometry/Box.hpp>
#include <Core/Geometry/Cylinder3.hpp>
#include <Core/Geometry/ImplicitSurfaceSet.hpp>
#include <Core/Geometry/ImplicitTriangleMesh3.hpp>
#include <Core/Geometry/MarchingCubes.hpp>
#include <Core/Geometry/Plane.hpp>
#include <Core/Geometry/RigidBodyCollider.hpp>
#include <Core/Geometry/Sphere.hpp>
#include <Core/Geometry/TriangleMesh3.hpp>
#include <Core/Grid/ScalarGrid.hpp>
#include <Core/Solver/LevelSet/LevelSetLiquidSolver3.hpp>
#include <Core/Utils/Logging.hpp>

#include <pystring/pystring.h>
#include <clara.hpp>

#ifdef CUBBYFLOW_WINDOWS
#include <direct.h>
#else
#include <sys/stat.h>
#endif

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#define APP_NAME "LevelSetLiquidSim"

using namespace CubbyFlow;

void SaveTriangleMesh(const TriangleMesh3& mesh, const std::string& rootDir,
                      int frameCnt)
{
    char baseName[256];
    snprintf(baseName, sizeof(baseName), "frame_%06d.obj", frameCnt);
    std::string fileName = pystring::os::path::join(rootDir, baseName);
    std::ofstream file(fileName.c_str());
    if (file)
    {
        printf("Writing %s...\n", fileName.c_str());
        mesh.WriteObj(&file);
        file.close();
    }
}

void TriangulateAndSave(const ScalarGrid3Ptr& sdf, const std::string& rootDir,
                        int frameCnt)
{
    TriangleMesh3 mesh;
    const int flag = DIRECTION_ALL & ~DIRECTION_DOWN;
    MarchingCubes(sdf->DataView(), sdf->GridSpacing(), sdf->DataOrigin(),
                  &mesh, 0.0, flag);
    SaveTriangleMesh(mesh, rootDir, frameCnt);
}

void PrintInfo(const LevelSetLiquidSolver3Ptr& solver)
{
    const auto grids = solver->GetGridSystemData();
    const Vector3UZ resolution = grids->Resolution();
    const BoundingBox3D domain = grids->GetBoundingBox();
    const Vector3D gridSpacing = grids->GridSpacing();

    printf("Resolution: %zu x %zu x %zu\n", resolution.x, resolution.y,
           resolution.z);
    printf("Domain: [%f, %f, %f] x [%f, %f, %f]\n", domain.lowerCorner.x,
           domain.lowerCorner.y, domain.lowerCorner.z, domain.upperCorner.x,
           domain.upperCorner.y, domain.upperCorner.z);
    printf("Grid spacing: [%f, %f, %f]\n", gridSpacing.x, gridSpacing.y,
           gridSpacing.z);
}

void RunSimulation(const std::string& rootDir,
                   const LevelSetLiquidSolver3Ptr& solver, int numberOfFrames,
                   double fps)
{
    const auto sdf = solver->GetSignedDistanceField();

    for (Frame frame(0, 1.0 / fps); frame.index < numberOfFrames; ++frame)
    {
        solver->Update(frame);

        TriangulateAndSave(sdf, rootDir, frame.index);
    }
}

// Water-drop example
void RunExample1(const std::string& rootDir, size_t resX, int numberOfFrames,
                 double fps)
{
    // Build solver
    auto solver = LevelSetLiquidSolver3::Builder()
                      .WithResolution({ resX, 2 * resX, resX })
                      .WithDomainSizeX(1.0)
                      .MakeShared();

    const auto grids = solver->GetGridSystemData();
    BoundingBox3D domain = grids->GetBoundingBox();

    // Build emitter
    const auto plane = Plane3::Builder()
                           .WithNormal({ 0, 1, 0 })
                           .WithPoint({ 0, 0.25 * domain.Height(), 0 })
                           .MakeShared();

    const auto sphere = Sphere3::Builder()
                            .WithCenter(domain.MidPoint())
                            .WithRadius(0.15 * domain.Width())
                            .MakeShared();

    const auto surfaceSet =
        ImplicitSurfaceSet3::Builder()
            .WithExplicitSurfaces(Array1<Surface3Ptr>{ plane, sphere })
            .MakeShared();

    auto emitter =
        VolumeGridEmitter3::Builder().WithSourceRegion(surfaceSet).MakeShared();

    solver->SetEmitter(emitter);
    emitter->AddSignedDistanceTarget(solver->GetSignedDistanceField());

    // Print simulation info
    printf("Running example 1 (water-drop)\n");
    PrintInfo(solver);

    // Run simulation
    RunSimulation(rootDir, solver, numberOfFrames, fps);
}

// Dam-breaking example
void RunExample2(const std::string& rootDir, size_t resX, int numberOfFrames,
                 double fps)
{
    // Build solver
    auto solver = LevelSetLiquidSolver3::Builder()
                      .WithResolution({ 3 * resX, 2 * resX, (3 * resX) / 2 })
                      .WithDomainSizeX(3.0)
                      .MakeShared();
    solver->SetUseCompressedLinearSystem(true);

    const auto grids = solver->GetGridSystemData();
    BoundingBox3D domain = grids->GetBoundingBox();
    const double lz = domain.Depth();

    // Build emitter
    const auto box1 = Box3::Builder()
                          .WithLowerCorner({ -0.5, -0.5, -0.5 * lz })
                          .WithUpperCorner({ 0.5, 0.75, 0.75 * lz })
                          .MakeShared();

    const auto box2 = Box3::Builder()
                          .WithLowerCorner({ 2.5, -0.5, 0.25 * lz })
                          .WithUpperCorner({ 3.5, 0.75, 1.5 * lz })
                          .MakeShared();

    const auto boxSet =
        ImplicitSurfaceSet3::Builder()
            .WithExplicitSurfaces(Array1<Surface3Ptr>{ box1, box2 })
            .MakeShared();

    auto emitter =
        VolumeGridEmitter3::Builder().WithSourceRegion(boxSet).MakeShared();

    solver->SetEmitter(emitter);
    emitter->AddSignedDistanceTarget(solver->GetSignedDistanceField());

    // Build collider
    const auto cyl1 = Cylinder3::Builder()
                          .WithCenter({ 1, 0.375, 0.375 })
                          .WithRadius(0.1)
                          .WithHeight(0.75)
                          .MakeShared();

    const auto cyl2 = Cylinder3::Builder()
                          .WithCenter({ 1.5, 0.375, 0.75 })
                          .WithRadius(0.1)
                          .WithHeight(0.75)
                          .MakeShared();

    const auto cyl3 = Cylinder3::Builder()
                          .WithCenter({ 2, 0.375, 1.125 })
                          .WithRadius(0.1)
                          .WithHeight(0.75)
                          .MakeShared();

    const auto cylSet =
        ImplicitSurfaceSet3::Builder()
            .WithExplicitSurfaces(Array1<Surface3Ptr>{ cyl1, cyl2, cyl3 })
            .MakeShared();

    const auto collider =
        RigidBodyCollider3::Builder().WithSurface(cylSet).MakeShared();

    solver->SetCollider(collider);

    // Print simulation info
    printf("Running example 2 (dam-breaking)\n");
    PrintInfo(solver);

    // Run simulation
    RunSimulation(rootDir, solver, numberOfFrames, fps);
}

// High-viscosity example (bunny-drop)
void RunExample3(const std::string& rootDir, size_t resX, int numberOfFrames,
                 double fps)
{
    // Build solver
    auto solver = LevelSetLiquidSolver3::Builder()
                      .WithResolution({ resX, resX, resX })
                      .WithDomainSizeX(1.0)
                      .MakeShared();
    solver->SetUseCompressedLinearSystem(true);

    solver->SetViscosityCoefficient(1.0);
    solver->SetIsGlobalCompensationEnabled(true);

    auto grids = solver->GetGridSystemData();

    // Build emitters
    auto bunnyMesh = TriangleMesh3::Builder().MakeShared();
    std::ifstream objFile(RESOURCES_DIR "/bunny.obj");
    if (objFile)
    {
        [[maybe_unused]] bool isLoaded = bunnyMesh->ReadObj(&objFile);
    }
    else
    {
        fprintf(stderr, "Cannot open Resources/bunny.obj\n");
        exit(EXIT_FAILURE);
    }

    const auto bunny = ImplicitTriangleMesh3::Builder()
                           .WithTriangleMesh(bunnyMesh)
                           .WithResolutionX(resX)
                           .MakeShared();

    auto emitter =
        VolumeGridEmitter3::Builder().WithSourceRegion(bunny).MakeShared();

    solver->SetEmitter(emitter);
    emitter->AddSignedDistanceTarget(solver->GetSignedDistanceField());

    // Print simulation info
    printf("Running example 3 (high-viscosity)\n");
    PrintInfo(solver);

    // Run simulation
    RunSimulation(rootDir, solver, numberOfFrames, fps);
}

// Low-viscosity example (bunny-drop)
void RunExample4(const std::string& rootDir, size_t resX, int numberOfFrames,
                 double fps)
{
    // Build solver
    auto solver = LevelSetLiquidSolver3::Builder()
                      .WithResolution({ resX, resX, resX })
                      .WithDomainSizeX(1.0)
                      .MakeShared();
    solver->SetUseCompressedLinearSystem(true);

    solver->SetViscosityCoefficient(0.0);
    solver->SetIsGlobalCompensationEnabled(true);

    auto grids = solver->GetGridSystemData();

    // Build emitters
    auto bunnyMesh = TriangleMesh3::Builder().MakeShared();
    std::ifstream objFile(RESOURCES_DIR "/bunny.obj");
    if (objFile)
    {
        [[maybe_unused]] bool isLoaded = bunnyMesh->ReadObj(&objFile);
    }
    else
    {
        fprintf(stderr, "Cannot open Resources/bunny.obj\n");
        exit(EXIT_FAILURE);
    }

    const auto bunny = ImplicitTriangleMesh3::Builder()
                           .WithTriangleMesh(bunnyMesh)
                           .WithResolutionX(resX)
                           .MakeShared();

    auto emitter =
        VolumeGridEmitter3::Builder().WithSourceRegion(bunny).MakeShared();

    solver->SetEmitter(emitter);
    emitter->AddSignedDistanceTarget(solver->GetSignedDistanceField());

    // Print simulation info
    printf("Running example 4 (low-viscosity)\n");
    PrintInfo(solver);

    // Run simulation
    RunSimulation(rootDir, solver, numberOfFrames, fps);
}

int main(int argc, char* argv[])
{
    bool showHelp = false;

    size_t resX = 50;
    int numberOfFrames = 100;
    double fps = 60.0;
    int exampleNum = 1;
    std::string logFileName = APP_NAME ".log";
    std::string outputDir = APP_NAME "_output";

    // Parsing
    auto parser =
        clara::Help(showHelp) |
        clara::Opt(resX, "resX")["-r"]["--resx"](
            "grid resolution in x-axis (default is 50)") |
        clara::Opt(numberOfFrames, "numberOfFrames")["-f"]["--frames"](
            "total number of frames (default is 100)") |
        clara::Opt(
            fps, "fps")["-p"]["--fps"]("frames per second (default is 60.0)") |
        clara::Opt(exampleNum, "exampleNum")["-e"]["--example"](
            "example number (between 1 and 4, default is 1)") |
        clara::Opt(logFileName, "logFileName")["-l"]["--log"](
            "log file name (default is " APP_NAME ".log)") |
        clara::Opt(outputDir, "outputDir")["-o"]["--output"](
            "output directory name (default is " APP_NAME "_output)");

    auto result = parser.parse(clara::Args(argc, argv));
    if (!result)
    {
        std::cerr << "Error in command line: " << result.errorMessage() << '\n';
        exit(EXIT_FAILURE);
    }

    if (showHelp)
    {
        std::cout << ToString(parser) << '\n';
        exit(EXIT_SUCCESS);
    }

#ifdef CUBBYFLOW_WINDOWS
    _mkdir(outputDir.c_str());
#else
    mkdir(outputDir.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
#endif

    std::ofstream logFile(logFileName.c_str());
    if (logFile)
    {
        Logging::SetAllStream(&logFile);
    }

    switch (exampleNum)
    {
        case 1:
            RunExample1(outputDir, resX, numberOfFrames, fps);
            break;
        case 2:
            RunExample2(outputDir, resX, numberOfFrames, fps);
            break;
        case 3:
            RunExample3(outputDir, resX, numberOfFrames, fps);
            break;
        case 4:
            RunExample4(outputDir, resX, numberOfFrames, fps);
            break;
        default:
            std::cout << ToString(parser) << '\n';
            exit(EXIT_FAILURE);
    }

    return EXIT_SUCCESS;
}