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

#include <Core/Array/ArrayUtils.hpp>
#include <Core/Emitter/ParticleEmitterSet3.hpp>
#include <Core/Emitter/VolumeParticleEmitter3.hpp>
#include <Core/Geometry/Box.hpp>
#include <Core/Geometry/Cylinder3.hpp>
#include <Core/Geometry/ImplicitSurfaceSet.hpp>
#include <Core/Geometry/Plane.hpp>
#include <Core/Geometry/RigidBodyCollider.hpp>
#include <Core/Geometry/Sphere.hpp>
#include <Core/Particle/ParticleSystemData.hpp>
#include <Core/PointGenerator/BccLatticePointGenerator.hpp>
#include <Core/PointGenerator/GridPointGenerator3.hpp>
#include <Core/Solver/Hybrid/APIC/APICSolver3.hpp>
#include <Core/Solver/Hybrid/FLIP/FLIPSolver3.hpp>
#include <Core/Solver/Hybrid/PIC/PICSolver3.hpp>
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

#define APP_NAME "HybridLiquidSim"

using namespace CubbyFlow;

void SaveParticleAsPos(const ParticleSystemData3Ptr& particles,
                       const std::string& rootDir, int frameCnt)
{
    Array1<Vector3D> positions(particles->NumberOfParticles());
    Copy(particles->Positions(), positions.View());
    char baseName[256];
    snprintf(baseName, sizeof(baseName), "frame_%06d.pos", frameCnt);
    std::string fileName = pystring::os::path::join(rootDir, baseName);
    std::ofstream file(fileName.c_str(), std::ios::binary);
    if (file)
    {
        printf("Writing %s...\n", fileName.c_str());
        std::vector<uint8_t> buffer;
        Serialize<Vector3D>(positions.View(), &buffer);
        file.write(reinterpret_cast<char*>(buffer.data()), buffer.size());
        file.close();
    }
}

void SaveParticleAsXYZ(const ParticleSystemData3Ptr& particles,
                       const std::string& rootDir, int frameCnt)
{
    Array1<Vector3D> positions(particles->NumberOfParticles());
    Copy(particles->Positions(), positions.View());
    char baseName[256];
    snprintf(baseName, sizeof(baseName), "frame_%06d.xyz", frameCnt);
    std::string fileName = pystring::os::path::join(rootDir, baseName);
    std::ofstream file(fileName.c_str());
    if (file)
    {
        printf("Writing %s...\n", fileName.c_str());
        for (const auto& pt : positions)
        {
            file << pt.x << ' ' << pt.y << ' ' << pt.z << std::endl;
        }
        file.close();
    }
}

void PrintInfo(const PICSolver3Ptr& solver)
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

void RunSimulation(const std::string& rootDir, const PICSolver3Ptr& solver,
                   int numberOfFrames, const std::string& format, double fps)
{
    const auto particles = solver->GetParticleSystemData();

    for (Frame frame(0, 1.0 / fps); frame.index < numberOfFrames; ++frame)
    {
        solver->Update(frame);
        if (format == "xyz")
        {
            SaveParticleAsXYZ(particles, rootDir, frame.index);
        }
        else if (format == "pos")
        {
            SaveParticleAsPos(particles, rootDir, frame.index);
        }
    }
}

// Water-drop example (FLIP)
void RunExample1(const std::string& rootDir, size_t resolutionX,
                 int numberOfFrames, const std::string& format, double fps)
{
    // Build solver
    auto solver =
        FLIPSolver3::Builder()
            .WithResolution({ resolutionX, 2 * resolutionX, resolutionX })
            .WithDomainSizeX(1.0)
            .MakeShared();

    const auto grids = solver->GetGridSystemData();
    auto particles = solver->GetParticleSystemData();

    const Vector3D gridSpacing = grids->GridSpacing();
    const double dx = gridSpacing.x;
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

    auto emitter1 = VolumeParticleEmitter3::Builder()
                        .WithSurface(plane)
                        .WithSpacing(0.5 * dx)
                        .WithMaxRegion(domain)
                        .WithIsOneShot(true)
                        .MakeShared();
    emitter1->SetPointGenerator(std::make_shared<GridPointGenerator3>());

    auto emitter2 = VolumeParticleEmitter3::Builder()
                        .WithSurface(sphere)
                        .WithSpacing(0.5 * dx)
                        .WithMaxRegion(domain)
                        .WithIsOneShot(true)
                        .MakeShared();
    emitter2->SetPointGenerator(std::make_shared<GridPointGenerator3>());

    const auto emitterSet = ParticleEmitterSet3::Builder()
                                .WithEmitters({ emitter1, emitter2 })
                                .MakeShared();

    solver->SetParticleEmitter(emitterSet);

    // Print simulation info
    printf("Running example 1 (water-drop With FLIP)\n");
    PrintInfo(solver);

    // Run simulation
    RunSimulation(rootDir, solver, numberOfFrames, format, fps);
}

// Water-drop example (PIC)
void RunExample2(const std::string& rootDir, size_t resolutionX,
                 int numberOfFrames, const std::string& format, double fps)
{
    // Build solver
    auto solver =
        PICSolver3::Builder()
            .WithResolution({ resolutionX, 2 * resolutionX, resolutionX })
            .WithDomainSizeX(1.0)
            .MakeShared();

    const auto grids = solver->GetGridSystemData();
    auto particles = solver->GetParticleSystemData();

    const Vector3D gridSpacing = grids->GridSpacing();
    double dx = gridSpacing.x;
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

    auto emitter1 = VolumeParticleEmitter3::Builder()
                        .WithSurface(plane)
                        .WithSpacing(0.5 * dx)
                        .WithMaxRegion(domain)
                        .WithIsOneShot(true)
                        .MakeShared();
    emitter1->SetPointGenerator(std::make_shared<GridPointGenerator3>());

    auto emitter2 = VolumeParticleEmitter3::Builder()
                        .WithSurface(sphere)
                        .WithSpacing(0.5 * dx)
                        .WithMaxRegion(domain)
                        .WithIsOneShot(true)
                        .MakeShared();
    emitter2->SetPointGenerator(std::make_shared<GridPointGenerator3>());

    const auto emitterSet = ParticleEmitterSet3::Builder()
                                .WithEmitters({ emitter1, emitter2 })
                                .MakeShared();

    solver->SetParticleEmitter(emitterSet);

    // Print simulation info
    printf("Running example 1 (water-drop With PIC)\n");
    PrintInfo(solver);

    // Run simulation
    RunSimulation(rootDir, solver, numberOfFrames, format, fps);
}

// Dam-breaking example (FLIP)
void RunExample3(const std::string& rootDir, size_t resolutionX,
                 int numberOfFrames, const std::string& format, double fps)
{
    // Build solver
    const Vector3UZ resolution{ 3 * resolutionX, 2 * resolutionX,
                                (3 * resolutionX) / 2 };
    auto solver = FLIPSolver3::Builder()
                      .WithResolution(resolution)
                      .WithDomainSizeX(3.0)
                      .MakeShared();
    solver->SetUseCompressedLinearSystem(true);

    const auto grids = solver->GetGridSystemData();
    const double dx = grids->GridSpacing().x;
    const BoundingBox3D domain = grids->GetBoundingBox();
    const double lz = domain.Depth();

    // Build emitter
    const auto box1 =
        Box3::Builder()
            .WithLowerCorner({ 0, 0, 0 })
            .WithUpperCorner({ 0.5 + 0.001, 0.75 + 0.001, 0.75 * lz + 0.001 })
            .MakeShared();

    const auto box2 =
        Box3::Builder()
            .WithLowerCorner({ 2.5 - 0.001, 0, 0.25 * lz - 0.001 })
            .WithUpperCorner({ 3.5 + 0.001, 0.75 + 0.001, 1.5 * lz + 0.001 })
            .MakeShared();

    const auto boxSet =
        ImplicitSurfaceSet3::Builder()
            .WithExplicitSurfaces(Array1<Surface3Ptr>{ box1, box2 })
            .MakeShared();

    auto emitter = VolumeParticleEmitter3::Builder()
                       .WithSurface(boxSet)
                       .WithMaxRegion(domain)
                       .WithSpacing(0.5 * dx)
                       .MakeShared();

    emitter->SetPointGenerator(std::make_shared<GridPointGenerator3>());
    solver->SetParticleEmitter(emitter);

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
    printf("Running example 3 (dam-breaking With FLIP)\n");
    PrintInfo(solver);

    // Run simulation
    RunSimulation(rootDir, solver, numberOfFrames, format, fps);
}

// Dam-breaking example (PIC)
void RunExample4(const std::string& rootDir, size_t resolutionX,
                 int numberOfFrames, const std::string& format, double fps)
{
    // Build solver
    const Vector3UZ resolution{ 3 * resolutionX, 2 * resolutionX,
                                (3 * resolutionX) / 2 };
    auto solver = PICSolver3::Builder()
                      .WithResolution(resolution)
                      .WithDomainSizeX(3.0)
                      .MakeShared();
    solver->SetUseCompressedLinearSystem(true);

    const auto grids = solver->GetGridSystemData();
    const double dx = grids->GridSpacing().x;
    const BoundingBox3D domain = grids->GetBoundingBox();
    const double lz = domain.Depth();

    // Build emitter
    const auto box1 =
        Box3::Builder()
            .WithLowerCorner({ 0, 0, 0 })
            .WithUpperCorner({ 0.5 + 0.001, 0.75 + 0.001, 0.75 * lz + 0.001 })
            .MakeShared();

    const auto box2 =
        Box3::Builder()
            .WithLowerCorner({ 2.5 - 0.001, 0, 0.25 * lz - 0.001 })
            .WithUpperCorner({ 3.5 + 0.001, 0.75 + 0.001, 1.5 * lz + 0.001 })
            .MakeShared();

    const auto boxSet =
        ImplicitSurfaceSet3::Builder()
            .WithExplicitSurfaces(Array1<Surface3Ptr>{ box1, box2 })
            .MakeShared();

    auto emitter = VolumeParticleEmitter3::Builder()
                       .WithSurface(boxSet)
                       .WithMaxRegion(domain)
                       .WithSpacing(0.5 * dx)
                       .MakeShared();

    emitter->SetPointGenerator(std::make_shared<GridPointGenerator3>());
    solver->SetParticleEmitter(emitter);

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
    printf("Running example 4 (dam-breaking With PIC)\n");
    PrintInfo(solver);

    // Run simulation
    RunSimulation(rootDir, solver, numberOfFrames, format, fps);
}

// Dam-breaking example (APIC)
void RunExample5(const std::string& rootDir, size_t resolutionX,
                 unsigned int numberOfFrames, const std::string& format,
                 double fps)
{
    // Build solver
    const Vector3UZ resolution{ 3 * resolutionX, 2 * resolutionX,
                                (3 * resolutionX) / 2 };
    auto solver = APICSolver3::Builder()
                      .WithResolution(resolution)
                      .WithDomainSizeX(3.0)
                      .MakeShared();
    solver->SetUseCompressedLinearSystem(true);

    const auto grids = solver->GetGridSystemData();
    const double dx = grids->GridSpacing().x;
    const BoundingBox3D domain = grids->GetBoundingBox();
    const double lz = domain.Depth();

    // Build emitter
    const auto box1 =
        Box3::Builder()
            .WithLowerCorner({ 0, 0, 0 })
            .WithUpperCorner({ 0.5 + 0.001, 0.75 + 0.001, 0.75 * lz + 0.001 })
            .MakeShared();

    const auto box2 =
        Box3::Builder()
            .WithLowerCorner({ 2.5 - 0.001, 0, 0.25 * lz - 0.001 })
            .WithUpperCorner({ 3.5 + 0.001, 0.75 + 0.001, 1.5 * lz + 0.001 })
            .MakeShared();

    const auto boxSet =
        ImplicitSurfaceSet3::Builder()
            .WithExplicitSurfaces(Array1<Surface3Ptr>{ box1, box2 })
            .MakeShared();

    auto emitter = VolumeParticleEmitter3::Builder()
                       .WithSurface(boxSet)
                       .WithMaxRegion(domain)
                       .WithSpacing(0.5 * dx)
                       .MakeShared();

    emitter->SetPointGenerator(std::make_shared<GridPointGenerator3>());
    solver->SetParticleEmitter(emitter);

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
    printf("Running example 5 (dam-breaking with APIC)\n");
    PrintInfo(solver);

    // Run simulation
    RunSimulation(rootDir, solver, numberOfFrames, format, fps);
}

// Sphere boundary with APIC
void RunExample6(const std::string& rootDir, size_t resolutionX,
                 unsigned int numberOfFrames, const std::string& format,
                 double fps)
{
    // Build solver
    auto solver = APICSolver3::Builder()
                      .WithResolution({ resolutionX, resolutionX, resolutionX })
                      .WithDomainSizeX(1.0)
                      .MakeShared();
    solver->SetUseCompressedLinearSystem(true);

    // Build collider
    const auto sphere = Sphere3::Builder()
                            .WithCenter({ 0.5, 0.5, 0.5 })
                            .WithRadius(0.4)
                            .WithIsNormalFlipped(true)
                            .MakeShared();

    const auto collider =
        RigidBodyCollider3::Builder().WithSurface(sphere).MakeShared();

    solver->SetCollider(collider);

    // Manually emit particles
    printf("Start emitting particles...\n");
    std::mt19937 rng;
    std::uniform_real_distribution<> dist(-0.1 * solver->GetGridSpacing().x,
                                          0.1 * solver->GetGridSpacing().x);
    const BccLatticePointGenerator pointGenerator;

    pointGenerator.ForEachPoint(
        BoundingBox3D({ 0.75, 0, 0 }, { 1, 1, 1 }),
        0.5 * solver->GetGridSpacing().x, [&](const Vector3D& pt) -> bool {
            Vector3D newPos = pt + Vector3D{ dist(rng), dist(rng), dist(rng) };

            if ((pt - Vector3D{ 0.5, 0.5, 0.5 }).Length() < 0.4)
            {
                solver->GetParticleSystemData()->AddParticle(newPos);
            }

            return true;
        });

    printf("Number of particles: %zu\n",
           solver->GetParticleSystemData()->NumberOfParticles());

    // Print simulation info
    printf("Running example 6 (sphere boundary with APIC)\n");
    PrintInfo(solver);

    // Run simulation
    RunSimulation(rootDir, solver, numberOfFrames, format, fps);
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
    std::string format = "xyz";

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
            "example number (between 1 and 6, default is 1)") |
        clara::Opt(logFileName, "logFileName")["-l"]["--log"](
            "log file name (default is " APP_NAME ".log)") |
        clara::Opt(outputDir, "outputDir")["-o"]["--output"](
            "output directory name (default is " APP_NAME "_output)") |
        clara::Opt(format, "format")["-m"]["--format"](
            "particle output format (xyz or pos. default is xyz)");

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
            RunExample1(outputDir, resX, numberOfFrames, format, fps);
            break;
        case 2:
            RunExample2(outputDir, resX, numberOfFrames, format, fps);
            break;
        case 3:
            RunExample3(outputDir, resX, numberOfFrames, format, fps);
            break;
        case 4:
            RunExample4(outputDir, resX, numberOfFrames, format, fps);
            break;
        case 5:
            RunExample5(outputDir, resX, numberOfFrames, format, fps);
            break;
        case 6:
            RunExample6(outputDir, resX, numberOfFrames, format, fps);
            break;
        default:
            std::cout << ToString(parser) << '\n';
            exit(EXIT_FAILURE);
    }

    return EXIT_SUCCESS;
}