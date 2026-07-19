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
#include <Core/Emitter/VolumeParticleEmitter3.hpp>
#include <Core/Geometry/Box.hpp>
#include <Core/Geometry/Cylinder3.hpp>
#include <Core/Geometry/ImplicitSurfaceSet.hpp>
#include <Core/Geometry/Plane.hpp>
#include <Core/Geometry/RigidBodyCollider.hpp>
#include <Core/Geometry/Sphere.hpp>
#include <Core/Particle/ParticleSystemData.hpp>
#include <Core/Solver/Particle/PCISPH/PCISPHSolver3.hpp>
#include <Core/Solver/Particle/SPH/SPHSolver3.hpp>
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

#define APP_NAME "SPHSim"

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
    std::string filename = pystring::os::path::join(rootDir, baseName);
    std::ofstream file(filename.c_str());
    if (file)
    {
        printf("Writing %s...\n", filename.c_str());
        for (const auto& pt : positions)
        {
            file << pt.x << ' ' << pt.y << ' ' << pt.z << std::endl;
        }
        file.close();
    }
}

void PrintInfo(const SPHSolver3Ptr& solver)
{
    const auto particles = solver->GetSPHSystemData();
    printf("Number of particles: %zu\n", particles->NumberOfParticles());
}

void RunSimulation(const std::string& rootDir, const SPHSolver3Ptr& solver,
                   int numberOfFrames, const std::string& format, double fps)
{
    const auto particles = solver->GetSPHSystemData();

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

// Water-drop example (PCISPH)
void RunExample1(const std::string& rootDir, double targetSpacing,
                 int numberOfFrames, const std::string& format, double fps)
{
    BoundingBox3D domain(Vector3D(), Vector3D(1, 2, 1));

    // Build solver
    auto solver = PCISPHSolver3::GetBuilder()
                      .WithTargetDensity(1000.0)
                      .WithTargetSpacing(targetSpacing)
                      .MakeShared();

    solver->SetPseudoViscosityCoefficient(0.0);

    // Build emitter
    BoundingBox3D sourceBound(domain);
    sourceBound.Expand(-targetSpacing);

    const auto plane = Plane3::GetBuilder()
                           .WithNormal({ 0, 1, 0 })
                           .WithPoint({ 0, 0.25 * domain.Height(), 0 })
                           .MakeShared();

    const auto sphere = Sphere3::GetBuilder()
                            .WithCenter(domain.MidPoint())
                            .WithRadius(0.15 * domain.Width())
                            .MakeShared();

    const auto surfaceSet =
        ImplicitSurfaceSet3::GetBuilder()
            .WithExplicitSurfaces(Array1<Surface3Ptr>{ plane, sphere })
            .MakeShared();

    const auto emitter = VolumeParticleEmitter3::GetBuilder()
                             .WithImplicitSurface(surfaceSet)
                             .WithSpacing(targetSpacing)
                             .WithMaxRegion(sourceBound)
                             .WithIsOneShot(true)
                             .MakeShared();

    solver->SetEmitter(emitter);

    // Build collider
    const auto box = Box3::GetBuilder()
                         .WithIsNormalFlipped(true)
                         .WithBoundingBox(domain)
                         .MakeShared();

    const auto collider =
        RigidBodyCollider3::GetBuilder().WithSurface(box).MakeShared();

    solver->SetCollider(collider);

    // Print simulation info
    printf("Running example 1 (water-drop with PCISPH)\n");
    PrintInfo(solver);

    // Run simulation
    RunSimulation(rootDir, solver, numberOfFrames, format, fps);
}

// Water-drop example (SPH)
void RunExample2(const std::string& rootDir, double targetSpacing,
                 int numberOfFrames, const std::string& format, double fps)
{
    BoundingBox3D domain(Vector3D(), Vector3D(1, 2, 1));

    auto solver = SPHSolver3::GetBuilder()
                      .WithTargetDensity(1000.0)
                      .WithTargetSpacing(targetSpacing)
                      .MakeShared();

    solver->SetPseudoViscosityCoefficient(0.0);

    // Build emitter
    BoundingBox3D sourceBound(domain);
    sourceBound.Expand(-targetSpacing);

    const auto plane = Plane3::GetBuilder()
                           .WithNormal({ 0, 1, 0 })
                           .WithPoint({ 0, 0.25 * domain.Height(), 0 })
                           .MakeShared();

    const auto sphere = Sphere3::GetBuilder()
                            .WithCenter(domain.MidPoint())
                            .WithRadius(0.15 * domain.Width())
                            .MakeShared();

    const auto surfaceSet =
        ImplicitSurfaceSet3::GetBuilder()
            .WithExplicitSurfaces(Array1<Surface3Ptr>{ plane, sphere })
            .MakeShared();

    const auto emitter = VolumeParticleEmitter3::GetBuilder()
                             .WithImplicitSurface(surfaceSet)
                             .WithSpacing(targetSpacing)
                             .WithMaxRegion(sourceBound)
                             .WithIsOneShot(true)
                             .MakeShared();

    solver->SetEmitter(emitter);

    // Build collider
    const auto box = Box3::GetBuilder()
                         .WithIsNormalFlipped(true)
                         .WithBoundingBox(domain)
                         .MakeShared();

    const auto collider =
        RigidBodyCollider3::GetBuilder().WithSurface(box).MakeShared();

    solver->SetCollider(collider);

    // Print simulation info
    printf("Running example 2 (water-drop with SPH)\n");
    PrintInfo(solver);

    // Run simulation
    RunSimulation(rootDir, solver, numberOfFrames, format, fps);
}

// Dam-breaking example
void RunExample3(const std::string& rootDir, double targetSpacing,
                 int numberOfFrames, const std::string& format, double fps)
{
    BoundingBox3D domain(Vector3D(), Vector3D(3, 2, 1.5));
    const double lz = domain.Depth();

    // Build solver
    auto solver = PCISPHSolver3::GetBuilder()
                      .WithTargetDensity(1000.0)
                      .WithTargetSpacing(targetSpacing)
                      .MakeShared();

    solver->SetPseudoViscosityCoefficient(0.0);
    solver->SetTimeStepLimitScale(10.0);

    // Build emitter
    BoundingBox3D sourceBound(domain);
    sourceBound.Expand(-targetSpacing);

    const auto box1 =
        Box3::GetBuilder()
            .WithLowerCorner({ 0, 0, 0 })
            .WithUpperCorner({ 0.5 + 0.001, 0.75 + 0.001, 0.75 * lz + 0.001 })
            .MakeShared();

    const auto box2 =
        Box3::GetBuilder()
            .WithLowerCorner({ 2.5 - 0.001, 0, 0.25 * lz - 0.001 })
            .WithUpperCorner({ 3.5 + 0.001, 0.75 + 0.001, 1.5 * lz + 0.001 })
            .MakeShared();

    const auto boxSet =
        ImplicitSurfaceSet3::GetBuilder()
            .WithExplicitSurfaces(Array1<Surface3Ptr>{ box1, box2 })
            .MakeShared();

    const auto emitter = VolumeParticleEmitter3::GetBuilder()
                             .WithSurface(boxSet)
                             .WithMaxRegion(sourceBound)
                             .WithSpacing(targetSpacing)
                             .MakeShared();

    solver->SetEmitter(emitter);

    // Build collider
    const auto cyl1 = Cylinder3::GetBuilder()
                          .WithCenter({ 1, 0.375, 0.375 })
                          .WithRadius(0.1)
                          .WithHeight(0.75)
                          .MakeShared();

    const auto cyl2 = Cylinder3::GetBuilder()
                          .WithCenter({ 1.5, 0.375, 0.75 })
                          .WithRadius(0.1)
                          .WithHeight(0.75)
                          .MakeShared();

    const auto cyl3 = Cylinder3::GetBuilder()
                          .WithCenter({ 2, 0.375, 1.125 })
                          .WithRadius(0.1)
                          .WithHeight(0.75)
                          .MakeShared();

    const auto box = Box3::GetBuilder()
                         .WithIsNormalFlipped(true)
                         .WithBoundingBox(domain)
                         .MakeShared();

    const auto surfaceSet =
        ImplicitSurfaceSet3::GetBuilder()
            .WithExplicitSurfaces(Array1<Surface3Ptr>{ cyl1, cyl2, cyl3, box })
            .MakeShared();

    const auto collider =
        RigidBodyCollider3::GetBuilder().WithSurface(surfaceSet).MakeShared();

    solver->SetCollider(collider);

    // Print simulation info
    printf("Running example 3 (dam-breaking with PCISPH)\n");
    PrintInfo(solver);

    // Run simulation
    RunSimulation(rootDir, solver, numberOfFrames, format, fps);
}

int main(int argc, char* argv[])
{
    bool showHelp = false;

    double targetSpacing = 0.02;
    int numberOfFrames = 100;
    double fps = 60.0;
    int exampleNum = 1;
    std::string logFileName = APP_NAME ".log";
    std::string outputDir = APP_NAME "_output";
    std::string format = "xyz";

    // Parsing
    auto parser =
        clara::Help(showHelp) |
        clara::Opt(targetSpacing, "targetSpacing")["-s"]["--spacing"](
            "target particle spacing (default is 0.02)") |
        clara::Opt(numberOfFrames, "numberOfFrames")["-f"]["--frames"](
            "total number of frames (default is 100)") |
        clara::Opt(
            fps, "fps")["-p"]["--fps"]("frames per second (default is 60.0)") |
        clara::Opt(exampleNum, "exampleNum")["-e"]["--example"](
            "example number (between 1 and 3, default is 1)") |
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
            RunExample1(outputDir, targetSpacing, numberOfFrames, format, fps);
            break;
        case 2:
            RunExample2(outputDir, targetSpacing, numberOfFrames, format, fps);
            break;
        case 3:
            RunExample3(outputDir, targetSpacing, numberOfFrames, format, fps);
            break;
        default:
            std::cout << ToString(parser) << '\n';
            exit(EXIT_FAILURE);
    }

    return EXIT_SUCCESS;
}