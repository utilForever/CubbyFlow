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
#include <Core/Geometry/ImplicitTriangleMesh3.hpp>
#include <Core/Geometry/RigidBodyCollider.hpp>
#include <Core/Geometry/Sphere.hpp>
#include <Core/Geometry/TriangleMesh3.hpp>
#include <Core/Grid/ScalarGrid.hpp>
#include <Core/Math/MathUtils.hpp>
#include <Core/Solver/Advection/CubicSemiLagrangian3.hpp>
#include <Core/Solver/Advection/SemiLagrangian3.hpp>
#include <Core/Solver/Grid/GridSmokeSolver3.hpp>
#include <Core/Utils/Logging.hpp>

#include <pystring/pystring.h>
#include <Clara/include/clara.hpp>

#ifdef CUBBYFLOW_WINDOWS
#include <direct.h>
#else
#include <sys/stat.h>
#endif

#include <array>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#define APP_NAME "SmokeSim"

using namespace CubbyFlow;

const size_t EDGE_BLUR = 3;
const float EDGE_BLUR_F = 3.f;
const double TGA_SCALE = 10.0;

inline float SmoothStep(float edge0, float edge1, float x)
{
    const float t = Clamp((x - edge0) / (edge1 - edge0), 0.f, 1.f);
    return t * t * (3.f - 2.f * t);
}

// Export density field to Mitsuba volume file.
void SaveVolumeAsVol(const ScalarGrid3Ptr& density, const std::string& rootDir,
                     int frameCnt)
{
    char baseName[256];
    snprintf(baseName, sizeof(baseName), "frame_%06d.vol", frameCnt);
    std::string fileName = pystring::os::path::join(rootDir, baseName);
    std::ofstream file(fileName.c_str(), std::ofstream::binary);
    if (file)
    {
        printf("Writing %s...\n", fileName.c_str());

        // Mitsuba 0.5.0 grid-volume format
        char header[48];
        memset(header, 0, sizeof(header));

        header[0] = 'V';
        header[1] = 'O';
        header[2] = 'L';
        header[3] = 3;

        const std::array<int32_t, 5> encoding = {
            1,  // 32-bit float
            static_cast<int32_t>(density->DataSize().x),
            static_cast<int32_t>(density->DataSize().y),
            static_cast<int32_t>(density->DataSize().z),
            1  // number of channels
        };
        std::memcpy(header + 4, encoding.data(), sizeof(encoding));

        const BoundingBox3D domain = density->GetBoundingBox();
        const std::array<float, 6> bbox = {
            static_cast<float>(domain.lowerCorner.x),
            static_cast<float>(domain.lowerCorner.y),
            static_cast<float>(domain.lowerCorner.z),
            static_cast<float>(domain.upperCorner.x),
            static_cast<float>(domain.upperCorner.y),
            static_cast<float>(domain.upperCorner.z)
        };
        std::memcpy(header + 4 + sizeof(encoding), bbox.data(), sizeof(bbox));

        file.write(header, sizeof(header));

        Array3<float> data(density->DataSize());
        ParallelForEachIndex(data.Size(), [&](size_t i, size_t j, size_t k) {
            float d = static_cast<float>((*density)(i, j, k));

            // Blur the edge for less-noisy rendering
            if (i < EDGE_BLUR)
            {
                d *= SmoothStep(0.f, EDGE_BLUR_F, static_cast<float>(i));
            }
            if (i > data.Size().x - 1 - EDGE_BLUR)
            {
                d *= SmoothStep(0.f, EDGE_BLUR_F,
                                static_cast<float>((data.Size().x - 1) - i));
            }
            if (j < EDGE_BLUR)
            {
                d *= SmoothStep(0.f, EDGE_BLUR_F, static_cast<float>(j));
            }
            if (j > data.Size().y - 1 - EDGE_BLUR)
            {
                d *= SmoothStep(0.f, EDGE_BLUR_F,
                                static_cast<float>((data.Size().y - 1) - j));
            }
            if (k < EDGE_BLUR)
            {
                d *= SmoothStep(0.f, EDGE_BLUR_F, static_cast<float>(k));
            }
            if (k > data.Size().z - 1 - EDGE_BLUR)
            {
                d *= SmoothStep(0.f, EDGE_BLUR_F,
                                static_cast<float>((data.Size().z - 1) - k));
            }

            data(i, j, k) = d;
        });

        file.write(
            reinterpret_cast<const char*>(data.data()),
            sizeof(float) * data.Size().x * data.Size().y * data.Size().z);

        file.close();
    }
}

void SaveVolumeAsTga(const ScalarGrid3Ptr& density, const std::string& rootDir,
                     int frameCnt)
{
    char baseName[256];
    snprintf(baseName, sizeof(baseName), "frame_%06d.tga", frameCnt);
    std::string fileName = pystring::os::path::join(rootDir, baseName);
    std::ofstream file(fileName.c_str(), std::ofstream::binary);
    if (file)
    {
        printf("Writing %s...\n", fileName.c_str());

        Vector3UZ dataSize = density->DataSize();

        std::array<char, 18> header;
        header.fill(0);

        const int imgWidth = static_cast<int>(dataSize.x);
        const int imgHeight = static_cast<int>(dataSize.y);

        header[2] = 2;
        header[12] = static_cast<char>(imgWidth & 0xff);
        header[13] = static_cast<char>((imgWidth & 0xff00) >> 8);
        header[14] = static_cast<char>(imgHeight & 0xff);
        header[15] = static_cast<char>((imgHeight & 0xff00) >> 8);
        header[16] = 24;

        file.write(header.data(), header.size());

        Array2<double> hdrImg(dataSize.x, dataSize.y);
        ParallelForEachIndex(hdrImg.Size(), [&](size_t i, size_t j) {
            double sum = 0.0;
            for (size_t k = 0; k < dataSize.z; ++k)
            {
                sum += (*density)(i, j, k);
            }
            hdrImg(i, j) = TGA_SCALE * sum / static_cast<double>(dataSize.z);
        });

        std::vector<char> img(3 * dataSize.x * dataSize.y);
        for (size_t i = 0; i < dataSize.x * dataSize.y; ++i)
        {
            const char val =
                static_cast<char>(Clamp(hdrImg[i], 0.0, 1.0) * 255.0);
            img[3 * i + 0] = val;
            img[3 * i + 1] = val;
            img[3 * i + 2] = val;
        }

        file.write(img.data(), img.size());

        file.close();
    }
}

void PrintInfo(const GridSmokeSolver3Ptr& solver)
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
                   const GridSmokeSolver3Ptr& solver, int numberOfFrames,
                   const std::string& format, double fps)
{
    const auto density = solver->GetSmokeDensity();

    for (Frame frame(0, 1.0 / fps); frame.index < numberOfFrames; ++frame)
    {
        solver->Update(frame);

        if (format == "vol")
        {
            SaveVolumeAsVol(density, rootDir, frame.index);
        }
        else if (format == "tga")
        {
            SaveVolumeAsTga(density, rootDir, frame.index);
        }
    }
}

void RunExample1(const std::string& rootDir, size_t resolutionX,
                 int numberOfFrames, const std::string& format, double fps)
{
    // Build solver
    auto solver =
        GridSmokeSolver3::Builder()
            .WithResolution({ resolutionX, 2 * resolutionX, resolutionX })
            .WithDomainSizeX(1.0)
            .MakeShared();

    solver->SetAdvectionSolver(std::make_shared<CubicSemiLagrangian3>());

    const auto grids = solver->GetGridSystemData();
    BoundingBox3D domain = grids->GetBoundingBox();

    // Build emitter
    const auto box = Box3::Builder()
                         .WithLowerCorner({ 0.45, -1, 0.45 })
                         .WithUpperCorner({ 0.55, 0.05, 0.55 })
                         .MakeShared();

    auto emitter = VolumeGridEmitter3::Builder()
                       .WithSourceRegion(box)
                       .WithIsOneShot(false)
                       .MakeShared();

    solver->SetEmitter(emitter);
    emitter->AddStepFunctionTarget(solver->GetSmokeDensity(), 0, 1);
    emitter->AddStepFunctionTarget(solver->GetTemperature(), 0, 1);

    // Build collider
    const auto sphere = Sphere3::Builder()
                            .WithCenter({ 0.5, 0.3, 0.5 })
                            .WithRadius(0.075 * domain.Width())
                            .MakeShared();

    const auto collider =
        RigidBodyCollider3::Builder().WithSurface(sphere).MakeShared();

    solver->SetCollider(collider);

    // Print simulation info
    printf("Running example 1 (rising smoke with cubic-spline advection)\n");
    PrintInfo(solver);

    // Run simulation
    RunSimulation(rootDir, solver, numberOfFrames, format, fps);
}

void RunExample2(const std::string& rootDir, size_t resolutionX,
                 int numberOfFrames, const std::string& format, double fps)
{
    // Build solver
    auto solver =
        GridSmokeSolver3::Builder()
            .WithResolution({ resolutionX, 2 * resolutionX, resolutionX })
            .WithDomainSizeX(1.0)
            .MakeShared();

    solver->SetAdvectionSolver(std::make_shared<SemiLagrangian3>());

    const auto grids = solver->GetGridSystemData();
    BoundingBox3D domain = grids->GetBoundingBox();

    // Build emitter
    const auto box = Box3::Builder()
                         .WithLowerCorner({ 0.45, -1, 0.45 })
                         .WithUpperCorner({ 0.55, 0.05, 0.55 })
                         .MakeShared();

    auto emitter = VolumeGridEmitter3::Builder()
                       .WithSourceRegion(box)
                       .WithIsOneShot(false)
                       .MakeShared();

    solver->SetEmitter(emitter);
    emitter->AddStepFunctionTarget(solver->GetSmokeDensity(), 0, 1);
    emitter->AddStepFunctionTarget(solver->GetTemperature(), 0, 1);

    // Build collider
    const auto sphere = Sphere3::Builder()
                            .WithCenter({ 0.5, 0.3, 0.5 })
                            .WithRadius(0.075 * domain.Width())
                            .MakeShared();

    const auto collider =
        RigidBodyCollider3::Builder().WithSurface(sphere).MakeShared();

    solver->SetCollider(collider);

    // Print simulation info
    printf("Running example 2 (rising smoke with linear advection)\n");
    PrintInfo(solver);

    // Run simulation
    RunSimulation(rootDir, solver, numberOfFrames, format, fps);
}

void RunExample3(const std::string& rootDir, size_t resolutionX,
                 int numberOfFrames, const std::string& format, double fps)
{
    // Build solver
    auto solver = GridSmokeSolver3::Builder()
                      .WithResolution(
                          { resolutionX, resolutionX / 4 * 5, resolutionX / 2 })
                      .WithDomainSizeX(2.0)
                      .WithOrigin({ -1, -0.15, -0.5 })
                      .MakeShared();

    // Build emitter
    auto dragonMesh = TriangleMesh3::Builder().MakeShared();
    std::ifstream objFile(RESOURCES_DIR "/dragon.obj");
    if (objFile)
    {
        [[maybe_unused]] bool isLoaded = dragonMesh->ReadObj(&objFile);
    }
    else
    {
        fprintf(stderr, "Cannot open Resources/dragon.obj\n");
        exit(EXIT_FAILURE);
    }

    const auto dragon = ImplicitTriangleMesh3::Builder()
                            .WithTriangleMesh(dragonMesh)
                            .WithResolutionX(resolutionX)
                            .MakeShared();

    auto emitter = VolumeGridEmitter3::Builder()
                       .WithSourceRegion(dragon)
                       .WithIsOneShot(false)
                       .MakeShared();

    solver->SetEmitter(emitter);
    emitter->AddStepFunctionTarget(solver->GetSmokeDensity(), 0, 1);
    emitter->AddStepFunctionTarget(solver->GetTemperature(), 0, 1);

    // Print simulation info
    printf("Running example 3 (rising dragon)\n");
    PrintInfo(solver);

    // Run simulation
    RunSimulation(rootDir, solver, numberOfFrames, format, fps);
}

void RunExample4(const std::string& rootDir, size_t resolutionX,
                 int numberOfFrames, const std::string& format, double fps)
{
    // Build solver
    auto solver = GridSmokeSolver3::Builder()
                      .WithResolution(
                          { resolutionX, 6 * resolutionX / 5, resolutionX / 2 })
                      .WithDomainSizeX(1.0)
                      .MakeShared();

    solver->SetBuoyancyTemperatureFactor(2.0);

    // Build emitter
    const auto box = Box3::Builder()
                         .WithLowerCorner({ 0.05, 0.1, 0.225 })
                         .WithUpperCorner({ 0.1, 0.15, 0.275 })
                         .MakeShared();

    auto emitter = VolumeGridEmitter3::Builder()
                       .WithSourceRegion(box)
                       .WithIsOneShot(false)
                       .MakeShared();

    solver->SetEmitter(emitter);
    emitter->AddStepFunctionTarget(solver->GetSmokeDensity(), 0, 1);
    emitter->AddStepFunctionTarget(solver->GetTemperature(), 0, 1);
    emitter->AddTarget(solver->GetVelocity(),
                       [](double sdf, const Vector3D&, const Vector3D& oldVal) {
                           if (sdf < 0.05)
                           {
                               return Vector3D(0.5, oldVal.y, oldVal.z);
                           }
                           else
                           {
                               return Vector3D(oldVal);
                           }
                       });

    // Print simulation info
    printf("Running example 4 (rising smoke with cubic-spline advection)\n");
    PrintInfo(solver);

    // Run simulation
    RunSimulation(rootDir, solver, numberOfFrames, format, fps);
}

void RunExample5(const std::string& rootDir, size_t resolutionX,
                 int numberOfFrames, const std::string& format, double fps)
{
    // Build solver
    auto solver = GridSmokeSolver3::Builder()
                      .WithResolution(
                          { resolutionX, 6 * resolutionX / 5, resolutionX / 2 })
                      .WithDomainSizeX(1.0)
                      .MakeShared();

    solver->SetBuoyancyTemperatureFactor(2.0);
    solver->SetAdvectionSolver(std::make_shared<SemiLagrangian3>());

    // Build emitter
    const auto box = Box3::Builder()
                         .WithLowerCorner({ 0.05, 0.1, 0.225 })
                         .WithUpperCorner({ 0.1, 0.15, 0.275 })
                         .MakeShared();

    auto emitter = VolumeGridEmitter3::Builder()
                       .WithSourceRegion(box)
                       .WithIsOneShot(false)
                       .MakeShared();

    solver->SetEmitter(emitter);
    emitter->AddStepFunctionTarget(solver->GetSmokeDensity(), 0, 1);
    emitter->AddStepFunctionTarget(solver->GetTemperature(), 0, 1);
    emitter->AddTarget(solver->GetVelocity(), [](double sdf, const Vector3D& pt,
                                                 const Vector3D& oldVal) {
        UNUSED_VARIABLE(pt);

        if (sdf < 0.05)
        {
            return Vector3D(0.5, oldVal.y, oldVal.z);
        }
        else
        {
            return Vector3D(oldVal);
        }
    });

    // Print simulation info
    printf("Running example 5 (rising smoke with linear advection)\n");
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
    std::string format = "tga";

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
            "example number (between 1 and 5, default is 1)") |
        clara::Opt(logFileName, "logFileName")["-l"]["--log"](
            "log file name (default is " APP_NAME ".log)") |
        clara::Opt(outputDir, "outputDir")["-o"]["--output"](
            "output directory name (default is " APP_NAME "_output)") |
        clara::Opt(format, "format")["-m"]["--format"](
            "particle output format (tag or vol. default is tag)");

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
        default:
            std::cout << ToString(parser) << '\n';
            exit(EXIT_FAILURE);
    }

    return EXIT_SUCCESS;
}
