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

#include "SnowBall.hpp"

#include <Core/Animation/Frame.hpp>
#include <Core/Array/Array.hpp>
#include <Core/Geometry/Plane.hpp>
#include <Core/Geometry/RigidBodyCollider.hpp>
#include <Core/Particle/MPM/MPMSystemData.hpp>
#include <Core/Solver/Particle/MPM/SnowMPMSolver.hpp>
#include <Core/Utils/Serialization.hpp>

#include <clara.hpp>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#define APP_NAME "SnowMPMSim"

using namespace CubbyFlow;

namespace
{
SnowMPMSolver3Ptr MakeSolver(size_t resolution, size_t particlesPerCell)
{
    constexpr double targetDensity = 400.0;
    const double gridSpacing = 1.0 / static_cast<double>(resolution);
    const double particleMass = targetDensity * std::pow(gridSpacing, 3.0) /
                                static_cast<double>(particlesPerCell);
    auto solver =
        SnowMPMSolver3::GetBuilder()
            .WithResolution({ resolution, resolution, resolution })
            .WithGridSpacing({ gridSpacing, gridSpacing, gridSpacing })
            .WithRadius(0.25 * gridSpacing)
            .WithMass(particleMass)
            .MakeShared();
    solver->SetDragCoefficient(0.0);
    solver->SetClosedDomainBoundaryFlag(DIRECTION_ALL);
    solver->SetTimeStepLimitScale(0.5);
    return solver;
}

void AddSnowBall(const SnowMPMSolver3Ptr& solver, const Vector3D& center,
                 double radius, const Vector3D& initialVelocity,
                 size_t particlesPerCell, uint32_t seed)
{
    const auto particles = solver->GetMPMSystemData();
    const double gridSpacing = particles->GridMass().GridSpacing().x;
    const auto snowBall = GeneratePaperSnowBall(center, radius, gridSpacing,
                                                particlesPerCell, seed);
    Array1<Vector3D> velocities(snowBall.positions.Length(), initialVelocity);
    const size_t first = particles->NumberOfParticles();
    particles->AddParticles(snowBall.positions, velocities);

    auto masses = particles->ParticleMasses();
    auto states = particles->DeformationStates();
    for (size_t i = 0; i < snowBall.positions.Length(); ++i)
    {
        masses[first + i] = particles->Mass() * snowBall.massScales[i];
        const double plasticVolume =
            1.0 - std::log(snowBall.hardeningScales[i]) / 10.0;
        states[first + i].plastic =
            std::cbrt(plasticVolume) * Matrix3x3D::MakeIdentity();
    }
}

void ValidatePositions(const MPMSystemData3Ptr& particles)
{
    for (const auto& point : particles->Positions())
    {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z))
        {
            throw std::runtime_error{ "Non-finite particle position." };
        }
    }
}

void SaveParticleAsXYZ(const MPMSystemData3Ptr& particles,
                       const std::filesystem::path& outputDirectory,
                       int frameIndex)
{
    std::ostringstream baseName;
    baseName << "frame_" << std::setfill('0') << std::setw(6) << frameIndex
             << ".xyz";
    const auto filePath = outputDirectory / baseName.str();
    std::ofstream file(filePath);

    if (!file)
    {
        throw std::runtime_error{ "Cannot open output file: " +
                                  filePath.string() };
    }

    file << std::setprecision(17);
    for (const auto& point : particles->Positions())
    {
        file << point.x << ' ' << point.y << ' ' << point.z << '\n';
    }

    if (!file)
    {
        throw std::runtime_error{ "Cannot write output file: " +
                                  filePath.string() };
    }
}

void SaveParticleAsPos(const MPMSystemData3Ptr& particles,
                       const std::filesystem::path& outputDirectory,
                       int frameIndex)
{
    std::ostringstream baseName;
    baseName << "frame_" << std::setfill('0') << std::setw(6) << frameIndex
             << ".pos";
    const auto filePath = outputDirectory / baseName.str();
    std::ofstream file(filePath, std::ios::binary);

    if (!file)
    {
        throw std::runtime_error{ "Cannot open output file: " +
                                  filePath.string() };
    }

    std::vector<uint8_t> buffer;
    Serialize<Vector3D>(particles->Positions(), &buffer);
    file.write(reinterpret_cast<const char*>(buffer.data()),
               static_cast<std::streamsize>(buffer.size()));

    if (!file)
    {
        throw std::runtime_error{ "Cannot write output file: " +
                                  filePath.string() };
    }
}

void RunSimulation(const SnowMPMSolver3Ptr& solver,
                   const std::filesystem::path& outputDirectory,
                   int numberOfFrames, double fps, const std::string& format)
{
    std::filesystem::create_directories(outputDirectory);
    const auto particles = solver->GetMPMSystemData();

    std::cout << "Number of particles: " << particles->NumberOfParticles()
              << '\n';

    for (Frame frame{ 0, 1.0 / fps }; frame.index < numberOfFrames; ++frame)
    {
        solver->Update(frame);
        ValidatePositions(particles);
        if (format == "xyz")
        {
            SaveParticleAsXYZ(particles, outputDirectory, frame.index);
        }
        else
        {
            SaveParticleAsPos(particles, outputDirectory, frame.index);
        }
    }
}

void RunExample1(size_t resolution, int numberOfFrames, double fps,
                 const std::filesystem::path& outputDirectory,
                 const std::string& format, size_t particlesPerCell,
                 uint32_t seed, bool useSemiImplicit, unsigned int subSteps)
{
    auto solver = MakeSolver(resolution, particlesPerCell);
    if (useSemiImplicit)
    {
        ConfigurePaperSemiImplicit(*solver, subSteps);
    }
    AddSnowBall(solver, { 0.5, 0.65, 0.5 }, 0.16, { 0.0, 0.0, 0.0 },
                particlesPerCell, seed);

    const auto ground = Plane3::GetBuilder()
                            .WithNormal({ 0.0, 1.0, 0.0 })
                            .WithPoint({ 0.0, 0.1, 0.0 })
                            .MakeShared();
    const auto collider =
        RigidBodyCollider3::GetBuilder().WithSurface(ground).MakeShared();
    collider->SetFrictionCoefficient(0.5);
    solver->SetCollider(collider);

    std::cout << "Running example 1 (snowball drop, "
              << (useSemiImplicit ? "semi-implicit" : "explicit");
    if (useSemiImplicit)
    {
        std::cout << (subSteps > 0 ? ", fixed substeps: "
                                   : ", adaptive substeps");
        if (subSteps > 0)
        {
            std::cout << subSteps;
        }
    }
    std::cout << ")\n";
    RunSimulation(solver, outputDirectory, numberOfFrames, fps, format);
}

void RunExample2(size_t resolution, int numberOfFrames, double fps,
                 const std::filesystem::path& outputDirectory,
                 const std::string& format, size_t particlesPerCell,
                 uint32_t seed, bool useSemiImplicit, unsigned int subSteps)
{
    auto solver = MakeSolver(resolution, particlesPerCell);
    if (useSemiImplicit)
    {
        ConfigurePaperSemiImplicit(*solver, subSteps);
    }
    AddSnowBall(solver, { 0.22, 0.5, 0.5 }, 0.16, { 6.0, 0.0, 0.0 },
                particlesPerCell, seed);

    const auto wall = Plane3::GetBuilder()
                          .WithNormal({ -1.0, 0.0, 0.0 })
                          .WithPoint({ 0.72, 0.0, 0.0 })
                          .MakeShared();
    const auto collider =
        RigidBodyCollider3::GetBuilder().WithSurface(wall).MakeShared();
    solver->SetCollider(collider);

    std::cout << "Running example 2 (non-sticky wall smash, "
              << (useSemiImplicit ? "semi-implicit" : "explicit");
    if (useSemiImplicit)
    {
        std::cout << (subSteps > 0 ? ", fixed substeps: "
                                   : ", adaptive substeps");
        if (subSteps > 0)
        {
            std::cout << subSteps;
        }
    }
    std::cout << ")\n";
    RunSimulation(solver, outputDirectory, numberOfFrames, fps, format);
}
}  // namespace

int main(int argc, char* argv[])
{
    bool showHelp = false;
    int example = 1;
    size_t resolution = 64;
    int numberOfFrames = 120;
    double fps = 60.0;
    size_t particlesPerCell = 4;
    uint32_t seed = 0;
    unsigned int subSteps = 0;
    std::string outputDirectory = APP_NAME "_output";
    std::string format = "xyz";
    std::string integration = "auto";

    const auto parser =
        clara::Help(showHelp) |
        clara::Opt(example, "example")["-e"]["--example"](
            "example number: 1 snowball drop, 2 snowball smash "
            "(default is 1)") |
        clara::Opt(resolution, "resolution")["-r"]["--resolution"](
            "grid resolution per axis, at least 8 (default is 64)") |
        clara::Opt(numberOfFrames, "frames")["-f"]["--frames"](
            "total number of frames (default is 120)") |
        clara::Opt(fps,
                   "fps")["-p"]["--fps"]("frames per second (default is 60)") |
        clara::Opt(outputDirectory, "output")["-o"]["--output"](
            "output directory (default is " APP_NAME "_output)") |
        clara::Opt(format, "format")["-m"]["--format"](
            "particle output format: xyz or pos (default is xyz)") |
        clara::Opt(particlesPerCell, "count")["--particles-per-cell"](
            "random samples per grid cell (default is 4)") |
        clara::Opt(seed,
                   "seed")["--seed"]("random sampling seed (default is 0)") |
        clara::Opt(integration, "mode")["--integration"](
            "auto, explicit, or semi-implicit (default is auto)") |
        clara::Opt(subSteps, "count")["--substeps"](
            "fixed semi-implicit steps per frame; 0 uses adaptive "
            "steps (default is 0)");

    const auto result = parser.parse(clara::Args(argc, argv));
    if (!result)
    {
        std::cerr << "Error in command line: " << result.errorMessage() << '\n';
        return EXIT_FAILURE;
    }

    if (showHelp)
    {
        std::cout << ToString(parser) << '\n';
        return EXIT_SUCCESS;
    }

    if (example < 1 || example > 2)
    {
        std::cerr << "Example must be 1 or 2.\n";
        return EXIT_FAILURE;
    }
    if (resolution < 8)
    {
        std::cerr << "Resolution must be at least 8.\n";
        return EXIT_FAILURE;
    }
    if (numberOfFrames < 0)
    {
        std::cerr << "Frame count cannot be negative.\n";
        return EXIT_FAILURE;
    }
    if (particlesPerCell == 0)
    {
        std::cerr << "Particles per cell must be positive.\n";
        return EXIT_FAILURE;
    }
    if (!std::isfinite(fps) || fps <= 0.0)
    {
        std::cerr << "FPS must be positive and finite.\n";
        return EXIT_FAILURE;
    }
    if (outputDirectory.empty())
    {
        std::cerr << "Output directory cannot be empty.\n";
        return EXIT_FAILURE;
    }
    if (format != "xyz" && format != "pos")
    {
        std::cerr << "Format must be xyz or pos.\n";
        return EXIT_FAILURE;
    }
    if (integration != "auto" && integration != "explicit" &&
        integration != "semi-implicit")
    {
        std::cerr << "Integration must be auto, explicit, or semi-implicit.\n";
        return EXIT_FAILURE;
    }

    const bool useSemiImplicit = integration == "semi-implicit" ||
                                 (integration == "auto" && example == 2);

    try
    {
        if (example == 1)
        {
            RunExample1(resolution, numberOfFrames, fps, outputDirectory,
                        format, particlesPerCell, seed, useSemiImplicit,
                        subSteps);
        }
        else
        {
            RunExample2(resolution, numberOfFrames, fps, outputDirectory,
                        format, particlesPerCell, seed, useSemiImplicit,
                        subSteps);
        }
    }
    catch (const std::exception& exception)
    {
        std::cerr << "Simulation failed: " << exception.what() << '\n';
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
