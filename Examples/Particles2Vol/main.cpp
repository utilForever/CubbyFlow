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

#include "MitsubaVolume.hpp"

#include <Core/Utils/Serialization.hpp>

#include <clara.hpp>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

using namespace CubbyFlow;

namespace
{
Array1<Vector3D> ReadPositions(const std::filesystem::path& path)
{
    std::ifstream file(path, std::ios::binary);
    if (!file)
    {
        throw std::runtime_error("Cannot read input file: " + path.string());
    }

    const std::vector<uint8_t> bytes{ std::istreambuf_iterator<char>(file),
                                      std::istreambuf_iterator<char>() };
    Array1<Vector3D> positions;
    Deserialize(bytes, &positions);
    return positions;
}

void WriteBytes(const std::filesystem::path& path,
                const std::vector<uint8_t>& bytes)
{
    std::ofstream file(path, std::ios::binary);
    if (!file)
    {
        throw std::runtime_error("Cannot open output file: " + path.string());
    }
    file.write(reinterpret_cast<const char*>(bytes.data()),
               static_cast<std::streamsize>(bytes.size()));
    if (!file)
    {
        throw std::runtime_error("Cannot write output file: " + path.string());
    }
}
}  // namespace

int main(int argc, char* argv[])
{
    bool showHelp = false;
    std::string inputFileName;
    std::string outputFileName;
    size_t resolution = 160;
    size_t sourceResolution = 0;
    double particlesPerCell = 4.0;

    const auto parser =
        clara::Help(showHelp) |
        clara::Opt(inputFileName, "input")["-i"]["--input"](
            "serialized particle position (.pos) file") |
        clara::Opt(outputFileName,
                   "output")["-o"]["--output"]("Mitsuba VOL v3 output file") |
        clara::Opt(resolution, "resolution")["-r"]["--resolution"](
            "density grid cell resolution (default is 160)") |
        clara::Opt(sourceResolution, "resolution")["--source-resolution"](
            "simulation grid resolution (default matches output)") |
        clara::Opt(particlesPerCell, "count")["--particles-per-cell"](
            "particle samples per simulation cell (default is 4)");

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
    if (inputFileName.empty() || outputFileName.empty())
    {
        std::cout << ToString(parser) << '\n';
        return EXIT_FAILURE;
    }
    if (resolution == 0 || !std::isfinite(particlesPerCell) ||
        particlesPerCell <= 0.0)
    {
        std::cerr << "Resolution and particles per cell must be positive.\n";
        return EXIT_FAILURE;
    }

    try
    {
        if (sourceResolution == 0)
        {
            sourceResolution = resolution;
        }
        const auto positions = ReadPositions(inputFileName);
        const auto volume = RasterizeMitsubaDensity(
            positions, resolution, sourceResolution, particlesPerCell,
            { { 0.0, 0.0, 0.0 }, { 1.0, 1.0, 1.0 } });
        WriteBytes(outputFileName, EncodeMitsubaVolume(volume));
        std::cout << "Rasterized " << positions.Length() << " particles to "
                  << volume.resolution.x << " x " << volume.resolution.y
                  << " x " << volume.resolution.z << " voxels.\n";
    }
    catch (const std::exception& exception)
    {
        std::cerr << "Conversion failed: " << exception.what() << '\n';
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
