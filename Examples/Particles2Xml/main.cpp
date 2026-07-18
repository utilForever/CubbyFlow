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
#include <Core/Matrix/Matrix.hpp>
#include <Core/Utils/Serialization.hpp>

#include <clara.hpp>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace CubbyFlow;

void PrintInfo(size_t numberOfParticles)
{
    printf("Number of particles: %zu\n", numberOfParticles);
}

void ParticlesToXML(const Array1<Vector3D>& positions,
                    const std::string& xmlFileName)
{
    PrintInfo(positions.Length());

    std::ofstream file(xmlFileName.c_str());
    if (file)
    {
        printf("Writing %s...\n", xmlFileName.c_str());

        file << "<scene version=\"0.5.0\">";

        for (const auto& pos : positions)
        {
            file << "<shape type=\"instance\">";
            file << "<ref id=\"spheres\"/>";
            file << "<transform name=\"toWorld\">";

            char buffer[64];
            snprintf(buffer, sizeof(buffer),
                     "<translate x=\"%f\" y=\"%f\" z=\"%f\"/>", pos.x, pos.y,
                     pos.z);
            file << buffer;

            file << "</transform>";
            file << "</shape>";
        }

        file << "</scene>";

        file.close();
    }
    else
    {
        printf("Cannot write file %s.\n", xmlFileName.c_str());
        exit(EXIT_FAILURE);
    }
}

int main(int argc, char* argv[])
{
    bool showHelp = false;

    std::string inputFileName;
    std::string outputFileName;

    // Parsing
    auto parser =
        clara::Help(showHelp) |
        clara::Opt(inputFileName, "inputFileName")["-i"]["--input"](
            "input particle position file name") |
        clara::Opt(outputFileName,
                   "outputFileName")["-o"]["--output"]("output xml file name");

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

    if (inputFileName.empty() || outputFileName.empty())
    {
        std::cout << ToString(parser) << '\n';
        exit(EXIT_FAILURE);
    }

    // Read particle positions
    Array1<Vector3D> positions;
    std::ifstream positionFile(inputFileName.c_str(), std::ifstream::binary);
    if (positionFile)
    {
        const std::vector<uint8_t> buffer(
            (std::istreambuf_iterator<char>(positionFile)),
            (std::istreambuf_iterator<char>()));
        Deserialize(buffer, &positions);
        positionFile.close();
    }
    else
    {
        printf("Cannot read file %s.\n", inputFileName.c_str());
        exit(EXIT_FAILURE);
    }

    // Run marching cube and save it to the disk
    ParticlesToXML(positions, outputFileName);

    return EXIT_SUCCESS;
}