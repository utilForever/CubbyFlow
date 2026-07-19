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

#include <Core/Geometry/MarchingCubes.hpp>
#include <Core/Geometry/TriangleMesh3.hpp>
#include <Core/Geometry/TriangleMeshToSDF.hpp>
#include <Core/Grid/VertexCenteredScalarGrid.hpp>

#include <clara.hpp>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace CubbyFlow;

void SaveTriangleMeshData(const TriangleMesh3& data,
                          const std::string& fileName)
{
    std::ofstream file(fileName.c_str());
    if (file)
    {
        data.WriteObj(&file);
        file.close();
    }
}

int main(int argc, char* argv[])
{
    bool showHelp = false;

    std::string inputFileName;
    std::string outputFileName;
    size_t resX = 100;
    double marginScale = 0.2;

    // Parsing
    auto parser =
        clara::Help(showHelp) |
        clara::Opt(inputFileName,
                   "inputFileName")["-i"]["--input"]("input obj file name") |
        clara::Opt(outputFileName,
                   "outputFileName")["-o"]["--output"]("output sdf file name") |
        clara::Opt(resX, "resX")["-r"]["--resx"](
            "grid resolution in x-axis (default is 100)") |
        clara::Opt(marginScale, "marginScale")["-m"]["--margin"](
            "margin scale around the sdf (default is 0.2)");

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

    TriangleMesh3 triMesh;

    std::ifstream objFile(inputFileName.c_str());
    if (objFile)
    {
        printf("Reading obj file %s\n", inputFileName.c_str());
        [[maybe_unused]] bool isLoaded = triMesh.ReadObj(&objFile);
        objFile.close();
    }
    else
    {
        fprintf(stderr, "Failed to open file %s\n", inputFileName.c_str());
        exit(EXIT_FAILURE);
    }

    BoundingBox3D box = triMesh.GetBoundingBox();
    const Vector3D scale(box.Width(), box.Height(), box.Depth());
    box.lowerCorner -= marginScale * scale;
    box.upperCorner += marginScale * scale;

    const size_t resolutionY =
        static_cast<size_t>(std::ceil(resX * box.Height() / box.Width()));
    const size_t resolutionZ =
        static_cast<size_t>(std::ceil(resX * box.Depth() / box.Width()));

    printf("Vertex-centered grid size: %zu x %zu x %zu\n", resX, resolutionY,
           resolutionZ);

    const double dx = box.Width() / resX;

    VertexCenteredScalarGrid3 grid(
        { resX, resolutionY, resolutionZ }, { dx, dx, dx },
        { box.lowerCorner.x, box.lowerCorner.y, box.lowerCorner.z });

    const BoundingBox3D domain = grid.GetBoundingBox();
    printf("Domain size: [%f, %f, %f] x [%f, %f, %f]\n", domain.lowerCorner.x,
           domain.lowerCorner.y, domain.lowerCorner.z, domain.upperCorner.x,
           domain.upperCorner.y, domain.upperCorner.z);
    printf("Generating SDF...");

    TriangleMeshToSDF(triMesh, &grid);

    printf("done\n");

    std::ofstream sdfFile(outputFileName.c_str(), std::ofstream::binary);
    if (sdfFile)
    {
        printf("Writing to vertex-centered grid %s\n", outputFileName.c_str());

        std::vector<uint8_t> buffer;
        grid.Serialize(&buffer);
        sdfFile.write(reinterpret_cast<char*>(buffer.data()), buffer.size());
        sdfFile.close();
    }
    else
    {
        fprintf(stderr, "Failed to write file %s\n", outputFileName.c_str());
        exit(EXIT_FAILURE);
    }

    TriangleMesh3 triMesh2;
    MarchingCubes(grid.DataView(), grid.GridSpacing(), grid.Origin(), &triMesh2,
                  0, DIRECTION_ALL);

    SaveTriangleMeshData(triMesh2, outputFileName + "_previz.obj");

    return EXIT_SUCCESS;
}