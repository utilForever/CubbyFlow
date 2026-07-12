// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include "CUDAPCISPHSolver3Example.hpp"

#include <Core/CUDA/CUDAArray.hpp>
#include <Core/CUDA/CUDAPCISPHSolver3.hpp>
#include <Core/PointGenerator/GridPointGenerator3.hpp>
#include <Core/Utils/Serialization.hpp>

#if defined(__HIP__)
#include <Core/CUDA/cuda_to_hip.h>
#else
#include <cuda_runtime.h>
#endif
#include <pystring/pystring.h>

#include <fstream>

using namespace CubbyFlow;

void SaveParticleAsPos(const CUDASPHSystemData3* particles,
                       const std::string& rootDir, int frameCnt)
{
    Array1<Vector3D> positions(particles->NumberOfParticles());
    ConstCUDAArrayView1<float4> cudaPositions = particles->Positions();
    for (std::size_t i = 0; i < particles->NumberOfParticles(); ++i)
    {
        const float4 position = cudaPositions.At(i);
        positions(i) = Vector3D{ static_cast<double>(position.x),
                                 static_cast<double>(position.y),
                                 static_cast<double>(position.z) };
    }
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

void SaveParticleAsXYZ(const CUDASPHSystemData3* particles,
                       const std::string& rootDir, int frameCnt)
{
    Array1<Vector3D> positions(particles->NumberOfParticles());
    ConstCUDAArrayView1<float4> cudaPositions = particles->Positions();
    for (std::size_t i = 0; i < particles->NumberOfParticles(); ++i)
    {
        const float4 position = cudaPositions.At(i);
        positions(i) = Vector3D{ static_cast<double>(position.x),
                                 static_cast<double>(position.y),
                                 static_cast<double>(position.z) };
    }
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

void RunSimulation(const std::string& rootDir,
                   const CUDAPCISPHSolver3Ptr& solver, int numberOfFrames,
                   const std::string& format, double fps)
{
    const auto particles = solver->SPHSystemData();

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

void CUDAPCISPHSolver3Example::RunExample(const std::string& rootDir,
                                          float targetSpacing,
                                          int numberOfFrames,
                                          const std::string& format, double fps)
{
    // Setup solver
    auto solver = CUDAPCISPHSolver3::Builder().MakeShared();
    solver->SetDragCoefficient(0.0f);
    solver->SetRestitutionCoefficient(0.0f);
    solver->SetViscosityCoefficient(0.1f);
    solver->SetPseudoViscosityCoefficient(10.0f);
    solver->SetIsUsingFixedSubTimeSteps(true);
    solver->SetNumberOfFixedSubTimeSteps(1);

    auto particles = solver->SPHSystemData();
    particles->SetTargetSpacing(targetSpacing);
    particles->SetRelativeKernelRadius(1.8f);

    // Seed particles
    BoundingBox3D vol(Vector3D(), Vector3D(0.5, 0.5, 0.5));
    vol.Expand(-targetSpacing);
    Array1<Vector3D> rawPoints;
    GridPointGenerator3 generator;
    generator.Generate(vol, targetSpacing, &rawPoints);
    Array1<float4> hostData(rawPoints.Length());
    for (std::size_t i = 0; i < rawPoints.Length(); ++i)
    {
        Vector3F rp = rawPoints[i].CastTo<float>();
        hostData[i] = make_float4(rp[0], rp[1], rp[2], 0.0f);
    }
    CUDAArray1<float4> deviceData(hostData);
    particles->AddParticles(deviceData);
    printf("Number of particles: %zu\n", particles->NumberOfParticles());

    // Run simulation
    RunSimulation(rootDir, solver, numberOfFrames, format, fps);
}
