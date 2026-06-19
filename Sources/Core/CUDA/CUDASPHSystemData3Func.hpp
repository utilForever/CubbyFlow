// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_CUDA_SPH_SYSTEM_DATA3_FUNC_HPP
#define CUBBYFLOW_CUDA_SPH_SYSTEM_DATA3_FUNC_HPP

#ifdef CUBBYFLOW_USE_CUDA

#include "CUDAParticleSystemData3Func.hpp"

#include <Core/CUDA/CUDASPHKernels3.hpp>

#if defined(__HIP__)
#include <Core/CUDA/cuda_to_hip.h>
#else
#include <cuda_runtime.h>
#endif

namespace CubbyFlow
{
class UpdateDensity
{
 public:
    CUBBYFLOW_CUDA_HOST_DEVICE UpdateDensity(float h, float mass,
                                             float* density)
        : m_mass(mass), m_density(density), m_stdKernel(h)
    {
        // Do nothing
    }

    inline CUBBYFLOW_CUDA_DEVICE void operator()(uint32_t i, float4 o, uint32_t,
                                                 float4 p)
    {
        float dist = Length(o - p);
        m_density[i] += m_mass * m_stdKernel(dist);
    }

 private:
    float m_mass;
    float* m_density;
    CUDASPHStdKernel3 m_stdKernel;
};

class BuildNeighborListsAndUpdateDensitiesFunc
{
 public:
    inline CUBBYFLOW_CUDA_HOST_DEVICE BuildNeighborListsAndUpdateDensitiesFunc(
        const uint32_t* neighborStarts, const uint32_t* neighborEnds, float h,
        float mass, uint32_t* neighborLists, float* densities)
        : m_neighborStarts(neighborStarts),
          m_neighborEnds(neighborEnds),
          m_mass(mass),
          m_neighborLists(neighborLists),
          m_densities(densities),
          m_stdKernel(h)
    {
        // Do nothing
    }

    template <typename Index>
    inline CUBBYFLOW_CUDA_HOST_DEVICE void operator()(size_t i, Index j,
                                                      Index cnt, float d2)
    {
        m_densities[i] += m_mass * m_stdKernel(sqrt(d2));

        if (i != j)
        {
            m_neighborLists[m_neighborStarts[i] + cnt] = j;
        }
    }

 private:
    const uint32_t* m_neighborStarts;
    const uint32_t* m_neighborEnds;
    float m_mass;
    uint32_t* m_neighborLists;
    float* m_densities;
    CUDASPHStdKernel3 m_stdKernel;
};
}  // namespace CubbyFlow

#endif

#endif