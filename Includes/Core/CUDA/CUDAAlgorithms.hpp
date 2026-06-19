// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_CUDA_ALGORITHMS_HPP
#define CUBBYFLOW_CUDA_ALGORITHMS_HPP

#ifdef CUBBYFLOW_USE_CUDA

#include <Core/CUDA/CUDAUtils.hpp>

#include <stdio.h>

namespace CubbyFlow
{
#if defined(__CUDACC__) || defined(__HIPCC__)

template <typename T>
__global__ void CUDAFillKernel(T* dst, size_t n, T val)
{
    size_t i = blockIdx.x * blockDim.x + threadIdx.x;

    if (i < n)
    {
        dst[i] = val;
    }
}

template <typename T>
void CUDAFill(T* dst, size_t n, const T& val)
{
    if (n == 0)
    {
        return;
    }

    unsigned int numBlocks, numThreads;
    CUDAComputeGridSize((unsigned int)n, 256, numBlocks, numThreads);
    CUDAFillKernel<<<numBlocks, numThreads>>>(dst, n, val);

    CUBBYFLOW_CUDA_CHECK_LAST_ERROR("Failed executing CUDAFillKernel");
}

#endif

template <typename T>
__host__ __device__ inline void CUDASwap(T& a, T& b)
{
    T tmp = a;
    a = b;
    b = tmp;
}

// TODO: Rename it to something else to indicate this is a memory copy.
// TODO: Also, having CUDA prefix may collide with CUDA API.
template <typename T>
void CUDACopy(const T* src, size_t n, T* dst,
              cudaMemcpyKind kind = cudaMemcpyDeviceToDevice)
{
    CUBBYFLOW_CUDA_CHECK(cudaMemcpy(dst, src, n * sizeof(T), kind));
}

template <typename T>
void CUDACopyDeviceToDevice(const T* src, size_t n, T* dst)
{
    CUDACopy(src, n, dst, cudaMemcpyDeviceToDevice);
}

template <typename T>
void CUDACopyHostToDevice(const T* src, size_t n, T* dst)
{
    CUDACopy(src, n, dst, cudaMemcpyHostToDevice);
}

template <typename T>
void CUDACopyDeviceToHost(const T* src, size_t n, T* dst)
{
    CUDACopy(src, n, dst, cudaMemcpyDeviceToHost);
}
}  // namespace CubbyFlow

#endif

#endif