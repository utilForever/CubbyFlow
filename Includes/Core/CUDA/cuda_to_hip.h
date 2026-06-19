// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Copyright (c) 2026 Advanced Micro Devices, Inc.
//
// HIP/ROCm compatibility shim. Force-included on every HIP translation unit so
// the project's CUDA spelling (runtime API, vector types, and the device-vs-host
// preprocessor idioms) maps onto HIP without rewriting the .cu/.hpp sources.
//
// \author Jeff Daily <jeff.daily@amd.com>

#ifndef CUBBYFLOW_CUDA_TO_HIP_H
#define CUBBYFLOW_CUDA_TO_HIP_H

#if defined(CUBBYFLOW_USE_CUDA) && defined(__HIP__)

#include <hip/hip_runtime.h>

// hipcc defines __HIPCC__ on both compile passes but NOT __CUDACC__. The project
// gates its kernel definitions and host/device attribute macros on __CUDACC__;
// those guards are extended in-place to also accept __HIPCC__. __CUDACC__ itself
// is deliberately left undefined here: rocThrust keys its backend selection on
// __CUDACC__ (thrust/detail/config/compiler.h) and would otherwise fall back to
// its CUDA backend (pulling a CUDA-only CUB header) instead of the HIP backend.

// __CUDA_ARCH__ is undefined under HIP; HIP signals the device compile pass with
// __HIP_DEVICE_COMPILE__. The project selects between device (T&) and host
// (copy-back wrapper) return types for accessors via #ifdef __CUDA_ARCH__. clang
// compiles host and device in separate passes like nvcc, so defining __CUDA_ARCH__
// only in the device pass makes that per-pass selection resolve correctly.
#if defined(__HIP_DEVICE_COMPILE__) && __HIP_DEVICE_COMPILE__ && \
    !defined(__CUDA_ARCH__)
#define __CUDA_ARCH__ 1
#endif

// Runtime API: the project uses only a handful of symbols.
#define cudaError_t hipError_t
#define cudaSuccess hipSuccess
#define cudaGetLastError hipGetLastError
#define cudaGetErrorString hipGetErrorString
#define cudaDeviceReset hipDeviceReset
#define cudaDeviceSynchronize hipDeviceSynchronize
#define cudaMalloc hipMalloc
#define cudaFree hipFree
#define cudaMemcpy hipMemcpy
#define cudaMemcpyKind hipMemcpyKind
#define cudaMemcpyHostToDevice hipMemcpyHostToDevice
#define cudaMemcpyDeviceToHost hipMemcpyDeviceToHost
#define cudaMemcpyDeviceToDevice hipMemcpyDeviceToDevice

#endif  // CUBBYFLOW_USE_CUDA && __HIP__

#endif  // CUBBYFLOW_CUDA_TO_HIP_H
