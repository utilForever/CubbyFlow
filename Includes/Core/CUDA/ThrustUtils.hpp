// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_THRUST_UTILS_HPP
#define CUBBYFLOW_THRUST_UTILS_HPP

#ifdef CUBBYFLOW_USE_CUDA

#include <Core/CUDA/CUDAArray.hpp>

#if defined(__HIP__)
#include <Core/CUDA/cuda_to_hip.h>
#else
#include <cuda_runtime.h>
#endif

#include <thrust/device_ptr.h>

#include <algorithm>
#include <cstdint>

namespace CubbyFlow
{
template <typename T, size_t N, typename D>
thrust::device_ptr<T> thrustBegin(CUDAArrayBase<T, N, D>& arr)
{
    return thrust::device_ptr<T>(arr.data());
}

template <typename T, size_t N, typename D>
thrust::device_ptr<const T> thrustCBegin(const CUDAArrayBase<T, N, D>& arr)
{
    return thrust::device_ptr<const T>(arr.data());
}

template <typename T, size_t N, typename D>
thrust::device_ptr<T> thrustEnd(CUDAArrayBase<T, N, D>& arr)
{
    return thrust::device_ptr<T>(arr.data() + arr.Length());
}

template <typename T, size_t N, typename D>
thrust::device_ptr<const T> thrustCEnd(const CUDAArrayBase<T, N, D>& arr)
{
    return thrust::device_ptr<const T>(arr.data() + arr.Length());
}
}  // namespace CubbyFlow

#endif

#endif