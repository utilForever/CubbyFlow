// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_CUDA_SPH_KERNELS3_IMPL_HPP
#define CUBBYFLOW_CUDA_SPH_KERNELS3_IMPL_HPP

#ifdef CUBBYFLOW_USE_CUDA

#include <Core/CUDA/CUDAUtils.hpp>
#include <Core/Utils/Constants.hpp>

namespace CubbyFlow
{
inline CUBBYFLOW_CUDA_HOST_DEVICE CUDASPHStdKernel3::CUDASPHStdKernel3() : h(0), h2(0), h3(0), h5(0)
{
    // Do nothing
}

inline CUBBYFLOW_CUDA_HOST_DEVICE CUDASPHStdKernel3::CUDASPHStdKernel3(float kernelRadius)
    : h(kernelRadius), h2(h * h), h3(h2 * h), h5(h2 * h3)
{
    // Do nothing
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float CUDASPHStdKernel3::operator()(float distance) const
{
    if (distance * distance >= h2)
    {
        return 0.0f;
    }

    const float x = 1.0f - distance * distance / h2;
    return 315.0f / (64.0f * PI_FLOAT * h3) * x * x * x;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float CUDASPHStdKernel3::FirstDerivative(float distance) const
{
    if (distance >= h)
    {
        return 0.0f;
    }

    const float x = 1.0f - distance * distance / h2;
    return -945.0f / (32.0f * PI_FLOAT * h5) * distance * x * x;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float CUDASPHStdKernel3::SecondDerivative(float distance) const
{
    if (distance * distance >= h2)
    {
        return 0.0f;
    }

    const float x = distance * distance / h2;
    return 945.0f / (32.0f * PI_FLOAT * h5) * (1 - x) * (3 * x - 1);
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float4 CUDASPHStdKernel3::Gradient(const float4& point) const
{
    float dist = Length(point);

    if (dist > 0.0f)
    {
        return Gradient(dist, point / dist);
    }

    return make_float4(0, 0, 0, 0);
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float4 CUDASPHStdKernel3::Gradient(float distance,
                                          const float4& directionToCenter) const
{
    return -FirstDerivative(distance) * directionToCenter;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE CUDASPHSpikyKernel3::CUDASPHSpikyKernel3()
    : h(0), h2(0), h3(0), h4(0), h5(0)
{
    // Do nothing
}

inline CUBBYFLOW_CUDA_HOST_DEVICE CUDASPHSpikyKernel3::CUDASPHSpikyKernel3(float kernelRadius)
    : h(kernelRadius), h2(h * h), h3(h2 * h), h4(h2 * h2), h5(h3 * h2)
{
    // Do nothing
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float CUDASPHSpikyKernel3::operator()(float distance) const
{
    if (distance >= h)
    {
        return 0.0f;
    }

    const float x = 1.0f - distance / h;
    return 15.0f / (PI_FLOAT * h3) * x * x * x;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float CUDASPHSpikyKernel3::FirstDerivative(float distance) const
{
    if (distance >= h)
    {
        return 0.0f;
    }

    const float x = 1.0f - distance / h;
    return -45.0f / (PI_FLOAT * h4) * x * x;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float CUDASPHSpikyKernel3::SecondDerivative(float distance) const
{
    if (distance >= h)
    {
        return 0.0f;
    }

    const float x = 1.0f - distance / h;
    return 90.0f / (PI_FLOAT * h5) * x;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float4 CUDASPHSpikyKernel3::Gradient(const float4& point) const
{
    float dist = Length(point);

    if (dist > 0.0f)
    {
        return Gradient(dist, point / dist);
    }

    return make_float4(0, 0, 0, 0);
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float4 CUDASPHSpikyKernel3::Gradient(
    float distance, const float4& directionToCenter) const
{
    return -FirstDerivative(distance) * directionToCenter;
}
}  // namespace CubbyFlow

#endif

#endif