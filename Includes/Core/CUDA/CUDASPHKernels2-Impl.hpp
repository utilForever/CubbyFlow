// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_CUDA_SPH_KERNELS2_IMPL_HPP
#define CUBBYFLOW_CUDA_SPH_KERNELS2_IMPL_HPP

#ifdef CUBBYFLOW_USE_CUDA

#include <Core/CUDA/CUDAUtils.hpp>
#include <Core/Utils/Constants.hpp>

namespace CubbyFlow
{
inline CUBBYFLOW_CUDA_HOST_DEVICE CUDASPHStdKernel2::CUDASPHStdKernel2() : h(0), h2(0), h3(0), h4(0)
{
    // Do nothing
}

inline CUBBYFLOW_CUDA_HOST_DEVICE CUDASPHStdKernel2::CUDASPHStdKernel2(float kernelRadius)
    : h(kernelRadius), h2(h * h), h3(h2 * h), h4(h2 * h2)
{
    // Do nothing
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float CUDASPHStdKernel2::operator()(float distance) const
{
    const float distanceSquared = distance * distance;

    if (distanceSquared >= h2)
    {
        return 0.0f;
    }

    const float x = 1.0f - distanceSquared / h2;
    return 4.0f / (PI_FLOAT * h2) * x * x * x;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float CUDASPHStdKernel2::FirstDerivative(float distance) const
{
    if (distance >= h)
    {
        return 0.0f;
    }

    const float x = 1.0f - distance * distance / h2;
    return -24.0f * distance / (PI_FLOAT * h4) * x * x;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float CUDASPHStdKernel2::SecondDerivative(float distance) const
{
    float distanceSquared = distance * distance;

    if (distanceSquared >= h2)
    {
        return 0.0f;
    }

    const float x = distanceSquared / h2;
    return 24.0f / (PI_FLOAT * h4) * (1 - x) * (5 * x - 1);
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float2 CUDASPHStdKernel2::Gradient(const float2& point) const
{
    float dist = Length(point);

    if (dist > 0.0f)
    {
        return Gradient(dist, point / dist);
    }

    return make_float2(0, 0);
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float2 CUDASPHStdKernel2::Gradient(float distance,
                                          const float2& directionToCenter) const
{
    return -FirstDerivative(distance) * directionToCenter;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE CUDASPHSpikyKernel2::CUDASPHSpikyKernel2()
    : h(0), h2(0), h3(0), h4(0), h5(0)
{
    // Do nothing
}

inline CUBBYFLOW_CUDA_HOST_DEVICE CUDASPHSpikyKernel2::CUDASPHSpikyKernel2(float kernelRadius)
    : h(kernelRadius), h2(h * h), h3(h2 * h), h4(h2 * h2), h5(h3 * h2)
{
    // Do nothing
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float CUDASPHSpikyKernel2::operator()(float distance) const
{
    if (distance >= h)
    {
        return 0.0f;
    }

    const float x = 1.0f - distance / h;
    return 10.0f / (PI_FLOAT * h2) * x * x * x;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float CUDASPHSpikyKernel2::FirstDerivative(float distance) const
{
    if (distance >= h)
    {
        return 0.0f;
    }

    const float x = 1.0f - distance / h;
    return -30.0f / (PI_FLOAT * h3) * x * x;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float CUDASPHSpikyKernel2::SecondDerivative(float distance) const
{
    if (distance >= h)
    {
        return 0.0f;
    }

    const float x = 1.0f - distance / h;
    return 60.0f / (PI_FLOAT * h4) * x;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float2 CUDASPHSpikyKernel2::Gradient(const float2& point) const
{
    float dist = Length(point);

    if (dist > 0.0f)
    {
        return Gradient(dist, point / dist);
    }

    return make_float2(0, 0);
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float2 CUDASPHSpikyKernel2::Gradient(
    float distance, const float2& directionToCenter) const
{
    return -FirstDerivative(distance) * directionToCenter;
}
}  // namespace CubbyFlow

#endif

#endif