// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_CUDA_UTILS_HPP
#define CUBBYFLOW_CUDA_UTILS_HPP

#ifdef CUBBYFLOW_USE_CUDA

#include <Core/Utils/Macros.hpp>

#if defined(__HIP__)
#include <Core/CUDA/cuda_to_hip.h>
#else
#include <cuda_runtime.h>
#endif

#include <algorithm>
#include <cstdint>

namespace CubbyFlow
{
inline CUBBYFLOW_CUDA_HOST_DEVICE unsigned int CUDADivRoundUp(unsigned int a,
                                                              unsigned int b)
{
    return (a % b != 0) ? (a / b + 1) : (a / b);
}

inline CUBBYFLOW_CUDA_HOST void CUDAComputeGridSize(unsigned int n,
                                                    unsigned int blockSize,
                                                    unsigned int& numBlocks,
                                                    unsigned int& numThreads)
{
    numThreads = std::min(blockSize, n);
    numBlocks = CUDADivRoundUp(n, numThreads);
}

// HIP's vector types (HIP_vector_type) already provide arithmetic, compound
// assignment, and equality operators for floatN, so these CUDA-side definitions
// would be ambiguous on ROCm. Only the named helpers below (Dot, Length, To*)
// are kept on both backends.
#if !defined(__HIP__)
inline CUBBYFLOW_CUDA_HOST_DEVICE float2 operator+(float2 a, float2 b)
{
    return make_float2(a.x + b.x, a.y + b.y);
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float3 operator+(float3 a, float3 b)
{
    return make_float3(a.x + b.x, a.y + b.y, a.z + b.z);
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float4 operator+(float4 a, float4 b)
{
    return make_float4(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float2 operator-(float2 a, float2 b)
{
    return make_float2(a.x - b.x, a.y - b.y);
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float3 operator-(float3 a, float3 b)
{
    return make_float3(a.x - b.x, a.y - b.y, a.z - b.z);
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float4 operator-(float4 a, float4 b)
{
    return make_float4(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w);
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float2 operator*(float a, float2 b)
{
    return make_float2(a * b.x, a * b.y);
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float3 operator*(float a, float3 b)
{
    return make_float3(a * b.x, a * b.y, a * b.z);
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float4 operator*(float a, float4 b)
{
    return make_float4(a * b.x, a * b.y, a * b.z, a * b.w);
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float2 operator*(float2 a, float b)
{
    return make_float2(a.x * b, a.y * b);
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float3 operator*(float3 a, float b)
{
    return make_float3(a.x * b, a.y * b, a.z * b);
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float4 operator*(float4 a, float b)
{
    return make_float4(a.x * b, a.y * b, a.z * b, a.w * b);
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float2 operator*(float2 a, float2 b)
{
    return make_float2(a.x * b.x, a.y * b.y);
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float3 operator*(float3 a, float3 b)
{
    return make_float3(a.x * b.x, a.y * b.y, a.z * b.z);
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float4 operator*(float4 a, float4 b)
{
    return make_float4(a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w);
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float2 operator/(float a, float2 b)
{
    return make_float2(a / b.x, a / b.y);
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float3 operator/(float a, float3 b)
{
    return make_float3(a / b.x, a / b.y, a / b.z);
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float4 operator/(float a, float4 b)
{
    return make_float4(a / b.x, a / b.y, a / b.z, a / b.w);
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float2 operator/(float2 a, float b)
{
    return make_float2(a.x / b, a.y / b);
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float3 operator/(float3 a, float b)
{
    return make_float3(a.x / b, a.y / b, a.z / b);
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float4 operator/(float4 a, float b)
{
    return make_float4(a.x / b, a.y / b, a.z / b, a.w / b);
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float2 operator/(float2 a, float2 b)
{
    return make_float2(a.x / b.x, a.y / b.y);
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float3 operator/(float3 a, float3 b)
{
    return make_float3(a.x / b.x, a.y / b.y, a.z / b.z);
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float4 operator/(float4 a, float4 b)
{
    return make_float4(a.x / b.x, a.y / b.y, a.z / b.z, a.w / b.w);
}

inline CUBBYFLOW_CUDA_HOST_DEVICE void operator+=(float2& a, float b)
{
    a.x += b;
    a.y += b;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE void operator+=(float3& a, float b)
{
    a.x += b;
    a.y += b;
    a.z += b;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE void operator+=(float4& a, float b)
{
    a.x += b;
    a.y += b;
    a.z += b;
    a.w += b;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE void operator+=(float2& a, float2 b)
{
    a.x += b.x;
    a.y += b.y;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE void operator+=(float3& a, float3 b)
{
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE void operator+=(float4& a, float4 b)
{
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    a.w += b.w;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE void operator-=(float2& a, float b)
{
    a.x -= b;
    a.y -= b;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE void operator-=(float3& a, float b)
{
    a.x -= b;
    a.y -= b;
    a.z -= b;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE void operator-=(float4& a, float b)
{
    a.x -= b;
    a.y -= b;
    a.z -= b;
    a.w -= b;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE void operator-=(float2& a, float2 b)
{
    a.x -= b.x;
    a.y -= b.y;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE void operator-=(float3& a, float3 b)
{
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE void operator-=(float4& a, float4 b)
{
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
    a.w -= b.w;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE void operator*=(float2& a, float b)
{
    a.x *= b;
    a.y *= b;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE void operator*=(float3& a, float b)
{
    a.x *= b;
    a.y *= b;
    a.z *= b;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE void operator*=(float4& a, float b)
{
    a.x *= b;
    a.y *= b;
    a.z *= b;
    a.w *= b;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE void operator*=(float2& a, float2 b)
{
    a.x *= b.x;
    a.y *= b.y;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE void operator*=(float3& a, float4 b)
{
    a.x *= b.x;
    a.y *= b.y;
    a.z *= b.z;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE void operator*=(float4& a, float4 b)
{
    a.x *= b.x;
    a.y *= b.y;
    a.z *= b.z;
    a.w *= b.w;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE void operator/=(float2& a, float b)
{
    a.x /= b;
    a.y /= b;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE void operator/=(float3& a, float b)
{
    a.x /= b;
    a.y /= b;
    a.z /= b;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE void operator/=(float4& a, float b)
{
    a.x /= b;
    a.y /= b;
    a.z /= b;
    a.w /= b;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE void operator/=(float2& a, float4 b)
{
    a.x /= b.x;
    a.y /= b.y;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE void operator/=(float3& a, float4 b)
{
    a.x /= b.x;
    a.y /= b.y;
    a.z /= b.z;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE void operator/=(float4& a, float4 b)
{
    a.x /= b.x;
    a.y /= b.y;
    a.z /= b.z;
    a.w /= b.w;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE bool operator==(float2 a, float2 b)
{
    return std::abs(a.x - b.x) < 1e-6f && std::abs(a.y - b.y) < 1e-6f;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE bool operator==(float3 a, float3 b)
{
    return std::abs(a.x - b.x) < 1e-6f && std::abs(a.y - b.y) < 1e-6f &&
           std::abs(a.z - b.z) < 1e-6f;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE bool operator==(float4 a, float4 b)
{
    return std::abs(a.x - b.x) < 1e-6f && std::abs(a.y - b.y) < 1e-6f &&
           std::abs(a.z - b.z) < 1e-6f && std::abs(a.w - b.w) < 1e-6f;
}
#endif  // !__HIP__

inline CUBBYFLOW_CUDA_HOST_DEVICE float Dot(float2 a, float2 b)
{
    return a.x * b.x + a.y * b.y;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float Dot(float3 a, float3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float Dot(float4 a, float4 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float LengthSquared(float2 v)
{
    return Dot(v, v);
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float LengthSquared(float3 v)
{
    return Dot(v, v);
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float LengthSquared(float4 v)
{
    return Dot(v, v);
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float Length(float2 v)
{
    return sqrtf(LengthSquared(v));
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float Length(float3 v)
{
    return sqrtf(LengthSquared(v));
}

inline CUBBYFLOW_CUDA_HOST_DEVICE float Length(float4 v)
{
    return sqrtf(LengthSquared(v));
}

template <typename VectorType>
inline CUBBYFLOW_CUDA_HOST_DEVICE float2 ToFloat2(const VectorType& vec)
{
    return make_float2(vec.x, vec.y);
}

template <typename VectorType>
inline CUBBYFLOW_CUDA_HOST_DEVICE float3 ToFloat3(const VectorType& vec)
{
    return make_float3(vec.x, vec.y, vec.z);
}

template <typename VectorType>
inline CUBBYFLOW_CUDA_HOST_DEVICE float4 ToFloat4(const VectorType& vec,
                                                  float w)
{
    return make_float4(vec.x, vec.y, vec.z, w);
}

template <typename VectorType>
inline CUBBYFLOW_CUDA_HOST_DEVICE float4 ToFloat4(const VectorType& vec)
{
    return make_float4(vec.x, vec.y, vec.z, vec.w);
}

template <typename SizeType>
inline CUBBYFLOW_CUDA_HOST_DEVICE int2 ToInt2(const SizeType& size)
{
    return make_int2(static_cast<int>(size.x), static_cast<int>(size.y));
}

template <typename SizeType>
inline CUBBYFLOW_CUDA_HOST_DEVICE int3 ToInt3(const SizeType& size)
{
    return make_int3(static_cast<int>(size.x), static_cast<int>(size.y),
                     static_cast<int>(size.z));
}

template <typename SizeType>
inline CUBBYFLOW_CUDA_HOST_DEVICE uint2 ToUInt2(const SizeType& size)
{
    return make_uint2(static_cast<uint32_t>(size.x),
                      static_cast<uint32_t>(size.y));
}

template <typename SizeType>
inline CUBBYFLOW_CUDA_HOST_DEVICE uint3 ToUInt3(const SizeType& size)
{
    return make_uint3(static_cast<uint32_t>(size.x),
                      static_cast<uint32_t>(size.y),
                      static_cast<uint32_t>(size.z));
}
}  // namespace CubbyFlow

#endif

#endif