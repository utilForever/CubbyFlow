// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_CUDA_STD_VECTOR_IMPL_HPP
#define CUBBYFLOW_CUDA_STD_VECTOR_IMPL_HPP

#ifdef CUBBYFLOW_USE_CUDA

#include <Core/CUDA/CUDAAlgorithms.hpp>

#include <algorithm>

namespace CubbyFlow
{
template <typename T>
CUDAStdVector<T>::CUDAStdVector(size_t n, const ValueType& initVal)
{
    ResizeUninitialized(n);
    CUDAFill(m_ptr, n, initVal);
}

template <typename T>
template <typename A>
CUDAStdVector<T>::CUDAStdVector(const std::vector<T, A>& other)
    : CUDAStdVector{ other.size() }
{
    CUDACopyHostToDevice(other.data(), m_size, m_ptr);
}

template <typename T>
CUDAStdVector<T>::CUDAStdVector(const CUDAStdVector& other)
    : CUDAStdVector{ other.Size() }
{
    CUDACopyDeviceToDevice(other.m_ptr, m_size, m_ptr);
}

template <typename T>
CUDAStdVector<T>::CUDAStdVector(CUDAStdVector&& other) noexcept
{
    *this = std::move(other);
}

template <typename T>
CUDAStdVector<T>::~CUDAStdVector()
{
    Clear();
}

template <typename T>
template <typename A>
CUDAStdVector<T>& CUDAStdVector<T>::operator=(const std::vector<T, A>& other)
{
    CopyFrom(other);
    return *this;
}

template <typename T>
CUDAStdVector<T>& CUDAStdVector<T>::operator=(const CUDAStdVector& other)
{
    CopyFrom(other);
    return *this;
}

template <typename T>
CUDAStdVector<T>& CUDAStdVector<T>::operator=(CUDAStdVector&& other) noexcept
{
    Clear();

    Swap(other);
    return *this;
}

template <typename T>
typename CUDAStdVector<T>::Pointer CUDAStdVector<T>::data()
{
    return m_ptr;
}

template <typename T>
typename CUDAStdVector<T>::ConstPointer CUDAStdVector<T>::data() const
{
    return m_ptr;
}

template <typename T>
size_t CUDAStdVector<T>::Size() const
{
    return m_size;
}

#if defined(__HIP__) || defined(__CUDA_ARCH__)
template <typename T>
__device__ typename CUDAStdVector<T>::Reference CUDAStdVector<T>::At(size_t i)
{
    return m_ptr[i];
}

template <typename T>
__device__ typename CUDAStdVector<T>::ConstReference CUDAStdVector<T>::At(
    size_t i) const
{
    return m_ptr[i];
}
#endif
#if defined(__HIP__) || !defined(__CUDA_ARCH__)
template <typename T>
CUBBYFLOW_CUDA_HOST typename CUDAStdVector<T>::ReferenceType
CUDAStdVector<T>::At(size_t i)
{
    ReferenceType r(m_ptr + i);
    return r;
}

template <typename T>
CUBBYFLOW_CUDA_HOST T CUDAStdVector<T>::At(size_t i) const
{
    T tmp;
    CUDACopyDeviceToHost(m_ptr + i, 1, &tmp);
    return tmp;
}
#endif

template <typename T>
void CUDAStdVector<T>::Clear()
{
    if (m_ptr != nullptr)
    {
        CUBBYFLOW_CUDA_CHECK(cudaFree(m_ptr));
    }

    m_ptr = nullptr;
    m_size = 0;
}

template <typename T>
void CUDAStdVector<T>::Fill(const ValueType& val)
{
    CUDAFill(m_ptr, m_size, val);
}

template <typename T>
void CUDAStdVector<T>::Resize(size_t n, const ValueType& initVal)
{
    CUDAStdVector newBuffer(n, initVal);

    CUDACopy(m_ptr, std::min(n, m_size), newBuffer.m_ptr);
    Swap(newBuffer);
}

template <typename T>
void CUDAStdVector<T>::ResizeUninitialized(size_t n)
{
    Clear();

    CUBBYFLOW_CUDA_CHECK(cudaMalloc(&m_ptr, n * sizeof(T)));
    m_size = n;
}

template <typename T>
void CUDAStdVector<T>::Swap(CUDAStdVector& other)
{
    std::swap(m_ptr, other.m_ptr);
    std::swap(m_size, other.m_size);
}

template <typename T>
void CUDAStdVector<T>::PushBack(const ValueType& val)
{
    CUDAStdVector newBuffer;
    newBuffer.ResizeUninitialized(m_size + 1);

    CUDACopy(m_ptr, m_size, newBuffer.m_ptr);
    CUDACopyHostToDevice(&val, 1, newBuffer.m_ptr + m_size);
    Swap(newBuffer);
}

template <typename T>
void CUDAStdVector<T>::Append(const ValueType& val)
{
    PushBack(val);
}

template <typename T>
void CUDAStdVector<T>::Append(const CUDAStdVector& other)
{
    CUDAStdVector newBuffer;
    newBuffer.ResizeUninitialized(m_size + other.m_size);

    CUDACopy(m_ptr, m_size, newBuffer.m_ptr);
    CUDACopy(other.m_ptr, other.m_size, newBuffer.m_ptr + m_size);
    Swap(newBuffer);
}

template <typename T>
template <typename A>
void CUDAStdVector<T>::CopyFrom(const std::vector<T, A>& other)
{
    if (m_size == other.size())
    {
        CUDACopyHostToDevice(other.data(), m_size, m_ptr);
    }
    else
    {
        CUDAStdVector newBuffer(other);
        Swap(newBuffer);
    }
}

template <typename T>
void CUDAStdVector<T>::CopyFrom(const CUDAStdVector& other)
{
    if (m_size == other.Size())
    {
        CUDACopyDeviceToDevice(other.data(), m_size, m_ptr);
    }
    else
    {
        CUDAStdVector newBuffer(other);
        Swap(newBuffer);
    }
}

template <typename T>
template <typename A>
void CUDAStdVector<T>::CopyTo(std::vector<T, A>& other)
{
    other.resize(m_size);
    CUDACopyDeviceToHost(m_ptr, m_size, other.data());
}

#if defined(__HIP__) || defined(__CUDA_ARCH__)
template <typename T>
__device__ typename CUDAStdVector<T>::Reference CUDAStdVector<T>::operator[](
    size_t i)
{
    return At(i);
}

template <typename T>
__device__ typename CUDAStdVector<T>::ConstReference
CUDAStdVector<T>::operator[](size_t i) const
{
    return At(i);
}
#endif
#if defined(__HIP__) || !defined(__CUDA_ARCH__)
template <typename T>
CUBBYFLOW_CUDA_HOST typename CUDAStdVector<T>::ReferenceType
CUDAStdVector<T>::operator[](size_t i)
{
    return At(i);
}

template <typename T>
CUBBYFLOW_CUDA_HOST T CUDAStdVector<T>::operator[](size_t i) const
{
    return At(i);
}
#endif
}  // namespace CubbyFlow

#endif

#endif