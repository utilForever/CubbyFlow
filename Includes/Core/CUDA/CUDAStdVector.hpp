// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_CUDA_STD_VECTOR_HPP
#define CUBBYFLOW_CUDA_STD_VECTOR_HPP

#ifdef CUBBYFLOW_USE_CUDA

#include <Core/CUDA/CUDAAlgorithms.hpp>

#include <vector>

namespace CubbyFlow
{
template <typename T>
class CUDAStdVector final
{
 public:
    using ValueType = T;
    using Reference = T&;
    using ConstReference = const T&;
    using Pointer = ValueType*;
    using ConstPointer = const ValueType*;

    class ReferenceType
    {
     public:
        CUBBYFLOW_CUDA_HOST_DEVICE ReferenceType(Pointer p) : m_ptr(p)
        {
            // Do nothing
        }

        CUBBYFLOW_CUDA_HOST_DEVICE ReferenceType(const ReferenceType& other)
            : m_ptr(other.m_ptr)
        {
            // Do nothing
        }

        CUBBYFLOW_CUDA_HOST_DEVICE ReferenceType(ReferenceType&& other) noexcept
            : m_ptr(std::move(other.m_ptr))
        {
            // Do nothing
        }

        ~ReferenceType() = default;

        CUBBYFLOW_CUDA_HOST_DEVICE ReferenceType& operator=(
            const ReferenceType& other)
        {
            m_ptr = other.m_ptr;
            return *this;
        }

        CUBBYFLOW_CUDA_HOST_DEVICE ReferenceType& operator=(
            ReferenceType&& other) noexcept
        {
            m_ptr = std::move(other.m_ptr);
            return *this;
        }

#if defined(__HIP__) || defined(__CUDA_ARCH__)
        __device__ ReferenceType& operator=(const ValueType& val)
        {
            *m_ptr = val;
            return *this;
        }

        __device__ operator ValueType() const
        {
            return *m_ptr;
        }
#endif
#if defined(__HIP__) || !defined(__CUDA_ARCH__)
        CUBBYFLOW_CUDA_HOST ReferenceType& operator=(const ValueType& val)
        {
            CUDACopyHostToDevice(&val, 1, m_ptr);
            return *this;
        }

        CUBBYFLOW_CUDA_HOST operator ValueType() const
        {
            std::remove_const_t<ValueType> tmp{};
            CUDACopyDeviceToHost(m_ptr, 1, &tmp);
            return tmp;
        }
#endif

     private:
        Pointer m_ptr;
    };

    CUDAStdVector() = default;

    CUDAStdVector(size_t n, const ValueType& initVal = ValueType{});

    template <typename A>
    CUDAStdVector(const std::vector<T, A>& other);

    CUDAStdVector(const CUDAStdVector& other);

    CUDAStdVector(CUDAStdVector&& other) noexcept;

    ~CUDAStdVector();

    template <typename A>
    CUDAStdVector& operator=(const std::vector<T, A>& other);

    CUDAStdVector& operator=(const CUDAStdVector& other);

    CUDAStdVector& operator=(CUDAStdVector&& other) noexcept;

    Pointer data();

    ConstPointer data() const;

    size_t Size() const;

#if defined(__HIP__) || defined(__CUDA_ARCH__)
    __device__ Reference At(size_t i);

    __device__ ConstReference At(size_t i) const;
#endif
#if defined(__HIP__) || !defined(__CUDA_ARCH__)
    CUBBYFLOW_CUDA_HOST ReferenceType At(size_t i);

    CUBBYFLOW_CUDA_HOST T At(size_t i) const;
#endif

    void Clear();

    void Fill(const ValueType& val);

    void Resize(size_t n, const ValueType& initVal = ValueType{});

    void ResizeUninitialized(size_t n);

    void Swap(CUDAStdVector& other);

    void PushBack(const ValueType& val);

    void Append(const ValueType& val);

    void Append(const CUDAStdVector& other);

    template <typename A>
    void CopyFrom(const std::vector<T, A>& other);

    void CopyFrom(const CUDAStdVector& other);

    template <typename A>
    void CopyTo(std::vector<T, A>& other);

#if defined(__HIP__) || defined(__CUDA_ARCH__)
    __device__ Reference operator[](size_t i);

    __device__ ConstReference operator[](size_t i) const;
#endif
#if defined(__HIP__) || !defined(__CUDA_ARCH__)
    CUBBYFLOW_CUDA_HOST ReferenceType operator[](size_t i);

    CUBBYFLOW_CUDA_HOST T operator[](size_t i) const;
#endif

 private:
    Pointer m_ptr = nullptr;
    size_t m_size = 0;
};
}  // namespace CubbyFlow

#include <Core/CUDA/CUDAStdVector-Impl.hpp>

#endif

#endif