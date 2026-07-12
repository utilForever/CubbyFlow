// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_CUDA_ARRAY_BASE_HPP
#define CUBBYFLOW_CUDA_ARRAY_BASE_HPP

#ifdef CUBBYFLOW_USE_CUDA

#include <Core/CUDA/CUDAStdArray.hpp>
#include <Core/CUDA/CUDAStdVector.hpp>

namespace CubbyFlow
{
template <typename T, size_t N, typename DerivedArray>
class CUDAArrayBase
{
 public:
    using Derived = DerivedArray;
    using ValueType = T;
    using HostReference = typename CUDAStdVector<T>::ReferenceType;
    using Reference = T&;
    using ConstReference = const T&;
    using Pointer = T*;
    using ConstPointer = const T*;

    CUBBYFLOW_CUDA_HOST_DEVICE size_t Index(size_t i) const;

    template <typename... Args>
    CUBBYFLOW_CUDA_HOST_DEVICE size_t Index(size_t i, Args... args) const;

    template <size_t... I>
    CUBBYFLOW_CUDA_HOST_DEVICE size_t
    Index(const CUDAStdArray<size_t, N>& idx) const;

    CUBBYFLOW_CUDA_HOST_DEVICE T* data();

    CUBBYFLOW_CUDA_HOST_DEVICE const T* data() const;

    CUBBYFLOW_CUDA_HOST_DEVICE const CUDAStdArray<size_t, N>& Size() const;

    template <size_t M = N>
    CUBBYFLOW_CUDA_HOST_DEVICE std::enable_if_t<(M > 0), size_t> Width() const;

    template <size_t M = N>
    CUBBYFLOW_CUDA_HOST_DEVICE std::enable_if_t<(M > 1), size_t> Height() const;

    template <size_t M = N>
    CUBBYFLOW_CUDA_HOST_DEVICE std::enable_if_t<(M > 2), size_t> Depth() const;

    CUBBYFLOW_CUDA_HOST_DEVICE size_t Length() const;

// Device accessors return live references into device memory; host accessors
// return a copy-back wrapper (or a value). nvcc selects between them with
// __CUDA_ARCH__ because it does not parse __host__ function bodies during the
// device pass. clang (HIP) parses both spaces in both passes and resolves these
// by the __host__/__device__ attributes instead, so under HIP both overload
// sets must be declared at once and overload resolution picks by call context.
#if defined(__HIP__)
    CUBBYFLOW_CUDA_DEVICE Reference At(size_t i);
    CUBBYFLOW_CUDA_HOST HostReference At(size_t i);

    CUBBYFLOW_CUDA_DEVICE ConstReference At(size_t i) const;
    CUBBYFLOW_CUDA_HOST ValueType At(size_t i) const;

    template <typename... Args>
    CUBBYFLOW_CUDA_DEVICE Reference At(size_t i, Args... args);
    template <typename... Args>
    CUBBYFLOW_CUDA_HOST HostReference At(size_t i, Args... args);

    template <typename... Args>
    CUBBYFLOW_CUDA_DEVICE ConstReference At(size_t i, Args... args) const;
    template <typename... Args>
    CUBBYFLOW_CUDA_HOST ValueType At(size_t i, Args... args) const;

    CUBBYFLOW_CUDA_DEVICE Reference At(const CUDAStdArray<size_t, N>& idx);
    CUBBYFLOW_CUDA_HOST HostReference At(const CUDAStdArray<size_t, N>& idx);

    CUBBYFLOW_CUDA_DEVICE ConstReference
    At(const CUDAStdArray<size_t, N>& idx) const;
    CUBBYFLOW_CUDA_HOST ValueType At(const CUDAStdArray<size_t, N>& idx) const;

    CUBBYFLOW_CUDA_DEVICE Reference operator[](size_t i);
    CUBBYFLOW_CUDA_HOST HostReference operator[](size_t i);

    CUBBYFLOW_CUDA_DEVICE ConstReference operator[](size_t i) const;
    CUBBYFLOW_CUDA_HOST ValueType operator[](size_t i) const;

    template <typename... Args>
    CUBBYFLOW_CUDA_DEVICE Reference operator()(size_t i, Args... args);
    template <typename... Args>
    CUBBYFLOW_CUDA_HOST HostReference operator()(size_t i, Args... args);

    template <typename... Args>
    CUBBYFLOW_CUDA_DEVICE ConstReference operator()(size_t i,
                                                    Args... args) const;
    template <typename... Args>
    CUBBYFLOW_CUDA_HOST ValueType operator()(size_t i, Args... args) const;

    CUBBYFLOW_CUDA_DEVICE Reference
    operator()(const CUDAStdArray<size_t, N>& idx);
    CUBBYFLOW_CUDA_HOST HostReference
    operator()(const CUDAStdArray<size_t, N>& idx);

    CUBBYFLOW_CUDA_DEVICE ConstReference
    operator()(const CUDAStdArray<size_t, N>& idx) const;
    CUBBYFLOW_CUDA_HOST ValueType
    operator()(const CUDAStdArray<size_t, N>& idx) const;
#elif defined(__CUDA_ARCH__)
    CUBBYFLOW_CUDA_DEVICE Reference At(size_t i);

    CUBBYFLOW_CUDA_DEVICE ConstReference At(size_t i) const;

    template <typename... Args>
    CUBBYFLOW_CUDA_DEVICE Reference At(size_t i, Args... args);

    template <typename... Args>
    CUBBYFLOW_CUDA_DEVICE ConstReference At(size_t i, Args... args) const;

    CUBBYFLOW_CUDA_DEVICE Reference At(const CUDAStdArray<size_t, N>& idx);

    CUBBYFLOW_CUDA_DEVICE ConstReference
    At(const CUDAStdArray<size_t, N>& idx) const;

    CUBBYFLOW_CUDA_DEVICE Reference operator[](size_t i);

    CUBBYFLOW_CUDA_DEVICE ConstReference operator[](size_t i) const;

    template <typename... Args>
    CUBBYFLOW_CUDA_DEVICE Reference operator()(size_t i, Args... args);

    template <typename... Args>
    CUBBYFLOW_CUDA_DEVICE ConstReference operator()(size_t i,
                                                    Args... args) const;

    CUBBYFLOW_CUDA_DEVICE Reference
    operator()(const CUDAStdArray<size_t, N>& idx);

    CUBBYFLOW_CUDA_DEVICE ConstReference
    operator()(const CUDAStdArray<size_t, N>& idx) const;
#else
    CUBBYFLOW_CUDA_HOST HostReference At(size_t i);

    CUBBYFLOW_CUDA_HOST ValueType At(size_t i) const;

    template <typename... Args>
    CUBBYFLOW_CUDA_HOST HostReference At(size_t i, Args... args);

    template <typename... Args>
    CUBBYFLOW_CUDA_HOST ValueType At(size_t i, Args... args) const;

    CUBBYFLOW_CUDA_HOST HostReference At(const CUDAStdArray<size_t, N>& idx);

    CUBBYFLOW_CUDA_HOST ValueType At(const CUDAStdArray<size_t, N>& idx) const;

    CUBBYFLOW_CUDA_HOST HostReference operator[](size_t i);

    CUBBYFLOW_CUDA_HOST ValueType operator[](size_t i) const;

    template <typename... Args>
    CUBBYFLOW_CUDA_HOST HostReference operator()(size_t i, Args... args);

    template <typename... Args>
    CUBBYFLOW_CUDA_HOST ValueType operator()(size_t i, Args... args) const;

    CUBBYFLOW_CUDA_HOST HostReference
    operator()(const CUDAStdArray<size_t, N>& idx);

    CUBBYFLOW_CUDA_HOST ValueType
    operator()(const CUDAStdArray<size_t, N>& idx) const;
#endif

 protected:
    CUBBYFLOW_CUDA_HOST_DEVICE CUDAArrayBase();

    CUBBYFLOW_CUDA_HOST_DEVICE CUDAArrayBase(const CUDAArrayBase& other);

    CUBBYFLOW_CUDA_HOST_DEVICE CUDAArrayBase(CUDAArrayBase&& other) noexcept;

    ~CUDAArrayBase() = default;

    CUBBYFLOW_CUDA_HOST_DEVICE CUDAArrayBase& operator=(
        const CUDAArrayBase& other);

    CUBBYFLOW_CUDA_HOST_DEVICE CUDAArrayBase& operator=(
        CUDAArrayBase&& other) noexcept;

    template <typename... Args>
    CUBBYFLOW_CUDA_HOST_DEVICE void SetPtrAndSize(Pointer ptr, size_t ni,
                                                  Args... args);

    CUBBYFLOW_CUDA_HOST_DEVICE void SetPtrAndSize(Pointer data,
                                                  CUDAStdArray<size_t, N> size);

    CUBBYFLOW_CUDA_HOST_DEVICE void SwapPtrAndSize(CUDAArrayBase& other);

    CUBBYFLOW_CUDA_HOST_DEVICE void ClearPtrAndSize();

    Pointer m_ptr = nullptr;
    CUDAStdArray<size_t, N> m_size;

 private:
    template <typename... Args>
    CUBBYFLOW_CUDA_HOST_DEVICE size_t IndexInternal(size_t d, size_t i,
                                                    Args... args) const;

    CUBBYFLOW_CUDA_HOST_DEVICE size_t IndexInternal(size_t, size_t i) const;

    template <size_t... I>
    CUBBYFLOW_CUDA_HOST_DEVICE size_t IndexInternal(
        const CUDAStdArray<size_t, N>& idx, std::index_sequence<I...>) const;
};
}  // namespace CubbyFlow

#include <Core/CUDA/CUDAArrayBase-Impl.hpp>

#endif

#endif