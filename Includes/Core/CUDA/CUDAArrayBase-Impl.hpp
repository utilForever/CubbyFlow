// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_CUDA_ARRAY_BASE_IMPL_HPP
#define CUBBYFLOW_CUDA_ARRAY_BASE_IMPL_HPP

#ifdef CUBBYFLOW_USE_CUDA

namespace CubbyFlow
{
template <typename T, size_t N, typename Derived>
CUBBYFLOW_CUDA_HOST_DEVICE size_t CUDAArrayBase<T, N, Derived>::Index(size_t i) const
{
    return i;
}

template <typename T, size_t N, typename Derived>
template <typename... Args>
CUBBYFLOW_CUDA_HOST_DEVICE size_t CUDAArrayBase<T, N, Derived>::Index(size_t i, Args... args) const
{
    static_assert(sizeof...(args) == N - 1, "Invalid number of indices.");
    return i + m_size[0] * IndexInternal(1, args...);
}

template <typename T, size_t N, typename Derived>
template <size_t... I>
CUBBYFLOW_CUDA_HOST_DEVICE size_t CUDAArrayBase<T, N, Derived>::Index(
    const CUDAStdArray<size_t, N>& idx) const
{
    return IndexInternal(idx, std::make_index_sequence<N>{});
}

template <typename T, size_t N, typename Derived>
CUBBYFLOW_CUDA_HOST_DEVICE T* CUDAArrayBase<T, N, Derived>::data()
{
    return m_ptr;
}

template <typename T, size_t N, typename Derived>
CUBBYFLOW_CUDA_HOST_DEVICE const T* CUDAArrayBase<T, N, Derived>::data() const
{
    return m_ptr;
}

template <typename T, size_t N, typename Derived>
CUBBYFLOW_CUDA_HOST_DEVICE const CUDAStdArray<size_t, N>& CUDAArrayBase<T, N, Derived>::Size() const
{
    return m_size;
}

template <typename T, size_t N, typename Derived>
template <size_t M>
CUBBYFLOW_CUDA_HOST_DEVICE std::enable_if_t<(M > 0), size_t> CUDAArrayBase<T, N, Derived>::Width() const
{
    return m_size[0];
}

template <typename T, size_t N, typename Derived>
template <size_t M>
CUBBYFLOW_CUDA_HOST_DEVICE std::enable_if_t<(M > 1), size_t> CUDAArrayBase<T, N, Derived>::Height() const
{
    return m_size[1];
}

template <typename T, size_t N, typename Derived>
template <size_t M>
CUBBYFLOW_CUDA_HOST_DEVICE std::enable_if_t<(M > 2), size_t> CUDAArrayBase<T, N, Derived>::Depth() const
{
    return m_size[2];
}

template <typename T, size_t N, typename Derived>
CUBBYFLOW_CUDA_HOST_DEVICE size_t CUDAArrayBase<T, N, Derived>::Length() const
{
    size_t l = m_size[0];

    for (size_t i = 1; i < N; ++i)
    {
        l *= m_size[i];
    }

    return l;
}

// See CUDAArrayBase.hpp: under HIP both spaces are defined and resolved by
// attribute, so emit device definitions when compiling for HIP or in nvcc's
// device pass, and host definitions when compiling for HIP or nvcc's host pass.
#if defined(__HIP__) || defined(__CUDA_ARCH__)
template <typename T, size_t N, typename Derived>
CUBBYFLOW_CUDA_DEVICE typename CUDAArrayBase<T, N, Derived>::Reference
CUDAArrayBase<T, N, Derived>::At(size_t i)
{
    return m_ptr[i];
}

template <typename T, size_t N, typename Derived>
CUBBYFLOW_CUDA_DEVICE typename CUDAArrayBase<T, N, Derived>::ConstReference
CUDAArrayBase<T, N, Derived>::At(size_t i) const
{
    return m_ptr[i];
}

template <typename T, size_t N, typename Derived>
template <typename... Args>
CUBBYFLOW_CUDA_DEVICE typename CUDAArrayBase<T, N, Derived>::Reference
CUDAArrayBase<T, N, Derived>::At(size_t i, Args... args)
{
    return At(Index(i, args...));
}

template <typename T, size_t N, typename Derived>
template <typename... Args>
CUBBYFLOW_CUDA_DEVICE typename CUDAArrayBase<T, N, Derived>::ConstReference
CUDAArrayBase<T, N, Derived>::At(size_t i, Args... args) const
{
    return At(Index(i, args...));
}

template <typename T, size_t N, typename Derived>
CUBBYFLOW_CUDA_DEVICE typename CUDAArrayBase<T, N, Derived>::Reference
CUDAArrayBase<T, N, Derived>::At(const CUDAStdArray<size_t, N>& idx)
{
    return At(Index(idx));
}

template <typename T, size_t N, typename Derived>
CUBBYFLOW_CUDA_DEVICE typename CUDAArrayBase<T, N, Derived>::ConstReference
CUDAArrayBase<T, N, Derived>::At(const CUDAStdArray<size_t, N>& idx) const
{
    return At(Index(idx));
}

template <typename T, size_t N, typename Derived>
CUBBYFLOW_CUDA_DEVICE typename CUDAArrayBase<T, N, Derived>::Reference
CUDAArrayBase<T, N, Derived>::operator[](size_t i)
{
    return At(i);
}

template <typename T, size_t N, typename Derived>
CUBBYFLOW_CUDA_DEVICE typename CUDAArrayBase<T, N, Derived>::ConstReference
CUDAArrayBase<T, N, Derived>::operator[](size_t i) const
{
    return At(i);
}

template <typename T, size_t N, typename Derived>
template <typename... Args>
CUBBYFLOW_CUDA_DEVICE typename CUDAArrayBase<T, N, Derived>::Reference
CUDAArrayBase<T, N, Derived>::operator()(size_t i, Args... args)
{
    return At(i, args...);
}

template <typename T, size_t N, typename Derived>
template <typename... Args>
CUBBYFLOW_CUDA_DEVICE typename CUDAArrayBase<T, N, Derived>::ConstReference
CUDAArrayBase<T, N, Derived>::operator()(size_t i, Args... args) const
{
    return At(i, args...);
}

template <typename T, size_t N, typename Derived>
CUBBYFLOW_CUDA_DEVICE typename CUDAArrayBase<T, N, Derived>::Reference
CUDAArrayBase<T, N, Derived>::operator()(const CUDAStdArray<size_t, N>& idx)
{
    return At(idx);
}

template <typename T, size_t N, typename Derived>
CUBBYFLOW_CUDA_DEVICE typename CUDAArrayBase<T, N, Derived>::ConstReference
CUDAArrayBase<T, N, Derived>::operator()(
    const CUDAStdArray<size_t, N>& idx) const
{
    return At(idx);
}
#endif
#if defined(__HIP__) || !defined(__CUDA_ARCH__)
template <typename T, size_t N, typename Derived>
CUBBYFLOW_CUDA_HOST typename CUDAArrayBase<T, N, Derived>::HostReference
CUDAArrayBase<T, N, Derived>::At(size_t i)
{
    return HostReference(m_ptr + i);
}

template <typename T, size_t N, typename Derived>
CUBBYFLOW_CUDA_HOST T CUDAArrayBase<T, N, Derived>::At(size_t i) const
{
    return (T)HostReference(m_ptr + i);
}

template <typename T, size_t N, typename Derived>
template <typename... Args>
CUBBYFLOW_CUDA_HOST typename CUDAArrayBase<T, N, Derived>::HostReference
CUDAArrayBase<T, N, Derived>::At(size_t i, Args... args)
{
    return At(Index(i, args...));
}

template <typename T, size_t N, typename Derived>
template <typename... Args>
CUBBYFLOW_CUDA_HOST T CUDAArrayBase<T, N, Derived>::At(size_t i, Args... args) const
{
    return At(Index(i, args...));
}

template <typename T, size_t N, typename Derived>
CUBBYFLOW_CUDA_HOST typename CUDAArrayBase<T, N, Derived>::HostReference
CUDAArrayBase<T, N, Derived>::At(const CUDAStdArray<size_t, N>& idx)
{
    return At(Index(idx));
}

template <typename T, size_t N, typename Derived>
CUBBYFLOW_CUDA_HOST T CUDAArrayBase<T, N, Derived>::At(const CUDAStdArray<size_t, N>& idx) const
{
    return At(Index(idx));
}

template <typename T, size_t N, typename Derived>
CUBBYFLOW_CUDA_HOST typename CUDAArrayBase<T, N, Derived>::HostReference
CUDAArrayBase<T, N, Derived>::operator[](size_t i)
{
    return At(i);
}

template <typename T, size_t N, typename Derived>
CUBBYFLOW_CUDA_HOST T CUDAArrayBase<T, N, Derived>::operator[](size_t i) const
{
    return At(i);
}

template <typename T, size_t N, typename Derived>
template <typename... Args>
CUBBYFLOW_CUDA_HOST typename CUDAArrayBase<T, N, Derived>::HostReference
CUDAArrayBase<T, N, Derived>::operator()(size_t i, Args... args)
{
    return At(i, args...);
}

template <typename T, size_t N, typename Derived>
template <typename... Args>
CUBBYFLOW_CUDA_HOST T CUDAArrayBase<T, N, Derived>::operator()(size_t i, Args... args) const
{
    return At(i, args...);
}

template <typename T, size_t N, typename Derived>
CUBBYFLOW_CUDA_HOST typename CUDAArrayBase<T, N, Derived>::HostReference
CUDAArrayBase<T, N, Derived>::operator()(const CUDAStdArray<size_t, N>& idx)
{
    return At(idx);
}

template <typename T, size_t N, typename Derived>
CUBBYFLOW_CUDA_HOST T CUDAArrayBase<T, N, Derived>::operator()(
    const CUDAStdArray<size_t, N>& idx) const
{
    return At(idx);
}
#endif

template <typename T, size_t N, typename Derived>
CUBBYFLOW_CUDA_HOST_DEVICE CUDAArrayBase<T, N, Derived>::CUDAArrayBase() : m_size{}
{
    // Do nothing
}

template <typename T, size_t N, typename Derived>
CUBBYFLOW_CUDA_HOST_DEVICE CUDAArrayBase<T, N, Derived>::CUDAArrayBase(const CUDAArrayBase& other)
{
    SetPtrAndSize(other.m_ptr, other.m_size);
}

template <typename T, size_t N, typename Derived>
CUBBYFLOW_CUDA_HOST_DEVICE CUDAArrayBase<T, N, Derived>::CUDAArrayBase(CUDAArrayBase&& other) noexcept
{
    *this = std::move(other);
}

template <typename T, size_t N, typename Derived>
CUBBYFLOW_CUDA_HOST_DEVICE CUDAArrayBase<T, N, Derived>& CUDAArrayBase<T, N, Derived>::operator=(
    const CUDAArrayBase& other)
{
    SetPtrAndSize(other.m_ptr, other.m_size);
    return *this;
}

template <typename T, size_t N, typename Derived>
CUBBYFLOW_CUDA_HOST_DEVICE CUDAArrayBase<T, N, Derived>& CUDAArrayBase<T, N, Derived>::operator=(
    CUDAArrayBase&& other) noexcept
{
    SetPtrAndSize(other.m_ptr, other.m_size);
    other.SetPtrAndSize(nullptr, CUDAStdArray<size_t, N>{});
    return *this;
}

template <typename T, size_t N, typename Derived>
template <typename... Args>
CUBBYFLOW_CUDA_HOST_DEVICE void CUDAArrayBase<T, N, Derived>::SetPtrAndSize(Pointer ptr, size_t ni,
                                                 Args... args)
{
    SetPtrAndSize(ptr, CUDAStdArray<size_t, N>{ ni, args... });
}

template <typename T, size_t N, typename Derived>
CUBBYFLOW_CUDA_HOST_DEVICE void CUDAArrayBase<T, N, Derived>::SetPtrAndSize(Pointer ptr,
                                                 CUDAStdArray<size_t, N> size)
{
    m_ptr = ptr;
    m_size = size;
}

template <typename T, size_t N, typename Derived>
CUBBYFLOW_CUDA_HOST_DEVICE void CUDAArrayBase<T, N, Derived>::SwapPtrAndSize(CUDAArrayBase& other)
{
    CUDASwap(m_ptr, other.m_ptr);
    CUDASwap(m_size, other.m_size);
}

template <typename T, size_t N, typename Derived>
CUBBYFLOW_CUDA_HOST_DEVICE void CUDAArrayBase<T, N, Derived>::ClearPtrAndSize()
{
    SetPtrAndSize(nullptr, CUDAStdArray<size_t, N>{});
}

template <typename T, size_t N, typename Derived>
template <typename... Args>
CUBBYFLOW_CUDA_HOST_DEVICE size_t CUDAArrayBase<T, N, Derived>::IndexInternal(size_t d, size_t i,
                                                   Args... args) const
{
    return i + m_size[d] * IndexInternal(d + 1, args...);
}

template <typename T, size_t N, typename Derived>
CUBBYFLOW_CUDA_HOST_DEVICE size_t CUDAArrayBase<T, N, Derived>::IndexInternal(size_t, size_t i) const
{
    return i;
}

template <typename T, size_t N, typename Derived>
template <size_t... I>
CUBBYFLOW_CUDA_HOST_DEVICE size_t CUDAArrayBase<T, N, Derived>::IndexInternal(
    const CUDAStdArray<size_t, N>& idx, std::index_sequence<I...>) const
{
    return Index(idx[I]...);
}
}  // namespace CubbyFlow

#endif

#endif
