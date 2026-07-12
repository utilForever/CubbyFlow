// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_CUDA_ARRAY_VIEW_IMPL_HPP
#define CUBBYFLOW_CUDA_ARRAY_VIEW_IMPL_HPP

#ifdef CUBBYFLOW_USE_CUDA

namespace CubbyFlow
{
template <typename T, size_t N>
CUBBYFLOW_CUDA_HOST_DEVICE CUDAArrayView<T, N>::CUDAArrayView() : Base()
{
    // Do nothing
}

template <typename T, size_t N>
CUBBYFLOW_CUDA_HOST_DEVICE CUDAArrayView<T, N>::CUDAArrayView(T* ptr, const CUDAStdArray<size_t, N>& size)
    : CUDAArrayView{}
{
    Base::SetPtrAndSize(ptr, size);
}

template <typename T, size_t N>
template <size_t M>
CUBBYFLOW_CUDA_HOST_DEVICE CUDAArrayView<T, N>::CUDAArrayView(
    typename std::enable_if<(M == 1), T>::type* ptr, size_t size)
    : CUDAArrayView(ptr, CUDAStdArray<size_t, N>{ size })
{
    // Do nothing
}

template <typename T, size_t N>
CUBBYFLOW_CUDA_HOST_DEVICE CUDAArrayView<T, N>::CUDAArrayView(CUDAArray<T, N>& other) : CUDAArrayView{}
{
    Set(other);
}

template <typename T, size_t N>
CUBBYFLOW_CUDA_HOST_DEVICE CUDAArrayView<T, N>::CUDAArrayView(const CUDAArrayView& other)
{
    Set(other);
}

template <typename T, size_t N>
CUBBYFLOW_CUDA_HOST_DEVICE CUDAArrayView<T, N>::CUDAArrayView(CUDAArrayView&& other) noexcept
    : CUDAArrayView{}
{
    *this = std::move(other);
}

template <typename T, size_t N>
CUBBYFLOW_CUDA_HOST_DEVICE CUDAArrayView<T, N>& CUDAArrayView<T, N>::operator=(const CUDAArrayView& other)
{
    Set(other);
    return *this;
}

template <typename T, size_t N>
CUBBYFLOW_CUDA_HOST_DEVICE CUDAArrayView<T, N>& CUDAArrayView<T, N>::operator=(
    CUDAArrayView&& other) noexcept
{
    Base::SetPtrAndSize(other.data(), other.Size());
    other.SetPtrAndSize(nullptr, CUDAStdArray<size_t, N>{});
    return *this;
}

template <typename T, size_t N>
CUBBYFLOW_CUDA_HOST_DEVICE void CUDAArrayView<T, N>::Set(CUDAArray<T, N>& other)
{
    Base::SetPtrAndSize(other.data(), other.Size());
}

template <typename T, size_t N>
CUBBYFLOW_CUDA_HOST_DEVICE void CUDAArrayView<T, N>::Set(const CUDAArrayView& other)
{
    Base::SetPtrAndSize(const_cast<T*>(other.data()), other.Size());
}

template <typename T, size_t N>
CUBBYFLOW_CUDA_HOST void CUDAArrayView<T, N>::Fill(const T& val)
{
    CUDAFill(data(), val);
}

template <typename T, size_t N>
CUBBYFLOW_CUDA_HOST_DEVICE CUDAArrayView<const T, N>::CUDAArrayView() : Base()
{
    // Do nothing
}

template <typename T, size_t N>
CUBBYFLOW_CUDA_HOST_DEVICE CUDAArrayView<const T, N>::CUDAArrayView(const T* ptr,
                                         const CUDAStdArray<size_t, N>& size)
    : CUDAArrayView{}
{
    Base::SetPtrAndSize(MemoryHandle(ptr), size);
}

template <typename T, size_t N>
template <size_t M>
CUBBYFLOW_CUDA_HOST_DEVICE CUDAArrayView<const T, N>::CUDAArrayView(
    const typename std::enable_if<(M == 1), T>::type* ptr, size_t size)
    : CUDAArrayView(ptr, CUDAStdArray<size_t, N>{ size })
{
    // Do nothing
}

template <typename T, size_t N>
CUBBYFLOW_CUDA_HOST_DEVICE CUDAArrayView<const T, N>::CUDAArrayView(const CUDAArray<T, N>& other)
    : CUDAArrayView{}
{
    Set(other);
}

template <typename T, size_t N>
CUBBYFLOW_CUDA_HOST_DEVICE CUDAArrayView<const T, N>::CUDAArrayView(const CUDAArrayView<T, N>& other)
{
    Set(other);
}

template <typename T, size_t N>
CUBBYFLOW_CUDA_HOST_DEVICE CUDAArrayView<const T, N>::CUDAArrayView(const CUDAArrayView& other)
{
    Set(other);
}

template <typename T, size_t N>
CUBBYFLOW_CUDA_HOST_DEVICE CUDAArrayView<const T, N>::CUDAArrayView(CUDAArrayView&& other) noexcept
    : CUDAArrayView{}
{
    *this = std::move(other);
}

template <typename T, size_t N>
CUBBYFLOW_CUDA_HOST_DEVICE CUDAArrayView<const T, N>& CUDAArrayView<const T, N>::operator=(
    const CUDAArrayView<T, N>& other)
{
    Set(other);
    return *this;
}

template <typename T, size_t N>
CUBBYFLOW_CUDA_HOST_DEVICE CUDAArrayView<const T, N>& CUDAArrayView<const T, N>::operator=(
    const CUDAArrayView& other)
{
    Set(other);
    return *this;
}

template <typename T, size_t N>
CUBBYFLOW_CUDA_HOST_DEVICE CUDAArrayView<const T, N>& CUDAArrayView<const T, N>::operator=(
    CUDAArrayView&& other) noexcept
{
    Base::SetPtrAndSize(other.data(), other.Size());
    other.SetPtrAndSize(nullptr, CUDAStdArray<size_t, N>{});
    return *this;
}

template <typename T, size_t N>
CUBBYFLOW_CUDA_HOST_DEVICE void CUDAArrayView<const T, N>::Set(const CUDAArray<T, N>& other)
{
    Base::SetPtrAndSize(other.data(), other.Size());
}

template <typename T, size_t N>
CUBBYFLOW_CUDA_HOST_DEVICE void CUDAArrayView<const T, N>::Set(const CUDAArrayView<T, N>& other)
{
    Base::SetPtrAndSize(other.data(), other.Size());
}

template <typename T, size_t N>
CUBBYFLOW_CUDA_HOST_DEVICE void CUDAArrayView<const T, N>::Set(const CUDAArrayView& other)
{
    Base::SetPtrAndSize(other.data(), other.Size());
}
}  // namespace CubbyFlow

#endif

#endif
