// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_CUDA_STD_ARRAY_IMPL_HPP
#define CUBBYFLOW_CUDA_STD_ARRAY_IMPL_HPP

#ifdef CUBBYFLOW_USE_CUDA

namespace CubbyFlow
{
template <typename T, size_t N>
CUBBYFLOW_CUDA_HOST_DEVICE CUDAStdArray<T, N>::CUDAStdArray()
{
    Fill(T{});
}

template <typename T, size_t N>
template <typename... Args>
CUBBYFLOW_CUDA_HOST_DEVICE CUDAStdArray<T, N>::CUDAStdArray(ConstReference first,
                                                            Args... rest)
{
    static_assert(
        sizeof...(Args) == N - 1,
        "Number of arguments should be equal to the size of the vector.");
    SetAt(0, first, rest...);
}

template <typename T, size_t N>
CUBBYFLOW_CUDA_HOST CUDAStdArray<T, N>::CUDAStdArray(const std::array<T, N>& other)
{
    for (size_t i = 0; i < N; ++i)
    {
        m_elements[i] = other[i];
    }
}

template <typename T, size_t N>
CUBBYFLOW_CUDA_HOST CUDAStdArray<T, N>::CUDAStdArray(const Vector<T, N>& other)
{
    for (size_t i = 0; i < N; ++i)
    {
        m_elements[i] = other[i];
    }
}

template <typename T, size_t N>
CUBBYFLOW_CUDA_HOST_DEVICE CUDAStdArray<T, N>::CUDAStdArray(
    const CUDAStdArray& other)
{
    for (size_t i = 0; i < N; ++i)
    {
        m_elements[i] = other[i];
    }
}

template <typename T, size_t N>
CUBBYFLOW_CUDA_HOST_DEVICE CUDAStdArray<T, N>::CUDAStdArray(
    CUDAStdArray&& other) noexcept
{
    for (size_t i = 0; i < N; ++i)
    {
        m_elements[i] = std::move(other[i]);
    }
}

template <typename T, size_t N>
CUBBYFLOW_CUDA_HOST_DEVICE CUDAStdArray<T, N>& CUDAStdArray<T, N>::operator=(
    const CUDAStdArray& other)
{
    for (size_t i = 0; i < N; ++i)
    {
        m_elements[i] = other[i];
    }

    return *this;
}

template <typename T, size_t N>
CUBBYFLOW_CUDA_HOST_DEVICE CUDAStdArray<T, N>& CUDAStdArray<T, N>::operator=(
    CUDAStdArray&& other) noexcept
{
    for (size_t i = 0; i < N; ++i)
    {
        m_elements[i] = std::move(other[i]);
    }

    return *this;
}

template <typename T, size_t N>
CUBBYFLOW_CUDA_HOST_DEVICE void CUDAStdArray<T, N>::Fill(ConstReference val)
{
    for (size_t i = 0; i < N; ++i)
    {
        m_elements[i] = val;
    }
}

template <typename T, size_t N>
CUBBYFLOW_CUDA_HOST Vector<T, N> CUDAStdArray<T, N>::ToVector() const
{
    Vector<T, N> vec;

    for (size_t i = 0; i < N; ++i)
    {
        vec[i] = m_elements[i];
    }

    return vec;
}

template <typename T, size_t N>
CUBBYFLOW_CUDA_HOST_DEVICE typename CUDAStdArray<T, N>::Reference
CUDAStdArray<T, N>::operator[](size_t i)
{
    return m_elements[i];
}

template <typename T, size_t N>
CUBBYFLOW_CUDA_HOST_DEVICE typename CUDAStdArray<T, N>::ConstReference
CUDAStdArray<T, N>::operator[](size_t i) const
{
    return m_elements[i];
}

template <typename T, size_t N>
CUBBYFLOW_CUDA_HOST_DEVICE bool CUDAStdArray<T, N>::operator==(
    const CUDAStdArray& other) const
{
    for (size_t i = 0; i < N; ++i)
    {
        if (m_elements[i] != other.m_elements[i])
        {
            return false;
        }
    }

    return true;
}

template <typename T, size_t N>
CUBBYFLOW_CUDA_HOST_DEVICE bool CUDAStdArray<T, N>::operator!=(
    const CUDAStdArray& other) const
{
    return !(*this == other);
}

template <typename T, size_t N>
template <typename... Args>
CUBBYFLOW_CUDA_HOST_DEVICE void CUDAStdArray<T, N>::SetAt(size_t i,
                                                         ConstReference first,
                                                         Args... rest)
{
    m_elements[i] = first;
    SetAt(i + 1, rest...);
}

template <typename T, size_t N>
template <typename... Args>
CUBBYFLOW_CUDA_HOST_DEVICE void CUDAStdArray<T, N>::SetAt(size_t i,
                                                         ConstReference first)
{
    m_elements[i] = first;
}
}  // namespace CubbyFlow

#endif

#endif