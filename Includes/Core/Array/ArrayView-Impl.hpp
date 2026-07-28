// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_ARRAY_VIEW_IMPL_HPP
#define CUBBYFLOW_ARRAY_VIEW_IMPL_HPP

#include <Core/Array/Array.hpp>

namespace CubbyFlow
{
template <typename T, size_t N>
ArrayView<T, N>::ArrayView() : Base()
{
    // Do nothing
}

template <typename T, size_t N>
ArrayView<T, N>::ArrayView(T* ptr, const Vector<size_t, N>& size) : ArrayView()
{
    Base::SetPtrAndSize(ptr, size);
}

template <typename T, size_t N>
template <size_t M>
ArrayView<T, N>::ArrayView(std::enable_if_t<(M == 1), T*> ptr, size_t size)
    : ArrayView(ptr, Vector<size_t, N>{ size })
{
    // Do nothing
}

template <typename T, size_t N>
ArrayView<T, N>::ArrayView(Array<T, N>& other) : ArrayView()
{
    Set(other);
}

template <typename T, size_t N>
ArrayView<T, N>::ArrayView(const ArrayView& other)
{
    Set(other);
}

template <typename T, size_t N>
ArrayView<T, N>::ArrayView(ArrayView&& other) noexcept : ArrayView()
{
    *this = std::move(other);
}

template <typename T, size_t N>
ArrayView<T, N>& ArrayView<T, N>::operator=(const ArrayView& other)
{
    Set(other);

    return *this;
}

template <typename T, size_t N>
ArrayView<T, N>& ArrayView<T, N>::operator=(ArrayView&& other) noexcept
{
    Base::SetPtrAndSize(other.data(), other.Size());
    other.SetPtrAndSize(nullptr, Vector<size_t, N>{});

    return *this;
}

template <typename T, size_t N>
void ArrayView<T, N>::Set(Array<T, N>& other)
{
    Base::SetPtrAndSize(other.data(), other.Size());
}

template <typename T, size_t N>
void ArrayView<T, N>::Set(const ArrayView& other)
{
    Base::SetPtrAndSize(other.m_ptr, other.Size());
}

template <typename T, size_t N>
void ArrayView<T, N>::Fill(const T& val)
{
    ForEachIndex(Vector<size_t, N>{}, m_size,
                 [this, &val](auto... idx) { this->At(idx...) = val; });
}

template <typename T, size_t N>
ArrayView<const T, N>::ArrayView() : Base()
{
    // Do nothing
}

template <typename T, size_t N>
ArrayView<const T, N>::ArrayView(const T* ptr, const Vector<size_t, N>& size)
    : ArrayView()
{
    Base::SetPtrAndSize(ptr, size);
}

template <typename T, size_t N>
template <size_t M>
ArrayView<const T, N>::ArrayView(const std::enable_if_t<(M == 1), T*> ptr,
                                 size_t size)
    : ArrayView(ptr, Vector<size_t, N>{ size })
{
    // Do nothing
}

template <typename T, size_t N>
ArrayView<const T, N>::ArrayView(const Array<T, N>& other) : ArrayView()
{
    Set(other);
}

template <typename T, size_t N>
ArrayView<const T, N>::ArrayView(const ArrayView<T, N>& other)
{
    Set(other);
}

template <typename T, size_t N>
ArrayView<const T, N>::ArrayView(const ArrayView<const T, N>& other)
{
    Set(other);
}

template <typename T, size_t N>
ArrayView<const T, N>::ArrayView(ArrayView&& other) noexcept : ArrayView()
{
    *this = std::move(other);
}

template <typename T, size_t N>
ArrayView<const T, N>& ArrayView<const T, N>::operator=(
    const ArrayView<T, N>& other)
{
    Set(other);

    return *this;
}

template <typename T, size_t N>
ArrayView<const T, N>& ArrayView<const T, N>::operator=(
    const ArrayView<const T, N>& other)
{
    Set(other);

    return *this;
}

template <typename T, size_t N>
ArrayView<const T, N>& ArrayView<const T, N>::operator=(
    ArrayView<const T, N>&& other) noexcept
{
    Base::SetPtrAndSize(other.data(), other.Size());
    other.SetPtrAndSize(nullptr, Vector<size_t, N>{});

    return *this;
}

template <typename T, size_t N>
void ArrayView<const T, N>::Set(const Array<T, N>& other)
{
    Base::SetPtrAndSize(other.data(), other.Size());
}

template <typename T, size_t N>
void ArrayView<const T, N>::Set(const ArrayView<T, N>& other)
{
    Base::SetPtrAndSize(other.data(), other.Size());
}

template <typename T, size_t N>
void ArrayView<const T, N>::Set(const ArrayView<const T, N>& other)
{
    Base::SetPtrAndSize(other.data(), other.Size());
}
}  // namespace CubbyFlow

#endif
