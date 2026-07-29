// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_ARRAY_HPP
#define CUBBYFLOW_ARRAY_HPP

#include <Core/Array/ArrayBase.hpp>

#include <vector>

namespace CubbyFlow
{
//!
//! \brief Generic N-dimensional array class interface.
//!
//! This class provides generic template class for N-dimensional array where N
//! must be either 1, 2 or 3. This particular class exists to provide generic
//! interface for 1, 2 or 3 dimensional arrays using template specialization
//! only, but it cannot create any instance by itself.
//!
//! \tparam T - Real number type.
//! \tparam N - Dimension.
//!
template <typename T, size_t N>
class ArrayView;

template <typename T, size_t N>
class Array final : public ArrayBase<T, N, Array<T, N>>
{
    using Base = ArrayBase<T, N, Array<T, N>>;
    using Base::At;
    using Base::ClearPtrAndSize;
    using Base::m_size;
    using Base::SetPtrAndSize;
    using Base::SwapPtrAndSize;

 public:
    Array();

    explicit Array(const Vector<size_t, N>& size, const T& initVal = T{});

    template <typename... Args>
    explicit Array(size_t nx, Args... args);

    Array(NestedInitializerListsT<T, N> lst);

    template <typename OtherDerived>
    Array(const ArrayBase<T, N, OtherDerived>& other);

    template <typename OtherDerived>
    Array(const ArrayBase<const T, N, OtherDerived>& other);

    ~Array() override = default;

    Array(const Array& other);

    Array(Array&& other) noexcept;

    Array& operator=(const Array& other);

    Array& operator=(Array&& other) noexcept;

    template <typename OtherDerived>
    Array& operator=(const ArrayBase<T, N, OtherDerived>& other);

    template <typename OtherDerived>
    Array& operator=(const ArrayBase<const T, N, OtherDerived>& other);

    template <typename D>
    void CopyFrom(const ArrayBase<T, N, D>& other);

    template <typename D>
    void CopyFrom(const ArrayBase<const T, N, D>& other);

    void Fill(const T& val);

    void Resize(Vector<size_t, N> size_, const T& initVal = T{});

    template <typename... Args>
    void Resize(size_t nx, Args... args);

    template <size_t M = N>
    std::enable_if_t<(M == 1), void> Append(const T& val);

    template <typename OtherDerived, size_t M = N>
    std::enable_if_t<(M == 1), void> Append(
        const ArrayBase<T, N, OtherDerived>& extra);

    template <typename OtherDerived, size_t M = N>
    std::enable_if_t<(M == 1), void> Append(
        const ArrayBase<const T, N, OtherDerived>& extra);

    void Clear();

    void Swap(Array& other);

    [[nodiscard]] ArrayView<T, N> View();

    [[nodiscard]] ArrayView<const T, N> View() const;

 private:
    std::vector<T> m_data;
};

template <class T>
using Array1 = Array<T, 1>;

template <class T>
using Array2 = Array<T, 2>;

template <class T>
using Array3 = Array<T, 3>;

template <class T>
using Array4 = Array<T, 4>;
}  // namespace CubbyFlow

#include <Core/Array/Array-Impl.hpp>

#endif
