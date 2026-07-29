// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_ARRAY_UTILS_IMPL_HPP
#define CUBBYFLOW_ARRAY_UTILS_IMPL_HPP

#include <Core/Array/Array.hpp>

namespace CubbyFlow
{
namespace Internal
{
template <typename T, typename U>
void ExtrapolatePoint(const Array2<char>& valid, const Vector2UZ& size,
                      ArrayView2<U> output, Array2<char>* next, size_t i,
                      size_t j)
{
    if (valid(i, j))
    {
        (*next)(i, j) = 1;
        return;
    }

    T sum{};
    unsigned int count = 0;
    const auto add = [&](size_t x, size_t y) {
        if (valid(x, y))
        {
            sum += output(x, y);
            ++count;
        }
    };
    if (i + 1 < size.x)
    {
        add(i + 1, j);
    }
    if (i > 0)
    {
        add(i - 1, j);
    }
    if (j + 1 < size.y)
    {
        add(i, j + 1);
    }
    if (j > 0)
    {
        add(i, j - 1);
    }

    if (count > 0)
    {
        output(i, j) =
            sum / static_cast<typename GetScalarType<T>::value>(count);
        (*next)(i, j) = 1;
    }
}

template <typename T, typename U>
void ExtrapolatePoint(const Array3<char>& valid, const Vector3UZ& size,
                      ArrayView3<U> output, Array3<char>* next, size_t i,
                      size_t j, size_t k)
{
    if (valid(i, j, k))
    {
        (*next)(i, j, k) = 1;
        return;
    }

    T sum{};
    unsigned int count = 0;
    const auto add = [&](size_t x, size_t y, size_t z) {
        if (valid(x, y, z))
        {
            sum += output(x, y, z);
            ++count;
        }
    };
    if (i + 1 < size.x)
    {
        add(i + 1, j, k);
    }
    if (i > 0)
    {
        add(i - 1, j, k);
    }
    if (j + 1 < size.y)
    {
        add(i, j + 1, k);
    }
    if (j > 0)
    {
        add(i, j - 1, k);
    }
    if (k + 1 < size.z)
    {
        add(i, j, k + 1);
    }
    if (k > 0)
    {
        add(i, j, k - 1);
    }

    if (count > 0)
    {
        output(i, j, k) =
            sum / static_cast<typename GetScalarType<T>::value>(count);
        (*next)(i, j, k) = 1;
    }
}
}  // namespace Internal

template <typename T, size_t N>
void Fill(ArrayView<T, N> a, const Vector<size_t, N>& begin,
          const Vector<size_t, N>& end, const T& val)
{
    ForEachIndex(begin, end, [&a, &val](auto... idx) { a(idx...) = val; });
}

template <typename T, size_t N>
void Fill(ArrayView<T, N> a, const T& val)
{
    Fill(a, Vector<size_t, N>{}, Vector<size_t, N>{ a.Size() }, val);
}

template <typename T>
void Fill(ArrayView<T, 1> a, size_t begin, size_t end, const T& val)
{
    Fill(a, Vector1UZ{ begin }, Vector1UZ{ end }, val);
}

template <typename T, typename U, size_t N>
void Copy(ArrayView<T, N> src, const Vector<size_t, N>& begin,
          const Vector<size_t, N>& end, ArrayView<U, N> dst)
{
    ForEachIndex(begin, end,
                 [&dst, &src](auto... idx) { dst(idx...) = src(idx...); });
}

template <typename T, typename U, size_t N>
void Copy(ArrayView<T, N> src, ArrayView<U, N> dst)
{
    Copy(src, Vector<size_t, N>{}, Vector<size_t, N>{ src.Size() }, dst);
}

template <typename T, typename U>
void Copy(ArrayView<T, 1> src, size_t begin, size_t end, ArrayView<U, 1> dst)
{
    Copy(src, Vector1UZ{ begin }, Vector1UZ{ end }, dst);
}

template <typename T, typename U>
void ExtrapolateToRegion(ArrayView2<T> input, ArrayView2<char> valid,
                         unsigned int numberOfIterations, ArrayView2<U> output)
{
    const Vector2UZ size = input.Size();

    assert(size == valid.Size());
    assert(size == output.Size());

    Array2<char> valid0(size);
    Array2<char> valid1(size);

    ParallelForEachIndex(
        valid0.Size(), [&valid0, &valid, &output, &input](size_t i, size_t j) {
            valid0(i, j) = valid(i, j);
            output(i, j) = input(i, j);
        });

    for (unsigned int iter = 0; iter < numberOfIterations; ++iter)
    {
        ForEachIndex(valid0.Size(), [&valid0, &size, &output, &valid1](
                                        size_t i, size_t j) {
            Internal::ExtrapolatePoint<T>(valid0, size, output, &valid1, i, j);
        });

        valid0.Swap(valid1);
    }
}

template <typename T, typename U>
void ExtrapolateToRegion(ArrayView3<T> input, ArrayView3<char> valid,
                         unsigned int numberOfIterations, ArrayView3<U> output)
{
    const Vector3UZ size = input.Size();

    assert(size == valid.Size());
    assert(size == output.Size());

    Array3<char> valid0(size);
    Array3<char> valid1(size);

    ParallelForEachIndex(valid0.Size(), [&valid0, &valid, &output, &input](
                                            size_t i, size_t j, size_t k) {
        valid0(i, j, k) = valid(i, j, k);
        output(i, j, k) = input(i, j, k);
    });

    for (unsigned int iter = 0; iter < numberOfIterations; ++iter)
    {
        ForEachIndex(valid0.Size(), [&valid0, &size, &output, &valid1](
                                        size_t i, size_t j, size_t k) {
            Internal::ExtrapolatePoint<T>(valid0, size, output, &valid1, i, j,
                                          k);
        });

        valid0.Swap(valid1);
    }
}
}  // namespace CubbyFlow

#endif
