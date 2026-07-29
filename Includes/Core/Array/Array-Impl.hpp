// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_ARRAY_IMPL_HPP
#define CUBBYFLOW_ARRAY_IMPL_HPP

#include <Core/Utils/IterationUtils.hpp>

namespace CubbyFlow
{
template <typename T, size_t N>
class Array;

namespace Internal
{
template <typename T, size_t N, size_t I>
struct GetSizeAndInitVal
{
    template <typename... Args>
    static void Call(Vector<size_t, N>& size, T& value, size_t n, Args... args)
    {
        size[N - I - 1] = n;

        GetSizeAndInitVal<T, N, I - 1>::Call(size, value, args...);
    }
};

template <typename T, size_t N>
struct GetSizeAndInitVal<T, N, 0>
{
    static void Call(Vector<size_t, N>& size, T& value, size_t n)
    {
        Call(size, value, n, T{});
    }

    static void Call(Vector<size_t, N>& size, T& value, size_t n,
                     const T& initVal)
    {
        size[N - 1] = n;
        value = initVal;
    }
};

template <typename T, size_t N, size_t I>
struct GetSizeFromInitList
{
    static size_t Call(Vector<size_t, N>& size,
                       NestedInitializerListsT<T, I> lst)
    {
        size[I - 1] = lst.size();

        size_t i = 0;

        for (auto subLst : lst)
        {
            if (i == 0)
            {
                GetSizeFromInitList<T, N, I - 1>::Call(size, subLst);
            }
            else
            {
                Vector<size_t, N> tempSizeN;
                [[maybe_unused]] size_t otherSize =
                    GetSizeFromInitList<T, N, I - 1>::Call(tempSizeN, subLst);

                assert(otherSize == tempSizeN[I - 2]);
            }

            ++i;
        }

        return size[I - 1];
    }
};

template <typename T, size_t N>
struct GetSizeFromInitList<T, N, 1>
{
    static size_t Call(Vector<size_t, N>& size,
                       NestedInitializerListsT<T, 1> lst)
    {
        size[0] = lst.size();

        return size[0];
    }
};

template <typename T, size_t N, size_t I>
struct SetArrayFromInitList
{
    static void Call(Array<T, N>& arr, NestedInitializerListsT<T, I> lst)
    {
        size_t i = 0;

        for (auto subLst : lst)
        {
            assert(i < arr.Size()[I - 1]);

            SetArrayFromInitList<T, N, I - 1>::Call(arr, subLst, i);

            ++i;
        }
    }

    template <typename... RemainingIndices>
    static void Call(Array<T, N>& arr, NestedInitializerListsT<T, I> lst,
                     RemainingIndices... indices)
    {
        size_t i = 0;

        for (auto subLst : lst)
        {
            assert(i < arr.Size()[I - 1]);

            SetArrayFromInitList<T, N, I - 1>::Call(arr, subLst, i, indices...);
            ++i;
        }
    }
};

template <typename T, size_t N>
struct SetArrayFromInitList<T, N, 1>
{
    static void call(Array<T, N>& arr, NestedInitializerListsT<T, 1> lst)
    {
        size_t i = 0;

        for (auto val : lst)
        {
            assert(i < arr.Size()[0]);

            arr(i) = val;
            ++i;
        }
    }

    template <typename... RemainingIndices>
    static void Call(Array<T, N>& arr, NestedInitializerListsT<T, 1> lst,
                     RemainingIndices... indices)
    {
        size_t i = 0;

        for (auto val : lst)
        {
            assert(i < arr.Size()[0]);

            arr(i, indices...) = val;
            ++i;
        }
    }
};
}  // namespace Internal

template <typename T, size_t N>
Array<T, N>::Array() : Base()
{
    // Do nothing
}

template <typename T, size_t N>
Array<T, N>::Array(const Vector<size_t, N>& size_, const T& initVal) : Array()
{
    m_data.resize(Product<size_t, N>(size_, 1), initVal);
    Base::SetPtrAndSize(m_data.data(), size_);
}

template <typename T, size_t N>
template <typename... Args>
Array<T, N>::Array(size_t nx, Args... args)
{
    Vector<size_t, N> size;
    T initVal;

    Internal::GetSizeAndInitVal<T, N, N - 1>::Call(size, initVal, nx, args...);
    m_data.resize(Product<size_t, N>(size, 1), initVal);
    Base::SetPtrAndSize(m_data.data(), size);
}

template <typename T, size_t N>
Array<T, N>::Array(NestedInitializerListsT<T, N> lst)
{
    Vector<size_t, N> newSize{};

    Internal::GetSizeFromInitList<T, N, N>::Call(newSize, lst);
    m_data.resize(Product<size_t, N>(newSize, 1));
    Base::SetPtrAndSize(m_data.data(), newSize);
    Internal::SetArrayFromInitList<T, N, N>::Call(*this, lst);
}

template <typename T, size_t N>
template <typename OtherDerived>
Array<T, N>::Array(const ArrayBase<T, N, OtherDerived>& other) : Array()
{
    CopyFrom(other);
}

template <typename T, size_t N>
template <typename OtherDerived>
Array<T, N>::Array(const ArrayBase<const T, N, OtherDerived>& other) : Array()
{
    CopyFrom(other);
}

template <typename T, size_t N>
Array<T, N>::Array(const Array& other) : Array()
{
    CopyFrom(other);
}

template <typename T, size_t N>
Array<T, N>::Array(Array&& other) noexcept : Array()
{
    *this = std::move(other);
}

template <typename T, size_t N>
Array<T, N>& Array<T, N>::operator=(const Array& other)
{
    CopyFrom(other);

    return *this;
}

template <typename T, size_t N>
Array<T, N>& Array<T, N>::operator=(Array&& other) noexcept
{
    m_data = std::move(other.m_data);

    Base::SetPtrAndSize(other.data(), other.Size());
    other.SetPtrAndSize(nullptr, Vector<size_t, N>{});

    return *this;
}

template <typename T, size_t N>
template <typename OtherDerived>
Array<T, N>& Array<T, N>::operator=(const ArrayBase<T, N, OtherDerived>& other)
{
    CopyFrom(other);

    return *this;
}

template <typename T, size_t N>
template <typename OtherDerived>
Array<T, N>& Array<T, N>::operator=(
    const ArrayBase<const T, N, OtherDerived>& other)
{
    CopyFrom(other);

    return *this;
}

template <typename T, size_t N>
template <typename D>
void Array<T, N>::CopyFrom(const ArrayBase<T, N, D>& other)
{
    Resize(other.Size());
    ForEachIndex(
        Vector<size_t, N>{}, other.Size(),
        [this, &other](auto... idx) { this->At(idx...) = other(idx...); });
}

template <typename T, size_t N>
template <typename D>
void Array<T, N>::CopyFrom(const ArrayBase<const T, N, D>& other)
{
    Resize(other.Size());
    ForEachIndex(
        Vector<size_t, N>{}, other.Size(),
        [this, &other](auto... idx) { this->At(idx...) = other(idx...); });
}

template <typename T, size_t N>
void Array<T, N>::Fill(const T& val)
{
    std::fill(m_data.begin(), m_data.end(), val);
}

template <typename T, size_t N>
void Array<T, N>::Resize(Vector<size_t, N> size_, const T& initVal)
{
    Array newArray(size_, initVal);
    Vector<size_t, N> minSize = Min(m_size, newArray.m_size);

    ForEachIndex(minSize, [&newArray, this](auto... idx) {
        newArray(idx...) = (*this)(idx...);
    });

    *this = std::move(newArray);
}

template <typename T, size_t N>
template <typename... Args>
void Array<T, N>::Resize(size_t nx, Args... args)
{
    Vector<size_t, N> size;
    T initVal;

    Internal::GetSizeAndInitVal<T, N, N - 1>::Call(size, initVal, nx, args...);

    Resize(size, initVal);
}

template <typename T, size_t N>
template <size_t M>
std::enable_if_t<(M == 1), void> Array<T, N>::Append(const T& val)
{
    m_data.push_back(val);

    Base::SetPtrAndSize(m_data.data(), m_data.size());
}

template <typename T, size_t N>
template <typename OtherDerived, size_t M>
std::enable_if_t<(M == 1), void> Array<T, N>::Append(
    const ArrayBase<T, N, OtherDerived>& extra)
{
    m_data.insert(m_data.end(), extra.begin(), extra.end());

    Base::SetPtrAndSize(m_data.data(), m_data.size());
}

template <typename T, size_t N>
template <typename OtherDerived, size_t M>
std::enable_if_t<(M == 1), void> Array<T, N>::Append(
    const ArrayBase<const T, N, OtherDerived>& extra)
{
    m_data.insert(m_data.end(), extra.begin(), extra.end());

    Base::SetPtrAndSize(m_data.data(), m_data.size());
}

template <typename T, size_t N>
void Array<T, N>::Clear()
{
    Base::ClearPtrAndSize();

    m_data.clear();
}

template <typename T, size_t N>
void Array<T, N>::Swap(Array& other)
{
    Base::SwapPtrAndSize(other);

    std::swap(m_data, other.m_data);
}

template <typename T, size_t N>
ArrayView<T, N> Array<T, N>::View()
{
    return ArrayView<T, N>(*this);
}

template <typename T, size_t N>
ArrayView<const T, N> Array<T, N>::View() const
{
    return ArrayView<const T, N>(*this);
}
}  // namespace CubbyFlow

#endif