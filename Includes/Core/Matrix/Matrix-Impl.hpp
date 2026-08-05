// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_MATRIX_IMPL_HPP
#define CUBBYFLOW_MATRIX_IMPL_HPP

#include <Core/Utils/Functors.hpp>

#include <numeric>

namespace CubbyFlow
{
namespace Internal
{
// TODO: With C++17, fold expression could be used instead.
template <typename M1, typename M2, size_t J>
struct DotProduct
{
    constexpr static auto call(const M1& a, const M2& b, size_t i, size_t j)
    {
        return DotProduct<M1, M2, J - 1>::call(a, b, i, j) + a(i, J) * b(J, j);
    }
};

template <typename M1, typename M2>
struct DotProduct<M1, M2, 0>
{
    constexpr static auto call(const M1& a, const M2& b, size_t i, size_t j)
    {
        return a(i, 0) * b(0, j);
    }
};

// TODO: With C++17, fold expression could be used instead.
template <typename T, size_t Rows, size_t Cols, typename ReduceOperation,
          typename UnaryOperation, size_t I>
struct Reduce
{
    // For vector-like Matrix
    template <typename U = T>
    constexpr static std::enable_if_t<(Cols == 1), U> Call(
        const Matrix<T, Rows, 1>& a, const T& init, ReduceOperation op,
        UnaryOperation uop)
    {
        return op(
            Reduce<T, Rows, Cols, ReduceOperation, UnaryOperation, I - 1>::Call(
                a, init, op, uop),
            uop(a(I, 0)));
    }

    // For vector-like Matrix with zero init
    template <typename U = T>
    constexpr static std::enable_if_t<(Cols == 1), U> Call(
        const Matrix<T, Rows, 1>& a, ReduceOperation op, UnaryOperation uop)
    {
        return op(
            Reduce<T, Rows, Cols, ReduceOperation, UnaryOperation, I - 1>::Call(
                a, op, uop),
            uop(a(I, 0)));
    }

    // For Matrix
    constexpr static T Call(const Matrix<T, Rows, Cols>& a, const T& init,
                            ReduceOperation op, UnaryOperation uop)
    {
        return op(
            Reduce<T, Rows, Cols, ReduceOperation, UnaryOperation, I - 1>::Call(
                a, init, op, uop),
            uop(a[I]));
    }

    // For Matrix with zero init
    constexpr static T Call(const Matrix<T, Rows, Cols>& a, ReduceOperation op,
                            UnaryOperation uop)
    {
        return op(
            Reduce<T, Rows, Cols, ReduceOperation, UnaryOperation, I - 1>::Call(
                a, op, uop),
            uop(a[I]));
    }

    // For diagonal elements on Matrix
    constexpr static T CallDiag(const Matrix<T, Rows, Cols>& a, const T& init,
                                ReduceOperation op, UnaryOperation uop)
    {
        return op(Reduce<T, Rows, Cols, ReduceOperation, UnaryOperation,
                         I - 1>::CallDiag(a, init, op, uop),
                  uop(a(I, I)));
    }
};

template <typename T, size_t Rows, size_t Cols, typename ReduceOperation,
          typename UnaryOperation>
struct Reduce<T, Rows, Cols, ReduceOperation, UnaryOperation, 0>
{
    // For vector-like Matrix
    template <typename U = T>
    constexpr static std::enable_if_t<(Cols > 1), U> Call(
        const Matrix<T, Rows, 1>& a, const T& init, ReduceOperation op,
        UnaryOperation uop)
    {
        return op(uop(a(0, 0)), init);
    }

    // For vector-like Matrix with zero init
    template <typename U = T>
    constexpr static std::enable_if_t<(Cols == 1), U> Call(
        const Matrix<T, Rows, 1>& a, ReduceOperation op, UnaryOperation uop)
    {
        return uop(a(0, 0));
    }

    // For Matrix
    constexpr static T Call(const Matrix<T, Rows, Cols>& a, const T& init,
                            ReduceOperation op, UnaryOperation uop)
    {
        return op(uop(a[0]), init);
    }

    // For MatrixBase with zero init
    constexpr static T Call(const Matrix<T, Rows, Cols>& a, ReduceOperation op,
                            UnaryOperation uop)
    {
        return uop(a[0]);
    }

    // For diagonal elements on MatrixBase
    constexpr static T CallDiag(const Matrix<T, Rows, Cols>& a, const T& init,
                                ReduceOperation op, UnaryOperation uop)
    {
        return op(uop(a(0, 0)), init);
    }
};

// We can use std::logical_and<>, but explicitly putting && helps compiler
// to early terminate the loop (at least for gcc 8.1 as I checked the
// assembly).
// TODO: With C++17, fold expression could be used instead.
template <typename T, size_t Rows, size_t Cols, typename BinaryOperation,
          size_t I>
struct FoldWithAnd
{
    constexpr static bool Call(const Matrix<T, Rows, Cols>& a,
                               const Matrix<T, Rows, Cols>& b,
                               BinaryOperation op)
    {
        return FoldWithAnd<T, Rows, Cols, BinaryOperation, I - 1>::Call(a, b,
                                                                        op) &&
               op(a[I], b[I]);
    }
};

template <typename T, size_t Rows, size_t Cols, typename BinaryOperation>
struct FoldWithAnd<T, Rows, Cols, BinaryOperation, 0>
{
    constexpr static bool Call(const Matrix<T, Rows, Cols>& a,
                               const Matrix<T, Rows, Cols>& b,
                               BinaryOperation op)
    {
        return op(a[0], b[0]);
    }
};

}  // namespace Internal

template <typename T, size_t Rows, size_t Cols>
Matrix<T, Rows, Cols>::Matrix(ConstReference value)
{
    Fill(value);
}

template <typename T, size_t Rows, size_t Cols>
template <size_t R, size_t C, typename E>
Matrix<T, Rows, Cols>::Matrix(const MatrixExpression<T, R, C, E>& expression)
{
    assert(expression.GetRows() == Rows && expression.GetCols() == Cols);

    CopyFrom(expression);
}

template <typename T, size_t Rows, size_t Cols>
Matrix<T, Rows, Cols>::Matrix(const NestedInitializerListsT<T, 2>& lst)
{
    size_t i = 0;

    for (auto rows : lst)
    {
        assert(i < Rows);

        size_t j = 0;

        for (auto col : rows)
        {
            assert(j < Cols);

            (*this)(i, j) = col;
            ++j;
        }

        ++i;
    }
}

template <typename T, size_t Rows, size_t Cols>
Matrix<T, Rows, Cols>::Matrix(ConstPointer ptr)
{
    size_t cnt = 0;

    for (size_t i = 0; i < Rows; ++i)
    {
        for (size_t j = 0; j < Cols; ++j)
        {
            (*this)(i, j) = ptr[cnt++];
        }
    }
}

template <typename T, size_t Rows, size_t Cols>
void Matrix<T, Rows, Cols>::Fill(const T& val)
{
    m_elements.fill(val);
}

template <typename T, size_t Rows, size_t Cols>
void Matrix<T, Rows, Cols>::Fill(const std::function<T(size_t i)>& func)
{
    for (size_t i = 0; i < Rows * Cols; ++i)
    {
        m_elements[i] = func(i);
    }
}

template <typename T, size_t Rows, size_t Cols>
void Matrix<T, Rows, Cols>::Fill(
    const std::function<T(size_t i, size_t j)>& func)
{
    for (size_t i = 0; i < Rows; ++i)
    {
        for (size_t j = 0; j < Cols; ++j)
        {
            (*this)(i, j) = func(i, j);
        }
    }
}

template <typename T, size_t Rows, size_t Cols>
void Matrix<T, Rows, Cols>::Swap(Matrix& other)
{
    m_elements.swap(other.m_elements);
}

template <typename T, size_t Rows, size_t Cols>
constexpr size_t Matrix<T, Rows, Cols>::GetRows() const
{
    return Rows;
}

template <typename T, size_t Rows, size_t Cols>
constexpr size_t Matrix<T, Rows, Cols>::GetCols() const
{
    return Cols;
}

template <typename T, size_t Rows, size_t Cols>
typename Matrix<T, Rows, Cols>::Iterator Matrix<T, Rows, Cols>::begin()
{
    return &m_elements[0];
}

template <typename T, size_t Rows, size_t Cols>
constexpr typename Matrix<T, Rows, Cols>::ConstIterator
Matrix<T, Rows, Cols>::begin() const
{
    return &m_elements[0];
}

template <typename T, size_t Rows, size_t Cols>
typename Matrix<T, Rows, Cols>::Iterator Matrix<T, Rows, Cols>::end()
{
    return begin() + Rows * Cols;
}

template <typename T, size_t Rows, size_t Cols>
constexpr typename Matrix<T, Rows, Cols>::ConstIterator
Matrix<T, Rows, Cols>::end() const
{
    return begin() + Rows * Cols;
}

template <typename T, size_t Rows, size_t Cols>
typename Matrix<T, Rows, Cols>::Pointer Matrix<T, Rows, Cols>::data()
{
    return &m_elements[0];
}

template <typename T, size_t Rows, size_t Cols>
constexpr typename Matrix<T, Rows, Cols>::ConstPointer
Matrix<T, Rows, Cols>::data() const
{
    return &m_elements[0];
}

template <typename T, size_t Rows, size_t Cols>
typename Matrix<T, Rows, Cols>::Reference Matrix<T, Rows, Cols>::operator[](
    size_t i)
{
    assert(i < Rows * Cols);

    return m_elements[i];
}

template <typename T, size_t Rows, size_t Cols>
typename Matrix<T, Rows, Cols>::ConstReference
Matrix<T, Rows, Cols>::operator[](size_t i) const
{
    assert(i < Rows * Cols);

    return m_elements[i];
}

template <typename T>
template <size_t R, size_t C, typename E>
Matrix<T, 1, 1>::Matrix(const MatrixExpression<T, R, C, E>& expression)
{
    assert(expression.GetRows() == 1 && expression.GetCols() == 1);

    x = expression.Eval(0, 0);
}

template <typename T>
Matrix<T, 1, 1>::Matrix(const std::initializer_list<T>& lst)
{
    assert(lst.size() > 0);

    x = *lst.begin();
}

template <typename T>
void Matrix<T, 1, 1>::Fill(const T& val)
{
    x = val;
}

template <typename T>
void Matrix<T, 1, 1>::Fill(const std::function<T(size_t i)>& func)
{
    x = func(0);
}

template <typename T>
void Matrix<T, 1, 1>::Fill(const std::function<T(size_t i, size_t j)>& func)
{
    x = func(0, 0);
}

template <typename T>
void Matrix<T, 1, 1>::Swap(Matrix& other)
{
    std::swap(x, other.x);
}

template <typename T>
constexpr size_t Matrix<T, 1, 1>::GetRows() const
{
    return 1;
}

template <typename T>
constexpr size_t Matrix<T, 1, 1>::GetCols() const
{
    return 1;
}

template <typename T>
typename Matrix<T, 1, 1>::Iterator Matrix<T, 1, 1>::begin()
{
    return &x;
}

template <typename T>
constexpr typename Matrix<T, 1, 1>::ConstIterator Matrix<T, 1, 1>::begin() const
{
    return &x;
}

template <typename T>
typename Matrix<T, 1, 1>::Iterator Matrix<T, 1, 1>::end()
{
    return begin() + 1;
}

template <typename T>
constexpr typename Matrix<T, 1, 1>::ConstIterator Matrix<T, 1, 1>::end() const
{
    return begin() + 1;
}

template <typename T>
typename Matrix<T, 1, 1>::Pointer Matrix<T, 1, 1>::data()
{
    return &x;
}

template <typename T>
constexpr typename Matrix<T, 1, 1>::ConstPointer Matrix<T, 1, 1>::data() const
{
    return &x;
}

template <typename T>
typename Matrix<T, 1, 1>::Reference Matrix<T, 1, 1>::operator[](size_t i)
{
    assert(i < 1);

    return (&x)[i];
}

template <typename T>
typename Matrix<T, 1, 1>::ConstReference Matrix<T, 1, 1>::operator[](
    size_t i) const
{
    assert(i < 1);

    return (&x)[i];
}

template <typename T>
constexpr Matrix<T, 1, 1> Matrix<T, 1, 1>::MakeUnitX()
{
    return Matrix<T, 1, 1>{ 1 };
}

template <typename T>
constexpr Matrix<T, 1, 1> Matrix<T, 1, 1>::MakeUnit(size_t)
{
    return MakeUnitX();
}

template <typename T>
template <size_t R, size_t C, typename E>
Matrix<T, 2, 1>::Matrix(const MatrixExpression<T, R, C, E>& expression)
{
    assert(expression.GetRows() == 2 && expression.GetCols() == 1);

    x = expression.Eval(0, 0);
    y = expression.Eval(1, 0);
}

template <typename T>
Matrix<T, 2, 1>::Matrix(const std::initializer_list<T>& lst)
{
    assert(lst.size() > 1);

    auto iter = lst.begin();
    x = *(iter++);
    y = *(iter);
}

template <typename T>
void Matrix<T, 2, 1>::Fill(const T& val)
{
    x = y = val;
}

template <typename T>
void Matrix<T, 2, 1>::Fill(const std::function<T(size_t i)>& func)
{
    x = func(0);
    y = func(1);
}

template <typename T>
void Matrix<T, 2, 1>::Fill(const std::function<T(size_t i, size_t j)>& func)
{
    x = func(0, 0);
    y = func(1, 0);
}

template <typename T>
void Matrix<T, 2, 1>::Swap(Matrix& other)
{
    std::swap(x, other.x);
    std::swap(y, other.y);
}

template <typename T>
constexpr size_t Matrix<T, 2, 1>::GetRows() const
{
    return 2;
}

template <typename T>
constexpr size_t Matrix<T, 2, 1>::GetCols() const
{
    return 1;
}

template <typename T>
typename Matrix<T, 2, 1>::Iterator Matrix<T, 2, 1>::begin()
{
    return &x;
}

template <typename T>
constexpr typename Matrix<T, 2, 1>::ConstIterator Matrix<T, 2, 1>::begin() const
{
    return &x;
}

template <typename T>
typename Matrix<T, 2, 1>::Iterator Matrix<T, 2, 1>::end()
{
    return begin() + 2;
}

template <typename T>
constexpr typename Matrix<T, 2, 1>::ConstIterator Matrix<T, 2, 1>::end() const
{
    return begin() + 2;
}

template <typename T>
typename Matrix<T, 2, 1>::Pointer Matrix<T, 2, 1>::data()
{
    return &x;
}

template <typename T>
constexpr typename Matrix<T, 2, 1>::ConstPointer Matrix<T, 2, 1>::data() const
{
    return &x;
}

template <typename T>
typename Matrix<T, 2, 1>::Reference Matrix<T, 2, 1>::operator[](size_t i)
{
    assert(i < 2);

    return (&x)[i];
}

template <typename T>
typename Matrix<T, 2, 1>::ConstReference Matrix<T, 2, 1>::operator[](
    size_t i) const
{
    assert(i < 2);

    return (&x)[i];
}

template <typename T>
constexpr Matrix<T, 2, 1> Matrix<T, 2, 1>::MakeUnitX()
{
    return Matrix<T, 2, 1>{ 1, 0 };
}

template <typename T>
constexpr Matrix<T, 2, 1> Matrix<T, 2, 1>::MakeUnitY()
{
    return Matrix<T, 2, 1>{ 0, 1 };
}

template <typename T>
constexpr Matrix<T, 2, 1> Matrix<T, 2, 1>::MakeUnit(size_t i)
{
    return Matrix<T, 2, 1>(i == 0, i == 1);
}

template <typename T>
template <size_t R, size_t C, typename E>
Matrix<T, 3, 1>::Matrix(const MatrixExpression<T, R, C, E>& expression)
{
    assert(expression.GetRows() == 3 && expression.GetCols() == 1);

    x = expression.Eval(0, 0);
    y = expression.Eval(1, 0);
    z = expression.Eval(2, 0);
}

template <typename T>
Matrix<T, 3, 1>::Matrix(const std::initializer_list<T>& lst)
{
    assert(lst.size() > 2);

    auto iter = lst.begin();
    x = *(iter++);
    y = *(iter++);
    z = *(iter);
}

template <typename T>
void Matrix<T, 3, 1>::Fill(const T& val)
{
    x = y = z = val;
}

template <typename T>
void Matrix<T, 3, 1>::Fill(const std::function<T(size_t i)>& func)
{
    x = func(0);
    y = func(1);
    z = func(2);
}

template <typename T>
void Matrix<T, 3, 1>::Fill(const std::function<T(size_t i, size_t j)>& func)
{
    x = func(0, 0);
    y = func(1, 0);
    z = func(2, 0);
}

template <typename T>
void Matrix<T, 3, 1>::Swap(Matrix& other)
{
    std::swap(x, other.x);
    std::swap(y, other.y);
    std::swap(z, other.z);
}

template <typename T>
constexpr size_t Matrix<T, 3, 1>::GetRows() const
{
    return 3;
}

template <typename T>
constexpr size_t Matrix<T, 3, 1>::GetCols() const
{
    return 1;
}

template <typename T>
typename Matrix<T, 3, 1>::Iterator Matrix<T, 3, 1>::begin()
{
    return &x;
}

template <typename T>
constexpr typename Matrix<T, 3, 1>::ConstIterator Matrix<T, 3, 1>::begin() const
{
    return &x;
}

template <typename T>
typename Matrix<T, 3, 1>::Iterator Matrix<T, 3, 1>::end()
{
    return begin() + 3;
}

template <typename T>
constexpr typename Matrix<T, 3, 1>::ConstIterator Matrix<T, 3, 1>::end() const
{
    return begin() + 3;
}

template <typename T>
typename Matrix<T, 3, 1>::Pointer Matrix<T, 3, 1>::data()
{
    return &x;
}

template <typename T>
constexpr typename Matrix<T, 3, 1>::ConstPointer Matrix<T, 3, 1>::data() const
{
    return &x;
}

template <typename T>
typename Matrix<T, 3, 1>::Reference Matrix<T, 3, 1>::operator[](size_t i)
{
    assert(i < 3);

    return (&x)[i];
}

template <typename T>
typename Matrix<T, 3, 1>::ConstReference Matrix<T, 3, 1>::operator[](
    size_t i) const
{
    assert(i < 3);

    return (&x)[i];
}

template <typename T>
constexpr Matrix<T, 3, 1> Matrix<T, 3, 1>::MakeUnitX()
{
    return Matrix<T, 3, 1>{ 1, 0, 0 };
}

template <typename T>
constexpr Matrix<T, 3, 1> Matrix<T, 3, 1>::MakeUnitY()
{
    return Matrix<T, 3, 1>{ 0, 1, 0 };
}

template <typename T>
constexpr Matrix<T, 3, 1> Matrix<T, 3, 1>::MakeUnitZ()
{
    return Matrix<T, 3, 1>{ 0, 0, 1 };
}

template <typename T>
constexpr Matrix<T, 3, 1> Matrix<T, 3, 1>::MakeUnit(size_t i)
{
    return Matrix<T, 3, 1>(i == 0, i == 1, i == 2);
}

template <typename T>
template <size_t R, size_t C, typename E>
Matrix<T, 4, 1>::Matrix(const MatrixExpression<T, R, C, E>& expression)
{
    assert(expression.GetRows() == 4 && expression.GetCols() == 1);

    x = expression.Eval(0, 0);
    y = expression.Eval(1, 0);
    z = expression.Eval(2, 0);
    w = expression.Eval(3, 0);
}

template <typename T>
Matrix<T, 4, 1>::Matrix(const std::initializer_list<T>& lst)
{
    assert(lst.size() > 3);

    auto iter = lst.begin();
    x = *(iter++);
    y = *(iter++);
    z = *(iter++);
    w = *(iter);
}

template <typename T>
void Matrix<T, 4, 1>::Fill(const T& val)
{
    x = y = z = w = val;
}

template <typename T>
void Matrix<T, 4, 1>::Fill(const std::function<T(size_t i)>& func)
{
    x = func(0);
    y = func(1);
    z = func(2);
    w = func(3);
}

template <typename T>
void Matrix<T, 4, 1>::Fill(const std::function<T(size_t i, size_t j)>& func)
{
    x = func(0, 0);
    y = func(1, 0);
    z = func(2, 0);
    w = func(3, 0);
}

template <typename T>
void Matrix<T, 4, 1>::Swap(Matrix& other)
{
    std::swap(x, other.x);
    std::swap(y, other.y);
    std::swap(z, other.z);
    std::swap(w, other.w);
}

template <typename T>
constexpr size_t Matrix<T, 4, 1>::GetRows() const
{
    return 4;
}

template <typename T>
constexpr size_t Matrix<T, 4, 1>::GetCols() const
{
    return 1;
}

template <typename T>
typename Matrix<T, 4, 1>::Iterator Matrix<T, 4, 1>::begin()
{
    return &x;
}

template <typename T>
constexpr typename Matrix<T, 4, 1>::ConstIterator Matrix<T, 4, 1>::begin() const
{
    return &x;
}

template <typename T>
typename Matrix<T, 4, 1>::Iterator Matrix<T, 4, 1>::end()
{
    return begin() + 4;
}

template <typename T>
constexpr typename Matrix<T, 4, 1>::ConstIterator Matrix<T, 4, 1>::end() const
{
    return begin() + 4;
}

template <typename T>
typename Matrix<T, 4, 1>::Pointer Matrix<T, 4, 1>::data()
{
    return &x;
}

template <typename T>
constexpr typename Matrix<T, 4, 1>::ConstPointer Matrix<T, 4, 1>::data() const
{
    return &x;
}

template <typename T>
typename Matrix<T, 4, 1>::Reference Matrix<T, 4, 1>::operator[](size_t i)
{
    assert(i < 4);

    return (&x)[i];
}

template <typename T>
typename Matrix<T, 4, 1>::ConstReference Matrix<T, 4, 1>::operator[](
    size_t i) const
{
    assert(i < 4);

    return (&x)[i];
}

template <typename T>
constexpr Matrix<T, 4, 1> Matrix<T, 4, 1>::MakeUnitX()
{
    return Matrix<T, 4, 1>{ 1, 0, 0, 0 };
}

template <typename T>
constexpr Matrix<T, 4, 1> Matrix<T, 4, 1>::MakeUnitY()
{
    return Matrix<T, 4, 1>{ 0, 1, 0, 0 };
}

template <typename T>
constexpr Matrix<T, 4, 1> Matrix<T, 4, 1>::MakeUnitZ()
{
    return Matrix<T, 4, 1>{ 0, 0, 1, 0 };
}

template <typename T>
constexpr Matrix<T, 4, 1> Matrix<T, 4, 1>::MakeUnitW()
{
    return Matrix<T, 4, 1>{ 0, 0, 0, 1 };
}

template <typename T>
constexpr Matrix<T, 4, 1> Matrix<T, 4, 1>::MakeUnit(size_t i)
{
    return Matrix<T, 4, 1>(i == 0, i == 1, i == 2, i == 3);
}

template <typename T>
Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>::Matrix() = default;

template <typename T>
Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>::Matrix(
    size_t rows, size_t cols, ConstReference value)
{
    m_elements.resize(rows * cols);
    m_rows = rows;
    m_cols = cols;

    Fill(value);
}

template <typename T>
template <size_t R, size_t C, typename E>
Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>::Matrix(
    const MatrixExpression<T, R, C, E>& expression)
    : Matrix(expression.GetRows(), expression.GetCols())
{
    CopyFrom(expression);
}

template <typename T>
Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>::Matrix(
    const NestedInitializerListsT<T, 2>& lst)
{
    size_t i = 0;

    for (auto rows : lst)
    {
        size_t j = 0;

        for (auto col : rows)
        {
            (void)col;
            ++j;
        }

        m_cols = j;
        ++i;
    }

    m_rows = i;
    m_elements.resize(m_rows * m_cols);

    i = 0;

    for (auto rows : lst)
    {
        assert(i < m_rows);

        size_t j = 0;

        for (auto col : rows)
        {
            assert(j < m_cols);

            (*this)(i, j) = col;
            ++j;
        }

        ++i;
    }
}

template <typename T>
Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>::Matrix(size_t rows,
                                                            size_t cols,
                                                            ConstPointer ptr)
    : Matrix(rows, cols)
{
    size_t cnt = 0;

    for (size_t i = 0; i < rows; ++i)
    {
        for (size_t j = 0; j < cols; ++j)
        {
            (*this)(i, j) = ptr[cnt++];
        }
    }
}

template <typename T>
Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>::Matrix(
    const Matrix& other) = default;

template <typename T>
Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>::Matrix(
    Matrix&& other) noexcept = default;

template <typename T>
Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>&
Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>::operator=(
    const Matrix& other) = default;

template <typename T>
Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>&
Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>::operator=(
    Matrix&& other) noexcept
{
    m_elements = std::move(other.m_elements);
    m_rows = other.m_rows;
    m_cols = other.m_cols;
    other.m_rows = 0;
    other.m_cols = 0;
    return *this;
}

template <typename T>
void Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>::Fill(const T& val)
{
    std::fill(m_elements.begin(), m_elements.end(), val);
}

template <typename T>
void Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>::Fill(
    const std::function<T(size_t i)>& func)
{
    for (size_t i = 0; i < m_elements.size(); ++i)
    {
        m_elements[i] = func(i);
    }
}

template <typename T>
void Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>::Fill(
    const std::function<T(size_t i, size_t j)>& func)
{
    for (size_t i = 0; i < GetRows(); ++i)
    {
        for (size_t j = 0; j < GetCols(); ++j)
        {
            (*this)(i, j) = func(i, j);
        }
    }
}

template <typename T>
void Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>::Swap(Matrix& other)
{
    m_elements.swap(other.m_elements);

    std::swap(m_rows, other.m_rows);
    std::swap(m_cols, other.m_cols);
}

template <typename T>
void Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>::Resize(
    size_t rows, size_t cols, ConstReference val)
{
    Matrix newMatrix{ rows, cols, val };
    const size_t minRows = std::min(rows, m_rows);
    const size_t minCols = std::min(cols, m_cols);

    for (size_t i = 0; i < minRows; ++i)
    {
        for (size_t j = 0; j < minCols; ++j)
        {
            newMatrix(i, j) = (*this)(i, j);
        }
    }

    *this = std::move(newMatrix);
}

template <typename T>
void Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>::Clear()
{
    m_elements.clear();

    m_rows = 0;
    m_cols = 0;
}

template <typename T>
size_t Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>::GetRows() const
{
    return m_rows;
}

template <typename T>
size_t Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>::GetCols() const
{
    return m_cols;
}

template <typename T>
typename Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>::Iterator
Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>::begin()
{
    return &m_elements[0];
}

template <typename T>
typename Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>::ConstIterator
Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>::begin() const
{
    return &m_elements[0];
}

template <typename T>
typename Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>::Iterator
Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>::end()
{
    return begin() + m_rows * m_cols;
}

template <typename T>
typename Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>::ConstIterator
Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>::end() const
{
    return begin() + m_rows * m_cols;
}

template <typename T>
typename Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>::Pointer
Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>::data()
{
    return &m_elements[0];
}

template <typename T>
typename Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>::ConstPointer
Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>::data() const
{
    return &m_elements[0];
}

template <typename T>
typename Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>::Reference
Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>::operator[](size_t i)
{
    assert(i < m_rows * m_cols);

    return m_elements[i];
}

template <typename T>
typename Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>::ConstReference
Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>::operator[](size_t i) const
{
    assert(i < m_rows * m_cols);

    return m_elements[i];
}

template <typename T>
Matrix<T, MATRIX_SIZE_DYNAMIC, 1>::Matrix() = default;

template <typename T>
Matrix<T, MATRIX_SIZE_DYNAMIC, 1>::Matrix(size_t rows, ConstReference value)
{
    m_elements.resize(rows, value);
}

template <typename T>
template <size_t R, size_t C, typename E>
Matrix<T, MATRIX_SIZE_DYNAMIC, 1>::Matrix(
    const MatrixExpression<T, R, C, E>& expression)
    : Matrix(expression.GetRows(), 1)
{
    CopyFrom(expression);
}

template <typename T>
Matrix<T, MATRIX_SIZE_DYNAMIC, 1>::Matrix(const std::initializer_list<T>& lst)
{
    size_t sz = lst.size();
    m_elements.resize(sz);

    size_t i = 0;

    for (auto row : lst)
    {
        m_elements[i] = static_cast<T>(row);
        ++i;
    }
}

template <typename T>
Matrix<T, MATRIX_SIZE_DYNAMIC, 1>::Matrix(size_t rows, ConstPointer ptr)
    : Matrix(rows)
{
    size_t cnt = 0;

    for (size_t i = 0; i < rows; ++i)
    {
        (*this)[i] = ptr[cnt++];
    }
}

template <typename T>
Matrix<T, MATRIX_SIZE_DYNAMIC, 1>::Matrix(const Matrix& other) = default;

template <typename T>
Matrix<T, MATRIX_SIZE_DYNAMIC, 1>::Matrix(Matrix&& other) noexcept = default;

template <typename T>
Matrix<T, MATRIX_SIZE_DYNAMIC, 1>& Matrix<T, MATRIX_SIZE_DYNAMIC, 1>::operator=(
    const Matrix& other) = default;

template <typename T>
Matrix<T, MATRIX_SIZE_DYNAMIC, 1>& Matrix<T, MATRIX_SIZE_DYNAMIC, 1>::operator=(
    Matrix&& other) noexcept = default;

template <typename T>
void Matrix<T, MATRIX_SIZE_DYNAMIC, 1>::Fill(const T& val)
{
    std::fill(m_elements.begin(), m_elements.end(), val);
}

template <typename T>
void Matrix<T, MATRIX_SIZE_DYNAMIC, 1>::Fill(
    const std::function<T(size_t i)>& func)
{
    for (size_t i = 0; i < m_elements.size(); ++i)
    {
        m_elements[i] = func(i);
    }
}

template <typename T>
void Matrix<T, MATRIX_SIZE_DYNAMIC, 1>::Fill(
    const std::function<T(size_t i, size_t j)>& func)
{
    for (size_t i = 0; i < GetRows(); ++i)
    {
        m_elements[i] = func(i, 0);
    }
}

template <typename T>
void Matrix<T, MATRIX_SIZE_DYNAMIC, 1>::Swap(Matrix& other)
{
    m_elements.swap(other.m_elements);
}

template <typename T>
void Matrix<T, MATRIX_SIZE_DYNAMIC, 1>::Resize(size_t rows, ConstReference val)
{
    m_elements.resize(rows, val);
}

template <typename T>
void Matrix<T, MATRIX_SIZE_DYNAMIC, 1>::AddElement(ConstReference newElem)
{
    m_elements.push_back(newElem);
}

template <typename T>
void Matrix<T, MATRIX_SIZE_DYNAMIC, 1>::AddElement(const Matrix& newElems)
{
    m_elements.insert(m_elements.end(), newElems.m_elements.begin(),
                      newElems.m_elements.end());
}

template <typename T>
void Matrix<T, MATRIX_SIZE_DYNAMIC, 1>::Clear()
{
    m_elements.clear();
}

template <typename T>
size_t Matrix<T, MATRIX_SIZE_DYNAMIC, 1>::GetRows() const
{
    return m_elements.size();
}

template <typename T>
constexpr size_t Matrix<T, MATRIX_SIZE_DYNAMIC, 1>::GetCols() const
{
    return 1;
}

template <typename T>
typename Matrix<T, MATRIX_SIZE_DYNAMIC, 1>::Iterator
Matrix<T, MATRIX_SIZE_DYNAMIC, 1>::begin()
{
    return &m_elements[0];
}

template <typename T>
typename Matrix<T, MATRIX_SIZE_DYNAMIC, 1>::ConstIterator
Matrix<T, MATRIX_SIZE_DYNAMIC, 1>::begin() const
{
    return &m_elements[0];
}

template <typename T>
typename Matrix<T, MATRIX_SIZE_DYNAMIC, 1>::Iterator
Matrix<T, MATRIX_SIZE_DYNAMIC, 1>::end()
{
    return begin() + m_elements.size();
}

template <typename T>
typename Matrix<T, MATRIX_SIZE_DYNAMIC, 1>::ConstIterator
Matrix<T, MATRIX_SIZE_DYNAMIC, 1>::end() const
{
    return begin() + m_elements.size();
}

template <typename T>
typename Matrix<T, MATRIX_SIZE_DYNAMIC, 1>::Pointer
Matrix<T, MATRIX_SIZE_DYNAMIC, 1>::data()
{
    return &m_elements[0];
}

template <typename T>
typename Matrix<T, MATRIX_SIZE_DYNAMIC, 1>::ConstPointer
Matrix<T, MATRIX_SIZE_DYNAMIC, 1>::data() const
{
    return &m_elements[0];
}

template <typename T>
typename Matrix<T, MATRIX_SIZE_DYNAMIC, 1>::Reference
Matrix<T, MATRIX_SIZE_DYNAMIC, 1>::operator[](size_t i)
{
    assert(i < m_elements.size());

    return m_elements[i];
}

template <typename T>
typename Matrix<T, MATRIX_SIZE_DYNAMIC, 1>::ConstReference
Matrix<T, MATRIX_SIZE_DYNAMIC, 1>::operator[](size_t i) const
{
    assert(i < m_elements.size());

    return m_elements[i];
}

template <typename T, size_t R1, size_t C1, size_t R2, size_t C2, typename M2>
void operator+=(Matrix<T, R1, C1>& a, const MatrixExpression<T, R2, C2, M2>& b)
{
    a = a + b;
}

template <typename T, size_t Rows, size_t Cols>
void operator+=(Matrix<T, Rows, Cols>& a, const T& b)
{
    a = a + b;
}

template <typename T, size_t R1, size_t C1, size_t R2, size_t C2, typename M2>
void operator-=(Matrix<T, R1, C1>& a, const MatrixExpression<T, R2, C2, M2>& b)
{
    a = a - b;
}

template <typename T, size_t Rows, size_t Cols>
void operator-=(Matrix<T, Rows, Cols>& a, const T& b)
{
    a = a - b;
}

template <typename T, size_t R1, size_t C1, size_t R2, size_t C2, typename M2>
void operator*=(Matrix<T, R1, C1>& a, const MatrixExpression<T, R2, C2, M2>& b)
{
    assert(a.GetCols() == b.GetRows());

    Matrix<T, R1, C2> c(a * b);
    a = c;
}

template <typename T, size_t R1, size_t C1, size_t R2, size_t C2, typename M2>
void ElemIMul(Matrix<T, R1, C1>& a, const MatrixExpression<T, R2, C2, M2>& b)
{
    assert(a.GetRows() == b.GetRows() && a.GetCols() == b.GetCols());

    a = MatrixElemWiseMul<T, R1, C1, const Matrix<T, R1, C1>&, const M2&>{
        a, b.GetDerived()
    };
}

template <typename T, size_t Rows, size_t Cols>
void operator*=(Matrix<T, Rows, Cols>& a, const T& b)
{
    a = MatrixScalarElemWiseMul<T, Rows, Cols, const Matrix<T, Rows, Cols>&>{
        a, b
    };
}

template <typename T, size_t R1, size_t C1, size_t R2, size_t C2, typename M2>
void ElemIDiv(Matrix<T, R1, C1>& a, const MatrixExpression<T, R2, C2, M2>& b)
{
    a = MatrixElemWiseDiv<T, R1, C1, const Matrix<T, R1, C1>&, const M2&>(
        a, b.GetDerived());
}

template <typename T, size_t Rows, size_t Cols>
void operator/=(Matrix<T, Rows, Cols>& a, const T& b)
{
    a = MatrixScalarElemWiseDiv<T, Rows, Cols, const Matrix<T, Rows, Cols>&>{
        a, b
    };
}

template <typename T, size_t Rows, size_t Cols, typename M1, typename M2>
constexpr std::enable_if_t<IsMatrixSizeStatic<Rows, Cols>(), bool> operator==(
    const MatrixExpression<T, Rows, Cols, M1>& a,
    const MatrixExpression<T, Rows, Cols, M2>& b)
{
    return Internal::FoldWithAnd<T, Rows, Cols, std::equal_to<T>,
                                 Rows * Cols - 1>::Call(a, b,
                                                        std::equal_to<T>());
}

template <typename T, size_t R1, size_t C1, size_t R2, size_t C2, typename M1,
          typename M2>
bool operator==(const MatrixExpression<T, R1, C1, M1>& a,
                const MatrixExpression<T, R2, C2, M2>& b)
{
    if (a.GetRows() != b.GetRows() || a.GetCols() != b.GetCols())
    {
        return false;
    }

    for (size_t i = 0; i < a.GetRows(); ++i)
    {
        for (size_t j = 0; j < a.GetCols(); ++j)
        {
            if (a.Eval(i, j) != b.Eval(i, j))
            {
                return false;
            }
        }
    }

    return true;
}

template <typename T, size_t R1, size_t C1, size_t R2, size_t C2, typename M1,
          typename M2>
bool operator!=(const MatrixExpression<T, R1, C1, M1>& a,
                const MatrixExpression<T, R2, C2, M2>& b)
{
    return !(a == b);
}

template <typename T, size_t Rows, size_t Cols, typename M1,
          typename BinaryOperation>
constexpr std::enable_if_t<TraitIsMatrixSizeStatic<Rows, Cols>::value, T>
Accumulate(const MatrixExpression<T, Rows, Cols, M1>& a, const T& init,
           BinaryOperation op)
{
    return Internal::Reduce<T, Rows, Cols, BinaryOperation, NoOp<T>,
                            Rows * Cols - 1>::Call(Matrix<T, Rows, Cols>(a),
                                                   init, op, NoOp<T>());
}

template <typename T, size_t Rows, size_t Cols, typename M1>
constexpr std::enable_if_t<TraitIsMatrixSizeStatic<Rows, Cols>::value, T>
Accumulate(const MatrixExpression<T, Rows, Cols, M1>& a, const T& init)
{
    return Internal::Reduce<T, Rows, Cols, std::plus<T>, NoOp<T>,
                            Rows * Cols - 1>::Call(Matrix<T, Rows, Cols>(a),
                                                   init, std::plus<T>(),
                                                   NoOp<T>());
}

template <typename T, size_t Rows, size_t Cols, typename M1>
constexpr std::enable_if_t<TraitIsMatrixSizeStatic<Rows, Cols>::value, T>
Accumulate(const MatrixExpression<T, Rows, Cols, M1>& a)
{
    return Internal::Reduce<T, Rows, Cols, std::plus<T>, NoOp<T>,
                            Rows * Cols - 1>::Call(Matrix<T, Rows, Cols>(a),
                                                   std::plus<T>(), NoOp<T>());
}

template <typename T, size_t Rows, size_t Cols, typename M1,
          typename BinaryOperation>
constexpr std::enable_if_t<TraitIsMatrixSizeDynamic<Rows, Cols>::value, T>
Accumulate(const MatrixExpression<T, Rows, Cols, M1>& a, const T& init,
           BinaryOperation op)
{
    return std::accumulate(a.begin(), a.end(), init, op);
}

template <typename T, size_t Rows, size_t Cols, typename M1>
constexpr std::enable_if_t<TraitIsMatrixSizeDynamic<Rows, Cols>::value, T>
Accumulate(const MatrixExpression<T, Rows, Cols, M1>& a, const T& init)
{
    return std::accumulate(a.begin(), a.end(), init, std::plus<T>());
}

template <typename T, size_t Rows, size_t Cols, typename M1>
constexpr std::enable_if_t<TraitIsMatrixSizeDynamic<Rows, Cols>::value, T>
Accumulate(const MatrixExpression<T, Rows, Cols, M1>& a)
{
    return std::accumulate(a.begin(), a.end(), T{}, std::plus<T>());
}

// Product

template <typename T, size_t Rows, size_t Cols, typename M1>
constexpr T Product(const MatrixExpression<T, Rows, Cols, M1>& a, const T& init)
{
    return Accumulate(a, init, std::multiplies<T>());
}

// Interpolation
template <typename T, size_t Rows, size_t Cols, typename M1, typename M2,
          typename M3, typename M4>
std::enable_if_t<IsMatrixSizeStatic<Rows, Cols>(), Matrix<T, Rows, Cols>>
MonotonicCatmullRom(const MatrixExpression<T, Rows, Cols, M1>& f0,
                    const MatrixExpression<T, Rows, Cols, M2>& f1,
                    const MatrixExpression<T, Rows, Cols, M3>& f2,
                    const MatrixExpression<T, Rows, Cols, M4>& f3, T f)
{
    Matrix<T, Rows, Cols> result;

    for (size_t i = 0; i < f0.GetRows(); ++i)
    {
        for (size_t j = 0; j < f0.GetCols(); ++j)
        {
            result(i, j) = MonotonicCatmullRom(f0.Eval(i, j), f1.Eval(i, j),
                                               f2.Eval(i, j), f3.Eval(i, j), f);
        }
    }

    return result;
}
}  // namespace CubbyFlow

#endif
