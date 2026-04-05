// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_MATRIX_CSR_IMPL_HPP
#define CUBBYFLOW_MATRIX_CSR_IMPL_HPP

#include <Core/Math/MathUtils.hpp>
#include <Core/Utils/CppUtils.hpp>
#include <Core/Utils/Parallel.hpp>

#include <utility>

namespace CubbyFlow
{
template <typename T, typename ME>
MatrixCSRMatrixMul<T, ME>::MatrixCSRMatrixMul(const MatrixCSR<T>& m1,
                                              const ME& m2)
    : m_m1(m1),
      m_m2(m2),
      m_nnz(m1.NonZeroData()),
      m_rp(m1.RowPointersData()),
      m_ci(m1.ColumnIndicesData())
{
    // Do nothing
}

template <typename T, typename ME>
size_t MatrixCSRMatrixMul<T, ME>::GetRows() const
{
    return m_m1.GetRows();
}

template <typename T, typename ME>
size_t MatrixCSRMatrixMul<T, ME>::GetCols() const
{
    return m_m2.GetCols();
}

template <typename T, typename ME>
T MatrixCSRMatrixMul<T, ME>::operator()(size_t i, size_t j) const
{
    const size_t colBegin = m_rp[i];
    const size_t colEnd = m_rp[i + 1];

    T sum = 0;

    for (size_t kk = colBegin; kk < colEnd; ++kk)
    {
        size_t k = m_ci[kk];
        sum += m_nnz[kk] * m_m2(k, j);
    }

    return sum;
}

template <typename T>
MatrixCSR<T>::Element::Element() : i(0), j(0), value(0)
{
    // Do nothing
}

template <typename T>
MatrixCSR<T>::Element::Element(size_t i_, size_t j_, const T& value_)
    : i(i_), j(j_), value(value_)
{
    // Do nothing
}

template <typename T>
MatrixCSR<T>::MatrixCSR()
{
    Clear();
}

template <typename T>
MatrixCSR<T>::MatrixCSR(
    const std::initializer_list<std::initializer_list<T>>& lst, T epsilon)
{
    Compress(lst, epsilon);
}

template <typename T>
template <size_t R, size_t C, typename ME>
MatrixCSR<T>::MatrixCSR(const MatrixExpression<T, R, C, ME>& other, T epsilon)
{
    Compress(other, epsilon);
}

template <typename T>
MatrixCSR<T>::MatrixCSR(const MatrixCSR& other)
{
    Set(other);
}

template <typename T>
MatrixCSR<T>::MatrixCSR(MatrixCSR&& other) noexcept
    : m_size(std::exchange(other.m_size, Vector2UZ{})),
      m_nonZeros(std::move(other.m_nonZeros)),
      m_rowPointers(std::move(other.m_rowPointers)),
      m_columnIndices(std::move(other.m_columnIndices))
{
    // Do nothing
}

template <typename T>
MatrixCSR<T>& MatrixCSR<T>::operator=(const MatrixCSR& other)
{
    Set(other);

    return *this;
}

template <typename T>
MatrixCSR<T>& MatrixCSR<T>::operator=(MatrixCSR&& other) noexcept
{
    m_size = other.m_size;
    other.m_size = Vector2UZ{};
    m_nonZeros = std::move(other.m_nonZeros);
    m_rowPointers = std::move(other.m_rowPointers);
    m_columnIndices = std::move(other.m_columnIndices);

    return *this;
}

template <typename T>
void MatrixCSR<T>::Clear()
{
    m_size = { 0, 0 };
    m_nonZeros.clear();
    m_rowPointers.clear();
    m_columnIndices.clear();
    m_rowPointers.push_back(0);
}

template <typename T>
void MatrixCSR<T>::Set(const T& s)
{
    std::fill(m_nonZeros.begin(), m_nonZeros.end(), s);
}

template <typename T>
void MatrixCSR<T>::Set(const MatrixCSR& other)
{
    m_size = other.m_size;
    m_nonZeros = other.m_nonZeros;
    m_rowPointers = other.m_rowPointers;
    m_columnIndices = other.m_columnIndices;
}

template <typename T>
void MatrixCSR<T>::Reserve(size_t rows, size_t cols, size_t numNonZeros)
{
    m_size = Vector2UZ(rows, cols);
    m_rowPointers.resize(m_size.x + 1);
    m_nonZeros.resize(numNonZeros);
    m_columnIndices.resize(numNonZeros);
}

template <typename T>
void MatrixCSR<T>::Compress(
    const std::initializer_list<std::initializer_list<T>>& lst, T epsilon)
{
    const size_t numRows = lst.size();
    const size_t numCols = (numRows > 0) ? lst.begin()->size() : 0;

    m_size = { numRows, numCols };
    m_nonZeros.clear();
    m_rowPointers.clear();
    m_columnIndices.clear();

    auto rowIter = lst.begin();
    for (size_t i = 0; i < numRows; ++i)
    {
        assert(numCols == rowIter->size());

        m_rowPointers.push_back(m_nonZeros.size());

        auto colIter = rowIter->begin();
        for (size_t j = 0; j < numCols; ++j)
        {
            if (std::fabs(*colIter) > epsilon)
            {
                m_nonZeros.push_back(*colIter);
                m_columnIndices.push_back(j);
            }

            ++colIter;
        }

        ++rowIter;
    }

    m_rowPointers.push_back(m_nonZeros.size());
}

template <typename T>
template <size_t R, size_t C, typename E>
void MatrixCSR<T>::Compress(const MatrixExpression<T, R, C, E>& other,
                            T epsilon)
{
    const size_t numRows = other.GetRows();
    const size_t numCols = other.GetCols();

    m_size = { numRows, numCols };
    m_nonZeros.clear();
    m_columnIndices.clear();

    const E& expression = other.GetDerived();

    for (size_t i = 0; i < numRows; ++i)
    {
        m_rowPointers.push_back(m_nonZeros.size());

        for (size_t j = 0; j < numCols; ++j)
        {
            if (T val = expression(i, j); std::fabs(val) > epsilon)
            {
                m_nonZeros.push_back(val);
                m_columnIndices.push_back(j);
            }
        }
    }

    m_rowPointers.push_back(m_nonZeros.size());
}

template <typename T>
void MatrixCSR<T>::AddElement(size_t i, size_t j, const T& value)
{
    AddElement({ i, j, value });
}

template <typename T>
void MatrixCSR<T>::AddElement(const Element& element)
{
    if (const ssize_t numRowsToAdd = static_cast<ssize_t>(element.i) -
                                     static_cast<ssize_t>(m_size.x) + 1;
        numRowsToAdd > 0)
    {
        for (ssize_t i = 0; i < numRowsToAdd; ++i)
        {
            AddRow({}, {});
        }
    }

    m_size.y = std::max(m_size.y, element.j + 1);

    const size_t rowBegin = m_rowPointers[element.i];
    const size_t rowEnd = m_rowPointers[element.i + 1];

    auto colIdxIter =
        std::lower_bound(m_columnIndices.begin() + rowBegin,
                         m_columnIndices.begin() + rowEnd, element.j);
    auto offset = colIdxIter - m_columnIndices.begin();

    m_columnIndices.insert(colIdxIter, element.j);
    m_nonZeros.insert(m_nonZeros.begin() + offset, element.value);

    for (size_t i = element.i + 1; i < m_rowPointers.size(); ++i)
    {
        ++m_rowPointers[i];
    }
}

template <typename T>
void MatrixCSR<T>::AddRow(const NonZeroContainerType& nonZeros,
                          const IndexContainerType& columnIndices)
{
    assert(nonZeros.size() == columnIndices.size());

    ++m_size.x;

    // TODO: Implement zip iterator
    std::vector<std::pair<T, size_t>> zipped;
    for (size_t i = 0; i < nonZeros.size(); ++i)
    {
        zipped.emplace_back(nonZeros[i], columnIndices[i]);
        m_size.y = std::max(m_size.y, columnIndices[i] + 1);
    }

    std::sort(zipped.begin(), zipped.end(),
              [](std::pair<T, size_t> a, std::pair<T, size_t> b) {
                  return a.second < b.second;
              });

    for (size_t i = 0; i < zipped.size(); ++i)
    {
        m_nonZeros.push_back(zipped[i].first);
        m_columnIndices.push_back(zipped[i].second);
    }

    m_rowPointers.push_back(m_nonZeros.size());
}

template <typename T>
void MatrixCSR<T>::SetElement(size_t i, size_t j, const T& value)
{
    SetElement({ i, j, value });
}

template <typename T>
void MatrixCSR<T>::SetElement(const Element& element)
{
    if (size_t nzIndex = HasElement(element.i, element.j);
        nzIndex == std::numeric_limits<size_t>::max())
    {
        AddElement(element);
    }
    else
    {
        m_nonZeros[nzIndex] = element.value;
    }
}

template <typename T>
bool MatrixCSR<T>::IsEqual(const MatrixCSR& other) const
{
    if (m_size != other.m_size)
    {
        return false;
    }

    if (m_nonZeros.size() != other.m_nonZeros.size())
    {
        return false;
    }

    for (size_t i = 0; i < m_nonZeros.size(); ++i)
    {
        if (m_nonZeros[i] != other.m_nonZeros[i])
        {
            return false;
        }

        if (m_columnIndices[i] != other.m_columnIndices[i])
        {
            return false;
        }
    }

    for (size_t i = 0; i < m_rowPointers.size(); ++i)
    {
        if (m_rowPointers[i] != other.m_rowPointers[i])
        {
            return false;
        }
    }

    return true;
}

template <typename T>
bool MatrixCSR<T>::IsSimilar(const MatrixCSR& other, double tol) const
{
    if (m_size != other.m_size)
    {
        return false;
    }

    if (m_nonZeros.size() != other.m_nonZeros.size())
    {
        return false;
    }

    for (size_t i = 0; i < m_nonZeros.size(); ++i)
    {
        if (std::fabs(m_nonZeros[i] - other.m_nonZeros[i]) > tol)
        {
            return false;
        }

        if (m_columnIndices[i] != other.m_columnIndices[i])
        {
            return false;
        }
    }

    for (size_t i = 0; i < m_rowPointers.size(); ++i)
    {
        if (m_rowPointers[i] != other.m_rowPointers[i])
        {
            return false;
        }
    }

    return true;
}

template <typename T>
bool MatrixCSR<T>::IsSquare() const
{
    return GetRows() == GetCols();
}

template <typename T>
Vector2UZ MatrixCSR<T>::Size() const
{
    return m_size;
}

template <typename T>
size_t MatrixCSR<T>::GetRows() const
{
    return m_size.x;
}

template <typename T>
size_t MatrixCSR<T>::GetCols() const
{
    return m_size.y;
}

template <typename T>
size_t MatrixCSR<T>::NumberOfNonZeros() const
{
    return m_nonZeros.size();
}

template <typename T>
const T& MatrixCSR<T>::NonZero(size_t i) const
{
    return m_nonZeros[i];
}

template <typename T>
T& MatrixCSR<T>::NonZero(size_t i)
{
    return m_nonZeros[i];
}

template <typename T>
const size_t& MatrixCSR<T>::RowPointer(size_t i) const
{
    return m_rowPointers[i];
}

template <typename T>
const size_t& MatrixCSR<T>::ColumnIndex(size_t i) const
{
    return m_columnIndices[i];
}

template <typename T>
T* MatrixCSR<T>::NonZeroData()
{
    return m_nonZeros.data();
}

template <typename T>
const T* MatrixCSR<T>::NonZeroData() const
{
    return m_nonZeros.data();
}

template <typename T>
const size_t* MatrixCSR<T>::RowPointersData() const
{
    return m_rowPointers.data();
}

template <typename T>
const size_t* MatrixCSR<T>::ColumnIndicesData() const
{
    return m_columnIndices.data();
}

template <typename T>
typename MatrixCSR<T>::NonZeroIterator MatrixCSR<T>::NonZeroBegin()
{
    return m_nonZeros.begin();
}

template <typename T>
typename MatrixCSR<T>::ConstNonZeroIterator MatrixCSR<T>::NonZeroBegin() const
{
    return m_nonZeros.cbegin();
}

template <typename T>
typename MatrixCSR<T>::NonZeroIterator MatrixCSR<T>::NonZeroEnd()
{
    return m_nonZeros.end();
}

template <typename T>
typename MatrixCSR<T>::ConstNonZeroIterator MatrixCSR<T>::NonZeroEnd() const
{
    return m_nonZeros.cend();
}

template <typename T>
typename MatrixCSR<T>::IndexIterator MatrixCSR<T>::RowPointersBegin()
{
    return m_rowPointers.begin();
}

template <typename T>
typename MatrixCSR<T>::ConstIndexIterator MatrixCSR<T>::RowPointersBegin() const
{
    return m_rowPointers.cbegin();
}

template <typename T>
typename MatrixCSR<T>::IndexIterator MatrixCSR<T>::RowPointersEnd()
{
    return m_rowPointers.end();
}

template <typename T>
typename MatrixCSR<T>::ConstIndexIterator MatrixCSR<T>::RowPointersEnd() const
{
    return m_rowPointers.cend();
}

template <typename T>
typename MatrixCSR<T>::IndexIterator MatrixCSR<T>::ColumnIndicesBegin()
{
    return m_columnIndices.begin();
}

template <typename T>
typename MatrixCSR<T>::ConstIndexIterator MatrixCSR<T>::ColumnIndicesBegin()
    const
{
    return m_columnIndices.cbegin();
}

template <typename T>
typename MatrixCSR<T>::IndexIterator MatrixCSR<T>::ColumnIndicesEnd()
{
    return m_columnIndices.end();
}

template <typename T>
typename MatrixCSR<T>::ConstIndexIterator MatrixCSR<T>::ColumnIndicesEnd() const
{
    return m_columnIndices.cend();
}

template <typename T>
MatrixCSR<T> MatrixCSR<T>::Add(const T& s) const
{
    MatrixCSR ret(*this);

    ParallelFor(ZERO_SIZE, ret.m_nonZeros.size(),
                [&](size_t i) { ret.m_nonZeros[i] += s; });

    return ret;
}

template <typename T>
MatrixCSR<T> MatrixCSR<T>::Add(const MatrixCSR& m) const
{
    return BinaryOp(m, std::plus<T>());
}

template <typename T>
MatrixCSR<T> MatrixCSR<T>::Sub(const T& s) const
{
    MatrixCSR ret(*this);

    ParallelFor(ZERO_SIZE, ret.m_nonZeros.size(),
                [&](size_t i) { ret.m_nonZeros[i] -= s; });

    return ret;
}

template <typename T>
MatrixCSR<T> MatrixCSR<T>::Sub(const MatrixCSR& m) const
{
    return BinaryOp(m, std::minus<T>());
}

template <typename T>
MatrixCSR<T> MatrixCSR<T>::Mul(const T& s) const
{
    MatrixCSR ret(*this);

    ParallelFor(ZERO_SIZE, ret.m_nonZeros.size(),
                [&](size_t i) { ret.m_nonZeros[i] *= s; });

    return ret;
}

template <typename T>
template <size_t R, size_t C, typename ME>
MatrixCSRMatrixMul<T, ME> MatrixCSR<T>::Mul(
    const MatrixExpression<T, R, C, ME>& m) const
{
    return MatrixCSRMatrixMul<T, ME>(*this, m.GetDerived());
}

template <typename T>
MatrixCSR<T> MatrixCSR<T>::Div(const T& s) const
{
    MatrixCSR ret(*this);

    ParallelFor(ZERO_SIZE, ret.m_nonZeros.size(),
                [&](size_t i) { ret.m_nonZeros[i] /= s; });

    return ret;
}

template <typename T>
MatrixCSR<T> MatrixCSR<T>::RAdd(const T& s) const
{
    return Add(s);
}

template <typename T>
MatrixCSR<T> MatrixCSR<T>::RAdd(const MatrixCSR& m) const
{
    return Add(m);
}

template <typename T>
MatrixCSR<T> MatrixCSR<T>::RSub(const T& s) const
{
    MatrixCSR ret(*this);

    ParallelFor(ZERO_SIZE, ret.m_nonZeros.size(),
                [&](size_t i) { ret.m_nonZeros[i] = s - ret.m_nonZeros[i]; });

    return ret;
}

template <typename T>
MatrixCSR<T> MatrixCSR<T>::RSub(const MatrixCSR& m) const
{
    return m.Sub(*this);
}

template <typename T>
MatrixCSR<T> MatrixCSR<T>::RMul(const T& s) const
{
    return Mul(s);
}

template <typename T>
MatrixCSR<T> MatrixCSR<T>::RDiv(const T& s) const
{
    MatrixCSR ret(*this);

    ParallelFor(ZERO_SIZE, ret.m_nonZeros.size(),
                [&](size_t i) { ret.m_nonZeros[i] = s / ret.m_nonZeros[i]; });

    return ret;
}

template <typename T>
void MatrixCSR<T>::IAdd(const T& s)
{
    ParallelFor(ZERO_SIZE, m_nonZeros.size(),
                [&](size_t i) { m_nonZeros[i] += s; });
}

template <typename T>
void MatrixCSR<T>::IAdd(const MatrixCSR& m)
{
    Set(Add(m));
}

template <typename T>
void MatrixCSR<T>::ISub(const T& s)
{
    ParallelFor(ZERO_SIZE, m_nonZeros.size(),
                [&](size_t i) { m_nonZeros[i] -= s; });
}

template <typename T>
void MatrixCSR<T>::ISub(const MatrixCSR& m)
{
    Set(Sub(m));
}

template <typename T>
void MatrixCSR<T>::IMul(const T& s)
{
    ParallelFor(ZERO_SIZE, m_nonZeros.size(),
                [&](size_t i) { m_nonZeros[i] *= s; });
}

template <typename T>
template <size_t R, size_t C, typename ME>
void MatrixCSR<T>::IMul(const MatrixExpression<T, R, C, ME>& m)
{
    MatrixCSRD result = Mul(m);

    *this = std::move(result);
}

template <typename T>
void MatrixCSR<T>::IDiv(const T& s)
{
    ParallelFor(ZERO_SIZE, m_nonZeros.size(),
                [&](size_t i) { m_nonZeros[i] /= s; });
}

template <typename T>
T MatrixCSR<T>::Sum() const
{
    return ParallelReduce(
        ZERO_SIZE, NumberOfNonZeros(), T(0),
        [&](size_t start, size_t end, T init) {
            T result = init;

            for (size_t i = start; i < end; ++i)
            {
                result += m_nonZeros[i];
            }

            return result;
        },
        std::plus<T>());
}

template <typename T>
T MatrixCSR<T>::Avg() const
{
    return Sum() / NumberOfNonZeros();
}

template <typename T>
T MatrixCSR<T>::Min() const
{
    const T& (*_min)(const T&, const T&) = std::min<T>;

    return ParallelReduce(
        ZERO_SIZE, NumberOfNonZeros(), std::numeric_limits<T>::max(),
        [&](size_t start, size_t end, T init) {
            T result = init;

            for (size_t i = start; i < end; ++i)
            {
                result = std::min(result, m_nonZeros[i]);
            }

            return result;
        },
        _min);
}

template <typename T>
T MatrixCSR<T>::Max() const
{
    const T& (*_max)(const T&, const T&) = std::max<T>;

    return ParallelReduce(
        ZERO_SIZE, NumberOfNonZeros(), std::numeric_limits<T>::min(),
        [&](size_t start, size_t end, T init) {
            T result = init;

            for (size_t i = start; i < end; ++i)
            {
                result = std::max(result, m_nonZeros[i]);
            }

            return result;
        },
        _max);
}

template <typename T>
T MatrixCSR<T>::AbsMin() const
{
    return ParallelReduce(
        ZERO_SIZE, NumberOfNonZeros(), std::numeric_limits<T>::max(),
        [&](size_t start, size_t end, T init) {
            T result = init;

            for (size_t i = start; i < end; ++i)
            {
                result = CubbyFlow::AbsMin(result, m_nonZeros[i]);
            }

            return result;
        },
        CubbyFlow::AbsMin<T>);
}

template <typename T>
T MatrixCSR<T>::AbsMax() const
{
    return ParallelReduce(
        ZERO_SIZE, NumberOfNonZeros(), T(0),
        [&](size_t start, size_t end, T init) {
            T result = init;

            for (size_t i = start; i < end; ++i)
            {
                result = CubbyFlow::AbsMax(result, m_nonZeros[i]);
            }

            return result;
        },
        CubbyFlow::AbsMax<T>);
}

template <typename T>
T MatrixCSR<T>::Trace() const
{
    assert(IsSquare());

    return ParallelReduce(
        ZERO_SIZE, GetRows(), T(0),
        [&](size_t start, size_t end, T init) {
            T result = init;

            for (size_t i = start; i < end; ++i)
            {
                result += (*this)(i, i);
            }

            return result;
        },
        std::plus<T>());
}

template <typename T>
template <typename U>
MatrixCSR<U> MatrixCSR<T>::CastTo() const
{
    MatrixCSR<U> ret;
    ret.Reserve(GetRows(), GetCols(), NumberOfNonZeros());

    auto nnz = ret.NonZeroBegin();
    auto ci = ret.ColumnIndicesBegin();
    auto rp = ret.RowPointersBegin();

    ParallelFor(ZERO_SIZE, m_nonZeros.size(), [&](size_t i) {
        nnz[i] = static_cast<U>(m_nonZeros[i]);
        ci[i] = m_columnIndices[i];
    });

    ParallelFor(ZERO_SIZE, m_rowPointers.size(),
                [&](size_t i) { rp[i] = m_rowPointers[i]; });

    return ret;
}

template <typename T>
template <size_t R, size_t C, typename E>
MatrixCSR<T>& MatrixCSR<T>::operator=(const MatrixExpression<T, R, C, E>& m)
{
    Set(m);

    return *this;
}

template <typename T>
MatrixCSR<T>& MatrixCSR<T>::operator+=(const T& s)
{
    IAdd(s);

    return *this;
}

template <typename T>
MatrixCSR<T>& MatrixCSR<T>::operator+=(const MatrixCSR& m)
{
    IAdd(m);

    return *this;
}

template <typename T>
MatrixCSR<T>& MatrixCSR<T>::operator-=(const T& s)
{
    ISub(s);

    return *this;
}

template <typename T>
MatrixCSR<T>& MatrixCSR<T>::operator-=(const MatrixCSR& m)
{
    ISub(m);

    return *this;
}

template <typename T>
MatrixCSR<T>& MatrixCSR<T>::operator*=(const T& s)
{
    IMul(s);

    return *this;
}

template <typename T>
template <size_t R, size_t C, typename ME>
MatrixCSR<T>& MatrixCSR<T>::operator*=(const MatrixExpression<T, R, C, ME>& m)
{
    IMul(m);

    return *this;
}

template <typename T>
MatrixCSR<T>& MatrixCSR<T>::operator/=(const T& s)
{
    IDiv(s);

    return *this;
}

template <typename T>
T MatrixCSR<T>::operator()(size_t i, size_t j) const
{
    size_t nzIndex = HasElement(i, j);

    if (nzIndex == std::numeric_limits<size_t>::max())
    {
        return 0.0;
    }

    return m_nonZeros[nzIndex];
}

template <typename T>
bool MatrixCSR<T>::operator==(const MatrixCSR& m) const
{
    return IsEqual(m);
}

template <typename T>
bool MatrixCSR<T>::operator!=(const MatrixCSR& m) const
{
    return !IsEqual(m);
}

template <typename T>
MatrixCSR<T> MatrixCSR<T>::MakeIdentity(size_t m)
{
    MatrixCSR ret;

    ret.m_size = Vector2UZ(m, m);
    ret.m_nonZeros.resize(m, 1.0);
    ret.m_columnIndices.resize(m);
    std::iota(ret.m_columnIndices.begin(), ret.m_columnIndices.end(), 0);
    ret.m_rowPointers.resize(m + 1);
    std::iota(ret.m_rowPointers.begin(), ret.m_rowPointers.end(), 0);

    return ret;
}

template <typename T>
size_t MatrixCSR<T>::HasElement(size_t i, size_t j) const
{
    if (i >= m_size.x || j >= m_size.y)
    {
        return std::numeric_limits<size_t>::max();
    }

    const size_t rowBegin = m_rowPointers[i];
    const size_t rowEnd = m_rowPointers[i + 1];

    if (const auto iter = BinaryFind(m_columnIndices.begin() + rowBegin,
                                     m_columnIndices.begin() + rowEnd, j);
        iter != m_columnIndices.begin() + rowEnd)
    {
        return static_cast<size_t>(iter - m_columnIndices.begin());
    }

    return std::numeric_limits<size_t>::max();
}

template <typename T>
template <typename Op>
MatrixCSR<T> MatrixCSR<T>::BinaryOp(const MatrixCSR& m, Op op) const
{
    assert(m_size == m.m_size);

    MatrixCSR ret;

    for (size_t i = 0; i < m_size.x; ++i)
    {
        std::vector<size_t> col;
        std::vector<double> nnz;

        auto colIterA = m_columnIndices.begin() + m_rowPointers[i];
        auto colIterB = m.m_columnIndices.begin() + m.m_rowPointers[i];
        auto colEndA = m_columnIndices.begin() + m_rowPointers[i + 1];
        auto colEndB = m.m_columnIndices.begin() + m.m_rowPointers[i + 1];
        auto nnzIterA = m_nonZeros.begin() + m_rowPointers[i];
        auto nnzIterB = m.m_nonZeros.begin() + m.m_rowPointers[i];

        while (colIterA != colEndA || colIterB != colEndB)
        {
            if (colIterB == colEndB || *colIterA < *colIterB)
            {
                col.push_back(*colIterA);
                nnz.push_back(op(*nnzIterA, 0));
                ++colIterA;
                ++nnzIterA;
            }
            else if (colIterA == colEndA || *colIterA > *colIterB)
            {
                col.push_back(*colIterB);
                nnz.push_back(op(0, *nnzIterB));
                ++colIterB;
                ++nnzIterB;
            }
            else
            {
                assert(*colIterA == *colIterB);

                col.push_back(*colIterB);
                nnz.push_back(op(*nnzIterA, *nnzIterB));
                ++colIterA;
                ++nnzIterA;
                ++colIterB;
                ++nnzIterB;
            }
        }

        ret.AddRow(nnz, col);
    }

    return ret;
}

template <typename T>
MatrixCSR<T> operator-(const MatrixCSR<T>& a)
{
    return a.Mul(-1);
}

template <typename T>
MatrixCSR<T> operator+(const MatrixCSR<T>& a, const MatrixCSR<T>& b)
{
    return a.Add(b);
}

template <typename T>
MatrixCSR<T> operator+(const MatrixCSR<T>& a, T b)
{
    return a.Add(b);
}

template <typename T>
MatrixCSR<T> operator+(T a, const MatrixCSR<T>& b)
{
    return b.Add(a);
}

template <typename T>
MatrixCSR<T> operator-(const MatrixCSR<T>& a, const MatrixCSR<T>& b)
{
    return a.Sub(b);
}

template <typename T>
MatrixCSR<T> operator-(const MatrixCSR<T>& a, T b)
{
    return a.Sub(b);
}

template <typename T>
MatrixCSR<T> operator-(T a, const MatrixCSR<T>& b)
{
    return b.RSub(a);
}

template <typename T>
MatrixCSR<T> operator*(const MatrixCSR<T>& a, T b)
{
    return a.Mul(b);
}

template <typename T>
MatrixCSR<T> operator*(T a, const MatrixCSR<T>& b)
{
    return b.RMul(a);
}

template <typename T, size_t R, size_t C, typename ME>
MatrixCSRMatrixMul<T, ME> operator*(const MatrixCSR<T>& a,
                                    const MatrixExpression<T, R, C, ME>& b)
{
    return a.Mul(b);
}

template <typename T>
MatrixCSR<T> operator/(const MatrixCSR<T>& a, T b)
{
    return a.Div(b);
}

template <typename T>
MatrixCSR<T> operator/(T a, const MatrixCSR<T>& b)
{
    return b.RDiv(a);
}
}  // namespace CubbyFlow

#endif