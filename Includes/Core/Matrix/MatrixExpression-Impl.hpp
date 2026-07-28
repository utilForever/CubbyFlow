// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_MATRIX_EXPRESSION_IMPL_HPP
#define CUBBYFLOW_MATRIX_EXPRESSION_IMPL_HPP

#include <Core/Math/MathUtils.hpp>

#include <cassert>

namespace CubbyFlow
{
template <typename T, size_t Rows, size_t Cols, typename D>
constexpr size_t MatrixExpression<T, Rows, Cols, D>::GetRows() const
{
    return static_cast<const D&>(*this).GetRows();
}

template <typename T, size_t Rows, size_t Cols, typename D>
constexpr size_t MatrixExpression<T, Rows, Cols, D>::GetCols() const
{
    return static_cast<const D&>(*this).GetCols();
}

template <typename T, size_t Rows, size_t Cols, typename D>
T MatrixExpression<T, Rows, Cols, D>::Eval(size_t i, size_t j) const
{
    return GetDerived()(i, j);
}

template <typename T, size_t Rows, size_t Cols, typename D>
Matrix<T, Rows, Cols> MatrixExpression<T, Rows, Cols, D>::Eval() const
{
    return Matrix<T, Rows, Cols>(*this);
}

template <typename T, size_t Rows, size_t Cols, typename D>
template <size_t R, size_t C, typename E>
bool MatrixExpression<T, Rows, Cols, D>::IsSimilar(
    const MatrixExpression<T, R, C, E>& expression, double tol) const
{
    if (expression.GetRows() != GetRows() || expression.GetCols() != GetCols())
    {
        return false;
    }

    SimilarTo<T> op{ tol };

    for (size_t i = 0; i < GetRows(); ++i)
    {
        for (size_t j = 0; j < GetCols(); ++j)
        {
            if (!op(Eval(i, j), expression.Eval(i, j)))
            {
                return false;
            }
        }
    }

    return true;
}

template <typename T, size_t Rows, size_t Cols, typename D>
constexpr bool MatrixExpression<T, Rows, Cols, D>::IsSquare() const
{
    return GetRows() == GetCols();
}

template <typename T, size_t Rows, size_t Cols, typename D>
T MatrixExpression<T, Rows, Cols, D>::Sum() const
{
    T s = 0;

    for (size_t i = 0; i < GetRows(); ++i)
    {
        for (size_t j = 0; j < GetCols(); ++j)
        {
            s += Eval(i, j);
        }
    }

    return s;
}

template <typename T, size_t Rows, size_t Cols, typename D>
T MatrixExpression<T, Rows, Cols, D>::Avg() const
{
    return Sum() / (GetRows() * GetCols());
}

template <typename T, size_t Rows, size_t Cols, typename D>
T MatrixExpression<T, Rows, Cols, D>::Min() const
{
    T s = Eval(0, 0);

    for (size_t j = 1; j < GetCols(); ++j)
    {
        s = std::min(s, Eval(0, j));
    }

    for (size_t i = 1; i < GetRows(); ++i)
    {
        for (size_t j = 0; j < GetCols(); ++j)
        {
            s = std::min(s, Eval(i, j));
        }
    }

    return s;
}

template <typename T, size_t Rows, size_t Cols, typename D>
T MatrixExpression<T, Rows, Cols, D>::Max() const
{
    T s = Eval(0, 0);

    for (size_t j = 1; j < GetCols(); ++j)
    {
        s = std::max(s, Eval(0, j));
    }

    for (size_t i = 1; i < GetRows(); ++i)
    {
        for (size_t j = 0; j < GetCols(); ++j)
        {
            s = std::max(s, Eval(i, j));
        }
    }

    return s;
}

template <typename T, size_t Rows, size_t Cols, typename D>
T MatrixExpression<T, Rows, Cols, D>::AbsMin() const
{
    T s = Eval(0, 0);

    for (size_t j = 1; j < GetCols(); ++j)
    {
        s = CubbyFlow::AbsMin(s, Eval(0, j));
    }

    for (size_t i = 1; i < GetRows(); ++i)
    {
        for (size_t j = 0; j < GetCols(); ++j)
        {
            s = CubbyFlow::AbsMin(s, Eval(i, j));
        }
    }

    return s;
}

template <typename T, size_t Rows, size_t Cols, typename D>
T MatrixExpression<T, Rows, Cols, D>::AbsMax() const
{
    T s = Eval(0, 0);

    for (size_t j = 1; j < GetCols(); ++j)
    {
        s = CubbyFlow::AbsMax(s, Eval(0, j));
    }

    for (size_t i = 1; i < GetRows(); ++i)
    {
        for (size_t j = 0; j < GetCols(); ++j)
        {
            s = CubbyFlow::AbsMax(s, Eval(i, j));
        }
    }

    return s;
}

template <typename T, size_t Rows, size_t Cols, typename D>
T MatrixExpression<T, Rows, Cols, D>::Trace() const
{
    assert(GetRows() == GetCols());

    T result = Eval(0, 0);

    for (size_t i = 1; i < GetRows(); ++i)
    {
        result += Eval(i, i);
    }

    return result;
}

template <typename T, size_t Rows, size_t Cols, typename D>
T MatrixExpression<T, Rows, Cols, D>::Determinant() const
{
    assert(GetRows() == GetCols());

    return Determinant(*this);
}

template <typename T, size_t Rows, size_t Cols, typename D>
size_t MatrixExpression<T, Rows, Cols, D>::DominantAxis() const
{
    assert(GetCols() == 1);

    size_t ret = 0;
    T best = Eval(0, 0);

    for (size_t i = 1; i < GetRows(); ++i)
    {
        T curr = Eval(i, 0);

        if (std::fabs(curr) > std::fabs(best))
        {
            best = curr;
            ret = i;
        }
    }

    return ret;
}

template <typename T, size_t Rows, size_t Cols, typename D>
size_t MatrixExpression<T, Rows, Cols, D>::SubdominantAxis() const
{
    assert(GetCols() == 1);

    size_t ret = 0;
    T best = Eval(0, 0);

    for (size_t i = 1; i < GetRows(); ++i)
    {
        T curr = Eval(i, 0);

        if (std::fabs(curr) < std::fabs(best))
        {
            best = curr;
            ret = i;
        }
    }

    return ret;
}

template <typename T, size_t Rows, size_t Cols, typename D>
T MatrixExpression<T, Rows, Cols, D>::Norm() const
{
    return std::sqrt(NormSquared());
}

template <typename T, size_t Rows, size_t Cols, typename D>
T MatrixExpression<T, Rows, Cols, D>::NormSquared() const
{
    T result = 0;

    for (size_t i = 0; i < GetRows(); ++i)
    {
        for (size_t j = 0; j < GetCols(); ++j)
        {
            result += Eval(i, j) * Eval(i, j);
        }
    }

    return result;
}

template <typename T, size_t Rows, size_t Cols, typename D>
T MatrixExpression<T, Rows, Cols, D>::FrobeniusNorm() const
{
    return Norm();
}

template <typename T, size_t Rows, size_t Cols, typename D>
T MatrixExpression<T, Rows, Cols, D>::Length() const
{
    assert(GetCols() == 1);

    return Norm();
}

template <typename T, size_t Rows, size_t Cols, typename D>
T MatrixExpression<T, Rows, Cols, D>::LengthSquared() const
{
    assert(GetCols() == 1);

    return NormSquared();
}

template <typename T, size_t Rows, size_t Cols, typename D>
template <size_t R, size_t C, typename E>
T MatrixExpression<T, Rows, Cols, D>::DistanceTo(
    const MatrixExpression<T, R, C, E>& other) const
{
    assert(GetCols() == 1);

    return std::sqrt(DistanceSquaredTo(other));
};

template <typename T, size_t Rows, size_t Cols, typename D>
template <size_t R, size_t C, typename E>
T MatrixExpression<T, Rows, Cols, D>::DistanceSquaredTo(
    const MatrixExpression<T, R, C, E>& other) const
{
    assert(GetCols() == 1);

    return D(GetDerived() - other.GetDerived()).NormSquared();
}

template <typename T, size_t Rows, size_t Cols, typename D>
MatrixScalarElemWiseDiv<T, Rows, Cols, const D&>
MatrixExpression<T, Rows, Cols, D>::Normalized() const
{
    return MatrixScalarElemWiseDiv<T, Rows, Cols, const D&>{ GetDerived(),
                                                             Norm() };
}

template <typename T, size_t Rows, size_t Cols, typename D>
MatrixDiagonal<T, Rows, Cols, const D&>
MatrixExpression<T, Rows, Cols, D>::Diagonal() const
{
    return MatrixDiagonal<T, Rows, Cols, const D&>{ GetDerived() };
}

template <typename T, size_t Rows, size_t Cols, typename D>
MatrixOffDiagonal<T, Rows, Cols, const D&>
MatrixExpression<T, Rows, Cols, D>::OffDiagonal() const
{
    return MatrixOffDiagonal<T, Rows, Cols, const D&>{ GetDerived() };
}

template <typename T, size_t Rows, size_t Cols, typename D>
MatrixTri<T, Rows, Cols, const D&>
MatrixExpression<T, Rows, Cols, D>::StrictLowerTri() const
{
    return MatrixTri<T, Rows, Cols, const D&>{ GetDerived(), false, true };
}

template <typename T, size_t Rows, size_t Cols, typename D>
MatrixTri<T, Rows, Cols, const D&>
MatrixExpression<T, Rows, Cols, D>::StrictUpperTri() const
{
    return MatrixTri<T, Rows, Cols, const D&>{ GetDerived(), true, true };
}

template <typename T, size_t Rows, size_t Cols, typename D>
MatrixTri<T, Rows, Cols, const D&>
MatrixExpression<T, Rows, Cols, D>::LowerTri() const
{
    return MatrixTri<T, Rows, Cols, const D&>{ GetDerived(), false, false };
}

template <typename T, size_t Rows, size_t Cols, typename D>
MatrixTri<T, Rows, Cols, const D&>
MatrixExpression<T, Rows, Cols, D>::UpperTri() const
{
    return MatrixTri<T, Rows, Cols, const D&>{ GetDerived(), true, false };
}

template <typename T, size_t Rows, size_t Cols, typename D>
MatrixTranspose<T, Rows, Cols, const D&>
MatrixExpression<T, Rows, Cols, D>::Transposed() const
{
    return MatrixTranspose<T, Rows, Cols, const D&>{ GetDerived() };
}

template <typename T, size_t Rows, size_t Cols, typename D>
Matrix<T, Rows, Cols> MatrixExpression<T, Rows, Cols, D>::Inverse() const
{
    Matrix<T, Rows, Cols> result;
    Inverse(*this, result);
    return result;
}

template <typename T, size_t Rows, size_t Cols, typename D>
template <typename U>
MatrixTypeCast<T, Rows, Cols, U, const D&>
MatrixExpression<T, Rows, Cols, D>::CastTo() const
{
    return MatrixTypeCast<T, Rows, Cols, U, const D&>{ GetDerived() };
}

template <typename T, size_t Rows, size_t Cols, typename D>
template <size_t R, size_t C, typename E, typename U>
std::enable_if_t<(IsMatrixSizeDynamic<Rows, Cols>() || Cols == 1) &&
                     (IsMatrixSizeDynamic<R, C>() || C == 1),
                 U>
MatrixExpression<T, Rows, Cols, D>::Dot(
    const MatrixExpression<T, R, C, E>& expression) const
{
    assert(expression.GetRows() == GetRows() && expression.GetCols() == 1);

    T sum = Eval(0, 0) * expression.Eval(0, 0);

    for (size_t i = 1; i < GetRows(); ++i)
    {
        sum += Eval(i, 0) * expression.Eval(i, 0);
    }

    return sum;
}

template <typename T, size_t Rows, size_t Cols, typename D>
template <size_t R, size_t C, typename E, typename U>
std::enable_if_t<(IsMatrixSizeDynamic<Rows, Cols>() ||
                  (Rows == 2 && Cols == 1)) &&
                     (IsMatrixSizeDynamic<R, C>() || (R == 2 && C == 1)),
                 U>
MatrixExpression<T, Rows, Cols, D>::Cross(
    const MatrixExpression<T, R, C, E>& expression) const
{
    assert(GetRows() == 2 && GetCols() == 1 && expression.GetRows() == 2 &&
           expression.GetCols() == 1);

    return Eval(0, 0) * expression.Eval(1, 0) -
           expression.Eval(0, 0) * Eval(1, 0);
}

template <typename T, size_t Rows, size_t Cols, typename D>
template <size_t R, size_t C, typename E, typename U>
std::enable_if_t<(IsMatrixSizeDynamic<Rows, Cols>() ||
                  (Rows == 3 && Cols == 1)) &&
                     (IsMatrixSizeDynamic<R, C>() || (R == 3 && C == 1)),
                 Matrix<U, 3, 1>>
MatrixExpression<T, Rows, Cols, D>::Cross(
    const MatrixExpression<T, R, C, E>& exp) const
{
    assert(GetRows() == 3 && GetCols() == 1 && exp.GetRows() == 3 &&
           exp.GetCols() == 1);

    return Matrix<U, 3, 1>{
        Eval(1, 0) * exp.Eval(2, 0) - exp.Eval(1, 0) * Eval(2, 0),
        Eval(2, 0) * exp.Eval(0, 0) - exp.Eval(2, 0) * Eval(0, 0),
        Eval(0, 0) * exp.Eval(1, 0) - exp.Eval(0, 0) * Eval(1, 0)
    };
}

template <typename T, size_t Rows, size_t Cols, typename D>
template <size_t R, size_t C, typename E, typename U>
std::enable_if_t<(IsMatrixSizeDynamic<Rows, Cols>() ||
                  ((Rows == 2 || Rows == 3) && Cols == 1)) &&
                     (IsMatrixSizeDynamic<R, C>() ||
                      ((R == 2 || R == 3) && C == 1)),
                 Matrix<U, Rows, 1>>
MatrixExpression<T, Rows, Cols, D>::Reflected(
    const MatrixExpression<T, R, C, E>& normal) const
{
    assert((GetRows() == 2 || GetRows() == 3) && GetCols() == 1 &&
           normal.GetRows() == GetRows() && normal.GetCols() == 1);

    // this - 2(this.n)n
    return (*this) - 2 * Dot(normal) * normal;
}

template <typename T, size_t Rows, size_t Cols, typename D>
template <size_t R, size_t C, typename E, typename U>
std::enable_if_t<(IsMatrixSizeDynamic<Rows, Cols>() ||
                  ((Rows == 2 || Rows == 3) && Cols == 1)) &&
                     (IsMatrixSizeDynamic<R, C>() ||
                      ((R == 2 || R == 3) && C == 1)),
                 Matrix<U, Rows, 1>>
MatrixExpression<T, Rows, Cols, D>::Projected(
    const MatrixExpression<T, R, C, E>& normal) const
{
    assert((GetRows() == 2 || GetRows() == 3) && GetCols() == 1 &&
           normal.GetRows() == GetRows() && normal.GetCols() == 1);

    // this - this.n n
    return (*this) - this->Dot(normal) * normal;
}

template <typename T, size_t Rows, size_t Cols, typename D>
template <typename U>
std::enable_if_t<(IsMatrixSizeDynamic<Rows, Cols>() ||
                  (Rows == 2 && Cols == 1)),
                 Matrix<U, 2, 1>>
MatrixExpression<T, Rows, Cols, D>::Tangential() const
{
    assert(GetRows() == 2 && GetCols() == 1);

    return Matrix<U, 2, 1>{ -Eval(1, 0), Eval(0, 0) };
}

template <typename T, size_t Rows, size_t Cols, typename D>
template <typename U>
std::enable_if_t<(IsMatrixSizeDynamic<Rows, Cols>() ||
                  (Rows == 3 && Cols == 1)),
                 std::tuple<Matrix<U, 3, 1>, Matrix<U, 3, 1>>>
MatrixExpression<T, Rows, Cols, D>::Tangentials() const
{
    assert(GetRows() == 3 && GetCols() == 1);

    using V = Matrix<T, 3, 1>;

    V a(((std::fabs(Eval(1, 0)) > 0 || std::fabs(Eval(2, 0)) > 0) ? V(1, 0, 0)
                                                                  : V(0, 1, 0))
            .Cross(*this)
            .Normalized());
    V b(this->Cross(a));

    return std::make_tuple(a, b);
}

template <typename T, size_t Rows, size_t Cols, typename D>
D& MatrixExpression<T, Rows, Cols, D>::GetDerived()
{
    return static_cast<D&>(*this);
}

template <typename T, size_t Rows, size_t Cols, typename D>
const D& MatrixExpression<T, Rows, Cols, D>::GetDerived() const
{
    return static_cast<const D&>(*this);
}

template <typename T, size_t Rows, size_t Cols, typename D>
constexpr T MatrixExpression<T, Rows, Cols, D>::Determinant(
    const MatrixExpression<T, 1, 1, D>& m)
{
    return m.Eval(0, 0);
}

template <typename T, size_t Rows, size_t Cols, typename D>
constexpr T MatrixExpression<T, Rows, Cols, D>::Determinant(
    const MatrixExpression<T, 2, 2, D>& m)
{
    return m.Eval(0, 0) * m.Eval(1, 1) - m.Eval(1, 0) * m.Eval(0, 1);
}

template <typename T, size_t Rows, size_t Cols, typename D>
constexpr T MatrixExpression<T, Rows, Cols, D>::Determinant(
    const MatrixExpression<T, 3, 3, D>& m)
{
    return m.Eval(0, 0) * m.Eval(1, 1) * m.Eval(2, 2) -
           m.Eval(0, 0) * m.Eval(1, 2) * m.Eval(2, 1) +
           m.Eval(0, 1) * m.Eval(1, 2) * m.Eval(2, 0) -
           m.Eval(0, 1) * m.Eval(1, 0) * m.Eval(2, 2) +
           m.Eval(0, 2) * m.Eval(1, 0) * m.Eval(2, 1) -
           m.Eval(0, 2) * m.Eval(1, 1) * m.Eval(2, 0);
}

template <typename T, size_t Rows, size_t Cols, typename D>
constexpr T MatrixExpression<T, Rows, Cols, D>::Determinant(
    const MatrixExpression<T, 4, 4, D>& m)
{
    return m.Eval(0, 0) * m.Eval(1, 1) * m.Eval(2, 2) * m.Eval(3, 3) +
           m.Eval(0, 0) * m.Eval(1, 2) * m.Eval(2, 3) * m.Eval(3, 1) +
           m.Eval(0, 0) * m.Eval(1, 3) * m.Eval(2, 1) * m.Eval(3, 2) +
           m.Eval(0, 1) * m.Eval(1, 0) * m.Eval(2, 3) * m.Eval(3, 2) +
           m.Eval(0, 1) * m.Eval(1, 2) * m.Eval(2, 0) * m.Eval(3, 3) +
           m.Eval(0, 1) * m.Eval(1, 3) * m.Eval(2, 2) * m.Eval(3, 0) +
           m.Eval(0, 2) * m.Eval(1, 0) * m.Eval(2, 1) * m.Eval(3, 3) +
           m.Eval(0, 2) * m.Eval(1, 1) * m.Eval(2, 3) * m.Eval(3, 0) +
           m.Eval(0, 2) * m.Eval(1, 3) * m.Eval(2, 0) * m.Eval(3, 1) +
           m.Eval(0, 3) * m.Eval(1, 0) * m.Eval(2, 2) * m.Eval(3, 1) +
           m.Eval(0, 3) * m.Eval(1, 1) * m.Eval(2, 0) * m.Eval(3, 2) +
           m.Eval(0, 3) * m.Eval(1, 2) * m.Eval(2, 1) * m.Eval(3, 0) -
           m.Eval(0, 0) * m.Eval(1, 1) * m.Eval(2, 3) * m.Eval(3, 2) -
           m.Eval(0, 0) * m.Eval(1, 2) * m.Eval(2, 1) * m.Eval(3, 3) -
           m.Eval(0, 0) * m.Eval(1, 3) * m.Eval(2, 2) * m.Eval(3, 1) -
           m.Eval(0, 1) * m.Eval(1, 0) * m.Eval(2, 2) * m.Eval(3, 3) -
           m.Eval(0, 1) * m.Eval(1, 2) * m.Eval(2, 3) * m.Eval(3, 0) -
           m.Eval(0, 1) * m.Eval(1, 3) * m.Eval(2, 0) * m.Eval(3, 2) -
           m.Eval(0, 2) * m.Eval(1, 0) * m.Eval(2, 3) * m.Eval(3, 1) -
           m.Eval(0, 2) * m.Eval(1, 1) * m.Eval(2, 0) * m.Eval(3, 3) -
           m.Eval(0, 2) * m.Eval(1, 3) * m.Eval(2, 1) * m.Eval(3, 0) -
           m.Eval(0, 3) * m.Eval(1, 0) * m.Eval(2, 1) * m.Eval(3, 2) -
           m.Eval(0, 3) * m.Eval(1, 1) * m.Eval(2, 2) * m.Eval(3, 0) -
           m.Eval(0, 3) * m.Eval(1, 2) * m.Eval(2, 0) * m.Eval(3, 1);
}

template <typename T, size_t Rows, size_t Cols, typename D>
template <typename U>
std::enable_if_t<(Rows > 4 && Cols > 4) || IsMatrixSizeDynamic<Rows, Cols>(), U>
MatrixExpression<T, Rows, Cols, D>::Determinant(const MatrixExpression& m)
{
    // Computes inverse matrix using Gaussian elimination method.
    // https://martin-thoma.com/solving-linear-equations-with-gaussian-elimination/
    Matrix<T, Rows, Cols> a{ m.GetDerived() };

    T result = 1;

    for (size_t i = 0; i < m.GetRows(); ++i)
    {
        // Search for maximum in this column
        T maxEl = std::fabs(a(i, i));
        size_t maxRow = i;

        for (size_t k = i + 1; k < m.GetRows(); ++k)
        {
            if (std::fabs(a(k, i)) > maxEl)
            {
                maxEl = std::fabs(a(k, i));
                maxRow = k;
            }
        }

        // Swap maximum row with current row (column by column)
        if (maxRow != i)
        {
            for (size_t k = i; k < m.GetRows(); ++k)
            {
                std::swap(a(maxRow, k), a(i, k));
            }
            result *= -1;
        }

        // Make all rows below this one 0 in current column
        for (size_t k = i + 1; k < m.GetRows(); ++k)
        {
            T c = -a(k, i) / a(i, i);

            for (size_t j = i; j < m.GetRows(); ++j)
            {
                if (i == j)
                {
                    a(k, j) = 0;
                }
                else
                {
                    a(k, j) += c * a(i, j);
                }
            }
        }
    }

    for (size_t i = 0; i < m.GetRows(); ++i)
    {
        result *= a(i, i);
    }

    return result;
}

template <typename T, size_t Rows, size_t Cols, typename D>
void MatrixExpression<T, Rows, Cols, D>::Inverse(
    const MatrixExpression<T, 1, 1, D>& m, Matrix<T, Rows, Cols>& result)
{
    result(0, 0) = 1 / m(0, 0);
}

template <typename T, size_t Rows, size_t Cols, typename D>
void MatrixExpression<T, Rows, Cols, D>::Inverse(
    const MatrixExpression<T, 2, 2, D>& m, Matrix<T, Rows, Cols>& result)
{
    T d = Determinant(m);

    result(0, 0) = m.Eval(1, 1) / d;
    result(0, 1) = -m.Eval(0, 1) / d;
    result(1, 0) = -m.Eval(1, 0) / d;
    result(1, 1) = m.Eval(0, 0) / d;
}

template <typename T, size_t Rows, size_t Cols, typename D>
void MatrixExpression<T, Rows, Cols, D>::Inverse(
    const MatrixExpression<T, 3, 3, D>& m, Matrix<T, Rows, Cols>& result)
{
    T d = Determinant(m);

    result(0, 0) =
        (m.Eval(1, 1) * m.Eval(2, 2) - m.Eval(1, 2) * m.Eval(2, 1)) / d;
    result(0, 1) =
        (m.Eval(0, 2) * m.Eval(2, 1) - m.Eval(0, 1) * m.Eval(2, 2)) / d;
    result(0, 2) =
        (m.Eval(0, 1) * m.Eval(1, 2) - m.Eval(0, 2) * m.Eval(1, 1)) / d;
    result(1, 0) =
        (m.Eval(1, 2) * m.Eval(2, 0) - m.Eval(1, 0) * m.Eval(2, 2)) / d;
    result(1, 1) =
        (m.Eval(0, 0) * m.Eval(2, 2) - m.Eval(0, 2) * m.Eval(2, 0)) / d;
    result(1, 2) =
        (m.Eval(0, 2) * m.Eval(1, 0) - m.Eval(0, 0) * m.Eval(1, 2)) / d;
    result(2, 0) =
        (m.Eval(1, 0) * m.Eval(2, 1) - m.Eval(1, 1) * m.Eval(2, 0)) / d;
    result(2, 1) =
        (m.Eval(0, 1) * m.Eval(2, 0) - m.Eval(0, 0) * m.Eval(2, 1)) / d;
    result(2, 2) =
        (m.Eval(0, 0) * m.Eval(1, 1) - m.Eval(0, 1) * m.Eval(1, 0)) / d;
}

template <typename T, size_t Rows, size_t Cols, typename D>
void MatrixExpression<T, Rows, Cols, D>::Inverse(
    const MatrixExpression<T, 4, 4, D>& m, Matrix<T, Rows, Cols>& result)
{
    T d = Determinant(m);

    result(0, 0) = (m.Eval(1, 1) * m.Eval(2, 2) * m.Eval(3, 3) +
                    m.Eval(1, 2) * m.Eval(2, 3) * m.Eval(3, 1) +
                    m.Eval(1, 3) * m.Eval(2, 1) * m.Eval(3, 2) -
                    m.Eval(1, 1) * m.Eval(2, 3) * m.Eval(3, 2) -
                    m.Eval(1, 2) * m.Eval(2, 1) * m.Eval(3, 3) -
                    m.Eval(1, 3) * m.Eval(2, 2) * m.Eval(3, 1)) /
                   d;
    result(0, 1) = (m.Eval(0, 1) * m.Eval(2, 3) * m.Eval(3, 2) +
                    m.Eval(0, 2) * m.Eval(2, 1) * m.Eval(3, 3) +
                    m.Eval(0, 3) * m.Eval(2, 2) * m.Eval(3, 1) -
                    m.Eval(0, 1) * m.Eval(2, 2) * m.Eval(3, 3) -
                    m.Eval(0, 2) * m.Eval(2, 3) * m.Eval(3, 1) -
                    m.Eval(0, 3) * m.Eval(2, 1) * m.Eval(3, 2)) /
                   d;
    result(0, 2) = (m.Eval(0, 1) * m.Eval(1, 2) * m.Eval(3, 3) +
                    m.Eval(0, 2) * m.Eval(1, 3) * m.Eval(3, 1) +
                    m.Eval(0, 3) * m.Eval(1, 1) * m.Eval(3, 2) -
                    m.Eval(0, 1) * m.Eval(1, 3) * m.Eval(3, 2) -
                    m.Eval(0, 2) * m.Eval(1, 1) * m.Eval(3, 3) -
                    m.Eval(0, 3) * m.Eval(1, 2) * m.Eval(3, 1)) /
                   d;
    result(0, 3) = (m.Eval(0, 1) * m.Eval(1, 3) * m.Eval(2, 2) +
                    m.Eval(0, 2) * m.Eval(1, 1) * m.Eval(2, 3) +
                    m.Eval(0, 3) * m.Eval(1, 2) * m.Eval(2, 1) -
                    m.Eval(0, 1) * m.Eval(1, 2) * m.Eval(2, 3) -
                    m.Eval(0, 2) * m.Eval(1, 3) * m.Eval(2, 1) -
                    m.Eval(0, 3) * m.Eval(1, 1) * m.Eval(2, 2)) /
                   d;
    result(1, 0) = (m.Eval(1, 0) * m.Eval(2, 3) * m.Eval(3, 2) +
                    m.Eval(1, 2) * m.Eval(2, 0) * m.Eval(3, 3) +
                    m.Eval(1, 3) * m.Eval(2, 2) * m.Eval(3, 0) -
                    m.Eval(1, 0) * m.Eval(2, 2) * m.Eval(3, 3) -
                    m.Eval(1, 2) * m.Eval(2, 3) * m.Eval(3, 0) -
                    m.Eval(1, 3) * m.Eval(2, 0) * m.Eval(3, 2)) /
                   d;
    result(1, 1) = (m.Eval(0, 0) * m.Eval(2, 2) * m.Eval(3, 3) +
                    m.Eval(0, 2) * m.Eval(2, 3) * m.Eval(3, 0) +
                    m.Eval(0, 3) * m.Eval(2, 0) * m.Eval(3, 2) -
                    m.Eval(0, 0) * m.Eval(2, 3) * m.Eval(3, 2) -
                    m.Eval(0, 2) * m.Eval(2, 0) * m.Eval(3, 3) -
                    m.Eval(0, 3) * m.Eval(2, 2) * m.Eval(3, 0)) /
                   d;
    result(1, 2) = (m.Eval(0, 0) * m.Eval(1, 3) * m.Eval(3, 2) +
                    m.Eval(0, 2) * m.Eval(1, 0) * m.Eval(3, 3) +
                    m.Eval(0, 3) * m.Eval(1, 2) * m.Eval(3, 0) -
                    m.Eval(0, 0) * m.Eval(1, 2) * m.Eval(3, 3) -
                    m.Eval(0, 2) * m.Eval(1, 3) * m.Eval(3, 0) -
                    m.Eval(0, 3) * m.Eval(1, 0) * m.Eval(3, 2)) /
                   d;
    result(1, 3) = (m.Eval(0, 0) * m.Eval(1, 2) * m.Eval(2, 3) +
                    m.Eval(0, 2) * m.Eval(1, 3) * m.Eval(2, 0) +
                    m.Eval(0, 3) * m.Eval(1, 0) * m.Eval(2, 2) -
                    m.Eval(0, 0) * m.Eval(1, 3) * m.Eval(2, 2) -
                    m.Eval(0, 2) * m.Eval(1, 0) * m.Eval(2, 3) -
                    m.Eval(0, 3) * m.Eval(1, 2) * m.Eval(2, 0)) /
                   d;
    result(2, 0) = (m.Eval(1, 0) * m.Eval(2, 1) * m.Eval(3, 3) +
                    m.Eval(1, 1) * m.Eval(2, 3) * m.Eval(3, 0) +
                    m.Eval(1, 3) * m.Eval(2, 0) * m.Eval(3, 1) -
                    m.Eval(1, 0) * m.Eval(2, 3) * m.Eval(3, 1) -
                    m.Eval(1, 1) * m.Eval(2, 0) * m.Eval(3, 3) -
                    m.Eval(1, 3) * m.Eval(2, 1) * m.Eval(3, 0)) /
                   d;
    result(2, 1) = (m.Eval(0, 0) * m.Eval(2, 3) * m.Eval(3, 1) +
                    m.Eval(0, 1) * m.Eval(2, 0) * m.Eval(3, 3) +
                    m.Eval(0, 3) * m.Eval(2, 1) * m.Eval(3, 0) -
                    m.Eval(0, 0) * m.Eval(2, 1) * m.Eval(3, 3) -
                    m.Eval(0, 1) * m.Eval(2, 3) * m.Eval(3, 0) -
                    m.Eval(0, 3) * m.Eval(2, 0) * m.Eval(3, 1)) /
                   d;
    result(2, 2) = (m.Eval(0, 0) * m.Eval(1, 1) * m.Eval(3, 3) +
                    m.Eval(0, 1) * m.Eval(1, 3) * m.Eval(3, 0) +
                    m.Eval(0, 3) * m.Eval(1, 0) * m.Eval(3, 1) -
                    m.Eval(0, 0) * m.Eval(1, 3) * m.Eval(3, 1) -
                    m.Eval(0, 1) * m.Eval(1, 0) * m.Eval(3, 3) -
                    m.Eval(0, 3) * m.Eval(1, 1) * m.Eval(3, 0)) /
                   d;
    result(2, 3) = (m.Eval(0, 0) * m.Eval(1, 3) * m.Eval(2, 1) +
                    m.Eval(0, 1) * m.Eval(1, 0) * m.Eval(2, 3) +
                    m.Eval(0, 3) * m.Eval(1, 1) * m.Eval(2, 0) -
                    m.Eval(0, 0) * m.Eval(1, 1) * m.Eval(2, 3) -
                    m.Eval(0, 1) * m.Eval(1, 3) * m.Eval(2, 0) -
                    m.Eval(0, 3) * m.Eval(1, 0) * m.Eval(2, 1)) /
                   d;
    result(3, 0) = (m.Eval(1, 0) * m.Eval(2, 2) * m.Eval(3, 1) +
                    m.Eval(1, 1) * m.Eval(2, 0) * m.Eval(3, 2) +
                    m.Eval(1, 2) * m.Eval(2, 1) * m.Eval(3, 0) -
                    m.Eval(1, 0) * m.Eval(2, 1) * m.Eval(3, 2) -
                    m.Eval(1, 1) * m.Eval(2, 2) * m.Eval(3, 0) -
                    m.Eval(1, 2) * m.Eval(2, 0) * m.Eval(3, 1)) /
                   d;
    result(3, 1) = (m.Eval(0, 0) * m.Eval(2, 1) * m.Eval(3, 2) +
                    m.Eval(0, 1) * m.Eval(2, 2) * m.Eval(3, 0) +
                    m.Eval(0, 2) * m.Eval(2, 0) * m.Eval(3, 1) -
                    m.Eval(0, 0) * m.Eval(2, 2) * m.Eval(3, 1) -
                    m.Eval(0, 1) * m.Eval(2, 0) * m.Eval(3, 2) -
                    m.Eval(0, 2) * m.Eval(2, 1) * m.Eval(3, 0)) /
                   d;
    result(3, 2) = (m.Eval(0, 0) * m.Eval(1, 2) * m.Eval(3, 1) +
                    m.Eval(0, 1) * m.Eval(1, 0) * m.Eval(3, 2) +
                    m.Eval(0, 2) * m.Eval(1, 1) * m.Eval(3, 0) -
                    m.Eval(0, 0) * m.Eval(1, 1) * m.Eval(3, 2) -
                    m.Eval(0, 1) * m.Eval(1, 2) * m.Eval(3, 0) -
                    m.Eval(0, 2) * m.Eval(1, 0) * m.Eval(3, 1)) /
                   d;
    result(3, 3) = (m.Eval(0, 0) * m.Eval(1, 1) * m.Eval(2, 2) +
                    m.Eval(0, 1) * m.Eval(1, 2) * m.Eval(2, 0) +
                    m.Eval(0, 2) * m.Eval(1, 0) * m.Eval(2, 1) -
                    m.Eval(0, 0) * m.Eval(1, 2) * m.Eval(2, 1) -
                    m.Eval(0, 1) * m.Eval(1, 0) * m.Eval(2, 2) -
                    m.Eval(0, 2) * m.Eval(1, 1) * m.Eval(2, 0)) /
                   d;
}

template <typename T, size_t Rows, size_t Cols, typename Derived>
template <typename M>
void MatrixExpression<T, Rows, Cols, Derived>::Inverse(
    const MatrixExpression<T, Rows, Cols, Derived>& m,
    std::enable_if_t<
        (Rows > 4 && Cols > 4) || IsMatrixSizeDynamic<Rows, Cols>(), M>& result)
{
    // Computes inverse matrix using Gaussian elimination method.
    // https://martin-thoma.com/solving-linear-equations-with-gaussian-elimination/
    Matrix<T, Rows, Cols> a{ m.GetDerived() };

    using ConstType = MatrixConstant<T, Rows, Cols>;

    result = MatrixDiagonal<T, Rows, Cols, ConstType>{ ConstType{
        a.GetRows(), a.GetCols(), 1 } };
    const size_t n = m.GetRows();

    for (size_t i = 0; i < n; ++i)
    {
        // Search for maximum in this column
        T maxEl = std::fabs(a(i, i));
        size_t maxRow = i;

        for (size_t k = i + 1; k < n; ++k)
        {
            if (std::fabs(a(k, i)) > maxEl)
            {
                maxEl = std::fabs(a(k, i));
                maxRow = k;
            }
        }

        // Swap maximum row with current row (column by column)
        if (maxRow != i)
        {
            for (size_t k = i; k < n; ++k)
            {
                std::swap(a(maxRow, k), a(i, k));
            }

            for (size_t k = 0; k < n; ++k)
            {
                std::swap(result(maxRow, k), result(i, k));
            }
        }

        // Make all rows except this one 0 in current column
        for (size_t k = 0; k < n; ++k)
        {
            if (k == i)
            {
                continue;
            }

            T c = -a(k, i) / a(i, i);

            for (size_t j = 0; j < n; ++j)
            {
                result(k, j) += c * result(i, j);

                if (i == j)
                {
                    a(k, j) = 0;
                }
                else if (i < j)
                {
                    a(k, j) += c * a(i, j);
                }
            }
        }

        // Scale
        for (size_t k = 0; k < n; ++k)
        {
            T c = 1 / a(k, k);

            for (size_t j = 0; j < n; ++j)
            {
                a(k, j) *= c;
                result(k, j) *= c;
            }
        }
    }
}

template <typename T, size_t Rows, size_t Cols>
constexpr size_t MatrixConstant<T, Rows, Cols>::GetRows() const
{
    return m_rows;
}

template <typename T, size_t Rows, size_t Cols>
constexpr size_t MatrixConstant<T, Rows, Cols>::GetCols() const
{
    return m_cols;
}

template <typename T, size_t Rows, size_t Cols>
constexpr T MatrixConstant<T, Rows, Cols>::operator()(size_t, size_t) const
{
    return m_val;
}

template <typename T, size_t Rows, size_t Cols, typename M1>
constexpr size_t MatrixDiagonal<T, Rows, Cols, M1>::GetRows() const
{
    return m_mat1.GetRows();
}

template <typename T, size_t Rows, size_t Cols, typename M1>
constexpr size_t MatrixDiagonal<T, Rows, Cols, M1>::GetCols() const
{
    return m_mat1.GetCols();
}

template <typename T, size_t Rows, size_t Cols, typename M1>
T MatrixDiagonal<T, Rows, Cols, M1>::operator()(size_t i, size_t j) const
{
    if (i == j)
    {
        return m_mat1(i, j);
    }

    return T{};
}

template <typename T, size_t Rows, size_t Cols, typename M1>
constexpr size_t MatrixOffDiagonal<T, Rows, Cols, M1>::GetRows() const
{
    return m_mat1.GetRows();
}

template <typename T, size_t Rows, size_t Cols, typename M1>
constexpr size_t MatrixOffDiagonal<T, Rows, Cols, M1>::GetCols() const
{
    return m_mat1.GetCols();
}

template <typename T, size_t Rows, size_t Cols, typename M1>
T MatrixOffDiagonal<T, Rows, Cols, M1>::operator()(size_t i, size_t j) const
{
    if (i != j)
    {
        return m_mat1(i, j);
    }

    return T{};
}

template <typename T, size_t Rows, size_t Cols, typename M1>
constexpr size_t MatrixTri<T, Rows, Cols, M1>::GetRows() const
{
    return m_mat1.GetRows();
}

template <typename T, size_t Rows, size_t Cols, typename M1>
constexpr size_t MatrixTri<T, Rows, Cols, M1>::GetCols() const
{
    return m_mat1.GetCols();
}

template <typename T, size_t Rows, size_t Cols, typename M1>
T MatrixTri<T, Rows, Cols, M1>::operator()(size_t i, size_t j) const
{
    if (m_isUpper)
    {
        if (m_isStrict)
        {
            return (j > i) ? m_mat1(i, j) : 0;
        }

        return (j >= i) ? m_mat1(i, j) : 0;
    }

    if (m_isStrict)
    {
        return (j < i) ? m_mat1(i, j) : 0;
    }

    return (j <= i) ? m_mat1(i, j) : 0;
}

template <typename T, size_t Rows, size_t Cols, typename M1>
constexpr size_t MatrixTranspose<T, Rows, Cols, M1>::GetRows() const
{
    return m_mat1.GetCols();
}

template <typename T, size_t Rows, size_t Cols, typename M1>
constexpr size_t MatrixTranspose<T, Rows, Cols, M1>::GetCols() const
{
    return m_mat1.GetRows();
}

template <typename T, size_t Rows, size_t Cols, typename M1>
constexpr T MatrixTranspose<T, Rows, Cols, M1>::operator()(size_t i,
                                                           size_t j) const
{
    return m_mat1(j, i);
}

template <typename T, size_t Rows, size_t Cols, typename M1, typename UOp>
constexpr size_t MatrixUnaryOp<T, Rows, Cols, M1, UOp>::GetRows() const
{
    return m_mat1.GetRows();
}

template <typename T, size_t Rows, size_t Cols, typename M1, typename UOp>
constexpr size_t MatrixUnaryOp<T, Rows, Cols, M1, UOp>::GetCols() const
{
    return m_mat1.GetCols();
}

template <typename T, size_t Rows, size_t Cols, typename M1, typename UOp>
constexpr T MatrixUnaryOp<T, Rows, Cols, M1, UOp>::operator()(size_t i,
                                                              size_t j) const
{
    return m_op(m_mat1(i, j));
}

template <typename T, size_t Rows, size_t Cols, typename M1>
constexpr auto Ceil(const MatrixExpression<T, Rows, Cols, M1>& a)
{
    return MatrixCeil<T, Rows, Cols, const M1&>{ a.GetDerived() };
}

template <typename T, size_t Rows, size_t Cols, typename M1>
constexpr auto Floor(const MatrixExpression<T, Rows, Cols, M1>& a)
{
    return MatrixFloor<T, Rows, Cols, const M1&>{ a.GetDerived() };
}

template <typename T, size_t Rows, size_t Cols, typename M1>
constexpr auto operator-(const MatrixExpression<T, Rows, Cols, M1>& m)
{
    return MatrixNegate<T, Rows, Cols, const M1&>{ m.GetDerived() };
}

template <typename T, size_t Rows, size_t Cols, typename E1, typename E2,
          typename BOp>
constexpr size_t MatrixElemWiseBinaryOp<T, Rows, Cols, E1, E2, BOp>::GetRows()
    const
{
    return m_mat1.GetRows();
}

template <typename T, size_t Rows, size_t Cols, typename E1, typename E2,
          typename BOp>
constexpr size_t MatrixElemWiseBinaryOp<T, Rows, Cols, E1, E2, BOp>::GetCols()
    const
{
    return m_mat1.GetCols();
}

template <typename T, size_t Rows, size_t Cols, typename E1, typename E2,
          typename BOp>
constexpr T MatrixElemWiseBinaryOp<T, Rows, Cols, E1, E2, BOp>::operator()(
    size_t i, size_t j) const
{
    return m_op(m_mat1(i, j), m_mat2(i, j));
}

template <typename T, size_t Rows, size_t Cols, typename M1, typename M2>
constexpr auto operator+(const MatrixExpression<T, Rows, Cols, M1>& a,
                         const MatrixExpression<T, Rows, Cols, M2>& b)
{
    return MatrixElemWiseAdd<T, Rows, Cols, const M1&, const M2&>{
        a.GetDerived(), b.GetDerived()
    };
}

template <typename T, size_t Rows, size_t Cols, typename M1, typename M2>
constexpr auto operator-(const MatrixExpression<T, Rows, Cols, M1>& a,
                         const MatrixExpression<T, Rows, Cols, M2>& b)
{
    return MatrixElemWiseSub<T, Rows, Cols, const M1&, const M2&>{
        a.GetDerived(), b.GetDerived()
    };
}

template <typename T, size_t Rows, size_t Cols, typename M1, typename M2>
constexpr auto ElemMul(const MatrixExpression<T, Rows, Cols, M1>& a,
                       const MatrixExpression<T, Rows, Cols, M2>& b)
{
    return MatrixElemWiseMul<T, Rows, Cols, const M1&, const M2&>{
        a.GetDerived(), b.GetDerived()
    };
}

template <typename T, size_t Rows, size_t Cols, typename M1, typename M2>
constexpr auto ElemDiv(const MatrixExpression<T, Rows, Cols, M1>& a,
                       const MatrixExpression<T, Rows, Cols, M2>& b)
{
    return MatrixElemWiseDiv<T, Rows, Cols, const M1&, const M2&>{
        a.GetDerived(), b.GetDerived()
    };
}

template <typename T, size_t Rows, size_t Cols, typename M1, typename M2>
constexpr auto Min(const MatrixExpression<T, Rows, Cols, M1>& a,
                   const MatrixExpression<T, Rows, Cols, M2>& b)
{
    return MatrixElemWiseMin<T, Rows, Cols, const M1&, const M2&>{
        a.GetDerived(), b.GetDerived()
    };
}

template <typename T, size_t Rows, size_t Cols, typename M1, typename M2>
constexpr auto Max(const MatrixExpression<T, Rows, Cols, M1>& a,
                   const MatrixExpression<T, Rows, Cols, M2>& b)
{
    return MatrixElemWiseMax<T, Rows, Cols, const M1&, const M2&>{
        a.GetDerived(), b.GetDerived()
    };
}

template <typename T, size_t Rows, size_t Cols, typename M1, typename BOp>
constexpr size_t MatrixScalarElemWiseBinaryOp<T, Rows, Cols, M1, BOp>::GetRows()
    const
{
    return m_mat1.GetRows();
}

template <typename T, size_t Rows, size_t Cols, typename M1, typename BOp>
constexpr size_t MatrixScalarElemWiseBinaryOp<T, Rows, Cols, M1, BOp>::GetCols()
    const
{
    return m_mat1.GetCols();
}

template <typename T, size_t Rows, size_t Cols, typename M1, typename BOp>
constexpr T MatrixScalarElemWiseBinaryOp<T, Rows, Cols, M1, BOp>::operator()(
    size_t i, size_t j) const
{
    return m_op(m_mat1(i, j), m_scalar2);
}

template <typename T, size_t Rows, size_t Cols, typename M1>
constexpr auto operator+(const MatrixExpression<T, Rows, Cols, M1>& a,
                         const T& b)
{
    return MatrixScalarElemWiseAdd<T, Rows, Cols, const M1&>{ a.GetDerived(),
                                                              b };
}

template <typename T, size_t Rows, size_t Cols, typename M1>
constexpr auto operator-(const MatrixExpression<T, Rows, Cols, M1>& a,
                         const T& b)
{
    return MatrixScalarElemWiseSub<T, Rows, Cols, const M1&>{ a.GetDerived(),
                                                              b };
}

template <typename T, size_t Rows, size_t Cols, typename M1>
constexpr auto operator*(const MatrixExpression<T, Rows, Cols, M1>& a,
                         const T& b)
{
    return MatrixScalarElemWiseMul<T, Rows, Cols, const M1&>{ a.GetDerived(),
                                                              b };
}

template <typename T, size_t Rows, size_t Cols, typename M1>
constexpr auto operator/(const MatrixExpression<T, Rows, Cols, M1>& a,
                         const T& b)
{
    return MatrixScalarElemWiseDiv<T, Rows, Cols, const M1&>{ a.GetDerived(),
                                                              b };
}

template <typename T, size_t Rows, size_t Cols, typename M2, typename BOp>
constexpr size_t ScalarMatrixElemWiseBinaryOp<T, Rows, Cols, M2, BOp>::GetRows()
    const
{
    return m_mat2.GetRows();
}

template <typename T, size_t Rows, size_t Cols, typename M2, typename BOp>
constexpr size_t ScalarMatrixElemWiseBinaryOp<T, Rows, Cols, M2, BOp>::GetCols()
    const
{
    return m_mat2.GetCols();
}

template <typename T, size_t Rows, size_t Cols, typename M2, typename BOp>
constexpr T ScalarMatrixElemWiseBinaryOp<T, Rows, Cols, M2, BOp>::operator()(
    size_t i, size_t j) const
{
    return m_op(m_scalar1, m_mat2(i, j));
}

template <typename T, size_t Rows, size_t Cols, typename M2>
constexpr auto operator+(const T& a,
                         const MatrixExpression<T, Rows, Cols, M2>& b)
{
    return ScalarMatrixElemWiseAdd<T, Rows, Cols, const M2&>{ a,
                                                              b.GetDerived() };
}

template <typename T, size_t Rows, size_t Cols, typename M2>
constexpr auto operator-(const T& a,
                         const MatrixExpression<T, Rows, Cols, M2>& b)
{
    return ScalarMatrixElemWiseSub<T, Rows, Cols, const M2&>{ a,
                                                              b.GetDerived() };
}

template <typename T, size_t Rows, size_t Cols, typename M2>
constexpr auto operator*(const T& a,
                         const MatrixExpression<T, Rows, Cols, M2>& b)
{
    return ScalarMatrixElemWiseMul<T, Rows, Cols, const M2&>{ a,
                                                              b.GetDerived() };
}

template <typename T, size_t Rows, size_t Cols, typename M2>
constexpr auto operator/(const T& a,
                         const MatrixExpression<T, Rows, Cols, M2>& b)
{
    return ScalarMatrixElemWiseDiv<T, Rows, Cols, const M2&>{ a,
                                                              b.GetDerived() };
}

template <typename T, size_t Rows, size_t Cols, typename M1, typename M2,
          typename M3, typename TOp>
constexpr size_t MatrixTernaryOp<T, Rows, Cols, M1, M2, M3, TOp>::GetRows()
    const
{
    return m_mat1.GetRows();
}

template <typename T, size_t Rows, size_t Cols, typename M1, typename M2,
          typename M3, typename TOp>
constexpr size_t MatrixTernaryOp<T, Rows, Cols, M1, M2, M3, TOp>::GetCols()
    const
{
    return m_mat1.GetCols();
}

template <typename T, size_t Rows, size_t Cols, typename M1, typename M2,
          typename M3, typename TOp>
constexpr T MatrixTernaryOp<T, Rows, Cols, M1, M2, M3, TOp>::operator()(
    size_t i, size_t j) const
{
    return m_op(m_mat1(i, j), m_mat2(i, j), m_mat3(i, j));
}

template <typename T, size_t Rows, size_t Cols, typename M1, typename M2,
          typename M3>
auto Clamp(const MatrixExpression<T, Rows, Cols, M1>& a,
           const MatrixExpression<T, Rows, Cols, M2>& low,
           const MatrixExpression<T, Rows, Cols, M3>& high)
{
    assert(a.GetRows() == low.GetRows() && a.GetRows() == high.GetRows());
    assert(a.GetCols() == low.GetCols() && a.GetCols() == high.GetCols());

    return MatrixClamp<T, Rows, Cols, const M1&, const M2&, const M3&>{
        a.GetDerived(), low.GetDerived(), high.GetDerived()
    };
}

template <typename T, size_t Rows, size_t Cols, typename M1, typename M2>
constexpr size_t MatrixMul<T, Rows, Cols, M1, M2>::GetRows() const
{
    return m_mat1.GetRows();
}

template <typename T, size_t Rows, size_t Cols, typename M1, typename M2>
constexpr size_t MatrixMul<T, Rows, Cols, M1, M2>::GetCols() const
{
    return m_mat2.GetCols();
}

template <typename T, size_t Rows, size_t Cols, typename M1, typename M2>
T MatrixMul<T, Rows, Cols, M1, M2>::operator()(size_t i, size_t j) const
{
    T sum = m_mat1(i, 0) * m_mat2(0, j);

    for (size_t k = 1; k < m_mat1.GetCols(); ++k)
    {
        sum += m_mat1(i, k) * m_mat2(k, j);
    }

    return sum;
}

template <typename T, size_t R1, size_t C1, size_t R2, size_t C2, typename M1,
          typename M2>
auto operator*(const MatrixExpression<T, R1, C1, M1>& a,
               const MatrixExpression<T, R2, C2, M2>& b)
{
    assert(a.GetCols() == b.GetRows());

    return MatrixMul<T, R1, C2, const M1&, const M2&>{ a.GetDerived(),
                                                       b.GetDerived() };
}
}  // namespace CubbyFlow

#endif
