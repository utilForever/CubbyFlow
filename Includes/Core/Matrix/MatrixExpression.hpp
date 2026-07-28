// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_MATRIX_EXPRESSION_HPP
#define CUBBYFLOW_MATRIX_EXPRESSION_HPP

#include <Core/Utils/Functors.hpp>

#include <functional>

namespace CubbyFlow
{
static constexpr size_t MATRIX_SIZE_DYNAMIC = 0;

template <size_t Rows, size_t Cols>
constexpr bool IsMatrixSizeDynamic()
{
    return (Rows == MATRIX_SIZE_DYNAMIC) || (Cols == MATRIX_SIZE_DYNAMIC);
}

template <size_t Rows, size_t Cols>
constexpr bool IsMatrixSizeStatic()
{
    return !IsMatrixSizeDynamic<Rows, Cols>();
}

template <size_t Rows, size_t Cols>
constexpr bool IsMatrixStaticSquare()
{
    return IsMatrixSizeStatic<Rows, Cols>() && (Rows == Cols);
}

template <size_t Rows, size_t Cols>
struct TraitIsMatrixSizeDynamic
{
    static const bool value = IsMatrixSizeDynamic<Rows, Cols>();
};

template <size_t Rows, size_t Cols>
struct TraitIsMatrixSizeStatic
{
    static const bool value = IsMatrixSizeStatic<Rows, Cols>();
};

template <size_t Rows, size_t Cols>
struct TraitIsMatrixSizeSquare
{
    static const bool value = IsMatrixStaticSquare<Rows, Cols>();
};

template <typename T, size_t Rows, size_t Cols>
class Matrix;

template <typename T, size_t Rows, size_t Cols, typename M1>
class MatrixDiagonal;

template <typename T, size_t Rows, size_t Cols, typename M1>
class MatrixOffDiagonal;

template <typename T, size_t Rows, size_t Cols, typename M1>
class MatrixTri;

template <typename T, size_t Rows, size_t Cols, typename M1>
class MatrixTranspose;

template <typename T, size_t Rows, size_t Cols, typename M1,
          typename UnaryOperation>
class MatrixUnaryOp;

template <typename T, size_t Rows, size_t Cols, typename M1,
          typename BinaryOperation>
class MatrixScalarElemWiseBinaryOp;

//!
//! \brief Base class for matrix expression.
//!
//! Matrix expression is a meta type that enables template expression
//! pattern.
//!
//! \tparam T           Real number type.
//! \tparam Rows        The number of rows.
//! \tparam Cols        The number of columns.
//! \tparam Derived     Subclass type.
//!
template <typename T, size_t Rows, size_t Cols, typename Derived>
class MatrixExpression
{
 public:
    using ValueType = T;

    //! Returns the number of rows.
    [[nodiscard]] constexpr size_t GetRows() const;

    //! Returns the number of columns.
    [[nodiscard]] constexpr size_t GetCols() const;

    //! Returns the evaluated value for (i, j).
    [[nodiscard]] T Eval(size_t i, size_t j) const;

    [[nodiscard]] Matrix<T, Rows, Cols> Eval() const;

    //! Returns true if this matrix is similar to the input matrix within the
    //! given tolerance.
    template <size_t R, size_t C, typename E>
    [[nodiscard]] bool IsSimilar(
        const MatrixExpression<T, R, C, E>& m,
        double tol = std::numeric_limits<double>::epsilon()) const;

    //! Returns true if this matrix is a square matrix.
    [[nodiscard]] constexpr bool IsSquare() const;

    [[nodiscard]] ValueType Sum() const;

    [[nodiscard]] ValueType Avg() const;

    [[nodiscard]] ValueType Min() const;

    [[nodiscard]] ValueType Max() const;

    [[nodiscard]] ValueType AbsMin() const;

    [[nodiscard]] ValueType AbsMax() const;

    [[nodiscard]] ValueType Trace() const;

    [[nodiscard]] ValueType Determinant() const;

    [[nodiscard]] size_t DominantAxis() const;

    [[nodiscard]] size_t SubdominantAxis() const;

    [[nodiscard]] ValueType Norm() const;

    [[nodiscard]] ValueType NormSquared() const;

    [[nodiscard]] ValueType FrobeniusNorm() const;

    [[nodiscard]] ValueType Length() const;

    [[nodiscard]] ValueType LengthSquared() const;

    //! Returns the distance to the other vector.
    template <size_t R, size_t C, typename E>
    [[nodiscard]] ValueType DistanceTo(
        const MatrixExpression<T, R, C, E>& other) const;

    //! Returns the squared distance to the other vector.
    template <size_t R, size_t C, typename E>
    [[nodiscard]] ValueType DistanceSquaredTo(
        const MatrixExpression<T, R, C, E>& other) const;

    [[nodiscard]] MatrixScalarElemWiseBinaryOp<T, Rows, Cols, const Derived&,
                                               std::divides<T>>
    Normalized() const;

    //! Returns diagonal part of this matrix.
    [[nodiscard]] MatrixDiagonal<T, Rows, Cols, const Derived&> Diagonal()
        const;

    //! Returns off-diagonal part of this matrix.
    [[nodiscard]] MatrixOffDiagonal<T, Rows, Cols, const Derived&> OffDiagonal()
        const;

    //! Returns strictly lower triangle part of this matrix.
    [[nodiscard]] MatrixTri<T, Rows, Cols, const Derived&> StrictLowerTri()
        const;

    //! Returns strictly upper triangle part of this matrix.
    [[nodiscard]] MatrixTri<T, Rows, Cols, const Derived&> StrictUpperTri()
        const;

    //! Returns lower triangle part of this matrix (including the diagonal).
    [[nodiscard]] MatrixTri<T, Rows, Cols, const Derived&> LowerTri() const;

    //! Returns upper triangle part of this matrix (including the diagonal).
    [[nodiscard]] MatrixTri<T, Rows, Cols, const Derived&> UpperTri() const;

    [[nodiscard]] MatrixTranspose<T, Rows, Cols, const Derived&> Transposed()
        const;

    //! Returns inverse matrix.
    [[nodiscard]] Matrix<T, Rows, Cols> Inverse() const;

    template <typename U>
    MatrixUnaryOp<U, Rows, Cols, const Derived&, TypeCast<T, U>> CastTo() const;

    template <size_t R, size_t C, typename E, typename U = ValueType>
    std::enable_if_t<(IsMatrixSizeDynamic<Rows, Cols>() || Cols == 1) &&
                         (IsMatrixSizeDynamic<R, C>() || C == 1),
                     U>
    Dot(const MatrixExpression<T, R, C, E>& expression) const;

    template <size_t R, size_t C, typename E, typename U = ValueType>
    std::enable_if_t<(IsMatrixSizeDynamic<Rows, Cols>() ||
                      (Rows == 2 && Cols == 1)) &&
                         (IsMatrixSizeDynamic<R, C>() || (R == 2 && C == 1)),
                     U>
    Cross(const MatrixExpression<T, R, C, E>& expression) const;

    template <size_t R, size_t C, typename E, typename U = ValueType>
    std::enable_if_t<(IsMatrixSizeDynamic<Rows, Cols>() ||
                      (Rows == 3 && Cols == 1)) &&
                         (IsMatrixSizeDynamic<R, C>() || (R == 3 && C == 1)),
                     Matrix<U, 3, 1>>
    Cross(const MatrixExpression<T, R, C, E>& expression) const;

    //! Returns the reflection vector to the surface with given surface normal.
    template <size_t R, size_t C, typename E, typename U = ValueType>
    std::enable_if_t<(IsMatrixSizeDynamic<Rows, Cols>() ||
                      ((Rows == 2 || Rows == 3) && Cols == 1)) &&
                         (IsMatrixSizeDynamic<R, C>() ||
                          ((R == 2 || R == 3) && C == 1)),
                     Matrix<U, Rows, 1>>
    Reflected(const MatrixExpression<T, R, C, E>& normal) const;

    //! Returns the projected vector to the surface with given surface normal.
    template <size_t R, size_t C, typename E, typename U = ValueType>
    std::enable_if_t<(IsMatrixSizeDynamic<Rows, Cols>() ||
                      ((Rows == 2 || Rows == 3) && Cols == 1)) &&
                         (IsMatrixSizeDynamic<R, C>() ||
                          ((R == 2 || R == 3) && C == 1)),
                     Matrix<U, Rows, 1>>
    Projected(const MatrixExpression<T, R, C, E>& normal) const;

    //! Returns the tangential vector for this vector.
    template <typename U = ValueType>
    std::enable_if_t<(IsMatrixSizeDynamic<Rows, Cols>() ||
                      (Rows == 2 && Cols == 1)),
                     Matrix<U, 2, 1>>
    Tangential() const;

    //! Returns the tangential vectors for this vector.
    template <typename U = ValueType>
    std::enable_if_t<(IsMatrixSizeDynamic<Rows, Cols>() ||
                      (Rows == 3 && Cols == 1)),
                     std::tuple<Matrix<U, 3, 1>, Matrix<U, 3, 1>>>
    Tangentials() const;

    //! Returns actual implementation (the subclass).
    [[nodiscard]] Derived& GetDerived();

    //! Returns actual implementation (the subclass).
    [[nodiscard]] const Derived& GetDerived() const;

 protected:
    // Prohibits constructing this class instance.
    MatrixExpression() = default;

    constexpr static T Determinant(const MatrixExpression<T, 1, 1, Derived>& m);

    constexpr static T Determinant(const MatrixExpression<T, 2, 2, Derived>& m);

    constexpr static T Determinant(const MatrixExpression<T, 3, 3, Derived>& m);

    constexpr static T Determinant(const MatrixExpression<T, 4, 4, Derived>& m);

    template <typename U = ValueType>
    static std::enable_if_t<
        (Rows > 4 && Cols > 4) || IsMatrixSizeDynamic<Rows, Cols>(), U>
    Determinant(const MatrixExpression<T, Rows, Cols, Derived>& m);

    static void Inverse(const MatrixExpression<T, 1, 1, Derived>& m,
                        Matrix<T, Rows, Cols>& result);

    static void Inverse(const MatrixExpression<T, 2, 2, Derived>& m,
                        Matrix<T, Rows, Cols>& result);

    static void Inverse(const MatrixExpression<T, 3, 3, Derived>& m,
                        Matrix<T, Rows, Cols>& result);

    static void Inverse(const MatrixExpression<T, 4, 4, Derived>& m,
                        Matrix<T, Rows, Cols>& result);

    template <typename M = Matrix<T, Rows, Cols>>
    static void Inverse(const MatrixExpression& m,
                        std::enable_if_t<(Rows > 4 && Cols > 4) ||
                                             IsMatrixSizeDynamic<Rows, Cols>(),
                                         M>& result);
};

template <typename T, size_t Rows, size_t Cols>
class MatrixConstant
    : public MatrixExpression<T, Rows, Cols, MatrixConstant<T, Rows, Cols>>
{
 public:
    constexpr MatrixConstant(size_t r, size_t c, const T& val)
        : m_rows(r), m_cols(c), m_val(val)
    {
        // Do nothing
    }

    [[nodiscard]] constexpr size_t GetRows() const;

    [[nodiscard]] constexpr size_t GetCols() const;

    constexpr T operator()(size_t, size_t) const;

 private:
    size_t m_rows;
    size_t m_cols;
    T m_val;
};

template <typename T, size_t Rows, size_t Cols, typename M1>
class MatrixDiagonal
    : public MatrixExpression<T, Rows, Cols, MatrixDiagonal<T, Rows, Cols, M1>>
{
 public:
    explicit constexpr MatrixDiagonal(const M1& mat1) : m_mat1(mat1)
    {
        // Do nothing
    }

    [[nodiscard]] constexpr size_t GetRows() const;

    [[nodiscard]] constexpr size_t GetCols() const;

    T operator()(size_t i, size_t j) const;

 private:
    M1 m_mat1;
};

template <typename T, size_t Rows, size_t Cols, typename M1>
class MatrixOffDiagonal
    : public MatrixExpression<T, Rows, Cols,
                              MatrixOffDiagonal<T, Rows, Cols, M1>>
{
 public:
    explicit constexpr MatrixOffDiagonal(const M1& mat1) : m_mat1(mat1)
    {
        // Do nothing
    }

    [[nodiscard]] constexpr size_t GetRows() const;

    [[nodiscard]] constexpr size_t GetCols() const;

    T operator()(size_t i, size_t j) const;

 private:
    M1 m_mat1;
};

template <typename T, size_t Rows, size_t Cols, typename M1>
class MatrixTri
    : public MatrixExpression<T, Rows, Cols, MatrixTri<T, Rows, Cols, M1>>
{
 public:
    constexpr MatrixTri(const M1& mat1, bool isUpper, bool isStrict)
        : m_mat1(mat1), m_isUpper(isUpper), m_isStrict(isStrict)
    {
        // Do nothing
    }

    [[nodiscard]] constexpr size_t GetRows() const;

    [[nodiscard]] constexpr size_t GetCols() const;

    T operator()(size_t i, size_t j) const;

 private:
    M1 m_mat1;
    bool m_isUpper;
    bool m_isStrict;
};

template <typename T, size_t Rows, size_t Cols, typename M1>
class MatrixTranspose
    : public MatrixExpression<T, Rows, Cols, MatrixTranspose<T, Rows, Cols, M1>>
{
 public:
    explicit constexpr MatrixTranspose(const M1& m_mat1) : m_mat1(m_mat1)
    {
        // Do nothing
    }

    [[nodiscard]] constexpr size_t GetRows() const;

    [[nodiscard]] constexpr size_t GetCols() const;

    constexpr T operator()(size_t i, size_t j) const;

 private:
    M1 m_mat1;
};

template <typename T, size_t Rows, size_t Cols, typename M1,
          typename UnaryOperation>
class MatrixUnaryOp
    : public MatrixExpression<T, Rows, Cols,
                              MatrixUnaryOp<T, Rows, Cols, M1, UnaryOperation>>
{
 public:
    explicit constexpr MatrixUnaryOp(const M1& mat1) : m_mat1(mat1)
    {
        // Do nothing
    }

    [[nodiscard]] constexpr size_t GetRows() const;

    [[nodiscard]] constexpr size_t GetCols() const;

    constexpr T operator()(size_t i, size_t j) const;

 private:
    M1 m_mat1;
    UnaryOperation m_op;
};

template <typename T, size_t Rows, size_t Cols, typename M1>
using MatrixNegate = MatrixUnaryOp<T, Rows, Cols, M1, std::negate<T>>;

template <typename T, size_t Rows, size_t Cols, typename M1>
using MatrixCeil = MatrixUnaryOp<T, Rows, Cols, M1, DoCeil<T>>;

template <typename T, size_t Rows, size_t Cols, typename M1>
using MatrixFloor = MatrixUnaryOp<T, Rows, Cols, M1, DoFloor<T>>;

template <typename T, size_t Rows, size_t Cols, typename U, typename M1>
using MatrixTypeCast = MatrixUnaryOp<U, Rows, Cols, M1, TypeCast<T, U>>;

template <typename T, size_t Rows, size_t Cols, typename M1>
constexpr auto Ceil(const MatrixExpression<T, Rows, Cols, M1>& a);

template <typename T, size_t Rows, size_t Cols, typename M1>
constexpr auto Floor(const MatrixExpression<T, Rows, Cols, M1>& a);

template <typename T, size_t Rows, size_t Cols, typename M1>
constexpr auto operator-(const MatrixExpression<T, Rows, Cols, M1>& m);

//!
//! \brief Matrix expression for element-wise binary operation.
//!
//! This matrix expression represents a binary matrix operation that takes
//! two input matrix expressions.
//!
//! \tparam T                   Real number type.
//! \tparam E1                  First input expression type.
//! \tparam E2                  Second input expression type.
//! \tparam BinaryOperation     Binary operation.
//!
template <typename T, size_t Rows, size_t Cols, typename E1, typename E2,
          typename BinaryOperation>
class MatrixElemWiseBinaryOp
    : public MatrixExpression<
          T, Rows, Cols,
          MatrixElemWiseBinaryOp<T, Rows, Cols, E1, E2, BinaryOperation>>
{
 public:
    constexpr MatrixElemWiseBinaryOp(const E1& mat1, const E2& mat2)
        : m_mat1(mat1), m_mat2(mat2)
    {
        // Do nothing
    }

    [[nodiscard]] constexpr size_t GetRows() const;

    [[nodiscard]] constexpr size_t GetCols() const;

    constexpr T operator()(size_t i, size_t j) const;

 private:
    E1 m_mat1;
    E2 m_mat2;
    BinaryOperation m_op;
};

//! Matrix expression for element-wise matrix-matrix addition.
template <typename T, size_t Rows, size_t Cols, typename E1, typename E2>
using MatrixElemWiseAdd =
    MatrixElemWiseBinaryOp<T, Rows, Cols, E1, E2, std::plus<T>>;

//! Matrix expression for element-wise matrix-matrix subtraction.
template <typename T, size_t Rows, size_t Cols, typename E1, typename E2>
using MatrixElemWiseSub =
    MatrixElemWiseBinaryOp<T, Rows, Cols, E1, E2, std::minus<T>>;

//! Matrix expression for element-wise matrix-matrix multiplication.
template <typename T, size_t Rows, size_t Cols, typename E1, typename E2>
using MatrixElemWiseMul =
    MatrixElemWiseBinaryOp<T, Rows, Cols, E1, E2, std::multiplies<T>>;

//! Matrix expression for element-wise matrix-matrix division.
template <typename T, size_t Rows, size_t Cols, typename E1, typename E2>
using MatrixElemWiseDiv =
    MatrixElemWiseBinaryOp<T, Rows, Cols, E1, E2, std::divides<T>>;

//! Matrix expression for element-wise matrix-matrix min operation.
template <typename T, size_t Rows, size_t Cols, typename E1, typename E2>
using MatrixElemWiseMin =
    MatrixElemWiseBinaryOp<T, Rows, Cols, E1, E2, DoMin<T>>;

//! Matrix expression for element-wise matrix-matrix max operation.
template <typename T, size_t Rows, size_t Cols, typename E1, typename E2>
using MatrixElemWiseMax =
    MatrixElemWiseBinaryOp<T, Rows, Cols, E1, E2, DoMax<T>>;

template <typename T, size_t Rows, size_t Cols, typename M1, typename M2>
constexpr auto operator+(const MatrixExpression<T, Rows, Cols, M1>& a,
                         const MatrixExpression<T, Rows, Cols, M2>& b);

template <typename T, size_t Rows, size_t Cols, typename M1, typename M2>
constexpr auto operator-(const MatrixExpression<T, Rows, Cols, M1>& a,
                         const MatrixExpression<T, Rows, Cols, M2>& b);

template <typename T, size_t Rows, size_t Cols, typename M1, typename M2>
constexpr auto ElemMul(const MatrixExpression<T, Rows, Cols, M1>& a,
                       const MatrixExpression<T, Rows, Cols, M2>& b);

template <typename T, size_t Rows, size_t Cols, typename M1, typename M2>
constexpr auto ElemDiv(const MatrixExpression<T, Rows, Cols, M1>& a,
                       const MatrixExpression<T, Rows, Cols, M2>& b);

template <typename T, size_t Rows, size_t Cols, typename M1, typename M2>
constexpr auto Min(const MatrixExpression<T, Rows, Cols, M1>& a,
                   const MatrixExpression<T, Rows, Cols, M2>& b);

template <typename T, size_t Rows, size_t Cols, typename M1, typename M2>
constexpr auto Max(const MatrixExpression<T, Rows, Cols, M1>& a,
                   const MatrixExpression<T, Rows, Cols, M2>& b);

template <typename T, size_t Rows, size_t Cols, typename M1,
          typename BinaryOperation>
class MatrixScalarElemWiseBinaryOp
    : public MatrixExpression<
          T, Rows, Cols,
          MatrixScalarElemWiseBinaryOp<T, Rows, Cols, M1, BinaryOperation>>
{
 public:
    constexpr MatrixScalarElemWiseBinaryOp(const M1& mat1, const T& scalar2)
        : m_mat1(mat1), m_scalar2(scalar2)
    {
        // Do nothing
    }

    [[nodiscard]] constexpr size_t GetRows() const;

    [[nodiscard]] constexpr size_t GetCols() const;

    constexpr T operator()(size_t i, size_t j) const;

 private:
    M1 m_mat1;
    T m_scalar2;
    BinaryOperation m_op;
};

template <typename T, size_t Rows, size_t Cols, typename M1>
using MatrixScalarElemWiseAdd =
    MatrixScalarElemWiseBinaryOp<T, Rows, Cols, M1, std::plus<T>>;

template <typename T, size_t Rows, size_t Cols, typename M1>
using MatrixScalarElemWiseSub =
    MatrixScalarElemWiseBinaryOp<T, Rows, Cols, M1, std::minus<T>>;

template <typename T, size_t Rows, size_t Cols, typename M1>
using MatrixScalarElemWiseMul =
    MatrixScalarElemWiseBinaryOp<T, Rows, Cols, M1, std::multiplies<T>>;

template <typename T, size_t Rows, size_t Cols, typename M1>
using MatrixScalarElemWiseDiv =
    MatrixScalarElemWiseBinaryOp<T, Rows, Cols, M1, std::divides<T>>;

template <typename T, size_t Rows, size_t Cols, typename M1>
constexpr auto operator+(const MatrixExpression<T, Rows, Cols, M1>& a,
                         const T& b);

template <typename T, size_t Rows, size_t Cols, typename M1>
constexpr auto operator-(const MatrixExpression<T, Rows, Cols, M1>& a,
                         const T& b);

template <typename T, size_t Rows, size_t Cols, typename M1>
constexpr auto operator*(const MatrixExpression<T, Rows, Cols, M1>& a,
                         const T& b);

template <typename T, size_t Rows, size_t Cols, typename M1>
constexpr auto operator/(const MatrixExpression<T, Rows, Cols, M1>& a,
                         const T& b);

template <typename T, size_t Rows, size_t Cols, typename M2,
          typename BinaryOperation>
class ScalarMatrixElemWiseBinaryOp
    : public MatrixExpression<
          T, Rows, Cols,
          ScalarMatrixElemWiseBinaryOp<T, Rows, Cols, M2, BinaryOperation>>
{
 public:
    constexpr ScalarMatrixElemWiseBinaryOp(const T& s1, const M2& m2)
        : m_scalar1(s1), m_mat2(m2)
    {
        // Do nothing
    }

    [[nodiscard]] constexpr size_t GetRows() const;

    [[nodiscard]] constexpr size_t GetCols() const;

    constexpr T operator()(size_t i, size_t j) const;

 private:
    T m_scalar1;
    M2 m_mat2;
    BinaryOperation m_op;
};

template <typename T, size_t Rows, size_t Cols, typename M2>
using ScalarMatrixElemWiseAdd =
    ScalarMatrixElemWiseBinaryOp<T, Rows, Cols, M2, std::plus<T>>;

template <typename T, size_t Rows, size_t Cols, typename M2>
using ScalarMatrixElemWiseSub =
    ScalarMatrixElemWiseBinaryOp<T, Rows, Cols, M2, std::minus<T>>;

template <typename T, size_t Rows, size_t Cols, typename M2>
using ScalarMatrixElemWiseMul =
    ScalarMatrixElemWiseBinaryOp<T, Rows, Cols, M2, std::multiplies<T>>;

template <typename T, size_t Rows, size_t Cols, typename M2>
using ScalarMatrixElemWiseDiv =
    ScalarMatrixElemWiseBinaryOp<T, Rows, Cols, M2, std::divides<T>>;

template <typename T, size_t Rows, size_t Cols, typename M2>
constexpr auto operator+(const T& a,
                         const MatrixExpression<T, Rows, Cols, M2>& b);

template <typename T, size_t Rows, size_t Cols, typename M2>
constexpr auto operator-(const T& a,
                         const MatrixExpression<T, Rows, Cols, M2>& b);

template <typename T, size_t Rows, size_t Cols, typename M2>
constexpr auto operator*(const T& a,
                         const MatrixExpression<T, Rows, Cols, M2>& b);

template <typename T, size_t Rows, size_t Cols, typename M2>
constexpr auto operator/(const T& a,
                         const MatrixExpression<T, Rows, Cols, M2>& b);

template <typename T, size_t Rows, size_t Cols, typename M1, typename M2,
          typename M3, typename TernaryOperation>
class MatrixTernaryOp
    : public MatrixExpression<
          T, Rows, Cols,
          MatrixTernaryOp<T, Rows, Cols, M1, M2, M3, TernaryOperation>>
{
 public:
    constexpr MatrixTernaryOp(const M1& mat1, const M2& mat2, const M3& mat3)
        : m_mat1(mat1), m_mat2(mat2), m_mat3(mat3)
    {
        // Do nothing
    }

    [[nodiscard]] constexpr size_t GetRows() const;

    [[nodiscard]] constexpr size_t GetCols() const;

    constexpr T operator()(size_t i, size_t j) const;

 private:
    M1 m_mat1;
    M2 m_mat2;
    M3 m_mat3;
    TernaryOperation m_op;
};

template <typename T, size_t Rows, size_t Cols, typename M1, typename M2,
          typename M3>
using MatrixClamp = MatrixTernaryOp<T, Rows, Cols, M1, M2, M3, DoClamp<T>>;

template <typename T, size_t Rows, size_t Cols, typename M1, typename M2,
          typename M3>
auto Clamp(const MatrixExpression<T, Rows, Cols, M1>& a,
           const MatrixExpression<T, Rows, Cols, M2>& low,
           const MatrixExpression<T, Rows, Cols, M3>& high);

template <typename T, size_t Rows, size_t Cols, typename M1, typename M2>
class MatrixMul
    : public MatrixExpression<T, Rows, Cols, MatrixMul<T, Rows, Cols, M1, M2>>
{
 public:
    constexpr MatrixMul(const M1& mat1, const M2& mat2)
        : m_mat1(mat1), m_mat2(mat2)
    {
        // Do nothing
    }

    [[nodiscard]] constexpr size_t GetRows() const;

    [[nodiscard]] constexpr size_t GetCols() const;

    T operator()(size_t i, size_t j) const;

 private:
    M1 m_mat1;
    M2 m_mat2;
};

template <typename T, size_t R1, size_t C1, size_t R2, size_t C2, typename M1,
          typename M2>
auto operator*(const MatrixExpression<T, R1, C1, M1>& a,
               const MatrixExpression<T, R2, C2, M2>& b);
}  // namespace CubbyFlow

#include <Core/Matrix/MatrixExpression-Impl.hpp>

#endif