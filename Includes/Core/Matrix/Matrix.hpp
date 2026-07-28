// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_MATRIX_HPP
#define CUBBYFLOW_MATRIX_HPP

#include <Core/Matrix/MatrixDenseBase.hpp>
#include <Core/Matrix/MatrixExpression.hpp>
#include <Core/Utils/Macros.hpp>
#include <Core/Utils/NestedInitializerList.hpp>
#include <Core/Utils/TypeHelpers.hpp>

#include <array>
#include <cstdint>
#include <vector>

namespace CubbyFlow
{
template <typename T, size_t Rows, size_t Cols>
class Matrix final
    : public MatrixExpression<T, Rows, Cols, Matrix<T, Rows, Cols>>,
      public MatrixDenseBase<T, Rows, Cols, Matrix<T, Rows, Cols>>
{
 public:
    static_assert(IsMatrixSizeStatic<Rows, Cols>(),
                  "This class should be a static-sized matrix.");

    using Base = MatrixDenseBase<T, Rows, Cols, Matrix<T, Rows, Cols>>;
    using Base::CopyFrom;
    using Base::operator();

    using ValueType = T;
    using Reference = T&;
    using ConstReference = const T&;
    using Pointer = T*;
    using ConstPointer = const T*;
    using Iterator = Pointer;
    using ConstIterator = ConstPointer;

    constexpr Matrix() : m_elements{}
    {
        // Do nothing
    }

    Matrix(ConstReference value);

    template <typename... Args>
    constexpr Matrix(ConstReference first, Args... rest)
        : m_elements{ { first, static_cast<ValueType>(rest)... } }
    {
        // Do nothing
    }

    template <size_t R, size_t C, typename E>
    Matrix(const MatrixExpression<T, R, C, E>& expression);

    Matrix(const NestedInitializerListsT<T, 2>& lst);

    explicit Matrix(ConstPointer ptr);

    ~Matrix() = default;

    constexpr Matrix(const Matrix& other) : m_elements(other.m_elements)
    {
        // Do nothing
    }

    constexpr Matrix(Matrix&& other) noexcept
        : m_elements(std::move(other.m_elements))
    {
        // Do nothing
    }

    Matrix& operator=(const Matrix& other)
    {
        m_elements = other.m_elements;
        return *this;
    }

    Matrix& operator=(Matrix&& other) noexcept
    {
        m_elements = std::move(other.m_elements);
        return *this;
    }

    void Fill(const T& val);

    void Fill(const std::function<T(size_t i)>& func);

    void Fill(const std::function<T(size_t i, size_t j)>& func);

    void Swap(Matrix& other);

    [[nodiscard]] constexpr size_t GetRows() const;

    [[nodiscard]] constexpr size_t GetCols() const;

    Iterator begin();

    constexpr ConstIterator begin() const;

    Iterator end();

    constexpr ConstIterator end() const;

    Pointer data();

    constexpr ConstPointer data() const;

    Reference operator[](size_t i);

    ConstReference operator[](size_t i) const;

 private:
    std::array<T, Rows * Cols> m_elements;
};

template <typename T>
class Matrix<T, 1, 1> final : public MatrixExpression<T, 1, 1, Matrix<T, 1, 1>>,
                              public MatrixDenseBase<T, 1, 1, Matrix<T, 1, 1>>
{
 public:
    using Base = MatrixDenseBase<T, 1, 1, Matrix<T, 1, 1>>;
    using Base::operator();

    using ValueType = T;
    using Reference = T&;
    using ConstReference = const T&;
    using Pointer = T*;
    using ConstPointer = const T*;
    using Iterator = Pointer;
    using ConstIterator = ConstPointer;

    constexpr Matrix() : x(T{})
    {
        // Do nothing
    }

    constexpr Matrix(const T& _x) : x(_x)
    {
        // Do nothing
    }

    template <size_t R, size_t C, typename E>
    Matrix(const MatrixExpression<T, R, C, E>& expression);

    Matrix(const std::initializer_list<T>& lst);

    ~Matrix() = default;

    constexpr Matrix(const Matrix& other) : x(other.x)
    {
        // Do nothing
    }

    constexpr Matrix(Matrix&& other) noexcept : x(std::move(other.x))
    {
        // Do nothing
    }

    Matrix& operator=(const Matrix& other)
    {
        x = other.x;
        return *this;
    }

    Matrix& operator=(Matrix&& other) noexcept
    {
        x = std::move(other.x);
        return *this;
    }

    void Fill(const T& val);

    void Fill(const std::function<T(size_t i)>& func);

    void Fill(const std::function<T(size_t i, size_t j)>& func);

    void Swap(Matrix& other);

    [[nodiscard]] constexpr size_t GetRows() const;

    [[nodiscard]] constexpr size_t GetCols() const;

    [[nodiscard]] Iterator begin();

    [[nodiscard]] constexpr ConstIterator begin() const;

    [[nodiscard]] Iterator end();

    [[nodiscard]] constexpr ConstIterator end() const;

    [[nodiscard]] Pointer data();

    [[nodiscard]] constexpr ConstPointer data() const;

    Reference operator[](size_t i);

    ConstReference operator[](size_t i) const;

    constexpr static Matrix MakeUnitX();

    constexpr static Matrix MakeUnit(size_t i);

    ValueType x;
};

template <typename T>
class Matrix<T, 2, 1> final : public MatrixExpression<T, 2, 1, Matrix<T, 2, 1>>,
                              public MatrixDenseBase<T, 2, 1, Matrix<T, 2, 1>>
{
 public:
    using Base = MatrixDenseBase<T, 2, 1, Matrix<T, 2, 1>>;
    using Base::operator();

    using ValueType = T;
    using Reference = T&;
    using ConstReference = const T&;
    using Pointer = T*;
    using ConstPointer = const T*;
    using Iterator = Pointer;
    using ConstIterator = ConstPointer;

    constexpr Matrix() : x(T{}), y(T{})
    {
        // Do nothing
    }

    constexpr Matrix(const T& _x, const T& _y) : x(_x), y(_y)
    {
        // Do nothing
    }

    template <size_t R, size_t C, typename E>
    Matrix(const MatrixExpression<T, R, C, E>& expression);

    Matrix(const std::initializer_list<T>& lst);

    ~Matrix() = default;

    constexpr Matrix(const Matrix& other) : x(other.x), y(other.y)
    {
        // Do nothing
    }

    constexpr Matrix(Matrix&& other) noexcept
        : x(std::move(other.x)), y(std::move(other.y))
    {
        // Do nothing
    }

    Matrix& operator=(const Matrix& other)
    {
        x = other.x;
        y = other.y;
        return *this;
    }

    Matrix& operator=(Matrix&& other) noexcept
    {
        x = std::move(other.x);
        y = std::move(other.y);
        return *this;
    }

    void Fill(const T& val);

    void Fill(const std::function<T(size_t i)>& func);

    void Fill(const std::function<T(size_t i, size_t j)>& func);

    void Swap(Matrix& other);

    [[nodiscard]] constexpr size_t GetRows() const;

    [[nodiscard]] constexpr size_t GetCols() const;

    [[nodiscard]] Iterator begin();

    [[nodiscard]] constexpr ConstIterator begin() const;

    [[nodiscard]] Iterator end();

    [[nodiscard]] constexpr ConstIterator end() const;

    [[nodiscard]] Pointer data();

    [[nodiscard]] constexpr ConstPointer data() const;

    Reference operator[](size_t i);

    ConstReference operator[](size_t i) const;

    constexpr static Matrix MakeUnitX();

    constexpr static Matrix MakeUnitY();

    constexpr static Matrix MakeUnit(size_t i);

    ValueType x;
    ValueType y;
};

template <typename T>
class Matrix<T, 3, 1> final : public MatrixExpression<T, 3, 1, Matrix<T, 3, 1>>,
                              public MatrixDenseBase<T, 3, 1, Matrix<T, 3, 1>>
{
 public:
    using Base = MatrixDenseBase<T, 3, 1, Matrix<T, 3, 1>>;
    using Base::operator();

    using ValueType = T;
    using Reference = T&;
    using ConstReference = const T&;
    using Pointer = T*;
    using ConstPointer = const T*;
    using Iterator = Pointer;
    using ConstIterator = ConstPointer;

    constexpr Matrix() : x(T{}), y(T{}), z(T{})
    {
        // Do nothing
    }

    constexpr Matrix(const Matrix<T, 2, 1>& _xy, const T& _z)
        : x(_xy.x), y(_xy.y), z(_z)
    {
        // Do nothing
    }

    constexpr Matrix(const T& _x, const T& _y, const T& _z)
        : x(_x), y(_y), z(_z)
    {
        // Do nothing
    }

    template <size_t R, size_t C, typename E>
    Matrix(const MatrixExpression<T, R, C, E>& expression);

    Matrix(const std::initializer_list<T>& lst);

    ~Matrix() = default;

    constexpr Matrix(const Matrix&) = default;

    constexpr Matrix(Matrix&&) noexcept = default;

    Matrix& operator=(const Matrix&) = default;

    Matrix& operator=(Matrix&&) noexcept = default;

    void Fill(const T& val);

    void Fill(const std::function<T(size_t i)>& func);

    void Fill(const std::function<T(size_t i, size_t j)>& func);

    void Swap(Matrix& other);

    [[nodiscard]] constexpr size_t GetRows() const;

    [[nodiscard]] constexpr size_t GetCols() const;

    [[nodiscard]] Iterator begin();

    [[nodiscard]] constexpr ConstIterator begin() const;

    [[nodiscard]] Iterator end();

    [[nodiscard]] constexpr ConstIterator end() const;

    [[nodiscard]] Pointer data();

    [[nodiscard]] constexpr ConstPointer data() const;

    Reference operator[](size_t i);

    ConstReference operator[](size_t i) const;

    constexpr static Matrix MakeUnitX();

    constexpr static Matrix MakeUnitY();

    constexpr static Matrix MakeUnitZ();

    constexpr static Matrix MakeUnit(size_t i);

    ValueType x;
    ValueType y;
    ValueType z;
};

template <typename T>
class Matrix<T, 4, 1> final : public MatrixExpression<T, 4, 1, Matrix<T, 4, 1>>,
                              public MatrixDenseBase<T, 4, 1, Matrix<T, 4, 1>>
{
 public:
    using Base = MatrixDenseBase<T, 4, 1, Matrix<T, 4, 1>>;
    using Base::operator();

    using ValueType = T;
    using Reference = T&;
    using ConstReference = const T&;
    using Pointer = T*;
    using ConstPointer = const T*;
    using Iterator = Pointer;
    using ConstIterator = ConstPointer;

    constexpr Matrix() : x(T{}), y(T{}), z(T{}), w(T{})
    {
        // Do nothing
    }

    constexpr Matrix(const T& _x, const T& _y, const T& _z, const T& _w)
        : x(_x), y(_y), z(_z), w(_w)
    {
        // Do nothing
    }

    template <size_t R, size_t C, typename E>
    Matrix(const MatrixExpression<T, R, C, E>& expression);

    Matrix(const std::initializer_list<T>& lst);

    ~Matrix() = default;

    constexpr Matrix(const Matrix& other)
        : x(other.x), y(other.y), z(other.z), w(other.w)
    {
        // Do nothing
    }

    constexpr Matrix(Matrix&& other) noexcept
        : x(std::move(other.x)),
          y(std::move(other.y)),
          z(std::move(other.z)),
          w(std::move(other.w))
    {
        // Do nothing
    }

    Matrix& operator=(const Matrix& other)
    {
        x = other.x;
        y = other.y;
        z = other.z;
        w = other.w;
        return *this;
    }

    Matrix& operator=(Matrix&& other) noexcept
    {
        x = std::move(other.x);
        y = std::move(other.y);
        z = std::move(other.z);
        w = std::move(other.w);
        return *this;
    }

    void Fill(const T& val);

    void Fill(const std::function<T(size_t i)>& func);

    void Fill(const std::function<T(size_t i, size_t j)>& func);

    void Swap(Matrix& other);

    [[nodiscard]] constexpr size_t GetRows() const;

    [[nodiscard]] constexpr size_t GetCols() const;

    [[nodiscard]] Iterator begin();

    [[nodiscard]] constexpr ConstIterator begin() const;

    [[nodiscard]] Iterator end();

    [[nodiscard]] constexpr ConstIterator end() const;

    [[nodiscard]] Pointer data();

    [[nodiscard]] constexpr ConstPointer data() const;

    Reference operator[](size_t i);

    ConstReference operator[](size_t i) const;

    constexpr static Matrix MakeUnitX();

    constexpr static Matrix MakeUnitY();

    constexpr static Matrix MakeUnitZ();

    constexpr static Matrix MakeUnitW();

    constexpr static Matrix MakeUnit(size_t i);

    ValueType x;
    ValueType y;
    ValueType z;
    ValueType w;
};

template <typename T>
class Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC> final
    : public MatrixExpression<
          T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC,
          Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>>,
      public MatrixDenseBase<
          T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC,
          Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>>
{
 public:
    using ValueType = T;
    using Reference = T&;
    using ConstReference = const T&;
    using Pointer = T*;
    using ConstPointer = const T*;
    using Iterator = Pointer;
    using ConstIterator = ConstPointer;
    using MatrixDenseBase<
        T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC,
        Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>>::CopyFrom;

    Matrix();

    Matrix(size_t rows, size_t cols, ConstReference value = ValueType{});

    template <size_t R, size_t C, typename E>
    Matrix(const MatrixExpression<T, R, C, E>& expression);

    Matrix(const NestedInitializerListsT<T, 2>& lst);

    explicit Matrix(size_t rows, size_t cols, ConstPointer ptr);

    ~Matrix() = default;

    Matrix(const Matrix& other);

    Matrix(Matrix&& other) noexcept;

    Matrix& operator=(const Matrix& other);

    Matrix& operator=(Matrix&& other) noexcept;

    void Fill(const T& val);

    void Fill(const std::function<T(size_t i)>& func);

    void Fill(const std::function<T(size_t i, size_t j)>& func);

    void Swap(Matrix& other);

    void Resize(size_t rows, size_t cols, ConstReference val = ValueType{});

    void Clear();

    [[nodiscard]] size_t GetRows() const;

    [[nodiscard]] size_t GetCols() const;

    [[nodiscard]] Iterator begin();

    [[nodiscard]] ConstIterator begin() const;

    [[nodiscard]] Iterator end();

    [[nodiscard]] ConstIterator end() const;

    [[nodiscard]] Pointer data();

    [[nodiscard]] ConstPointer data() const;

    Reference operator[](size_t i);

    ConstReference operator[](size_t i) const;

 private:
    std::vector<T> m_elements;
    size_t m_rows = 0;
    size_t m_cols = 0;
};

template <typename T>
class Matrix<T, MATRIX_SIZE_DYNAMIC, 1> final
    : public MatrixExpression<T, MATRIX_SIZE_DYNAMIC, 1,
                              Matrix<T, MATRIX_SIZE_DYNAMIC, 1>>,
      public MatrixDenseBase<T, MATRIX_SIZE_DYNAMIC, 1,
                             Matrix<T, MATRIX_SIZE_DYNAMIC, 1>>
{
 public:
    using ValueType = T;
    using Reference = T&;
    using ConstReference = const T&;
    using Pointer = T*;
    using ConstPointer = const T*;
    using Iterator = Pointer;
    using ConstIterator = ConstPointer;
    using MatrixDenseBase<T, MATRIX_SIZE_DYNAMIC, 1,
                          Matrix<T, MATRIX_SIZE_DYNAMIC, 1>>::CopyFrom;

    Matrix();

    explicit Matrix(size_t rows, ConstReference value = ValueType{});

    template <size_t R, size_t C, typename E>
    Matrix(const MatrixExpression<T, R, C, E>& expression);

    Matrix(const std::initializer_list<T>& lst);

    explicit Matrix(size_t rows, ConstPointer ptr);

    ~Matrix() = default;

    Matrix(const Matrix& other);

    Matrix(Matrix&& other) noexcept;

    Matrix& operator=(const Matrix& other);

    Matrix& operator=(Matrix&& other) noexcept;

    void Fill(const T& val);

    void Fill(const std::function<T(size_t i)>& func);

    void Fill(const std::function<T(size_t i, size_t j)>& func);

    void Swap(Matrix& other);

    void Resize(size_t rows, ConstReference val = ValueType{});

    void AddElement(ConstReference newElem);

    void AddElement(const Matrix& newElems);

    void Clear();

    [[nodiscard]] size_t GetRows() const;

    [[nodiscard]] constexpr size_t GetCols() const;

    [[nodiscard]] Iterator begin();

    [[nodiscard]] ConstIterator begin() const;

    [[nodiscard]] Iterator end();

    [[nodiscard]] ConstIterator end() const;

    [[nodiscard]] Pointer data();

    [[nodiscard]] ConstPointer data() const;

    Reference operator[](size_t i);

    ConstReference operator[](size_t i) const;

 private:
    std::vector<T> m_elements;
};

template <typename T>
using Matrix2x2 = Matrix<T, 2, 2>;

template <typename T>
using Matrix3x3 = Matrix<T, 3, 3>;

template <typename T>
using Matrix4x4 = Matrix<T, 4, 4>;

using Matrix2x2B = Matrix2x2<int8_t>;
using Matrix2x2UB = Matrix2x2<uint8_t>;
using Matrix2x2S = Matrix2x2<int16_t>;
using Matrix2x2US = Matrix2x2<uint16_t>;
using Matrix2x2I = Matrix2x2<int32_t>;
using Matrix2x2UI = Matrix2x2<uint32_t>;
using Matrix2x2L = Matrix2x2<int64_t>;
using Matrix2x2UL = Matrix2x2<uint64_t>;
using Matrix2x2F = Matrix2x2<float>;
using Matrix2x2D = Matrix2x2<double>;
using Matrix2x2Z = Matrix2x2<ssize_t>;
using Matrix2x2UZ = Matrix2x2<size_t>;

using Matrix3x3B = Matrix3x3<int8_t>;
using Matrix3x3UB = Matrix3x3<uint8_t>;
using Matrix3x3S = Matrix3x3<int16_t>;
using Matrix3x3US = Matrix3x3<uint16_t>;
using Matrix3x3I = Matrix3x3<int32_t>;
using Matrix3x3UI = Matrix3x3<uint32_t>;
using Matrix3x3L = Matrix3x3<int64_t>;
using Matrix3x3UL = Matrix3x3<uint64_t>;
using Matrix3x3F = Matrix3x3<float>;
using Matrix3x3D = Matrix3x3<double>;
using Matrix3x3Z = Matrix3x3<ssize_t>;
using Matrix3x3UZ = Matrix3x3<size_t>;

using Matrix4x4B = Matrix4x4<int8_t>;
using Matrix4x4UB = Matrix4x4<uint8_t>;
using Matrix4x4S = Matrix4x4<int16_t>;
using Matrix4x4US = Matrix4x4<uint16_t>;
using Matrix4x4I = Matrix4x4<int32_t>;
using Matrix4x4UI = Matrix4x4<uint32_t>;
using Matrix4x4L = Matrix4x4<int64_t>;
using Matrix4x4UL = Matrix4x4<uint64_t>;
using Matrix4x4F = Matrix4x4<float>;
using Matrix4x4D = Matrix4x4<double>;
using Matrix4x4Z = Matrix4x4<ssize_t>;
using Matrix4x4UZ = Matrix4x4<size_t>;

template <typename T, size_t Rows>
using Vector = Matrix<T, Rows, 1>;

template <typename T>
using Vector1 = Vector<T, 1>;

template <typename T>
using Vector2 = Vector<T, 2>;

template <typename T>
using Vector3 = Vector<T, 3>;

template <typename T>
using Vector4 = Vector<T, 4>;

using Vector1B = Vector1<int8_t>;
using Vector1UB = Vector1<uint8_t>;
using Vector1S = Vector1<int16_t>;
using Vector1US = Vector1<uint16_t>;
using Vector1I = Vector1<int32_t>;
using Vector1UI = Vector1<uint32_t>;
using Vector1L = Vector1<int64_t>;
using Vector1UL = Vector1<uint64_t>;
using Vector1F = Vector1<float>;
using Vector1D = Vector1<double>;
using Vector1Z = Vector1<ssize_t>;
using Vector1UZ = Vector1<size_t>;

using Vector2B = Vector2<int8_t>;
using Vector2UB = Vector2<uint8_t>;
using Vector2S = Vector2<int16_t>;
using Vector2US = Vector2<uint16_t>;
using Vector2I = Vector2<int32_t>;
using Vector2UI = Vector2<uint32_t>;
using Vector2L = Vector2<int64_t>;
using Vector2UL = Vector2<uint64_t>;
using Vector2F = Vector2<float>;
using Vector2D = Vector2<double>;
using Vector2Z = Vector2<ssize_t>;
using Vector2UZ = Vector2<size_t>;

using Vector3B = Vector3<int8_t>;
using Vector3UB = Vector3<uint8_t>;
using Vector3S = Vector3<int16_t>;
using Vector3US = Vector3<uint16_t>;
using Vector3I = Vector3<int32_t>;
using Vector3UI = Vector3<uint32_t>;
using Vector3L = Vector3<int64_t>;
using Vector3UL = Vector3<uint64_t>;
using Vector3F = Vector3<float>;
using Vector3D = Vector3<double>;
using Vector3Z = Vector3<ssize_t>;
using Vector3UZ = Vector3<size_t>;

using Vector4B = Vector4<int8_t>;
using Vector4UB = Vector4<uint8_t>;
using Vector4S = Vector4<int16_t>;
using Vector4US = Vector4<uint16_t>;
using Vector4I = Vector4<int32_t>;
using Vector4UI = Vector4<uint32_t>;
using Vector4L = Vector4<int64_t>;
using Vector4UL = Vector4<uint64_t>;
using Vector4F = Vector4<float>;
using Vector4D = Vector4<double>;
using Vector4Z = Vector4<ssize_t>;
using Vector4UZ = Vector4<size_t>;

template <typename T>
using MatrixMxN = Matrix<T, MATRIX_SIZE_DYNAMIC, MATRIX_SIZE_DYNAMIC>;

using MatrixMxNB = MatrixMxN<int8_t>;
using MatrixMxNUB = MatrixMxN<uint8_t>;
using MatrixMxNS = MatrixMxN<int16_t>;
using MatrixMxNUS = MatrixMxN<uint16_t>;
using MatrixMxNI = MatrixMxN<int32_t>;
using MatrixMxNUI = MatrixMxN<uint32_t>;
using MatrixMxNL = MatrixMxN<int64_t>;
using MatrixMxNUL = MatrixMxN<uint64_t>;
using MatrixMxNF = MatrixMxN<float>;
using MatrixMxND = MatrixMxN<double>;
using MatrixMxNZ = MatrixMxN<ssize_t>;
using MatrixMxNUZ = MatrixMxN<size_t>;

template <typename T>
using VectorN = Matrix<T, MATRIX_SIZE_DYNAMIC, 1>;

using VectorNB = VectorN<int8_t>;
using VectorNUB = VectorN<uint8_t>;
using VectorNS = VectorN<int16_t>;
using VectorNUS = VectorN<uint16_t>;
using VectorNI = VectorN<int32_t>;
using VectorNUI = VectorN<uint32_t>;
using VectorNL = VectorN<int64_t>;
using VectorNUL = VectorN<uint64_t>;
using VectorNF = VectorN<float>;
using VectorND = VectorN<double>;
using VectorNZ = VectorN<ssize_t>;
using VectorNUZ = VectorN<size_t>;

template <typename T, size_t Rows, size_t Cols>
struct GetScalarType<Matrix<T, Rows, Cols>>
{
    using value = T;
};

template <typename T, size_t R1, size_t C1, size_t R2, size_t C2, typename M2>
void operator+=(Matrix<T, R1, C1>& a, const MatrixExpression<T, R2, C2, M2>& b);

template <typename T, size_t Rows, size_t Cols>
void operator+=(Matrix<T, Rows, Cols>& a, const T& b);

template <typename T, size_t R1, size_t C1, size_t R2, size_t C2, typename M2>
void operator-=(Matrix<T, R1, C1>& a, const MatrixExpression<T, R2, C2, M2>& b);

template <typename T, size_t Rows, size_t Cols>
void operator-=(Matrix<T, Rows, Cols>& a, const T& b);

template <typename T, size_t R1, size_t C1, size_t R2, size_t C2, typename M2>
void operator*=(Matrix<T, R1, C1>& a, const MatrixExpression<T, R2, C2, M2>& b);

template <typename T, size_t R1, size_t C1, size_t R2, size_t C2, typename M2>
void ElemIMul(Matrix<T, R1, C1>& a, const MatrixExpression<T, R2, C2, M2>& b);

template <typename T, size_t Rows, size_t Cols>
void operator*=(Matrix<T, Rows, Cols>& a, const T& b);

template <typename T, size_t R1, size_t C1, size_t R2, size_t C2, typename M2>
void ElemIDiv(Matrix<T, R1, C1>& a, const MatrixExpression<T, R2, C2, M2>& b);

template <typename T, size_t Rows, size_t Cols>
void operator/=(Matrix<T, Rows, Cols>& a, const T& b);

template <typename T, size_t Rows, size_t Cols, typename M1, typename M2>
constexpr std::enable_if_t<IsMatrixSizeStatic<Rows, Cols>(), bool> operator==(
    const MatrixExpression<T, Rows, Cols, M1>& a,
    const MatrixExpression<T, Rows, Cols, M2>& b);

template <typename T, size_t R1, size_t C1, size_t R2, size_t C2, typename M1,
          typename M2>
bool operator==(const MatrixExpression<T, R1, C1, M1>& a,
                const MatrixExpression<T, R2, C2, M2>& b);

template <typename T, size_t R1, size_t C1, size_t R2, size_t C2, typename M1,
          typename M2>
bool operator!=(const MatrixExpression<T, R1, C1, M1>& a,
                const MatrixExpression<T, R2, C2, M2>& b);

template <typename T, size_t Rows, size_t Cols, typename M1,
          typename BinaryOperation>
constexpr std::enable_if_t<TraitIsMatrixSizeStatic<Rows, Cols>::value, T>
Accumulate(const MatrixExpression<T, Rows, Cols, M1>& a, const T& init,
           BinaryOperation op);

template <typename T, size_t Rows, size_t Cols, typename M1>
constexpr std::enable_if_t<TraitIsMatrixSizeStatic<Rows, Cols>::value, T>
Accumulate(const MatrixExpression<T, Rows, Cols, M1>& a, const T& init);

template <typename T, size_t Rows, size_t Cols, typename M1>
constexpr std::enable_if_t<TraitIsMatrixSizeStatic<Rows, Cols>::value, T>
Accumulate(const MatrixExpression<T, Rows, Cols, M1>& a);

template <typename T, size_t Rows, size_t Cols, typename M1,
          typename BinaryOperation>
constexpr std::enable_if_t<TraitIsMatrixSizeDynamic<Rows, Cols>::value, T>
Accumulate(const MatrixExpression<T, Rows, Cols, M1>& a, const T& init,
           BinaryOperation op);

template <typename T, size_t Rows, size_t Cols, typename M1>
constexpr std::enable_if_t<TraitIsMatrixSizeDynamic<Rows, Cols>::value, T>
Accumulate(const MatrixExpression<T, Rows, Cols, M1>& a, const T& init);

template <typename T, size_t Rows, size_t Cols, typename M1>
constexpr std::enable_if_t<TraitIsMatrixSizeDynamic<Rows, Cols>::value, T>
Accumulate(const MatrixExpression<T, Rows, Cols, M1>& a);

template <typename T, size_t Rows, size_t Cols, typename M1>
constexpr T Product(const MatrixExpression<T, Rows, Cols, M1>& a,
                    const T& init);

template <typename T, size_t Rows, size_t Cols, typename M1, typename M2,
          typename M3, typename M4>
std::enable_if_t<IsMatrixSizeStatic<Rows, Cols>(), Matrix<T, Rows, Cols>>
MonotonicCatmullRom(const MatrixExpression<T, Rows, Cols, M1>& f0,
                    const MatrixExpression<T, Rows, Cols, M2>& f1,
                    const MatrixExpression<T, Rows, Cols, M3>& f2,
                    const MatrixExpression<T, Rows, Cols, M4>& f3, T f);
}  // namespace CubbyFlow

#include <Core/Matrix/Matrix-Impl.hpp>

#endif
