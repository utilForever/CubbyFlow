// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_QUATERNION_IMPL_HPP
#define CUBBYFLOW_QUATERNION_IMPL_HPP

namespace CubbyFlow
{
template <typename T>
Quaternion<T>::Quaternion()
{
    SetIdentity();
}

template <typename T>
Quaternion<T>::Quaternion(T newW, T newX, T newY, T newZ)
{
    Set(newW, newX, newY, newZ);
}

template <typename T>
Quaternion<T>::Quaternion(const std::initializer_list<T>& list)
{
    Set(list);
}

template <typename T>
Quaternion<T>::Quaternion(const Vector3<T>& axis, T angle)
{
    Set(axis, angle);
}

template <typename T>
Quaternion<T>::Quaternion(const Vector3<T>& from, const Vector3<T>& to)
{
    Set(from, to);
}

template <typename T>
Quaternion<T>::Quaternion(const Vector3<T>& axis0, const Vector3<T>& axis1,
                          const Vector3<T>& axis2)
{
    Set(axis0, axis1, axis2);
}

template <typename T>
Quaternion<T>::Quaternion(const Matrix3x3<T>& m33)
{
    Set(m33);
}

template <typename T>
Quaternion<T>::Quaternion(const Quaternion& other) = default;

template <typename T>
Quaternion<T>::Quaternion(Quaternion&& other) noexcept = default;

template <typename T>
Quaternion<T>& Quaternion<T>::operator=(const Quaternion& other) = default;

template <typename T>
Quaternion<T>& Quaternion<T>::operator=(Quaternion&& other) noexcept = default;

template <typename T>
void Quaternion<T>::Set(const Quaternion& other)
{
    Set(other.w, other.x, other.y, other.z);
}

template <typename T>
void Quaternion<T>::Set(T newW, T newX, T newY, T newZ)
{
    w = newW;
    x = newX;
    y = newY;
    z = newZ;
}

template <typename T>
void Quaternion<T>::Set(const std::initializer_list<T>& list)
{
    assert(list.size() == 4);

    auto inputElem = list.begin();
    w = *inputElem;
    x = *(++inputElem);
    y = *(++inputElem);
    z = *(++inputElem);
}

template <typename T>
void Quaternion<T>::Set(const Vector3<T>& axis, T angle)
{
    static const T eps = std::numeric_limits<T>::epsilon();

    T axisLengthSquared = axis.LengthSquared();

    if (axisLengthSquared < eps)
    {
        SetIdentity();
    }
    else
    {
        Vector3<T> normalizedAxis = axis.Normalized();
        T s = std::sin(angle / 2);

        x = normalizedAxis.x * s;
        y = normalizedAxis.y * s;
        z = normalizedAxis.z * s;
        w = std::cos(angle / 2);
    }
}

template <typename T>
void Quaternion<T>::Set(const Vector3<T>& from, const Vector3<T>& to)
{
    static const T eps = std::numeric_limits<T>::epsilon();

    Vector3<T> axis = from.Cross(to);

    T fromLengthSquared = from.LengthSquared();
    T toLengthSquared = to.LengthSquared();

    if (fromLengthSquared < eps || toLengthSquared < eps)
    {
        SetIdentity();
    }
    else
    {
        T axisLengthSquared = axis.LengthSquared();

        // In case two vectors are exactly the opposite, pick orthogonal vector
        // for axis.
        if (axisLengthSquared < eps)
        {
            axis = std::get<0>(from.Tangentials());
        }

        Set(from.Dot(to), axis.x, axis.y, axis.z);
        w += L2Norm();

        Normalize();
    }
}

template <typename T>
void Quaternion<T>::Set(const Vector3<T>& rotationBasis0,
                        const Vector3<T>& rotationBasis1,
                        const Vector3<T>& rotationBasis2)
{
    Matrix3x3<T> matrix3;

    matrix3.SetColumn(0, rotationBasis0.Normalized());
    matrix3.SetColumn(1, rotationBasis1.Normalized());
    matrix3.SetColumn(2, rotationBasis2.Normalized());

    Set(matrix3);
}

template <typename T>
void Quaternion<T>::Set(const Matrix3x3<T>& m)
{
    static const T eps = std::numeric_limits<T>::epsilon();
    static const T quarter = static_cast<T>(0.25);

    T onePlusTrace = m.Trace() + 1;

    if (onePlusTrace > eps)
    {
        T S = std::sqrt(onePlusTrace) * 2;
        w = quarter * S;
        x = (m(2, 1) - m(1, 2)) / S;
        y = (m(0, 2) - m(2, 0)) / S;
        z = (m(1, 0) - m(0, 1)) / S;
    }
    else if (m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2))
    {
        T S = std::sqrt(1 + m(0, 0) - m(1, 1) - m(2, 2)) * 2;
        w = (m(2, 1) - m(1, 2)) / S;
        x = quarter * S;
        y = (m(0, 1) + m(1, 0)) / S;
        z = (m(0, 2) + m(2, 0)) / S;
    }
    else if (m(1, 1) > m(2, 2))
    {
        T S = std::sqrt(1 + m(1, 1) - m(0, 0) - m(2, 2)) * 2;
        w = (m(0, 2) - m(2, 0)) / S;
        x = (m(0, 1) + m(1, 0)) / S;
        y = quarter * S;
        z = (m(1, 2) + m(2, 1)) / S;
    }
    else
    {
        T S = std::sqrt(1 + m(2, 2) - m(0, 0) - m(1, 1)) * 2;
        w = (m(1, 0) - m(0, 1)) / S;
        x = (m(0, 2) + m(2, 0)) / S;
        y = (m(1, 2) + m(2, 1)) / S;
        z = quarter * S;
    }
}

template <typename T>
template <typename U>
Quaternion<U> Quaternion<T>::CastTo() const
{
    return Quaternion<U>{ static_cast<U>(w), static_cast<U>(x),
                          static_cast<U>(y), static_cast<U>(z) };
}

template <typename T>
Quaternion<T> Quaternion<T>::Normalized() const
{
    Quaternion q{ *this };
    q.Normalize();
    return q;
}

template <typename T>
Vector3<T> Quaternion<T>::Mul(const Vector3<T>& v) const
{
    T _2xx = 2 * x * x;
    T _2yy = 2 * y * y;
    T _2zz = 2 * z * z;
    T _2xy = 2 * x * y;
    T _2xz = 2 * x * z;
    T _2xw = 2 * x * w;
    T _2yz = 2 * y * z;
    T _2yw = 2 * y * w;
    T _2zw = 2 * z * w;

    return Vector3<T>{
        (1 - _2yy - _2zz) * v.x + (_2xy - _2zw) * v.y + (_2xz + _2yw) * v.z,
        (_2xy + _2zw) * v.x + (1 - _2zz - _2xx) * v.y + (_2yz - _2xw) * v.z,
        (_2xz - _2yw) * v.x + (_2yz + _2xw) * v.y + (1 - _2yy - _2xx) * v.z
    };
}

template <typename T>
Quaternion<T> Quaternion<T>::Mul(const Quaternion& other) const
{
    return Quaternion{ w * other.w - x * other.x - y * other.y - z * other.z,
                       w * other.x + x * other.w + y * other.z - z * other.y,
                       w * other.y - x * other.z + y * other.w + z * other.x,
                       w * other.z + x * other.y - y * other.x + z * other.w };
}

template <typename T>
T Quaternion<T>::Dot(const Quaternion<T>& other) const
{
    return w * other.w + x * other.x + y * other.y + z * other.z;
}

template <typename T>
Quaternion<T> Quaternion<T>::RMul(const Quaternion& other) const
{
    return Quaternion{ other.w * w - other.x * x - other.y * y - other.z * z,
                       other.w * x + other.x * w + other.y * z - other.z * y,
                       other.w * y - other.x * z + other.y * w + other.z * x,
                       other.w * z + other.x * y - other.y * x + other.z * w };
}

template <typename T>
void Quaternion<T>::IMul(const Quaternion& other)
{
    *this = Mul(other);
}

template <typename T>
void Quaternion<T>::SetIdentity()
{
    Set(1, 0, 0, 0);
}

template <typename T>
void Quaternion<T>::Rotate(T angleInRadians)
{
    Vector3<T> axis;
    T currentAngle;

    GetAxisAngle(&axis, &currentAngle);

    currentAngle += angleInRadians;

    Set(axis, currentAngle);
}

template <typename T>
void Quaternion<T>::Normalize()
{
    T norm = L2Norm();

    if (norm > 0)
    {
        w /= norm;
        x /= norm;
        y /= norm;
        z /= norm;
    }
}

template <typename T>
Vector3<T> Quaternion<T>::Axis() const
{
    Vector3<T> result{ x, y, z };
    result.Normalize();

    if (2 * std::acos(w) < PI<T>())
    {
        return result;
    }

    return -result;
}

template <typename T>
T Quaternion<T>::Angle() const
{
    T result = 2 * std::acos(w);

    if (result < PI<T>())
    {
        return result;
    }

    // Wrap around
    return 2 * PI<T>() - result;
}

template <typename T>
void Quaternion<T>::GetAxisAngle(Vector3<T>* axis, T* angle) const
{
    *axis = Vector3<T>(x, y, z);
    axis->Normalize();
    *angle = 2 * std::acos(w);

    if (*angle > PI<T>())
    {
        // Wrap around
        (*axis) = -(*axis);
        *angle = 2 * PI<T>() - (*angle);
    }
}

template <typename T>
Quaternion<T> Quaternion<T>::Inverse() const
{
    const T denom = w * w + x * x + y * y + z * z;
    return Quaternion{ w / denom, -x / denom, -y / denom, -z / denom };
}

template <typename T>
Matrix3x3<T> Quaternion<T>::Matrix3() const
{
    T _2xx = 2 * x * x;
    T _2yy = 2 * y * y;
    T _2zz = 2 * z * z;
    T _2xy = 2 * x * y;
    T _2xz = 2 * x * z;
    T _2xw = 2 * x * w;
    T _2yz = 2 * y * z;
    T _2yw = 2 * y * w;
    T _2zw = 2 * z * w;

    Matrix3x3<T> m{ 1 - _2yy - _2zz, _2xy - _2zw,     _2xz + _2yw,
                    _2xy + _2zw,     1 - _2zz - _2xx, _2yz - _2xw,
                    _2xz - _2yw,     _2yz + _2xw,     1 - _2yy - _2xx };

    return m;
}

template <typename T>
Matrix4x4<T> Quaternion<T>::Matrix4() const
{
    T _2xx = 2 * x * x;
    T _2yy = 2 * y * y;
    T _2zz = 2 * z * z;
    T _2xy = 2 * x * y;
    T _2xz = 2 * x * z;
    T _2xw = 2 * x * w;
    T _2yz = 2 * y * z;
    T _2yw = 2 * y * w;
    T _2zw = 2 * z * w;

    Matrix4x4<T> m{ 1 - _2yy - _2zz,
                    _2xy - _2zw,
                    _2xz + _2yw,
                    0,
                    _2xy + _2zw,
                    1 - _2zz - _2xx,
                    _2yz - _2xw,
                    0,
                    _2xz - _2yw,
                    _2yz + _2xw,
                    1 - _2yy - _2xx,
                    0,
                    0,
                    0,
                    0,
                    1 };

    return m;
}

template <typename T>
T Quaternion<T>::L2Norm() const
{
    return std::sqrt(w * w + x * x + y * y + z * z);
}

template <typename T>
Quaternion<T>& Quaternion<T>::operator*=(const Quaternion& other)
{
    IMul(other);
    return *this;
}

template <typename T>
T& Quaternion<T>::operator[](size_t i)
{
    assert(i >= 0 && i < 4);

    if (i == 0)
    {
        return w;
    }

    if (i == 1)
    {
        return x;
    }

    if (i == 2)
    {
        return y;
    }

    return z;
}

template <typename T>
const T& Quaternion<T>::operator[](size_t i) const
{
    assert(i >= 0 && i < 4);

    if (i == 0)
    {
        return w;
    }

    if (i == 1)
    {
        return x;
    }

    if (i == 2)
    {
        return y;
    }

    return z;
}

template <typename T>
bool Quaternion<T>::operator==(const Quaternion& other) const
{
    return (w == other.w && x == other.x && y == other.y && z == other.z);
}

template <typename T>
bool Quaternion<T>::operator!=(const Quaternion& other) const
{
    return (w != other.w || x != other.x || y != other.y || z != other.z);
}

template <typename T>
Quaternion<T> Quaternion<T>::MakeIdentity()
{
    return Quaternion{};
}

template <typename T>
Quaternion<T> Slerp(const Quaternion<T>& a, const Quaternion<T>& b, T t)
{
    static const double threshold = 0.01;
    static const T eps = std::numeric_limits<T>::epsilon();

    T cosHalfAngle = a.Dot(b);
    T weightA, weightB;

    // For better accuracy, return lerp result when a and b are close enough.
    if (1.0 - std::fabs(cosHalfAngle) < threshold)
    {
        weightA = 1.0 - t;
        weightB = t;
    }
    else
    {
        T halfAngle = std::acos(cosHalfAngle);
        T sinHalfAngle = std::sqrt(1 - cosHalfAngle * cosHalfAngle);

        // In case of angle ~ 180, pick middle value.
        // If not, perform slerp.
        if (std::fabs(sinHalfAngle) < eps)
        {
            weightA = static_cast<T>(0.5);
            weightB = static_cast<T>(0.5);
        }
        else
        {
            weightA = std::sin((1 - t) * halfAngle) / sinHalfAngle;
            weightB = std::sin(t * halfAngle) / sinHalfAngle;
        }
    }

    return Quaternion<T>{ weightA * a.w + weightB * b.w,
                          weightA * a.x + weightB * b.x,
                          weightA * a.y + weightB * b.y,
                          weightA * a.z + weightB * b.z };
}

template <typename T>
Vector<T, 3> operator*(const Quaternion<T>& q, const Vector<T, 3>& v)
{
    return q.Mul(v);
}

template <typename T>
Quaternion<T> operator*(const Quaternion<T>& a, const Quaternion<T>& b)
{
    return a.Mul(b);
}
}  // namespace CubbyFlow

#endif
