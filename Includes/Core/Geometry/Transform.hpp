// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_TRANSFORM_HPP
#define CUBBYFLOW_TRANSFORM_HPP

#include <Core/Geometry/BoundingBox.hpp>
#include <Core/Geometry/Ray.hpp>
#include <Core/Math/Quaternion.hpp>
#include <Core/Matrix/Matrix.hpp>

namespace CubbyFlow
{
template <size_t N>
class Orientation
{
    // Do nothing
};

template <>
class Orientation<2>
{
 public:
    Orientation();

    Orientation(double angleInRadian);

    [[nodiscard]] double GetRotation() const;

    void SetRotation(double angleInRadian);

    //! Rotates a point in world coordinate to the local frame.
    [[nodiscard]] Vector2D ToLocal(const Vector2D& pointInWorld) const;

    //! Rotates a point in local space to the world coordinate.
    [[nodiscard]] Vector2D ToWorld(const Vector2D& pointInLocal) const;

 private:
    double m_angle = 0.0;
    double m_cosAngle = 1.0;
    double m_sinAngle = 0.0;
};

template <>
class Orientation<3>
{
 public:
    Orientation();

    Orientation(const QuaternionD& quat);

    [[nodiscard]] const QuaternionD& GetRotation() const;

    void SetRotation(const QuaternionD& quat);

    //! Rotates a point in world coordinate to the local frame.
    [[nodiscard]] Vector3D ToLocal(const Vector3D& pointInWorld) const;

    //! Rotates a point in local space to the world coordinate.
    [[nodiscard]] Vector3D ToWorld(const Vector3D& pointInLocal) const;

 private:
    QuaternionD m_quat;
    Matrix3x3D m_rotationMat3 = Matrix3x3D::MakeIdentity();
    Matrix3x3D m_inverseRotationMat3 = Matrix3x3D::MakeIdentity();
};

using Orientation2 = Orientation<2>;
using Orientation3 = Orientation<3>;

//!
//! \brief Represents N-D rigid body transform.
//!
template <size_t N>
class Transform
{
 public:
    //! Constructs identity transform.
    Transform() = default;

    //! Constructs a transform with translation and orientation.
    Transform(const Vector<double, N>& translation,
              const Orientation<N>& orientation);

    //! Returns the translation.
    [[nodiscard]] const Vector<double, N>& GetTranslation() const;

    //! Sets the translation.
    void SetTranslation(const Vector<double, N>& translation);

    //! Returns the orientation.
    [[nodiscard]] const Orientation<N>& GetOrientation() const;

    //! Sets the orientation.
    void SetOrientation(const Orientation<N>& orientation);

    //! Transforms a point in world coordinate to the local frame.
    [[nodiscard]] Vector<double, N> ToLocal(
        const Vector<double, N>& pointInWorld) const;

    //! Transforms a direction in world coordinate to the local frame.
    [[nodiscard]] Vector<double, N> ToLocalDirection(
        const Vector<double, N>& dirInWorld) const;

    //! Transforms a ray in world coordinate to the local frame.
    [[nodiscard]] Ray<double, N> ToLocal(
        const Ray<double, N>& rayInWorld) const;

    //! Transforms a bounding box in world coordinate to the local frame.
    [[nodiscard]] BoundingBox<double, N> ToLocal(
        const BoundingBox<double, N>& bboxInWorld) const;

    //! Transforms a point in local space to the world coordinate.
    [[nodiscard]] Vector<double, N> ToWorld(
        const Vector<double, N>& pointInLocal) const;

    //! Transforms a direction in local space to the world coordinate.
    [[nodiscard]] Vector<double, N> ToWorldDirection(
        const Vector<double, N>& dirInLocal) const;

    //! Transforms a ray in local space to the world coordinate.
    [[nodiscard]] Ray<double, N> ToWorld(
        const Ray<double, N>& rayInLocal) const;

    //! Transforms a bounding box in local space to the world coordinate.
    [[nodiscard]] BoundingBox<double, N> ToWorld(
        const BoundingBox<double, N>& bboxInLocal) const;

 private:
    Vector<double, N> m_translation;
    Orientation<N> m_orientation;
};

using Transform2 = Transform<2>;
using Transform3 = Transform<3>;
}  // namespace CubbyFlow

#endif
