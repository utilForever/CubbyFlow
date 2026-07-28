// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_TRIANGLE3_HPP
#define CUBBYFLOW_TRIANGLE3_HPP

#include <Core/Geometry/Surface.hpp>

namespace CubbyFlow
{
//!
//! \brief 3-D triangle geometry.
//!
//! This class represents 3-D triangle geometry which extends Surface3 by
//! overriding surface-related queries.
//!
class Triangle3 final : public Surface3
{
 public:
    class Builder;

    //! Constructs an empty triangle.
    explicit Triangle3(const Transform3& _transform = Transform3{},
                       bool _isNormalFlipped = false);

    //! Constructs a triangle with given \p points, \p normals, and \p uvs.
    Triangle3(std::array<Vector3D, 3> points, std::array<Vector3D, 3> normals,
              std::array<Vector2D, 3> uvs,
              const Transform3& _transform = Transform3{},
              bool _isNormalFlipped = false);

    //! Default copy constructor.
    Triangle3(const Triangle3&) = default;

    //! Default move constructor.
    Triangle3(Triangle3&&) noexcept = default;

    //! Default virtual destructor.
    ~Triangle3() override = default;

    //! Default copy assignment operator.
    Triangle3& operator=(const Triangle3&) = default;

    //! Default move assignment operator.
    Triangle3& operator=(Triangle3&&) noexcept = default;

    //! Returns the area of this triangle.
    [[nodiscard]] double Area() const;

    //! Returns barycentric coordinates for the given point \p pt.
    void GetBarycentricCoords(const Vector3D& pt, double* b0, double* b1,
                              double* b2) const;

    //! Returns the face normal of the triangle.
    [[nodiscard]] Vector3D FaceNormal() const;

    //! Set Triangle3::normals to the face normal.
    void SetNormalsToFaceNormal();

    //! Returns builder fox Triangle3.
    [[nodiscard]] static Builder GetBuilder();

    //! Three points.
    std::array<Vector3D, 3> points;

    //! Three normals.
    std::array<Vector3D, 3> normals;

    //! Three UV coordinates.
    std::array<Vector2D, 3> uvs;

 protected:
    [[nodiscard]] Vector3D ClosestPointLocal(
        const Vector3D& otherPoint) const override;

    [[nodiscard]] bool IntersectsLocal(const Ray3D& ray) const override;

    [[nodiscard]] BoundingBox3D BoundingBoxLocal() const override;

    [[nodiscard]] Vector3D ClosestNormalLocal(
        const Vector3D& otherPoint) const override;

    [[nodiscard]] SurfaceRayIntersection3 ClosestIntersectionLocal(
        const Ray3D& ray) const override;
};

//! Shared pointer for the Triangle3 type.
using Triangle3Ptr = std::shared_ptr<Triangle3>;

//!
//! \brief Front-end to create Triangle3 objects step by step.
//!
class Triangle3::Builder final : public SurfaceBuilderBase3<Builder>
{
 public:
    //! Returns builder with points.
    [[nodiscard]] Builder& WithPoints(const std::array<Vector3D, 3>& _points);

    //! Returns builder with normals.
    [[nodiscard]] Builder& WithNormals(const std::array<Vector3D, 3>& _normals);

    //! Returns builder with uvs.
    [[nodiscard]] Builder& WithUVs(const std::array<Vector2D, 3>& _uvs);

    //! Builds Triangle3.
    [[nodiscard]] Triangle3 Build() const;

    //! Builds shared pointer of Triangle3 instance.
    [[nodiscard]] Triangle3Ptr MakeShared() const;

 private:
    std::array<Vector3D, 3> m_points;
    std::array<Vector3D, 3> m_normals;
    std::array<Vector2D, 3> m_uvs;
};
}  // namespace CubbyFlow

#endif