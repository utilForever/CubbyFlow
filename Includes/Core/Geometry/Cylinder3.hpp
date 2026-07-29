// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_CYLINDER3_HPP
#define CUBBYFLOW_CYLINDER3_HPP

#include <Core/Geometry/Surface.hpp>

namespace CubbyFlow
{
//!
//! \brief 3-D cylinder geometry.
//!
//! This class represents 3-D cylinder geometry which extends Surface3 by
//! overriding surface-related queries. The cylinder is aligned with the y-axis.
//!
class Cylinder3 final : public Surface3
{
 public:
    class Builder;

    //! Constructs a cylinder with default parameters.
    Cylinder3() : Cylinder3(Transform3{})
    {
        // Do nothing
    }

    //! Constructs a cylinder with \p _transform and \p _isNormalFlipped.
    explicit Cylinder3(const Transform3& _transform,
                       bool _isNormalFlipped = false);

    //! Constructs a cylinder with \p _center, \p _radius, \p _height,
    //! \p _transform and \p _isNormalFlipped.
    Cylinder3(Vector3D _center, double _radius, double _height,
              const Transform3& _transform = Transform3{},
              bool _isNormalFlipped = false);

    //! Default copy constructor.
    Cylinder3(const Cylinder3&) = default;

    //! Default move constructor.
    Cylinder3(Cylinder3&&) noexcept = default;

    //! Default virtual destructor.
    ~Cylinder3() override = default;

    //! Default copy assignment operator.
    Cylinder3& operator=(const Cylinder3&) = default;

    //! Default move assignment operator.
    Cylinder3& operator=(Cylinder3&&) noexcept = default;

    //! Returns builder fox Cylinder3.
    [[nodiscard]] static Builder GetBuilder();

    //! Center of the cylinder.
    Vector3D center;

    //! Radius of the cylinder.
    double radius = 1.0;

    //! Height of the cylinder.
    double height = 1.0;

 protected:
    [[nodiscard]] Vector3D ClosestPointLocal(
        const Vector3D& otherPoint) const override;

    [[nodiscard]] double ClosestDistanceLocal(
        const Vector3D& otherPoint) const override;

    [[nodiscard]] bool IntersectsLocal(const Ray3D& ray) const override;

    [[nodiscard]] BoundingBox3D BoundingBoxLocal() const override;

    [[nodiscard]] Vector3D ClosestNormalLocal(
        const Vector3D& otherPoint) const override;

    [[nodiscard]] SurfaceRayIntersection3 ClosestIntersectionLocal(
        const Ray3D& ray) const override;
};

//! Shared pointer type for the Cylinder3.
using Cylinder3Ptr = std::shared_ptr<Cylinder3>;

//!
//! \brief Front-end to create Cylinder3 objects step by step.
//!
class Cylinder3::Builder final : public SurfaceBuilderBase3<Builder>
{
 public:
    //! Returns builder with center.
    [[nodiscard]] Builder& WithCenter(const Vector3D& _center);

    //! Returns builder with radius.
    [[nodiscard]] Builder& WithRadius(double _radius);

    //! Returns builder with height.
    [[nodiscard]] Builder& WithHeight(double _height);

    //! Builds Cylinder3.
    [[nodiscard]] Cylinder3 Build() const;

    //! Builds shared pointer of Cylinder3 instance.
    [[nodiscard]] Cylinder3Ptr MakeShared() const;

 private:
    Vector3D m_center{ 0, 0, 0 };
    double m_radius = 1.0;
    double m_height = 1.0;
};
}  // namespace CubbyFlow

#endif
