// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_BOX_HPP
#define CUBBYFLOW_BOX_HPP

#include <Core/Geometry/Surface.hpp>

namespace CubbyFlow
{
//!
//! \brief N-D box geometry.
//!
//! This class represents N-D box geometry which extends Surface class by
//! overriding surface-related queries. This box implementation is an
//! axis-aligned box that wraps lower-level primitive type, BoundingBox.
//!
template <size_t N>
class Box final : public Surface<N>
{
 public:
    class Builder;

    //! Bounding box of this box.
    BoundingBox<double, N> bound = BoundingBox<double, N>(
        Vector<double, N>{}, Vector<double, N>::MakeConstant(1.0));

    //! Constructs (0, 0, ...) x (1, 1, ...) box.
    Box() : Box(Transform<N>{})
    {
        // Do nothing
    }

    //! Constructs (0, 0, ...) x (1, 1, ...) box with a transform.
    explicit Box(const Transform<N>& _transform, bool _isNormalFlipped = false);

    //! Constructs a box with given \p lowerCorner and \p upperCorner.
    Box(const Vector<double, N>& lowerCorner,
        const Vector<double, N>& upperCorner,
        const Transform<N>& _transform = Transform<N>{},
        bool _isNormalFlipped = false);

    //! Constructs a box with BoundingBox instance.
    explicit Box(const BoundingBox<double, N>& boundingBox,
                 const Transform<N>& _transform = Transform<N>{},
                 bool _isNormalFlipped = false);

    //! Default virtual destructor.
    ~Box() override = default;

    //! Copy constructor.
    Box(const Box& other);

    //! Move constructor.
    Box(Box&& other) noexcept;

    //! Copy assignment operator.
    Box& operator=(const Box& other);

    //! Move assignment operator.
    Box& operator=(Box&& other) noexcept;

    //! Returns builder fox Box.
    static Builder GetBuilder();

 protected:
    [[nodiscard]] Vector<double, N> ClosestPointLocal(
        const Vector<double, N>& otherPoint) const override;

    [[nodiscard]] bool IntersectsLocal(
        const Ray<double, N>& ray) const override;

    [[nodiscard]] BoundingBox<double, N> BoundingBoxLocal() const override;

    [[nodiscard]] Vector<double, N> ClosestNormalLocal(
        const Vector<double, N>& otherPoint) const override;

    [[nodiscard]] SurfaceRayIntersection<N> ClosestIntersectionLocal(
        const Ray<double, N>& ray) const override;
};

//! 2-D Box type.
using Box2 = Box<2>;

//! 3-D Box type.
using Box3 = Box<3>;

//! Shared pointer type for the Box2.
using Box2Ptr = std::shared_ptr<Box2>;

//! Shared pointer type for the Box3.
using Box3Ptr = std::shared_ptr<Box3>;

//!
//! \brief Front-end to create Box objects step by step.
//!
template <size_t N>
class Box<N>::Builder final
    : public SurfaceBuilderBase<N, typename Box<N>::Builder>
{
 public:
    //! Returns builder with lower corner set.
    Builder& WithLowerCorner(const Vector<double, N>& pt);

    //! Returns builder with upper corner set.
    Builder& WithUpperCorner(const Vector<double, N>& pt);

    //! Returns builder with bounding box.
    Builder& WithBoundingBox(const BoundingBox<double, N>& bbox);

    //! Builds Box.
    Box Build() const;

    //! Builds shared pointer of Box instance.
    std::shared_ptr<Box<N>> MakeShared() const;

 private:
    using Base = SurfaceBuilderBase<N, typename Box<N>::Builder>;
    using Base::m_isNormalFlipped;
    using Base::m_transform;

    Vector<double, N> m_lowerCorner;
    Vector<double, N> m_upperCorner = Vector<double, N>::MakeConstant(1.0);
};
}  // namespace CubbyFlow

#endif
