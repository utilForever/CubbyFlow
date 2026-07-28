// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_TRIANGLE_MESH3_HPP
#define CUBBYFLOW_TRIANGLE_MESH3_HPP

#include <Core/Array/Array.hpp>
#include <Core/Geometry/BVH.hpp>
#include <Core/Geometry/Surface.hpp>
#include <Core/Geometry/Triangle3.hpp>

namespace CubbyFlow
{
//!
//! \brief 3-D triangle mesh geometry.
//!
//! This class represents 3-D triangle mesh geometry which extends Surface3 by
//! overriding surface-related queries. The mesh structure stores point,
//! normals, and UV coordinates.
//!
class TriangleMesh3 final : public Surface3
{
 public:
    class Builder;

    using Vector2DArray = Array1<Vector2D>;
    using Vector3DArray = Array1<Vector3D>;
    using IndexArray = Array1<Vector3UZ>;

    using PointArray = Vector3DArray;
    using NormalArray = Vector3DArray;
    using UVArray = Vector2DArray;

    //! Constructs an empty triangle mesh.
    TriangleMesh3() : TriangleMesh3(Transform3{})
    {
        // Do nothing
    }

    //! Constructs an empty triangle mesh with a transform.
    explicit TriangleMesh3(const Transform3& _transform,
                           bool _isNormalFlipped = false);

    //! Constructs mesh with points, normals, uvs, and their indices.
    TriangleMesh3(PointArray points, NormalArray normals, UVArray uvs,
                  IndexArray pointIndices, IndexArray normalIndices,
                  IndexArray uvIndices,
                  const Transform3& _transform = Transform3{},
                  bool _isNormalFlipped = false);

    //! Copy constructor.
    TriangleMesh3(const TriangleMesh3& other);

    //! Deleted move constructor.
    TriangleMesh3(TriangleMesh3&& other) noexcept = delete;

    // Default virtual destructor.
    ~TriangleMesh3() override = default;

    //! Deleted copy assignment operator.
    TriangleMesh3& operator=(const TriangleMesh3& other);

    //! Deleted move assignment operator.
    TriangleMesh3& operator=(TriangleMesh3&& other) noexcept = delete;

    //! Updates internal spatial query engine.
    void UpdateQueryEngine() override;

    //! Updates internal spatial query engine.
    void UpdateQueryEngine() const;

    //! Clears all content.
    void Clear();

    //! Copies the contents from \p other mesh.
    void Set(const TriangleMesh3& other);

    //! Swaps the contents with \p other mesh.
    void Swap(TriangleMesh3& other);

    //! Returns area of this mesh.
    [[nodiscard]] double Area() const;

    //! Returns volume of this mesh.
    [[nodiscard]] double Volume() const;

    //! Returns constant reference to the i-th point.
    [[nodiscard]] const Vector3D& Point(size_t i) const;

    //! Returns reference to the i-th point.
    [[nodiscard]] Vector3D& Point(size_t i);

    //! Returns constant reference to the i-th normal.
    [[nodiscard]] const Vector3D& Normal(size_t i) const;

    //! Returns reference to the i-th normal.
    [[nodiscard]] Vector3D& Normal(size_t i);

    //! Returns constant reference to the i-th UV coordinates.
    [[nodiscard]] const Vector2D& UV(size_t i) const;

    //! Returns reference to the i-th UV coordinates.
    [[nodiscard]] Vector2D& UV(size_t i);

    //! Returns constant reference to the point indices of i-th triangle.
    [[nodiscard]] const Vector3UZ& PointIndex(size_t i) const;

    //! Returns reference to the point indices of i-th triangle.
    [[nodiscard]] Vector3UZ& PointIndex(size_t i);

    //! Returns constant reference to the normal indices of i-th triangle.
    [[nodiscard]] const Vector3UZ& NormalIndex(size_t i) const;

    //! Returns reference to the normal indices of i-th triangle.
    [[nodiscard]] Vector3UZ& NormalIndex(size_t i);

    //! Returns constant reference to the UV indices of i-th triangle.
    [[nodiscard]] const Vector3UZ& UVIndex(size_t i) const;

    //! Returns reference to the UV indices of i-th triangle.
    [[nodiscard]] Vector3UZ& UVIndex(size_t i);

    //! Returns i-th triangle.
    [[nodiscard]] Triangle3 Triangle(size_t i) const;

    //! Returns number of points.
    [[nodiscard]] size_t NumberOfPoints() const;

    //! Returns number of normals.
    [[nodiscard]] size_t NumberOfNormals() const;

    //! Returns number of UV coordinates.
    [[nodiscard]] size_t NumberOfUVs() const;

    //! Returns number of triangles.
    [[nodiscard]] size_t NumberOfTriangles() const;

    //! Returns true if the mesh has normals.
    [[nodiscard]] bool HasNormals() const;

    //! Returns true if the mesh has UV coordinates.
    [[nodiscard]] bool HasUVs() const;

    //! Adds a point.
    void AddPoint(const Vector3D& pt);

    //! Adds a normal.
    void AddNormal(const Vector3D& n);

    //! Adds a UV.
    void AddUV(const Vector2D& t);

    //! Adds a triangle with point.
    void AddPointTriangle(const Vector3UZ& newPointIndices);

    //! Adds a triangle with normal.
    void AddNormalTriangle(const Vector3UZ& newNormalIndices);

    //! Adds a triangle with UV.
    void AddUVTriangle(const Vector3UZ& newUVIndices);

    //! Add a triangle.
    void AddTriangle(const Triangle3& tri);

    //! Sets entire normals to the face normals.
    void SetFaceNormal();

    //! Sets angle weighted vertex normal.
    void SetAngleWeightedVertexNormal();

    //! Scales the mesh by given factor.
    void Scale(double factor);

    //! Translates the mesh.
    void Translate(const Vector3D& t);

    //! Rotates the mesh.
    void Rotate(const QuaternionD& q);

    //! Writes the mesh in obj format to the output stream.
    void WriteObj(std::ostream* stream) const;

    //! Writes the mesh in obj format to the file.
    [[nodiscard]] bool WriteObj(const std::string& fileName) const;

    //! Reads the mesh in obj format from the input stream.
    [[nodiscard]] bool ReadObj(std::istream* stream);

    //! Reads the mesh in obj format from the file.
    [[nodiscard]] bool ReadObj(const std::string& fileName);

    //! Returns builder fox TriangleMesh3.
    [[nodiscard]] static Builder GetBuilder();

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

    [[nodiscard]] bool IsInsideLocal(const Vector3D& otherPoint) const override;

 private:
    void InvalidateCache() const;

    void InvalidateBVH() const;

    void BuildBVH() const;

    void BuildWindingNumbers() const;

    [[nodiscard]] double GetWindingNumber(const Vector3D& queryPoint,
                                          size_t triIndex) const;

    [[nodiscard]] double GetFastWindingNumber(const Vector3D& queryPoint,
                                              double accuracy) const;

    [[nodiscard]] double GetFastWindingNumber(const Vector3D& queryPoint,
                                              size_t rootNodeIndex,
                                              double accuracy) const;

    PointArray m_points;
    NormalArray m_normals;
    UVArray m_uvs;
    IndexArray m_pointIndices;
    IndexArray m_normalIndices;
    IndexArray m_uvIndices;

    mutable BVH3<size_t> m_bvh;
    mutable bool m_bvhInvalidated = true;

    mutable Array1<Vector3D> m_wnAreaWeightedNormalSums;
    mutable Array1<Vector3D> m_wnAreaWeightedAvgPositions;
    mutable bool m_wnInvalidated = true;
};

//! Shared pointer for the TriangleMesh3 type.
using TriangleMesh3Ptr = std::shared_ptr<TriangleMesh3>;

//!
//! \brief Front-end to create TriangleMesh3 objects step by step.
//!
class TriangleMesh3::Builder final : public SurfaceBuilderBase3<Builder>
{
 public:
    //! Returns builder with points.
    [[nodiscard]] Builder& WithPoints(const PointArray& points);

    //! Returns builder with normals.
    [[nodiscard]] Builder& WithNormals(const NormalArray& normals);

    //! Returns builder with uvs.
    [[nodiscard]] Builder& WithUVs(const UVArray& uvs);

    //! Returns builder with point indices.
    [[nodiscard]] Builder& WithPointIndices(const IndexArray& pointIndices);

    //! Returns builder with normal indices.
    [[nodiscard]] Builder& WithNormalIndices(const IndexArray& normalIndices);

    //! Returns builder with uv indices.
    [[nodiscard]] Builder& WithUVIndices(const IndexArray& uvIndices);

    //! Builds TriangleMesh3.
    [[nodiscard]] TriangleMesh3 Build() const;

    //! Builds shared pointer of TriangleMesh3 instance.
    [[nodiscard]] TriangleMesh3Ptr MakeShared() const;

 private:
    PointArray m_points;
    NormalArray m_normals;
    UVArray m_uvs;
    IndexArray m_pointIndices;
    IndexArray m_normalIndices;
    IndexArray m_uvIndices;
};
}  // namespace CubbyFlow

#endif
