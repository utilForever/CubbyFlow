// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Geometry/Triangle3.hpp>

#include <utility>

namespace CubbyFlow
{
inline Vector3D ClosestPointOnLine(const Vector3D& v0, const Vector3D& v1,
                                   const Vector3D& pt)
{
    const double lenSquared = (v1 - v0).LengthSquared();
    if (lenSquared < std::numeric_limits<double>::epsilon())
    {
        return v0;
    }

    const double t = (pt - v0).Dot(v1 - v0) / lenSquared;
    if (t < 0.0)
    {
        return v0;
    }
    if (t > 1.0)
    {
        return v1;
    }

    return v0 + t * (v1 - v0);
}

inline Vector3D ClosestNormalOnLine(const Vector3D& v0, const Vector3D& v1,
                                    const Vector3D& n0, const Vector3D& n1,
                                    const Vector3D& pt)
{
    const double lenSquared = (v1 - v0).LengthSquared();
    if (lenSquared < std::numeric_limits<double>::epsilon())
    {
        return n0;
    }

    const double t = (pt - v0).Dot(v1 - v0) / lenSquared;
    if (t < 0.0)
    {
        return n0;
    }
    if (t > 1.0)
    {
        return n1;
    }

    return (n0 + t * (n1 - n0)).Normalized();
}

Triangle3::Triangle3(const Transform3& _transform, bool _isNormalFlipped)
    : Surface3{ _transform, _isNormalFlipped }
{
    // Do nothing
}

Triangle3::Triangle3(std::array<Vector3D, 3> newPoints,
                     std::array<Vector3D, 3> newNormals,
                     std::array<Vector2D, 3> newUvs,
                     const Transform3& _transform, bool _isNormalFlipped)
    : Surface3{ _transform, _isNormalFlipped },
      points(std::move(newPoints)),
      normals(std::move(newNormals)),
      uvs(std::move(newUvs))
{
    // Do nothing
}

Vector3D Triangle3::ClosestPointLocal(const Vector3D& otherPoint) const
{
    const Vector3D n = FaceNormal();
    const double nd = n.Dot(n);
    const double d = n.Dot(points[0]);
    const double t = (d - n.Dot(otherPoint)) / nd;

    const Vector3D q = t * n + otherPoint;

    const Vector3D q01 = (points[1] - points[0]).Cross(q - points[0]);
    if (n.Dot(q01) < 0)
    {
        return ClosestPointOnLine(points[0], points[1], q);
    }

    const Vector3D q12 = (points[2] - points[1]).Cross(q - points[1]);
    if (n.Dot(q12) < 0)
    {
        return ClosestPointOnLine(points[1], points[2], q);
    }

    const Vector3D q02 = (points[0] - points[2]).Cross(q - points[2]);
    if (n.Dot(q02) < 0)
    {
        return ClosestPointOnLine(points[0], points[2], q);
    }

    const double a = Area();
    const double b0 = 0.5 * q12.Length() / a;
    const double b1 = 0.5 * q02.Length() / a;
    const double b2 = 0.5 * q01.Length() / a;

    return b0 * points[0] + b1 * points[1] + b2 * points[2];
}

Vector3D Triangle3::ClosestNormalLocal(const Vector3D& otherPoint) const
{
    const Vector3D n = FaceNormal();
    const double nd = n.Dot(n);
    const double d = n.Dot(points[0]);
    const double t = (d - n.Dot(otherPoint)) / nd;

    const Vector3D q = t * n + otherPoint;

    const Vector3D q01 = (points[1] - points[0]).Cross(q - points[0]);
    if (n.Dot(q01) < 0)
    {
        return ClosestNormalOnLine(points[0], points[1], normals[0], normals[1],
                                   q);
    }

    const Vector3D q12 = (points[2] - points[1]).Cross(q - points[1]);
    if (n.Dot(q12) < 0)
    {
        return ClosestNormalOnLine(points[1], points[2], normals[1], normals[2],
                                   q);
    }

    const Vector3D q02 = (points[0] - points[2]).Cross(q - points[2]);
    if (n.Dot(q02) < 0)
    {
        return ClosestNormalOnLine(points[0], points[2], normals[0], normals[2],
                                   q);
    }

    const double a = Area();
    const double b0 = 0.5 * q12.Length() / a;
    const double b1 = 0.5 * q02.Length() / a;
    const double b2 = 0.5 * q01.Length() / a;

    return (b0 * normals[0] + b1 * normals[1] + b2 * normals[2]).Normalized();
}

bool Triangle3::IntersectsLocal(const Ray3D& ray) const
{
    const Vector3D n = FaceNormal();
    const double nd = n.Dot(ray.direction);

    if (nd < std::numeric_limits<double>::epsilon())
    {
        return false;
    }

    const double d = n.Dot(points[0]);
    const double t = (d - n.Dot(ray.origin)) / nd;

    if (t < 0.0)
    {
        return false;
    }

    const Vector3D q = ray.PointAt(t);

    const Vector3D q01 = (points[1] - points[0]).Cross(q - points[0]);
    if (n.Dot(q01) <= 0.0)
    {
        return false;
    }

    const Vector3D q12 = (points[2] - points[1]).Cross(q - points[1]);
    if (n.Dot(q12) <= 0.0)
    {
        return false;
    }

    const Vector3D q02 = (points[0] - points[2]).Cross(q - points[2]);
    if (n.Dot(q02) <= 0.0)
    {
        return false;
    }

    return true;
}

SurfaceRayIntersection3 Triangle3::ClosestIntersectionLocal(
    const Ray3D& ray) const
{
    SurfaceRayIntersection3 intersection;
    const Vector3D n = FaceNormal();
    const double nd = n.Dot(ray.direction);

    if (nd < std::numeric_limits<double>::epsilon())
    {
        intersection.isIntersecting = false;
        return intersection;
    }

    const double d = n.Dot(points[0]);
    const double t = (d - n.Dot(ray.origin)) / nd;

    if (t < 0.0)
    {
        intersection.isIntersecting = false;
        return intersection;
    }

    const Vector3D q = ray.PointAt(t);

    const Vector3D q01 = (points[1] - points[0]).Cross(q - points[0]);
    if (n.Dot(q01) <= 0.0)
    {
        intersection.isIntersecting = false;
        return intersection;
    }

    const Vector3D q12 = (points[2] - points[1]).Cross(q - points[1]);
    if (n.Dot(q12) <= 0.0)
    {
        intersection.isIntersecting = false;
        return intersection;
    }

    const Vector3D q02 = (points[0] - points[2]).Cross(q - points[2]);
    if (n.Dot(q02) <= 0.0)
    {
        intersection.isIntersecting = false;
        return intersection;
    }

    const double a = Area();
    const double b0 = 0.5 * q12.Length() / a;
    const double b1 = 0.5 * q02.Length() / a;
    const double b2 = 0.5 * q01.Length() / a;

    const Vector3D normal = b0 * normals[0] + b1 * normals[1] + b2 * normals[2];

    intersection.isIntersecting = true;
    intersection.distance = t;
    intersection.point = q;
    intersection.normal = normal.Normalized();

    return intersection;
}

BoundingBox3D Triangle3::BoundingBoxLocal() const
{
    BoundingBox3D box{ points[0], points[1] };
    box.Merge(points[2]);
    return box;
}

double Triangle3::Area() const
{
    return 0.5 * (points[1] - points[0]).Cross(points[2] - points[0]).Length();
}

void Triangle3::GetBarycentricCoords(const Vector3D& pt, double* b0, double* b1,
                                     double* b2) const
{
    const Vector3D q01 = (points[1] - points[0]).Cross(pt - points[0]);
    const Vector3D q12 = (points[2] - points[1]).Cross(pt - points[1]);
    const Vector3D q02 = (points[0] - points[2]).Cross(pt - points[2]);

    const double a = Area();
    *b0 = 0.5 * q12.Length() / a;
    *b1 = 0.5 * q02.Length() / a;
    *b2 = 0.5 * q01.Length() / a;
}

Vector3D Triangle3::FaceNormal() const
{
    const Vector3D ret = (points[1] - points[0]).Cross(points[2] - points[0]);
    return ret.Normalized();
}

void Triangle3::SetNormalsToFaceNormal()
{
    normals[0] = normals[1] = normals[2] = FaceNormal();
}

Triangle3::Builder Triangle3::GetBuilder()
{
    return Builder{};
}

Triangle3::Builder& Triangle3::Builder::WithPoints(
    const std::array<Vector3D, 3>& _points)
{
    m_points = _points;
    return *this;
}

Triangle3::Builder& Triangle3::Builder::WithNormals(
    const std::array<Vector3D, 3>& _normals)
{
    m_normals = _normals;
    return *this;
}

Triangle3::Builder& Triangle3::Builder::WithUVs(
    const std::array<Vector2D, 3>& _uvs)
{
    m_uvs = _uvs;
    return *this;
}

Triangle3 Triangle3::Builder::Build() const
{
    return Triangle3{ m_points, m_normals, m_uvs, m_transform,
                      m_isNormalFlipped };
}

Triangle3Ptr Triangle3::Builder::MakeShared() const
{
    return std::make_shared<Triangle3>(m_points, m_normals, m_uvs, m_transform,
                                       m_isNormalFlipped);
}
}  // namespace CubbyFlow