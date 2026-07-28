// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Geometry/TriangleMesh3.hpp>
#include <Core/Math/MathUtils.hpp>
#include <Core/Utils/Parallel.hpp>

#include <tiny_obj_loader.h>

#include <array>
#include <cassert>
#include <fstream>
#include <iostream>
#include <utility>

namespace CubbyFlow
{
namespace
{
constexpr double DEFAULT_FAST_WINDING_NUMBER_ACCURACY = 2.0;

struct WindingNumberGatherData
{
    WindingNumberGatherData operator+(
        const WindingNumberGatherData& other) const
    {
        WindingNumberGatherData sum;
        sum.areaSums = areaSums + other.areaSums;
        sum.areaWeightedNormalSums =
            areaWeightedNormalSums + other.areaWeightedNormalSums;
        sum.areaWeightedPositionSums =
            areaWeightedPositionSums + other.areaWeightedPositionSums;

        return sum;
    }

    double areaSums = 0.0;
    Vector3D areaWeightedNormalSums;
    Vector3D areaWeightedPositionSums;
};

template <typename GatherFunc, typename LeafGatherFunc>
WindingNumberGatherData PostOrderTraversal(
    const BVH3<size_t>& bvh, size_t nodeIndex, const GatherFunc& visitorFunc,
    const LeafGatherFunc& leafFunc,
    const WindingNumberGatherData& initGatherData)
{
    WindingNumberGatherData data = initGatherData;

    if (bvh.IsLeaf(nodeIndex))
    {
        data = leafFunc(nodeIndex);
    }
    else
    {
        const auto children = bvh.Children(nodeIndex);
        data = data + PostOrderTraversal(bvh, children.first, visitorFunc,
                                         leafFunc, initGatherData);
        data = data + PostOrderTraversal(bvh, children.second, visitorFunc,
                                         leafFunc, initGatherData);
    }

    visitorFunc(nodeIndex, data);

    return data;
}

std::ostream& operator<<(std::ostream& stream, const Vector2D& v)
{
    stream << v.x << ' ' << v.y;
    return stream;
}

std::ostream& operator<<(std::ostream& stream, const Vector3D& v)
{
    stream << v.x << ' ' << v.y << ' ' << v.z;
    return stream;
}
}  // namespace

TriangleMesh3::TriangleMesh3(const Transform3& _transform,
                             bool _isNormalFlipped)
    : Surface3{ _transform, _isNormalFlipped }
{
    // Do nothing
}

TriangleMesh3::TriangleMesh3(PointArray points, NormalArray normals,
                             UVArray uvs, IndexArray pointIndices,
                             IndexArray normalIndices, IndexArray uvIndices,
                             const Transform3& _transform,
                             bool _isNormalFlipped)
    : Surface3(_transform, _isNormalFlipped),
      m_points(std::move(points)),
      m_normals(std::move(normals)),
      m_uvs(std::move(uvs)),
      m_pointIndices(std::move(pointIndices)),
      m_normalIndices(std::move(normalIndices)),
      m_uvIndices(std::move(uvIndices))
{
    // Do nothing
}

TriangleMesh3::TriangleMesh3(const TriangleMesh3& other) : Surface3{ other }
{
    Set(other);
}

TriangleMesh3& TriangleMesh3::operator=(const TriangleMesh3& other)
{
    Set(other);
    return *this;
}

void TriangleMesh3::UpdateQueryEngine()
{
    BuildBVH();
    BuildWindingNumbers();
}

void TriangleMesh3::UpdateQueryEngine() const
{
    BuildBVH();
    BuildWindingNumbers();
}

void TriangleMesh3::Clear()
{
    m_points.Clear();
    m_normals.Clear();
    m_uvs.Clear();
    m_pointIndices.Clear();
    m_normalIndices.Clear();
    m_uvIndices.Clear();

    InvalidateCache();
}

void TriangleMesh3::Set(const TriangleMesh3& other)
{
    m_points.CopyFrom(other.m_points);
    m_normals.CopyFrom(other.m_normals);
    m_uvs.CopyFrom(other.m_uvs);
    m_pointIndices.CopyFrom(other.m_pointIndices);
    m_normalIndices.CopyFrom(other.m_normalIndices);
    m_uvIndices.CopyFrom(other.m_uvIndices);

    InvalidateCache();
}

void TriangleMesh3::Swap(TriangleMesh3& other)
{
    m_points.Swap(other.m_points);
    m_normals.Swap(other.m_normals);
    m_uvs.Swap(other.m_uvs);
    m_pointIndices.Swap(other.m_pointIndices);
    m_normalIndices.Swap(other.m_normalIndices);
    m_uvIndices.Swap(other.m_uvIndices);
}

double TriangleMesh3::Area() const
{
    double a = 0;

    for (size_t i = 0; i < NumberOfTriangles(); ++i)
    {
        Triangle3 tri = Triangle(i);
        a += tri.Area();
    }

    return a;
}

double TriangleMesh3::Volume() const
{
    double vol = 0;

    for (size_t i = 0; i < NumberOfTriangles(); ++i)
    {
        Triangle3 tri = Triangle(i);
        vol += tri.points[0].Dot(tri.points[1].Cross(tri.points[2])) / 6.0;
    }

    return vol;
}

const Vector3D& TriangleMesh3::Point(size_t i) const
{
    return m_points[i];
}

Vector3D& TriangleMesh3::Point(size_t i)
{
    InvalidateCache();
    return m_points[i];
}

const Vector3D& TriangleMesh3::Normal(size_t i) const
{
    return m_normals[i];
}

Vector3D& TriangleMesh3::Normal(size_t i)
{
    return m_normals[i];
}

const Vector2D& TriangleMesh3::UV(size_t i) const
{
    return m_uvs[i];
}

Vector2D& TriangleMesh3::UV(size_t i)
{
    return m_uvs[i];
}

const Vector3UZ& TriangleMesh3::PointIndex(size_t i) const
{
    return m_pointIndices[i];
}

Vector3UZ& TriangleMesh3::PointIndex(size_t i)
{
    return m_pointIndices[i];
}

const Vector3UZ& TriangleMesh3::NormalIndex(size_t i) const
{
    return m_normalIndices[i];
}

Vector3UZ& TriangleMesh3::NormalIndex(size_t i)
{
    return m_normalIndices[i];
}

const Vector3UZ& TriangleMesh3::UVIndex(size_t i) const
{
    return m_uvIndices[i];
}

Vector3UZ& TriangleMesh3::UVIndex(size_t i)
{
    return m_uvIndices[i];
}

Triangle3 TriangleMesh3::Triangle(size_t i) const
{
    Triangle3 tri;

    for (int j = 0; j < 3; ++j)
    {
        tri.points[j] = m_points[m_pointIndices[i][j]];

        if (HasUVs())
        {
            tri.uvs[j] = m_uvs[m_uvIndices[i][j]];
        }
    }

    const Vector3D n = tri.FaceNormal();

    for (int j = 0; j < 3; ++j)
    {
        tri.normals[j] = HasNormals() ? m_normals[m_normalIndices[i][j]] : n;
    }

    return tri;
}

size_t TriangleMesh3::NumberOfPoints() const
{
    return m_points.Length();
}

size_t TriangleMesh3::NumberOfNormals() const
{
    return m_normals.Length();
}

size_t TriangleMesh3::NumberOfUVs() const
{
    return m_uvs.Length();
}

size_t TriangleMesh3::NumberOfTriangles() const
{
    return m_pointIndices.Length();
}

bool TriangleMesh3::HasNormals() const
{
    return m_normals.Length() > 0;
}

bool TriangleMesh3::HasUVs() const
{
    return m_uvs.Length() > 0;
}

void TriangleMesh3::AddPoint(const Vector3D& pt)
{
    m_points.Append(pt);
}

void TriangleMesh3::AddNormal(const Vector3D& n)
{
    m_normals.Append(n);
}

void TriangleMesh3::AddUV(const Vector2D& t)
{
    m_uvs.Append(t);
}

void TriangleMesh3::AddPointTriangle(const Vector3UZ& newPointIndices)
{
    m_pointIndices.Append(newPointIndices);
    InvalidateCache();
}

void TriangleMesh3::AddNormalTriangle(const Vector3UZ& newNormalIndices)
{
    m_normalIndices.Append(newNormalIndices);
    InvalidateCache();
}

void TriangleMesh3::AddUVTriangle(const Vector3UZ& newUVIndices)
{
    m_uvIndices.Append(newUVIndices);
    InvalidateCache();
}

void TriangleMesh3::AddTriangle(const Triangle3& tri)
{
    const size_t vStart = m_points.Length();
    const size_t nStart = m_normals.Length();
    const size_t tStart = m_uvs.Length();
    Vector3UZ newPointIndices;
    Vector3UZ newNormalIndices;
    Vector3UZ newUvIndices;

    for (size_t i = 0; i < 3; ++i)
    {
        m_points.Append(tri.points[i]);
        m_normals.Append(tri.normals[i]);
        m_uvs.Append(tri.uvs[i]);

        newPointIndices[i] = vStart + i;
        newNormalIndices[i] = nStart + i;
        newUvIndices[i] = tStart + i;
    }

    m_pointIndices.Append(newPointIndices);
    m_normalIndices.Append(newNormalIndices);
    m_uvIndices.Append(newUvIndices);

    InvalidateCache();
}

void TriangleMesh3::SetFaceNormal()
{
    m_normals.Resize(m_points.Length());
    m_normalIndices.CopyFrom(m_pointIndices);

    for (size_t i = 0; i < NumberOfTriangles(); ++i)
    {
        Triangle3 tri = Triangle(i);
        const Vector3D n = tri.FaceNormal();
        const Vector3UZ f = m_pointIndices[i];

        m_normals[f.x] = n;
        m_normals[f.y] = n;
        m_normals[f.z] = n;
    }
}

void TriangleMesh3::SetAngleWeightedVertexNormal()
{
    m_normals.Clear();
    m_normalIndices.Clear();

    Array1<double> angleWeights(m_points.Length());
    Vector3DArray pseudoNormals(m_points.Length());

    for (size_t i = 0; i < m_points.Length(); ++i)
    {
        angleWeights[i] = 0;
        pseudoNormals[i] = Vector3D{};
    }

    for (size_t i = 0; i < NumberOfTriangles(); ++i)
    {
        Vector3D pts[3];
        size_t idx[3];

        // Quick references
        for (int j = 0; j < 3; ++j)
        {
            idx[j] = m_pointIndices[i][j];
            pts[j] = m_points[idx[j]];
        }

        // Angle for point 0
        Vector3D e0 = pts[1] - pts[0];
        Vector3D e1 = pts[2] - pts[0];
        e0.Normalize();
        e1.Normalize();
        Vector3D normal = e0.Cross(e1);
        normal.Normalize();
        double cosAngle = Clamp(e0.Dot(e1), -1.0, 1.0);
        double angle = std::acos(cosAngle);
        angleWeights[idx[0]] += angle;
        pseudoNormals[idx[0]] += angle * normal;

        // Angle for point 1
        e0 = pts[2] - pts[1];
        e1 = pts[0] - pts[1];
        e0.Normalize();
        e1.Normalize();
        normal = e0.Cross(e1);
        normal.Normalize();
        cosAngle = Clamp(e0.Dot(e1), -1.0, 1.0);
        angle = std::acos(cosAngle);
        angleWeights[idx[1]] += angle;
        pseudoNormals[idx[1]] += angle * normal;

        // Angle for point 2
        e0 = pts[0] - pts[2];
        e1 = pts[1] - pts[2];
        e0.Normalize();
        e1.Normalize();
        normal = e0.Cross(e1);
        normal.Normalize();
        cosAngle = Clamp(e0.Dot(e1), -1.0, 1.0);
        angle = std::acos(cosAngle);
        angleWeights[idx[2]] += angle;
        pseudoNormals[idx[2]] += angle * normal;
    }

    for (size_t i = 0; i < m_points.Length(); ++i)
    {
        if (angleWeights[i] > 0)
        {
            pseudoNormals[i] /= angleWeights[i];
        }
    }

    std::swap(pseudoNormals, m_normals);
    m_normalIndices.CopyFrom(m_pointIndices);
}

void TriangleMesh3::Scale(double factor)
{
    ParallelFor(ZERO_SIZE, NumberOfPoints(),
                [this, factor](size_t i) { m_points[i] *= factor; });

    InvalidateCache();
}

void TriangleMesh3::Translate(const Vector3D& t)
{
    ParallelFor(ZERO_SIZE, NumberOfPoints(),
                [this, t](size_t i) { m_points[i] += t; });

    InvalidateCache();
}

void TriangleMesh3::Rotate(const QuaternionD& q)
{
    ParallelFor(ZERO_SIZE, NumberOfPoints(),
                [this, q](size_t i) { m_points[i] = q * m_points[i]; });

    ParallelFor(ZERO_SIZE, NumberOfNormals(),
                [this, q](size_t i) { m_normals[i] = q * m_normals[i]; });

    InvalidateCache();
}

void TriangleMesh3::WriteObj(std::ostream* stream) const
{
    // vertex
    for (const auto& pt : m_points)
    {
        *stream << "v " << pt << std::endl;
    }

    // UV coordinates
    for (const auto& uv : m_uvs)
    {
        *stream << "vt " << uv << std::endl;
    }

    // normals
    for (const auto& n : m_normals)
    {
        *stream << "vn " << n << std::endl;
    }

    // faces
    const bool hasUVs = HasUVs();
    const bool hasNormals = HasNormals();
    for (size_t i = 0; i < NumberOfTriangles(); ++i)
    {
        *stream << "f ";

        for (int j = 0; j < 3; ++j)
        {
            *stream << m_pointIndices[i][j] + 1;

            if (hasNormals || hasUVs)
            {
                *stream << '/';
            }

            if (hasUVs)
            {
                *stream << m_uvIndices[i][j] + 1;
            }

            if (hasNormals)
            {
                *stream << '/' << m_normalIndices[i][j] + 1;
            }
            *stream << ' ';
        }

        *stream << std::endl;
    }
}

bool TriangleMesh3::WriteObj(const std::string& fileName) const
{
    std::ofstream file(fileName.c_str());

    if (file)
    {
        WriteObj(&file);
        file.close();

        return true;
    }

    return false;
}

bool TriangleMesh3::ReadObj(std::istream* stream)
{
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn, err;

    const bool ret = LoadObj(&attrib, &shapes, &materials, &warn, &err, stream);

    // `err` may contain warning message.
    if (!err.empty())
    {
        std::cerr << err << '\n';
        return false;
    }

    // Failed to load obj.
    if (!ret)
    {
        return false;
    }

    InvalidateCache();

    // Read vertices
    for (size_t idx = 0; idx < attrib.vertices.size() / 3; ++idx)
    {
        // Access to vertex
        const tinyobj::real_t vx = attrib.vertices[3 * idx + 0];
        const tinyobj::real_t vy = attrib.vertices[3 * idx + 1];
        const tinyobj::real_t vz = attrib.vertices[3 * idx + 2];

        AddPoint({ vx, vy, vz });
    }

    // Read normals
    for (size_t idx = 0; idx < attrib.normals.size() / 3; ++idx)
    {
        // Access to normal
        const tinyobj::real_t vx = attrib.normals[3 * idx + 0];
        const tinyobj::real_t vy = attrib.normals[3 * idx + 1];
        const tinyobj::real_t vz = attrib.normals[3 * idx + 2];

        AddNormal({ vx, vy, vz });
    }

    // Read UVs
    for (size_t idx = 0; idx < attrib.texcoords.size() / 2; ++idx)
    {
        // Access to UV
        const tinyobj::real_t tu = attrib.texcoords[2 * idx + 0];
        const tinyobj::real_t tv = attrib.texcoords[2 * idx + 1];

        AddUV({ tu, tv });
    }

    // Read faces
    for (auto& shape : shapes)
    {
        size_t idx = 0;

        for (size_t f = 0; f < shape.mesh.num_face_vertices.size(); ++f)
        {
            const size_t fv = shape.mesh.num_face_vertices[f];

            if (fv == 3)
            {
                if (!attrib.vertices.empty())
                {
                    AddPointTriangle(
                        { static_cast<size_t>(
                              shape.mesh.indices[idx].vertex_index),
                          static_cast<size_t>(
                              shape.mesh.indices[idx + 1].vertex_index),
                          static_cast<size_t>(
                              shape.mesh.indices[idx + 2].vertex_index) });
                }

                if (!attrib.normals.empty())
                {
                    AddNormalTriangle(
                        { static_cast<size_t>(
                              shape.mesh.indices[idx].normal_index),
                          static_cast<size_t>(
                              shape.mesh.indices[idx + 1].normal_index),
                          static_cast<size_t>(
                              shape.mesh.indices[idx + 2].normal_index) });
                }

                if (!attrib.texcoords.empty())
                {
                    AddUVTriangle(
                        { static_cast<size_t>(
                              shape.mesh.indices[idx].texcoord_index),
                          static_cast<size_t>(
                              shape.mesh.indices[idx + 1].texcoord_index),
                          static_cast<size_t>(
                              shape.mesh.indices[idx + 2].texcoord_index) });
                }
            }

            idx += fv;
        }
    }

    return true;
}

bool TriangleMesh3::ReadObj(const std::string& fileName)
{
    std::ifstream file{ fileName.c_str() };

    if (file)
    {
        const bool result = ReadObj(&file);
        file.close();

        return result;
    }

    return false;
}

TriangleMesh3::Builder TriangleMesh3::GetBuilder()
{
    return Builder{};
}

Vector3D TriangleMesh3::ClosestPointLocal(const Vector3D& otherPoint) const
{
    BuildBVH();

    const auto distanceFunc = [this](const size_t& triIdx, const Vector3D& pt) {
        const Triangle3 tri = Triangle(triIdx);
        return tri.ClosestDistance(pt);
    };

    const auto queryResult = m_bvh.Nearest(otherPoint, distanceFunc);
    return Triangle(*queryResult.item).ClosestPoint(otherPoint);
}

double TriangleMesh3::ClosestDistanceLocal(const Vector3D& otherPoint) const
{
    BuildBVH();

    const auto distanceFunc = [this](const size_t& triIdx, const Vector3D& pt) {
        const Triangle3 tri = Triangle(triIdx);
        return tri.ClosestDistance(pt);
    };

    const auto queryResult = m_bvh.Nearest(otherPoint, distanceFunc);
    return queryResult.distance;
}

bool TriangleMesh3::IntersectsLocal(const Ray3D& ray) const
{
    BuildBVH();

    const auto testFunc = [this](const size_t& triIdx, const Ray3D& _ray) {
        const Triangle3 tri = Triangle(triIdx);
        return tri.Intersects(_ray);
    };

    return m_bvh.Intersects(ray, testFunc);
}

BoundingBox3D TriangleMesh3::BoundingBoxLocal() const
{
    BuildBVH();

    return m_bvh.GetBoundingBox();
}

Vector3D TriangleMesh3::ClosestNormalLocal(const Vector3D& otherPoint) const
{
    BuildBVH();

    const auto distanceFunc = [this](const size_t& triIdx, const Vector3D& pt) {
        const Triangle3 tri = Triangle(triIdx);
        return tri.ClosestDistance(pt);
    };

    const auto queryResult = m_bvh.Nearest(otherPoint, distanceFunc);
    return Triangle(*queryResult.item).ClosestNormal(otherPoint);
}

SurfaceRayIntersection3 TriangleMesh3::ClosestIntersectionLocal(
    const Ray3D& ray) const
{
    BuildBVH();

    const auto testFunc = [this](const size_t& triIdx, const Ray3D& _ray) {
        const Triangle3 tri = Triangle(triIdx);
        const SurfaceRayIntersection3 result = tri.ClosestIntersection(_ray);

        return result.distance;
    };

    const auto queryResult = m_bvh.ClosestIntersection(ray, testFunc);

    SurfaceRayIntersection3 result;
    result.distance = queryResult.distance;
    result.isIntersecting = queryResult.item != nullptr;

    if (queryResult.item != nullptr)
    {
        result.point = ray.PointAt(queryResult.distance);
        result.normal = Triangle(*queryResult.item).ClosestNormal(result.point);
    }

    return result;
}

bool TriangleMesh3::IsInsideLocal(const Vector3D& otherPoint) const
{
    return GetFastWindingNumber(otherPoint,
                                DEFAULT_FAST_WINDING_NUMBER_ACCURACY) > 0.5;
}

void TriangleMesh3::InvalidateCache() const
{
    m_bvhInvalidated = true;
    m_wnInvalidated = true;
}

void TriangleMesh3::InvalidateBVH() const
{
    m_bvhInvalidated = true;
}

void TriangleMesh3::BuildBVH() const
{
    if (m_bvhInvalidated)
    {
        const size_t nTris = NumberOfTriangles();

        Array1<size_t> ids(nTris);
        Array1<BoundingBox3D> bounds(nTris);
        for (size_t i = 0; i < nTris; ++i)
        {
            ids[i] = i;
            bounds[i] = Triangle(i).GetBoundingBox();
        }

        m_bvh.Build(ids, bounds);
        m_bvhInvalidated = false;
    }
}

void TriangleMesh3::BuildWindingNumbers() const
{
    // Barill et al., Fast Winding Numbers for Soups and Clouds,
    // ACM SIGGRAPH 2018
    if (m_wnInvalidated)
    {
        BuildBVH();

        const size_t numNodes = m_bvh.NumberOfNodes();
        m_wnAreaWeightedNormalSums.Resize(numNodes);
        m_wnAreaWeightedAvgPositions.Resize(numNodes);

        const auto visitorFunc = [this](size_t nodeIndex,
                                        const WindingNumberGatherData& data) {
            m_wnAreaWeightedNormalSums[nodeIndex] = data.areaWeightedNormalSums;
            m_wnAreaWeightedAvgPositions[nodeIndex] =
                data.areaWeightedPositionSums / data.areaSums;
        };

        const auto leafFunc =
            [this](size_t nodeIndex) -> WindingNumberGatherData {
            WindingNumberGatherData result;

            const auto iter = m_bvh.ItemOfNode(nodeIndex);
            assert(iter != m_bvh.end());

            Triangle3 tri = Triangle(*iter);
            const double area = tri.Area();
            result.areaSums = area;
            result.areaWeightedNormalSums = area * tri.FaceNormal();
            result.areaWeightedPositionSums =
                area * (tri.points[0] + tri.points[1] + tri.points[2]) / 3.0;

            return result;
        };

        PostOrderTraversal(m_bvh, 0, visitorFunc, leafFunc,
                           WindingNumberGatherData{});

        m_wnInvalidated = false;
    }
}

double TriangleMesh3::GetWindingNumber(const Vector3D& queryPoint,
                                       size_t triIndex) const
{
    // Jacobson et al., Robust Inside-Outside Segmentation using Generalized
    // Winding Numbers, ACM SIGGRAPH 2013.
    const Vector3D& vi = m_points[m_pointIndices[triIndex][0]];
    const Vector3D& vj = m_points[m_pointIndices[triIndex][1]];
    const Vector3D& vk = m_points[m_pointIndices[triIndex][2]];
    const Vector3D va = vi - queryPoint;
    const Vector3D vb = vj - queryPoint;
    const Vector3D vc = vk - queryPoint;
    const double a = va.Length();
    const double b = vb.Length();
    const double c = vc.Length();

    const Matrix3x3D mat(va.x, vb.x, vc.x, va.y, vb.y, vc.y, va.z, vb.z, vc.z);
    const double det = mat.Determinant();
    const double denom =
        a * b * c + va.Dot(vb) * c + vb.Dot(vc) * a + vc.Dot(va) * b;

    const double solidAngle = 2.0 * std::atan2(det, denom);

    return solidAngle;
}

double TriangleMesh3::GetFastWindingNumber(const Vector3D& queryPoint,
                                           double accuracy) const
{
    BuildWindingNumbers();

    return GetFastWindingNumber(queryPoint, 0, accuracy);
}

double TriangleMesh3::GetFastWindingNumber(const Vector3D& q,
                                           size_t rootNodeIndex,
                                           double accuracy) const
{
    // Barill et al., Fast Winding Numbers for Soups and Clouds,
    // ACM SIGGRAPH 2018.
    const Vector3D& treeP = m_wnAreaWeightedAvgPositions[rootNodeIndex];
    const double qToP2 = q.DistanceSquaredTo(treeP);

    const Vector3D& treeN = m_wnAreaWeightedNormalSums[rootNodeIndex];
    const BoundingBox3D& treeBound = m_bvh.NodeBound(rootNodeIndex);
    const Vector3D treeRVec =
        Max(treeP - treeBound.lowerCorner, treeBound.upperCorner - treeP);
    const double treeR = treeRVec.Length();

    if (qToP2 > Square(accuracy * treeR))
    {
        // Case: q is sufficiently far from all elements in tree
        // TODO: This is zero-th order approximation. Higher-order approximation
        // from Section 3.2.1 could be implemented for better accuracy in the
        // future.
        return (treeP - q).Dot(treeN) /
               (FOUR_PI_DOUBLE * Cubic(std::sqrt(qToP2)));
    }
    else
    {
        if (m_bvh.IsLeaf(rootNodeIndex))
        {
            // Case: q is nearby; use direct sum for tree's elements
            const auto iter = m_bvh.ItemOfNode(rootNodeIndex);
            return GetWindingNumber(q, *iter) * INV_FOUR_PI_DOUBLE;
        }
        else
        {
            // Case: Recursive call
            const auto children = m_bvh.Children(rootNodeIndex);
            double wn = 0.0;
            wn += GetFastWindingNumber(q, children.first, accuracy);
            wn += GetFastWindingNumber(q, children.second, accuracy);
            return wn;
        }
    }
}

TriangleMesh3::Builder& TriangleMesh3::Builder::WithPoints(
    const PointArray& points)
{
    m_points = points;
    return *this;
}

TriangleMesh3::Builder& TriangleMesh3::Builder::WithNormals(
    const NormalArray& normals)
{
    m_normals = normals;
    return *this;
}

TriangleMesh3::Builder& TriangleMesh3::Builder::WithUVs(const UVArray& uvs)
{
    m_uvs = uvs;
    return *this;
}

TriangleMesh3::Builder& TriangleMesh3::Builder::WithPointIndices(
    const IndexArray& pointIndices)
{
    m_pointIndices = pointIndices;
    return *this;
}

TriangleMesh3::Builder& TriangleMesh3::Builder::WithNormalIndices(
    const IndexArray& normalIndices)
{
    m_normalIndices = normalIndices;
    return *this;
}

TriangleMesh3::Builder& TriangleMesh3::Builder::WithUVIndices(
    const IndexArray& uvIndices)
{
    m_uvIndices = uvIndices;
    return *this;
}

TriangleMesh3 TriangleMesh3::Builder::Build() const
{
    return TriangleMesh3{ m_points,       m_normals,        m_uvs,
                          m_pointIndices, m_normalIndices,  m_uvIndices,
                          m_transform,    m_isNormalFlipped };
}

TriangleMesh3Ptr TriangleMesh3::Builder::MakeShared() const
{
    return std::make_shared<TriangleMesh3>(
        m_points, m_normals, m_uvs, m_pointIndices, m_normalIndices,
        m_uvIndices, m_transform, m_isNormalFlipped);
}
}  // namespace CubbyFlow
