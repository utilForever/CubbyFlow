// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Geometry/ImplicitTriangleMesh3.hpp>
#include <Core/Geometry/TriangleMeshToSDF.hpp>

#include <utility>

namespace CubbyFlow
{
ImplicitTriangleMesh3::ImplicitTriangleMesh3(TriangleMesh3Ptr mesh,
                                             size_t resolutionX, double margin,
                                             const Transform3& _transform,
                                             bool _isNormalFlipped)
    : ImplicitSurface3{ _transform, _isNormalFlipped }, m_mesh(std::move(mesh))
{
    BoundingBox3D box = m_mesh->GetBoundingBox();
    const Vector3D scale{ box.Width(), box.Height(), box.Depth() };
    box.lowerCorner -= margin * scale;
    box.upperCorner += margin * scale;

    const auto resolutionY = static_cast<size_t>(std::ceil(
        static_cast<double>(resolutionX) * box.Height() / box.Width()));
    const auto resolutionZ = static_cast<size_t>(std::ceil(
        static_cast<double>(resolutionX) * box.Depth() / box.Width()));

    const double dx = box.Width() / static_cast<double>(resolutionX);

    m_grid = std::make_shared<VertexCenteredScalarGrid3>();
    m_grid->Resize({ resolutionX, resolutionY, resolutionZ }, { dx, dx, dx },
                   { box.lowerCorner.x, box.lowerCorner.y, box.lowerCorner.z });

    TriangleMeshToSDF(*m_mesh, m_grid.get());

    m_customImplicitSurface =
        CustomImplicitSurface3::Builder{}
            .WithSignedDistanceFunction([&](const Vector3D& pt) -> double {
                return m_grid->Sample(pt);
            })
            .WithDomain(m_grid->GetBoundingBox())
            .WithResolution(dx)
            .MakeShared();
}

Vector3D ImplicitTriangleMesh3::ClosestPointLocal(
    const Vector3D& otherPoint) const
{
    return m_customImplicitSurface->ClosestPoint(otherPoint);
}

double ImplicitTriangleMesh3::ClosestDistanceLocal(
    const Vector3D& otherPoint) const
{
    return m_customImplicitSurface->ClosestDistance(otherPoint);
}

bool ImplicitTriangleMesh3::IntersectsLocal(const Ray3D& ray) const
{
    return m_customImplicitSurface->Intersects(ray);
}

BoundingBox3D ImplicitTriangleMesh3::BoundingBoxLocal() const
{
    return m_mesh->GetBoundingBox();
}

Vector3D ImplicitTriangleMesh3::ClosestNormalLocal(
    const Vector3D& otherPoint) const
{
    return m_customImplicitSurface->ClosestNormal(otherPoint);
}

double ImplicitTriangleMesh3::SignedDistanceLocal(
    const Vector3D& otherPoint) const
{
    return m_customImplicitSurface->SignedDistance(otherPoint);
}

SurfaceRayIntersection3 ImplicitTriangleMesh3::ClosestIntersectionLocal(
    const Ray3D& ray) const
{
    return m_customImplicitSurface->ClosestIntersection(ray);
}

ImplicitTriangleMesh3::Builder ImplicitTriangleMesh3::GetBuilder()
{
    return Builder{};
}

const VertexCenteredScalarGrid3Ptr& ImplicitTriangleMesh3::GetGrid() const
{
    return m_grid;
}

ImplicitTriangleMesh3::Builder&
ImplicitTriangleMesh3::Builder::WithTriangleMesh(const TriangleMesh3Ptr& mesh)
{
    m_mesh = mesh;
    return *this;
}

ImplicitTriangleMesh3::Builder& ImplicitTriangleMesh3::Builder::WithResolutionX(
    size_t resolutionX)
{
    m_resolutionX = resolutionX;
    return *this;
}

ImplicitTriangleMesh3::Builder& ImplicitTriangleMesh3::Builder::WithMargin(
    double margin)
{
    m_margin = margin;
    return *this;
}

ImplicitTriangleMesh3 ImplicitTriangleMesh3::Builder::Build() const
{
    return ImplicitTriangleMesh3{ m_mesh, m_resolutionX, m_margin, m_transform,
                                  m_isNormalFlipped };
}

ImplicitTriangleMesh3Ptr ImplicitTriangleMesh3::Builder::MakeShared() const
{
    return std::make_shared<ImplicitTriangleMesh3>(
        m_mesh, m_resolutionX, m_margin, m_transform, m_isNormalFlipped);
}
}  // namespace CubbyFlow