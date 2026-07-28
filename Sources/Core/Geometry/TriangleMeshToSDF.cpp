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
#include <Core/Geometry/TriangleMeshToSDF.hpp>
#include <Core/Grid/ScalarGrid.hpp>
#include <Core/Matrix/Matrix.hpp>

namespace CubbyFlow
{
void TriangleMeshToSDF(const TriangleMesh3& mesh, ScalarGrid3* sdf)
{
    const Vector3UZ size = sdf->DataSize();
    if (size.x * size.y * size.z == 0)
    {
        return;
    }

    const GridDataPositionFunc<3> pos = sdf->DataPosition();
    mesh.UpdateQueryEngine();
    sdf->ParallelForEachDataPointIndex(
        [&pos, &mesh, &sdf](size_t i, size_t j, size_t k) {
            const Vector3D p = pos(i, j, k);
            const double d = mesh.ClosestDistance(p);
            const double sd = mesh.IsInside(p) ? -d : d;

            (*sdf)(i, j, k) = sd;
        });
}
}  // namespace CubbyFlow
