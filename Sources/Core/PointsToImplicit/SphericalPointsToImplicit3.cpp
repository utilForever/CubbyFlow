// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Particle/ParticleSystemData.hpp>
#include <Core/PointsToImplicit/SphericalPointsToImplicit3.hpp>
#include <Core/Solver/LevelSet/FMMLevelSetSolver3.hpp>
#include <Core/Utils/Logging.hpp>

namespace CubbyFlow
{
SphericalPointsToImplicit3::SphericalPointsToImplicit3(double radius,
                                                       bool isOutputSDF)
    : m_radius(radius), m_isOutputSDF(isOutputSDF)
{
    // Do nothing
}

void SphericalPointsToImplicit3::Convert(
    const ConstArrayView1<Vector3D>& points, ScalarGrid3* output) const
{
    if (output == nullptr)
    {
        CUBBYFLOW_WARN << "Null scalar grid output pointer provided.";
        return;
    }

    const Vector3UZ& res = output->Resolution();
    if (res.x * res.y * res.z == 0)
    {
        CUBBYFLOW_WARN << "Empty grid is provided.";
        return;
    }

    const BoundingBox3D& bbox = output->GetBoundingBox();
    if (bbox.IsEmpty())
    {
        CUBBYFLOW_WARN << "Empty domain is provided.";
        return;
    }

    ParticleSystemData3 particles;
    particles.AddParticles(points);
    particles.BuildNeighborSearcher(2.0 * m_radius);

    const PointNeighborSearcher3Ptr neighborSearcher =
        particles.NeighborSearcher();

    std::shared_ptr<ScalarGrid3> temp = output->Clone();
    temp->Fill([this, &neighborSearcher](const Vector3D& x) {
        double minDist = 2.0 * m_radius;
        neighborSearcher->ForEachNearbyPoint(
            x, 2.0 * m_radius,
            [this, &minDist, &x](size_t, const Vector3D& xj) {
                minDist = std::min(minDist, (x - xj).Length());
            });

        return minDist - m_radius;
    });

    if (m_isOutputSDF)
    {
        FMMLevelSetSolver3 solver;
        solver.Reinitialize(*temp, std::numeric_limits<double>::max(), output);
    }
    else
    {
        temp->Swap(output);
    }
}
}  // namespace CubbyFlow