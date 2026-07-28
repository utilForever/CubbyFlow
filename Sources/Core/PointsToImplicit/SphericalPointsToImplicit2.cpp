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
#include <Core/PointsToImplicit/SphericalPointsToImplicit2.hpp>
#include <Core/Solver/LevelSet/FMMLevelSetSolver2.hpp>
#include <Core/Utils/Logging.hpp>

namespace CubbyFlow
{
SphericalPointsToImplicit2::SphericalPointsToImplicit2(double radius,
                                                       bool isOutputSDF)
    : m_radius(radius), m_isOutputSDF(isOutputSDF)
{
    // Do nothing
}

void SphericalPointsToImplicit2::Convert(
    const ConstArrayView1<Vector2D>& points, ScalarGrid2* output) const
{
    if (output == nullptr)
    {
        CUBBYFLOW_WARN << "Null scalar grid output pointer provided.";
        return;
    }

    const Vector2UZ& res = output->Resolution();
    if (res.x * res.y == 0)
    {
        CUBBYFLOW_WARN << "Empty grid is provided.";
        return;
    }

    const BoundingBox2D& bbox = output->GetBoundingBox();
    if (bbox.IsEmpty())
    {
        CUBBYFLOW_WARN << "Empty domain is provided.";
        return;
    }

    ParticleSystemData2 particles;
    particles.AddParticles(points);
    particles.BuildNeighborSearcher(2.0 * m_radius);

    const PointNeighborSearcher2Ptr neighborSearcher =
        particles.NeighborSearcher();

    std::shared_ptr<ScalarGrid2> temp = output->Clone();
    temp->Fill([this, &neighborSearcher](const Vector2D& x) {
        double minDist = 2.0 * m_radius;
        neighborSearcher->ForEachNearbyPoint(
            x, 2.0 * m_radius,
            [this, &minDist, &x](size_t, const Vector2D& xj) {
                minDist = std::min(minDist, (x - xj).Length());
            });

        return minDist - m_radius;
    });

    if (m_isOutputSDF)
    {
        FMMLevelSetSolver2 solver;
        solver.Reinitialize(*temp, std::numeric_limits<double>::max(), output);
    }
    else
    {
        temp->Swap(output);
    }
}
}  // namespace CubbyFlow
