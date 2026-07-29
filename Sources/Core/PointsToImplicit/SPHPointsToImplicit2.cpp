// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Particle/SPHSystemData.hpp>
#include <Core/PointsToImplicit/SPHPointsToImplicit2.hpp>
#include <Core/Solver/LevelSet/FMMLevelSetSolver2.hpp>
#include <Core/Utils/Logging.hpp>

namespace CubbyFlow
{
SPHPointsToImplicit2::SPHPointsToImplicit2(double kernelRadius,
                                           double cutOffDensity,
                                           bool isOutputSDF)
    : m_kernelRadius(kernelRadius),
      m_cutOffDensity(cutOffDensity),
      m_isOutputSDF(isOutputSDF)
{
    // Do nothing
}

void SPHPointsToImplicit2::Convert(const ConstArrayView1<Vector2D>& points,
                                   ScalarGrid2* output) const
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

    SPHSystemData2 sphParticles;
    sphParticles.AddParticles(points);
    sphParticles.SetKernelRadius(m_kernelRadius);
    sphParticles.BuildNeighborSearcher();
    sphParticles.UpdateDensities();

    Array1<double> constData(sphParticles.NumberOfParticles(), 1.0);
    std::shared_ptr<ScalarGrid2> temp = output->Clone();
    temp->Fill([&sphParticles, &constData, this](const Vector2D& x) {
        const double d = sphParticles.Interpolate(x, constData);
        return m_cutOffDensity - d;
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
