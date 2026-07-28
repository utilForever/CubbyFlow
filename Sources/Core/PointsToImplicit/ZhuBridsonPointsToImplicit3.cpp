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
#include <Core/PointsToImplicit/ZhuBridsonPointsToImplicit3.hpp>
#include <Core/Solver/LevelSet/FMMLevelSetSolver3.hpp>
#include <Core/Utils/Logging.hpp>

namespace CubbyFlow
{
inline double k(double s)
{
    return std::max(0.0, Cubic(1.0 - s * s));
}

ZhuBridsonPointsToImplicit3::ZhuBridsonPointsToImplicit3(double kernelRadius,
                                                         double cutOffThreshold,
                                                         bool isOutputSDF)
    : m_kernelRadius(kernelRadius),
      m_cutOffThreshold(cutOffThreshold),
      m_isOutputSDF(isOutputSDF)
{
    // Do nothing
}

void ZhuBridsonPointsToImplicit3::Convert(
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
    particles.BuildNeighborSearcher(m_kernelRadius);

    const PointNeighborSearcher3Ptr neighborSearcher =
        particles.NeighborSearcher();
    const double isoContValue = m_cutOffThreshold * m_kernelRadius;

    auto temp = output->Clone();
    temp->Fill([&neighborSearcher, this, &isoContValue,
                &output](const Vector3D& x) -> double {
        Vector3D xAvg;
        double wSum = 0.0;
        const auto func = [&x, this, &wSum, &xAvg](size_t, const Vector3D& xi) {
            const double wi = k((x - xi).Length() / m_kernelRadius);
            wSum += wi;
            xAvg += wi * xi;
        };
        neighborSearcher->ForEachNearbyPoint(x, m_kernelRadius, func);

        if (wSum > 0.0)
        {
            xAvg /= wSum;
            return (x - xAvg).Length() - isoContValue;
        }
        else
        {
            return output->GetBoundingBox().DiagonalLength();
        }
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