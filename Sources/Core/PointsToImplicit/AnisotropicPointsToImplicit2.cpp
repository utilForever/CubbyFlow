// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Math/SVD.hpp>
#include <Core/Matrix/Matrix.hpp>
#include <Core/Particle/SPHSystemData.hpp>
#include <Core/PointsToImplicit/AnisotropicPointsToImplicit2.hpp>
#include <Core/Searcher/PointKdTreeSearcher.hpp>
#include <Core/Solver/LevelSet/FMMLevelSetSolver2.hpp>
#include <Core/Utils/Logging.hpp>

#include <utility>

namespace CubbyFlow
{
inline double P(double distance)
{
    const double distanceSquared = distance * distance;

    if (distanceSquared >= 1.0)
    {
        return 0.0;
    }

    const double x = 1.0 - distanceSquared;
    return x * x * x;
}

inline double Wij(double distance, double r)
{
    if (distance < r)
    {
        return 1.0 - Cubic(distance / r);
    }

    return 0.0;
}

inline Matrix2x2D Vvt(const Vector2D& v)
{
    return Matrix2x2D{ v.x * v.x, v.x * v.y, v.y * v.x, v.y * v.y };
}

inline double W(const Vector2D& r, const Matrix2x2D& g, double gDet)
{
    static const double sigma = 4.0 / PI_DOUBLE;
    return sigma * gDet * P((g * r).Length());
}

inline std::pair<Vector2D, size_t> ComputeMean(
    const Vector2D& x, double r, const PointKdTreeSearcher2& searcher)
{
    Vector2D mean;
    double weightSum = 0.0;
    size_t numNeighbors = 0;
    searcher.ForEachNearbyPoint(x, r, [&](size_t, const Vector2D& neighbor) {
        const double weight = Wij((x - neighbor).Length(), r);
        weightSum += weight;
        mean += weight * neighbor;
        ++numNeighbors;
    });

    assert(weightSum > 0.0);
    return { mean / weightSum, numNeighbors };
}

inline Matrix2x2D ComputeAnisotropy(const Vector2D& x, const Vector2D& mean,
                                    double r, double h, double invH,
                                    const PointKdTreeSearcher2& searcher)
{
    auto covariance = Matrix2x2D::MakeScaleMatrix(h * h, h * h);
    double weightSum = 0.0;
    searcher.ForEachNearbyPoint(x, r, [&](size_t, const Vector2D& neighbor) {
        const double weight = Wij((mean - neighbor).Length(), r);
        weightSum += weight;
        covariance += weight * Vvt(neighbor - mean);
    });
    covariance /= weightSum;

    Matrix2x2D u;
    Vector2D singularValues;
    Matrix2x2D w;
    SVD(covariance, u, singularValues, w);

    singularValues.x = std::fabs(singularValues.x);
    singularValues.y = std::fabs(singularValues.y);
    const double minSingularValue = singularValues.Max() / 4.0;
    singularValues.x = std::max(singularValues.x, minSingularValue);
    singularValues.y = std::max(singularValues.y, minSingularValue);

    const Matrix2x2D invSigma =
        Matrix2x2D::MakeScaleMatrix(1.0 / singularValues);
    const double scale = std::sqrt(singularValues.x * singularValues.y);
    return invH * scale * (w * invSigma * u.Transposed());
}

AnisotropicPointsToImplicit2::AnisotropicPointsToImplicit2(
    double kernelRadius, double cutOffDensity, double positionSmoothingFactor,
    size_t minNumNeighbors, bool isOutputSDF)
    : m_kernelRadius(kernelRadius),
      m_cutOffDensity(cutOffDensity),
      m_positionSmoothingFactor(positionSmoothingFactor),
      m_minNumNeighbors(minNumNeighbors),
      m_isOutputSDF(isOutputSDF)
{
    // Do nothing
}

void AnisotropicPointsToImplicit2::Convert(
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

    const double h = m_kernelRadius;
    const double invH = 1 / h;
    const double r = 2.0 * h;

    // Mean estimator for cov. mat.
    const PointKdTreeSearcher2Ptr meanNeighborSearcher =
        PointKdTreeSearcher2::Builder{}.MakeShared();
    meanNeighborSearcher->Build(points);

    CUBBYFLOW_INFO << "Built neighbor searcher.";

    SPHSystemData2 meanParticles;
    meanParticles.AddParticles(points);
    meanParticles.SetNeighborSearcher(meanNeighborSearcher);
    meanParticles.SetKernelRadius(r);

    // Compute G and xMean
    std::vector<Matrix2x2D> gs(points.Length());
    Array1<Vector2D> xMeans{ points.Length() };

    ParallelFor(ZERO_SIZE, points.Length(),
                [&points, &meanNeighborSearcher, &r, &xMeans, this, &invH, &gs,
                 &h](size_t i) {
                    const Vector2D& x = points[i];
                    const auto [xMean, numNeighbors] =
                        ComputeMean(x, r, *meanNeighborSearcher);
                    xMeans[i] = Lerp(x, xMean, m_positionSmoothingFactor);
                    gs[i] = numNeighbors < m_minNumNeighbors
                                ? Matrix2x2D::MakeScaleMatrix(invH, invH)
                                : ComputeAnisotropy(x, xMean, r, h, invH,
                                                    *meanNeighborSearcher);
                });

    CUBBYFLOW_INFO << "Computed G and means.";

    // SPH estimator
    meanParticles.SetKernelRadius(h);
    meanParticles.UpdateDensities();
    const ArrayView1<double> d = meanParticles.Densities();
    const double m = meanParticles.Mass();

    PointKdTreeSearcher2 meanNeighborSearcher2;
    meanNeighborSearcher2.Build(xMeans);

    // Compute SDF
    std::shared_ptr<ScalarGrid2> temp = output->Clone();
    temp->Fill(
        [&meanNeighborSearcher2, this, &r, &m, &d, &gs](const Vector2D& x) {
            double sum = 0.0;
            meanNeighborSearcher2.ForEachNearbyPoint(
                x, r,
                [&sum, &m, &d, &x, &gs](size_t i,
                                        const Vector2D& neighborPosition) {
                    sum += m / d[i] *
                           W(neighborPosition - x, gs[i], gs[i].Determinant());
                });

            return m_cutOffDensity - sum;
        });

    CUBBYFLOW_INFO << "Computed SDF.";

    if (m_isOutputSDF)
    {
        FMMLevelSetSolver2 solver;
        solver.Reinitialize(*temp, std::numeric_limits<double>::max(), output);

        CUBBYFLOW_INFO << "Completed initialization.";
    }
    else
    {
        temp->Swap(output);
    }

    CUBBYFLOW_INFO << "Done converting points to implicit surface.";
}
}  // namespace CubbyFlow
