#include "gtest/gtest.h"

#include <Core/Math/BLAS.hpp>
#include <Core/Matrix/Matrix.hpp>
#include <Core/Utils/IterationUtils.hpp>
#include <Core/Utils/MG.hpp>

using namespace CubbyFlow;

namespace
{
using BLASType = BLAS<double, VectorND, MatrixMxND>;

void Relax(const typename BLASType::MatrixType& a,
           const typename BLASType::VectorType& b,
           unsigned int numberOfIterations, double maxTolerance,
           typename BLASType::VectorType* x,
           typename BLASType::VectorType* buffer)
{
    UNUSED_VARIABLE(maxTolerance);
    UNUSED_VARIABLE(buffer);

    size_t n = a.GetRows();

    for (unsigned int iter = 0; iter < numberOfIterations; ++iter)
    {
        ForEachIndex(x->GetRows(), [&n, &a, &x, &b](size_t i) {
            double sum = 0.0;

            for (size_t j = 0; j < n; ++j)
            {
                if (i != j)
                {
                    sum += a(j, i) * (*x)[j];
                }
            }

            (*x)[i] = (b[i] - sum) / a(i, i);
        });
    }
}

void Rest(const typename BLASType::VectorType& finer,
          typename BLASType::VectorType* coarser)
{
    size_t n = coarser->GetRows();
    ParallelForEachIndex(coarser->GetRows(), [&n, &coarser, &finer](size_t i) {
        // --*--|--*--|--*--|--*--
        //  1/8   3/8   3/8   1/8
        //           to
        // -----|-----*-----|-----
        size_t _2im1 = (i > 0) ? 2 * i - 1 : 2 * i;
        size_t _2ip2 = (i + 1 < n) ? 2 * i + 2 : 2 * i + 1;

        (*coarser)[i] = 0.125 * (finer[_2im1] + 3.0 * finer[2 * i] +
                                 3.0 * finer[2 * i + 1] + finer[_2ip2]);
    });
}

void Corr(const typename BLASType::VectorType& coarser,
          typename BLASType::VectorType* finer)
{
    size_t n = coarser.GetRows();

    ForEachIndex(coarser.GetRows(), [&n, &finer, &coarser](size_t i) {
        // -----|-----*-----|-----
        //           to
        //  1/4   3/4   3/4   1/4
        // --*--|--*--|--*--|--*--
        size_t _2im1 = (i > 0) ? 2 * i - 1 : 2 * i;
        size_t _2ip2 = (i + 1 < n) ? 2 * i + 2 : 2 * i + 1;

        (*finer)[_2im1] += 0.25 * coarser[i];
        (*finer)[2 * i + 0] += 0.75 * coarser[i];
        (*finer)[2 * i + 1] += 0.75 * coarser[i];
        (*finer)[_2ip2] += 0.25 * coarser[i];
    });
}
}  // namespace

TEST(MG, Solve)
{
    MGMatrix<BLASType> A;
    MGVector<BLASType> x, b, tmp;
    MGParameters<BLASType> params;

    size_t n = 128;
    size_t levels = 6;

    // Build matrix
    A.levels.resize(levels);
    x.levels.resize(levels);
    b.levels.resize(levels);
    tmp.levels.resize(levels);

    for (size_t l = 0; l < levels; ++l)
    {
        size_t m = n >> l;
        A[l].Resize(m, m, 0.0);
        x[l].Resize(m, 0.0);
        b[l].Resize(m, 0.0);
        tmp[l].Resize(m, 0.0);
    }

    // Simple Poisson eq.
    for (size_t l = 0; l < levels; ++l)
    {
        size_t m = n >> l;
        double invdx = pow(0.5, l);
        auto& Al = A[l];
        auto& bl = b[l];

        for (size_t i = 0; i < m; ++i)
        {
            if (i > 0)
            {
                Al(i, i) += invdx * invdx;
                Al(i - 1, i) -= invdx * invdx;
                bl[i] += invdx;
            }

            if (i < m - 1)
            {
                Al(i, i) += invdx * invdx;
                Al(i + 1, i) -= invdx * invdx;
                bl[i] -= invdx;
            }
        }
    }

    // Test relax
    BLASType::Residual(A[0], x[0], b[0], &tmp[0]);
    double r0 = BLASType::L2Norm(tmp[0]);

    Relax(A[0], b[0], 100, 0.0, &x[0], &tmp[0]);

    BLASType::Residual(A[0], x[0], b[0], &tmp[0]);
    double r1 = BLASType::L2Norm(tmp[0]);

    EXPECT_GT(r0, r1);

    // Reset solution
    x[0].Fill(0.0);

    // Now MultiGrid
    params.maxNumberOfLevels = levels;
    params.relaxFunc = Relax;
    params.restrictFunc = Rest;
    params.correctFunc = Corr;

    auto result = MGVCycle(A, params, &x, &b, &tmp);
    EXPECT_GT(r0, result.lastResidualNorm);
    EXPECT_GT(r1, result.lastResidualNorm);
}