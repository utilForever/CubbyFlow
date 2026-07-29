#include "gtest/gtest.h"

#include <Core/Solver/FDM/FDMMGSolver3.hpp>

using namespace CubbyFlow;

TEST(FDMMGSolver3, Solve)
{
    const size_t levels = 6;
    FDMMGLinearSystem3 system;
    system.ResizeWithCoarsest({ 4, 4, 4 }, levels);

    // Simple Poisson eq.
    for (size_t l = 0; l < system.GetNumberOfLevels(); ++l)
    {
        double invdx = pow(0.5, l);
        FDMMatrix3& A = system.A[l];
        FDMVector3& b = system.b[l];

        system.x[l].Fill(0);

        ForEachIndex(A.Size(), [&A, &invdx, &b](size_t i, size_t j, size_t k) {
            if (i > 0)
            {
                A(i, j, k).center += invdx * invdx;
            }
            if (i < A.Width() - 1)
            {
                A(i, j, k).center += invdx * invdx;
                A(i, j, k).right -= invdx * invdx;
            }

            if (j > 0)
            {
                A(i, j, k).center += invdx * invdx;
            }
            else
            {
                b(i, j, k) += invdx;
            }

            if (j < A.Height() - 1)
            {
                A(i, j, k).center += invdx * invdx;
                A(i, j, k).up -= invdx * invdx;
            }
            else
            {
                b(i, j, k) -= invdx;
            }

            if (k > 0)
            {
                A(i, j, k).center += invdx * invdx;
            }
            if (k < A.Depth() - 1)
            {
                A(i, j, k).center += invdx * invdx;
                A(i, j, k).front -= invdx * invdx;
            }
        });
    }

    auto buffer = system.x[0];
    FDMBLAS3::Residual(system.A[0], system.x[0], system.b[0], &buffer);
    double norm0 = FDMBLAS3::L2Norm(buffer);

    FDMMGSolver3 solver(levels, 5, 5, 20, 20, 1e-9);
    solver.Solve(&system);

    FDMBLAS3::Residual(system.A[0], system.x[0], system.b[0], &buffer);
    double norm1 = FDMBLAS3::L2Norm(buffer);

    EXPECT_LT(norm1, norm0);
}