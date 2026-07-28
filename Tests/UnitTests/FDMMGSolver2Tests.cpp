#include "gtest/gtest.h"

#include <Core/Solver/FDM/FDMMGSolver2.hpp>

using namespace CubbyFlow;

TEST(FDMMGSolver2, Solve)
{
    const size_t levels = 6;
    FDMMGLinearSystem2 system;
    system.ResizeWithCoarsest({ 4, 4 }, levels);

    // Simple Poisson eq.
    for (size_t l = 0; l < system.GetNumberOfLevels(); ++l)
    {
        double invdx = pow(0.5, l);
        FDMMatrix2& A = system.A[l];
        FDMVector2& b = system.b[l];

        system.x[l].Fill(0);

        ForEachIndex(A.Size(), [&A, &invdx, &b](size_t i, size_t j) {
            if (i > 0)
            {
                A(i, j).center += invdx * invdx;
            }
            if (i < A.Width() - 1)
            {
                A(i, j).center += invdx * invdx;
                A(i, j).right -= invdx * invdx;
            }

            if (j > 0)
            {
                A(i, j).center += invdx * invdx;
            }
            else
            {
                b(i, j) += invdx;
            }

            if (j < A.Height() - 1)
            {
                A(i, j).center += invdx * invdx;
                A(i, j).up -= invdx * invdx;
            }
            else
            {
                b(i, j) -= invdx;
            }
        });
    }

    auto buffer = system.x[0];
    FDMBLAS2::Residual(system.A[0], system.x[0], system.b[0], &buffer);
    double norm0 = FDMBLAS2::L2Norm(buffer);

    FDMMGSolver2 solver(levels, 5, 5, 20, 20, 1e-9);
    solver.Solve(&system);

    FDMBLAS2::Residual(system.A[0], system.x[0], system.b[0], &buffer);
    double norm1 = FDMBLAS2::L2Norm(buffer);

    EXPECT_LT(norm1, norm0);
}