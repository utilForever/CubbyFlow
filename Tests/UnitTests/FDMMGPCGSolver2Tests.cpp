#include "gtest/gtest.h"

#include <Core/Solver/FDM/FDMMGPCGSolver2.hpp>

using namespace CubbyFlow;

TEST(FDMMGPCGSolver2, Solve)
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

    FDMMGPCGSolver2 solver(200, levels, 5, 5, 10, 10, 1e-4);
    EXPECT_TRUE(solver.Solve(&system));
}