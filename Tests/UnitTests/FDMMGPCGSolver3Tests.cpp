#include "gtest/gtest.h"

#include <Core/Solver/FDM/FDMMGPCGSolver3.hpp>

using namespace CubbyFlow;

TEST(FDMMGPCGSolver3, Solve)
{
    const size_t levels = 4;
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

    FDMMGPCGSolver3 solver(50, levels, 5, 5, 10, 10, 1e-4, 1.5, false);
    EXPECT_TRUE(solver.Solve(&system));
}