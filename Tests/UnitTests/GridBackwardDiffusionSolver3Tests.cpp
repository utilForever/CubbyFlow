#include "gtest/gtest.h"

#include <Core/Grid/CellCenteredScalarGrid.hpp>
#include <Core/Solver/Grid/GridBackwardEulerDiffusionSolver3.hpp>

using namespace CubbyFlow;

TEST(GridBackwardEulerDiffusionSolver3, Solve)
{
    CellCenteredScalarGrid3 src({ 3, 3, 3 }, { 1.0, 1.0, 1.0 },
                                { 0.0, 0.0, 0.0 });
    CellCenteredScalarGrid3 dst({ 3, 3, 3 }, { 1.0, 1.0, 1.0 },
                                { 0.0, 0.0, 0.0 });

    src(1, 1, 1) = 1.0;

    GridBackwardEulerDiffusionSolver3 diffusionSolver;
    diffusionSolver.Solve(src, 1.0 / 12.0, 1.0, &dst);

    Array3<double> solution = { { { 0.001058, 0.005291, 0.001058 },
                                  { 0.005291, 0.041270, 0.005291 },
                                  { 0.001058, 0.005291, 0.001058 } },
                                { { 0.005291, 0.041270, 0.005291 },
                                  { 0.041270, 0.680423, 0.041270 },
                                  { 0.005291, 0.041270, 0.005291 } },
                                { { 0.001058, 0.005291, 0.001058 },
                                  { 0.005291, 0.041270, 0.005291 },
                                  { 0.001058, 0.005291, 0.001058 } } };

    dst.ForEachDataPointIndex([&solution, &dst](size_t i, size_t j, size_t k) {
        EXPECT_NEAR(solution(i, j, k), dst(i, j, k), 1e-6);
    });
}