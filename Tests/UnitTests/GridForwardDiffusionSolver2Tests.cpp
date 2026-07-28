#include "gtest/gtest.h"

#include <Core/Grid/CellCenteredScalarGrid.hpp>
#include <Core/Solver/Grid/GridForwardEulerDiffusionSolver2.hpp>

using namespace CubbyFlow;

TEST(GridForwardEulerDiffusionSolver2, Solve)
{
    CellCenteredScalarGrid2 src({ 3, 3 }, { 1.0, 1.0 }, { 0.0, 0.0 });
    CellCenteredScalarGrid2 dst({ 3, 3 }, { 1.0, 1.0 }, { 0.0, 0.0 });

    src(1, 1) = 1.0;

    GridForwardEulerDiffusionSolver2 diffusionSolver;
    diffusionSolver.Solve(src, 1.0 / 8.0, 1.0, &dst);

    Array2<double> solution = { { 0.0, 1.0 / 8.0, 0.0 },
                                { 1.0 / 8.0, 1.0 / 2.0, 1.0 / 8.0 },
                                { 0.0, 1.0 / 8.0, 0.0 } };

    dst.ForEachDataPointIndex([&solution, &dst](size_t i, size_t j) {
        EXPECT_NEAR(solution(i, j), dst(i, j), 1e-6);
    });
}