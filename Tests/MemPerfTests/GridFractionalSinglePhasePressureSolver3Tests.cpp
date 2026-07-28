#include "MemPerfTestsUtils.hpp"

#include "gtest/gtest.h"

#include <Core/Grid/CellCenteredScalarGrid.hpp>
#include <Core/Grid/FaceCenteredGrid.hpp>
#include <Core/Solver/Grid/GridFractionalSinglePhasePressureSolver3.hpp>

using namespace CubbyFlow;

namespace
{
void RunExperiment(size_t n, double height, bool compressed)
{
    FaceCenteredGrid3 vel({ n, n, n });
    CellCenteredScalarGrid3 fluidSDF({ n, n, n });

    vel.Fill(Vector3D());

    for (size_t k = 0; k < n; ++k)
    {
        for (size_t j = 0; j < n + 1; ++j)
        {
            for (size_t i = 0; i < n; ++i)
            {
                if (j == 0 || j == n)
                {
                    vel.V(i, j, k) = 0.0;
                }
                else
                {
                    vel.V(i, j, k) = 1.0;
                }
            }
        }
    }

    fluidSDF.Fill(
        [&height, &n](const Vector3D& x) { return x.y - height * n; });

    GridFractionalSinglePhasePressureSolver3 solver;
    solver.Solve(vel, 1.0, &vel,
                 ConstantScalarField3(std::numeric_limits<double>::max()),
                 ConstantVectorField3({ 0, 0, 0 }), fluidSDF, compressed);
}
}  // namespace

TEST(GridFractionalSinglePhasePressureSolver3, FullUncompressed)
{
    const size_t mem0 = GetCurrentRSS();

    RunExperiment(128, 1.0, false);

    const size_t mem1 = GetCurrentRSS();

    const auto msg = MakeReadableByteSize(mem1 - mem0);

    PrintMemReport(msg.first, msg.second);
}

TEST(GridFractionalSinglePhasePressureSolver3, FullCompressed)
{
    const size_t mem0 = GetCurrentRSS();

    RunExperiment(128, 1.0, true);

    const size_t mem1 = GetCurrentRSS();

    const auto msg = MakeReadableByteSize(mem1 - mem0);

    PrintMemReport(msg.first, msg.second);
}

TEST(GridFractionalSinglePhasePressureSolver3, FreeSurfaceUncompressed)
{
    const size_t mem0 = GetCurrentRSS();

    RunExperiment(128, 0.25, false);

    const size_t mem1 = GetCurrentRSS();

    const auto msg = MakeReadableByteSize(mem1 - mem0);

    PrintMemReport(msg.first, msg.second);
}

TEST(GridFractionalSinglePhasePressureSolver3, FreeSurfaceCompressed)
{
    const size_t mem0 = GetCurrentRSS();

    RunExperiment(128, 0.25, true);

    const size_t mem1 = GetCurrentRSS();

    const auto msg = MakeReadableByteSize(mem1 - mem0);

    PrintMemReport(msg.first, msg.second);
}
