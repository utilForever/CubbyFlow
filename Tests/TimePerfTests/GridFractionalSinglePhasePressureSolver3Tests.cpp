#include "benchmark/benchmark.h"

#include <Core/Field/ConstantScalarField.hpp>
#include <Core/Field/ConstantVectorField.hpp>
#include <Core/Grid/CellCenteredScalarGrid.hpp>
#include <Core/Grid/FaceCenteredGrid.hpp>
#include <Core/Matrix/Matrix.hpp>
#include <Core/Solver/Grid/GridFractionalSinglePhasePressureSolver3.hpp>

using CubbyFlow::CellCenteredScalarGrid3;
using CubbyFlow::ConstantScalarField3;
using CubbyFlow::ConstantVectorField3;
using CubbyFlow::FaceCenteredGrid3;
using CubbyFlow::Vector3D;

class GridFractionalSinglePhasePressureSolver3 : public ::benchmark::Fixture
{
 public:
    FaceCenteredGrid3 vel;
    CellCenteredScalarGrid3 fluidSDF;
    CubbyFlow::GridFractionalSinglePhasePressureSolver3 solver;

    void SetUp(const ::benchmark::State& state)
    {
        const auto n = static_cast<size_t>(state.range(0));
        const auto height = static_cast<double>(state.range(1));

        vel.Resize({ n, n, n });
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

        fluidSDF.Resize({ n, n, n });
        fluidSDF.Fill([&height](const Vector3D& x) { return x.y - height; });
    }
};

BENCHMARK_DEFINE_F(GridFractionalSinglePhasePressureSolver3, Solve)
(benchmark::State& state)
{
    bool compressed = state.range(2) == 1;
    while (state.KeepRunning())
    {
        solver.Solve(vel, 1.0, &vel,
                     ConstantScalarField3(std::numeric_limits<double>::max()),
                     ConstantVectorField3({ 0, 0, 0 }), fluidSDF, compressed);
    }
}

BENCHMARK_REGISTER_F(GridFractionalSinglePhasePressureSolver3, Solve)
    ->Args({ 128, 128, 0 })
    ->Args({ 128, 128, 1 })
    ->Args({ 128, 64, 0 })
    ->Args({ 128, 64, 1 })
    ->Args({ 128, 32, 0 })
    ->Args({ 128, 32, 1 });
