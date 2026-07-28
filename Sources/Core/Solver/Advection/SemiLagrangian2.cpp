// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Solver/Advection/SemiLagrangian2.hpp>

namespace CubbyFlow
{
void SemiLagrangian2::Advect(const ScalarGrid2& input, const VectorField2& flow,
                             double dt, ScalarGrid2* output,
                             const ScalarField2& boundarySDF)
{
    std::function<double(const Vector2D&)> inputSamplerFunc =
        GetScalarSamplerFunc(input);
    double h = std::min(output->GridSpacing().x, output->GridSpacing().y);

    GridDataPositionFunc<2> inputDataPos = input.DataPosition();
    GridDataPositionFunc<2> outputDataPos = output->DataPosition();
    ArrayView<double, 2> outputDataAcc = output->DataView();

    output->ParallelForEachDataPointIndex(
        [&boundarySDF, &inputDataPos, this, &flow, &dt, &h, &outputDataPos,
         &outputDataAcc, &inputSamplerFunc](size_t i, size_t j) {
            if (boundarySDF.Sample(inputDataPos(i, j)) > 0.0)
            {
                const Vector2D pt =
                    BackTrace(flow, dt, h, outputDataPos(i, j), boundarySDF);
                outputDataAcc(i, j) = inputSamplerFunc(pt);
            }
        });
}

void SemiLagrangian2::Advect(const CollocatedVectorGrid2& input,
                             const VectorField2& flow, double dt,
                             CollocatedVectorGrid2* output,
                             const ScalarField2& boundarySDF)
{
    std::function<Vector2D(const Vector2D&)> inputSamplerFunc =
        GetVectorSamplerFunc(input);
    double h = std::min(output->GridSpacing().x, output->GridSpacing().y);

    GridDataPositionFunc<2> inputDataPos = input.DataPosition();
    GridDataPositionFunc<2> outputDataPos = output->DataPosition();
    ArrayView<Vector<double, 2>, 2> outputDataAcc = output->DataView();

    output->ParallelForEachDataPointIndex(
        [&boundarySDF, &inputDataPos, this, &flow, &dt, &h, &outputDataPos,
         &outputDataAcc, &inputSamplerFunc](size_t i, size_t j) {
            if (boundarySDF.Sample(inputDataPos(i, j)) > 0.0)
            {
                const Vector2D pt =
                    BackTrace(flow, dt, h, outputDataPos(i, j), boundarySDF);
                outputDataAcc(i, j) = inputSamplerFunc(pt);
            }
        });
}

void SemiLagrangian2::Advect(const FaceCenteredGrid2& input,
                             const VectorField2& flow, double dt,
                             FaceCenteredGrid2* output,
                             const ScalarField2& boundarySDF)
{
    std::function<Vector2D(const Vector2D&)> inputSamplerFunc =
        GetVectorSamplerFunc(input);
    double h = std::min(output->GridSpacing().x, output->GridSpacing().y);

    auto uSourceDataPos = input.UPosition();
    auto uTargetDataPos = output->UPosition();
    ArrayView<double, 2> uTargetDataAcc = output->UView();

    output->ParallelForEachUIndex([&boundarySDF, &uSourceDataPos, this, &flow,
                                   &dt, &h, &uTargetDataPos, &uTargetDataAcc,
                                   &inputSamplerFunc](const Vector2UZ& idx) {
        if (boundarySDF.Sample(uSourceDataPos(idx)) > 0.0)
        {
            const Vector2D pt =
                BackTrace(flow, dt, h, uTargetDataPos(idx), boundarySDF);
            uTargetDataAcc(idx) = inputSamplerFunc(pt).x;
        }
    });

    auto vSourceDataPos = input.VPosition();
    auto vTargetDataPos = output->VPosition();
    ArrayView<double, 2> vTargetDataAcc = output->VView();

    output->ParallelForEachVIndex([&boundarySDF, &vSourceDataPos, this, &flow,
                                   &dt, &h, &vTargetDataPos, &vTargetDataAcc,
                                   &inputSamplerFunc](const Vector2UZ& idx) {
        if (boundarySDF.Sample(vSourceDataPos(idx)) > 0.0)
        {
            const Vector2D pt =
                BackTrace(flow, dt, h, vTargetDataPos(idx), boundarySDF);
            vTargetDataAcc(idx) = inputSamplerFunc(pt).y;
        }
    });
}

Vector2D SemiLagrangian2::BackTrace(const VectorField2& flow, double dt,
                                    double h, const Vector2D& startPt,
                                    const ScalarField2& boundarySDF) const
{
    double remainingT = dt;
    Vector2D pt0 = startPt;
    Vector2D pt1 = startPt;

    while (remainingT > std::numeric_limits<double>::epsilon())
    {
        // Adaptive time-stepping
        Vector2D vel0 = flow.Sample(pt0);
        const double numSubSteps =
            std::max(std::ceil(vel0.Length() * remainingT / h), 1.0);
        dt = remainingT / numSubSteps;

        // Mid-point rule
        Vector2D midPt = pt0 - 0.5 * dt * vel0;
        Vector2D midVel = flow.Sample(midPt);
        pt1 = pt0 - dt * midVel;

        // Boundary handling
        const double phi0 = boundarySDF.Sample(pt0);
        const double phi1 = boundarySDF.Sample(pt1);

        if (phi0 * phi1 < 0.0)
        {
            const double w =
                std::fabs(phi1) / (std::fabs(phi0) + std::fabs(phi1));
            pt1 = w * pt0 + (1.0 - w) * pt1;
            break;
        }

        remainingT -= dt;
        pt0 = pt1;
    }

    return pt1;
}

std::function<double(const Vector2D&)> SemiLagrangian2::GetScalarSamplerFunc(
    const ScalarGrid2& input) const
{
    return input.Sampler();
}

std::function<Vector2D(const Vector2D&)> SemiLagrangian2::GetVectorSamplerFunc(
    const CollocatedVectorGrid2& input) const
{
    return input.Sampler();
}

std::function<Vector2D(const Vector2D&)> SemiLagrangian2::GetVectorSamplerFunc(
    const FaceCenteredGrid2& input) const
{
    return input.Sampler();
}
}  // namespace CubbyFlow