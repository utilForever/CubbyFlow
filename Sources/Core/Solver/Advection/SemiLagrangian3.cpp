// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Solver/Advection/SemiLagrangian3.hpp>

namespace CubbyFlow
{
void SemiLagrangian3::Advect(const ScalarGrid3& input, const VectorField3& flow,
                             double dt, ScalarGrid3* output,
                             const ScalarField3& boundarySDF)
{
    std::function<double(const Vector3D&)> inputSamplerFunc =
        GetScalarSamplerFunc(input);
    double h = std::min(output->GridSpacing().x, output->GridSpacing().y);

    GridDataPositionFunc<3> inputDataPos = input.DataPosition();
    GridDataPositionFunc<3> outputDataPos = output->DataPosition();
    ArrayView<double, 3> outputDataAcc = output->DataView();

    output->ParallelForEachDataPointIndex(
        [&boundarySDF, &inputDataPos, this, &flow, &dt, &h, &outputDataPos,
         &outputDataAcc, &inputSamplerFunc](size_t i, size_t j, size_t k) {
            if (boundarySDF.Sample(inputDataPos(i, j, k)) > 0.0)
            {
                const Vector3D pt =
                    BackTrace(flow, dt, h, outputDataPos(i, j, k), boundarySDF);
                outputDataAcc(i, j, k) = inputSamplerFunc(pt);
            }
        });
}

void SemiLagrangian3::Advect(const CollocatedVectorGrid3& input,
                             const VectorField3& flow, double dt,
                             CollocatedVectorGrid3* output,
                             const ScalarField3& boundarySDF)
{
    std::function<Vector3D(const Vector3D&)> inputSamplerFunc =
        GetVectorSamplerFunc(input);
    double h = std::min(output->GridSpacing().x, output->GridSpacing().y);

    GridDataPositionFunc<3> inputDataPos = input.DataPosition();
    GridDataPositionFunc<3> outputDataPos = output->DataPosition();
    ArrayView<Vector<double, 3>, 3> outputDataAcc = output->DataView();

    output->ParallelForEachDataPointIndex(
        [&boundarySDF, &inputDataPos, this, &flow, &dt, &h, &outputDataPos,
         &outputDataAcc, &inputSamplerFunc](size_t i, size_t j, size_t k) {
            if (boundarySDF.Sample(inputDataPos(i, j, k)) > 0.0)
            {
                const Vector3D pt =
                    BackTrace(flow, dt, h, outputDataPos(i, j, k), boundarySDF);
                outputDataAcc(i, j, k) = inputSamplerFunc(pt);
            }
        });
}

void SemiLagrangian3::Advect(const FaceCenteredGrid3& input,
                             const VectorField3& flow, double dt,
                             FaceCenteredGrid3* output,
                             const ScalarField3& boundarySDF)
{
    std::function<Vector3D(const Vector3D&)> inputSamplerFunc =
        GetVectorSamplerFunc(input);
    double h = std::min(output->GridSpacing().x, output->GridSpacing().y);

    auto uSourceDataPos = input.UPosition();
    auto uTargetDataPos = output->UPosition();
    ArrayView<double, 3> uTargetDataAcc = output->UView();

    output->ParallelForEachUIndex([&boundarySDF, &uSourceDataPos, this, &flow,
                                   &dt, &h, &uTargetDataPos, &uTargetDataAcc,
                                   &inputSamplerFunc](const Vector3UZ& idx) {
        if (boundarySDF.Sample(uSourceDataPos(idx)) > 0.0)
        {
            const Vector3D pt =
                BackTrace(flow, dt, h, uTargetDataPos(idx), boundarySDF);
            uTargetDataAcc(idx) = inputSamplerFunc(pt).x;
        }
    });

    auto vSourceDataPos = input.VPosition();
    auto vTargetDataPos = output->VPosition();
    ArrayView<double, 3> vTargetDataAcc = output->VView();

    output->ParallelForEachVIndex([&boundarySDF, &vSourceDataPos, this, &flow,
                                   &dt, &h, &vTargetDataPos, &vTargetDataAcc,
                                   &inputSamplerFunc](const Vector3UZ& idx) {
        if (boundarySDF.Sample(vSourceDataPos(idx)) > 0.0)
        {
            const Vector3D pt =
                BackTrace(flow, dt, h, vTargetDataPos(idx), boundarySDF);
            vTargetDataAcc(idx) = inputSamplerFunc(pt).y;
        }
    });

    auto wSourceDataPos = input.WPosition();
    auto wTargetDataPos = output->WPosition();
    ArrayView<double, 3> wTargetDataAcc = output->WView();

    output->ParallelForEachWIndex([&boundarySDF, &wSourceDataPos, this, &flow,
                                   &dt, &h, &wTargetDataPos, &wTargetDataAcc,
                                   &inputSamplerFunc](const Vector3UZ& idx) {
        if (boundarySDF.Sample(wSourceDataPos(idx)) > 0.0)
        {
            const Vector3D pt =
                BackTrace(flow, dt, h, wTargetDataPos(idx), boundarySDF);
            wTargetDataAcc(idx) = inputSamplerFunc(pt).z;
        }
    });
}

Vector3D SemiLagrangian3::BackTrace(const VectorField3& flow, double dt,
                                    double h, const Vector3D& startPt,
                                    const ScalarField3& boundarySDF) const
{
    double remainingT = dt;
    Vector3D pt0 = startPt;
    Vector3D pt1 = startPt;

    while (remainingT > std::numeric_limits<double>::epsilon())
    {
        // Adaptive time-stepping
        Vector3D vel0 = flow.Sample(pt0);
        const double numSubSteps =
            std::max(std::ceil(vel0.Length() * remainingT / h), 1.0);
        dt = remainingT / numSubSteps;

        // Mid-point rule
        Vector3D midPt = pt0 - 0.5 * dt * vel0;
        Vector3D midVel = flow.Sample(midPt);
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

std::function<double(const Vector3D&)> SemiLagrangian3::GetScalarSamplerFunc(
    const ScalarGrid3& input) const
{
    return input.Sampler();
}

std::function<Vector3D(const Vector3D&)> SemiLagrangian3::GetVectorSamplerFunc(
    const CollocatedVectorGrid3& input) const
{
    return input.Sampler();
}

std::function<Vector3D(const Vector3D&)> SemiLagrangian3::GetVectorSamplerFunc(
    const FaceCenteredGrid3& input) const
{
    return input.Sampler();
}
}  // namespace CubbyFlow