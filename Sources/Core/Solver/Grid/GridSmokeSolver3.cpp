// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Grid/CellCenteredScalarGrid.hpp>
#include <Core/Solver/Grid/GridSmokeSolver3.hpp>

namespace CubbyFlow
{
GridSmokeSolver3::GridSmokeSolver3()
    : GridSmokeSolver3{ { 1, 1, 1 }, { 1, 1, 1 }, { 0, 0, 0 } }
{
    // Do nothing
}

GridSmokeSolver3::GridSmokeSolver3(const Vector3UZ& resolution,
                                   const Vector3D& gridSpacing,
                                   const Vector3D& gridOrigin)
    : GridFluidSolver3{ resolution, gridSpacing, gridOrigin }
{
    GridSystemData3Ptr grids = GetGridSystemData();
    m_smokeDensityDataID = grids->AddAdvectableScalarData(
        std::make_shared<CellCenteredScalarGrid3::Builder>(), 0.0);
    m_temperatureDataID = grids->AddAdvectableScalarData(
        std::make_shared<CellCenteredScalarGrid3::Builder>(), 0.0);
}

double GridSmokeSolver3::GetSmokeDiffusionCoefficient() const
{
    return m_smokeDiffusionCoefficient;
}

void GridSmokeSolver3::SetSmokeDiffusionCoefficient(double newValue)
{
    m_smokeDiffusionCoefficient = std::max(newValue, 0.0);
}

double GridSmokeSolver3::GetTemperatureDiffusionCoefficient() const
{
    return m_temperatureDiffusionCoefficient;
}

void GridSmokeSolver3::SetTemperatureDiffusionCoefficient(double newValue)
{
    m_temperatureDiffusionCoefficient = std::max(newValue, 0.0);
}

double GridSmokeSolver3::GetBuoyancySmokeDensityFactor() const
{
    return m_buoyancySmokeDensityFactor;
}

void GridSmokeSolver3::SetBuoyancySmokeDensityFactor(double newValue)
{
    m_buoyancySmokeDensityFactor = newValue;
}

double GridSmokeSolver3::GetBuoyancyTemperatureFactor() const
{
    return m_buoyancyTemperatureFactor;
}

void GridSmokeSolver3::SetBuoyancyTemperatureFactor(double newValue)
{
    m_buoyancyTemperatureFactor = newValue;
}

double GridSmokeSolver3::GetSmokeDecayFactor() const
{
    return m_smokeDecayFactor;
}

void GridSmokeSolver3::SetSmokeDecayFactor(double newValue)
{
    m_smokeDecayFactor = std::clamp(newValue, 0.0, 1.0);
}

double GridSmokeSolver3::GetTemperatureDecayFactor() const
{
    return m_temperatureDecayFactor;
}

void GridSmokeSolver3::SetTemperatureDecayFactor(double newValue)
{
    m_temperatureDecayFactor = std::clamp(newValue, 0.0, 1.0);
}

ScalarGrid3Ptr GridSmokeSolver3::GetSmokeDensity() const
{
    return GetGridSystemData()->AdvectableScalarDataAt(m_smokeDensityDataID);
}

ScalarGrid3Ptr GridSmokeSolver3::GetTemperature() const
{
    return GetGridSystemData()->AdvectableScalarDataAt(m_temperatureDataID);
}

void GridSmokeSolver3::OnEndAdvanceTimeStep(double timeIntervalInSeconds)
{
    ComputeDiffusion(timeIntervalInSeconds);
}

void GridSmokeSolver3::ComputeExternalForces(double timeIntervalInSeconds)
{
    ComputeBuoyancyForce(timeIntervalInSeconds);
}

void GridSmokeSolver3::ComputeDiffusion(double timeIntervalInSeconds)
{
    if (GetDiffusionSolver() != nullptr)
    {
        if (m_smokeDiffusionCoefficient >
            std::numeric_limits<double>::epsilon())
        {
            const ScalarGrid3Ptr den = GetSmokeDensity();
            const std::shared_ptr<CellCenteredScalarGrid3> den0 =
                std::dynamic_pointer_cast<CellCenteredScalarGrid3>(
                    den->Clone());

            GetDiffusionSolver()->Solve(*den0, m_smokeDiffusionCoefficient,
                                        timeIntervalInSeconds, den.get(),
                                        *GetColliderSDF());
            ExtrapolateIntoCollider(den.get());
        }

        if (m_temperatureDiffusionCoefficient >
            std::numeric_limits<double>::epsilon())
        {
            const ScalarGrid3Ptr temp = GetTemperature();
            const std::shared_ptr<CellCenteredScalarGrid3> temp0 =
                std::dynamic_pointer_cast<CellCenteredScalarGrid3>(
                    temp->Clone());

            GetDiffusionSolver()->Solve(
                *temp0, m_temperatureDiffusionCoefficient,
                timeIntervalInSeconds, temp.get(), *GetColliderSDF());
            ExtrapolateIntoCollider(temp.get());
        }
    }

    ScalarGrid3Ptr den = GetSmokeDensity();
    den->ParallelForEachDataPointIndex([&](size_t i, size_t j, size_t k) {
        (*den)(i, j, k) *= 1.0 - m_smokeDecayFactor;
    });
    ScalarGrid3Ptr temp = GetTemperature();
    temp->ParallelForEachDataPointIndex([&](size_t i, size_t j, size_t k) {
        (*temp)(i, j, k) *= 1.0 - m_temperatureDecayFactor;
    });
}

void GridSmokeSolver3::ComputeBuoyancyForce(double timeIntervalInSeconds)
{
    const GridSystemData3Ptr grids = GetGridSystemData();
    FaceCenteredGrid3Ptr vel = grids->Velocity();

    Vector3D up{ 0, 0, 1 };
    if (GetGravity().LengthSquared() > std::numeric_limits<double>::epsilon())
    {
        up = -GetGravity().Normalized();
    }

    if (std::abs(m_buoyancySmokeDensityFactor) >
            std::numeric_limits<double>::epsilon() ||
        std::abs(m_buoyancyTemperatureFactor) >
            std::numeric_limits<double>::epsilon())
    {
        ScalarGrid3Ptr den = GetSmokeDensity();
        ScalarGrid3Ptr temp = GetTemperature();

        double tAmb = 0.0;
        temp->ForEachCellIndex(
            [&](size_t i, size_t j, size_t k) { tAmb += (*temp)(i, j, k); });

        tAmb /= static_cast<double>(
            temp->Resolution().x * temp->Resolution().y * temp->Resolution().z);

        ArrayView3<double> u = vel->UView();
        ArrayView3<double> v = vel->VView();
        ArrayView3<double> w = vel->WView();
        auto uPos = vel->UPosition();
        auto vPos = vel->VPosition();
        auto wPos = vel->WPosition();

        if (std::abs(up.x) > std::numeric_limits<double>::epsilon())
        {
            vel->ParallelForEachUIndex([&](const Vector3UZ& idx) {
                const Vector3D pt = uPos(idx);
                const double fBuoy =
                    m_buoyancySmokeDensityFactor * den->Sample(pt) +
                    m_buoyancyTemperatureFactor * (temp->Sample(pt) - tAmb);
                u(idx) += timeIntervalInSeconds * fBuoy * up.x;
            });
        }

        if (std::abs(up.y) > std::numeric_limits<double>::epsilon())
        {
            vel->ParallelForEachVIndex([&](const Vector3UZ& idx) {
                const Vector3D pt = vPos(idx);
                const double fBuoy =
                    m_buoyancySmokeDensityFactor * den->Sample(pt) +
                    m_buoyancyTemperatureFactor * (temp->Sample(pt) - tAmb);
                v(idx) += timeIntervalInSeconds * fBuoy * up.y;
            });
        }

        if (std::abs(up.z) > std::numeric_limits<double>::epsilon())
        {
            vel->ParallelForEachWIndex([&](const Vector3UZ& idx) {
                const Vector3D pt = wPos(idx);
                const double fBuoy =
                    m_buoyancySmokeDensityFactor * den->Sample(pt) +
                    m_buoyancyTemperatureFactor * (temp->Sample(pt) - tAmb);
                w(idx) += timeIntervalInSeconds * fBuoy * up.z;
            });
        }

        ApplyBoundaryCondition();
    }
}

GridSmokeSolver3::Builder GridSmokeSolver3::GetBuilder()
{
    return Builder{};
}

GridSmokeSolver3 GridSmokeSolver3::Builder::Build() const
{
    return GridSmokeSolver3{ m_resolution, GetGridSpacing(), m_gridOrigin };
}

GridSmokeSolver3Ptr GridSmokeSolver3::Builder::MakeShared() const
{
    return std::make_shared<GridSmokeSolver3>(m_resolution, GetGridSpacing(),
                                              m_gridOrigin);
}
}  // namespace CubbyFlow