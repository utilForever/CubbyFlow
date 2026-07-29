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
#include <Core/Solver/Grid/GridSmokeSolver2.hpp>

namespace CubbyFlow
{
GridSmokeSolver2::GridSmokeSolver2()
    : GridSmokeSolver2{ { 1, 1 }, { 1, 1 }, { 0, 0 } }
{
    // Do nothing
}

GridSmokeSolver2::GridSmokeSolver2(const Vector2UZ& resolution,
                                   const Vector2D& gridSpacing,
                                   const Vector2D& gridOrigin)
    : GridFluidSolver2{ resolution, gridSpacing, gridOrigin }
{
    GridSystemData2Ptr grids = GetGridSystemData();
    m_smokeDensityDataID = grids->AddAdvectableScalarData(
        std::make_shared<CellCenteredScalarGrid2::Builder>(), 0.0);
    m_temperatureDataID = grids->AddAdvectableScalarData(
        std::make_shared<CellCenteredScalarGrid2::Builder>(), 0.0);
}

double GridSmokeSolver2::GetSmokeDiffusionCoefficient() const
{
    return m_smokeDiffusionCoefficient;
}

void GridSmokeSolver2::SetSmokeDiffusionCoefficient(double newValue)
{
    m_smokeDiffusionCoefficient = std::max(newValue, 0.0);
}

double GridSmokeSolver2::GetTemperatureDiffusionCoefficient() const
{
    return m_temperatureDiffusionCoefficient;
}

void GridSmokeSolver2::SetTemperatureDiffusionCoefficient(double newValue)
{
    m_temperatureDiffusionCoefficient = std::max(newValue, 0.0);
}

double GridSmokeSolver2::GetBuoyancySmokeDensityFactor() const
{
    return m_buoyancySmokeDensityFactor;
}

void GridSmokeSolver2::SetBuoyancySmokeDensityFactor(double newValue)
{
    m_buoyancySmokeDensityFactor = newValue;
}

double GridSmokeSolver2::GetBuoyancyTemperatureFactor() const
{
    return m_buoyancyTemperatureFactor;
}

void GridSmokeSolver2::SetBuoyancyTemperatureFactor(double newValue)
{
    m_buoyancyTemperatureFactor = newValue;
}

double GridSmokeSolver2::GetSmokeDecayFactor() const
{
    return m_smokeDecayFactor;
}

void GridSmokeSolver2::SetSmokeDecayFactor(double newValue)
{
    m_smokeDecayFactor = std::clamp(newValue, 0.0, 1.0);
}

double GridSmokeSolver2::GetTemperatureDecayFactor() const
{
    return m_temperatureDecayFactor;
}

void GridSmokeSolver2::SetTemperatureDecayFactor(double newValue)
{
    m_temperatureDecayFactor = std::clamp(newValue, 0.0, 1.0);
}

ScalarGrid2Ptr GridSmokeSolver2::GetSmokeDensity() const
{
    return GetGridSystemData()->AdvectableScalarDataAt(m_smokeDensityDataID);
}

ScalarGrid2Ptr GridSmokeSolver2::GetTemperature() const
{
    return GetGridSystemData()->AdvectableScalarDataAt(m_temperatureDataID);
}

void GridSmokeSolver2::OnEndAdvanceTimeStep(double timeIntervalInSeconds)
{
    ComputeDiffusion(timeIntervalInSeconds);
}

void GridSmokeSolver2::ComputeExternalForces(double timeIntervalInSeconds)
{
    ComputeBuoyancyForce(timeIntervalInSeconds);
}

void GridSmokeSolver2::ComputeDiffusion(double timeIntervalInSeconds)
{
    if (GetDiffusionSolver() != nullptr)
    {
        if (m_smokeDiffusionCoefficient >
            std::numeric_limits<double>::epsilon())
        {
            const ScalarGrid2Ptr den = GetSmokeDensity();
            const std::shared_ptr<CellCenteredScalarGrid2> den0 =
                std::dynamic_pointer_cast<CellCenteredScalarGrid2>(
                    den->Clone());

            GetDiffusionSolver()->Solve(*den0, m_smokeDiffusionCoefficient,
                                        timeIntervalInSeconds, den.get(),
                                        *GetColliderSDF());
            ExtrapolateIntoCollider(den.get());
        }

        if (m_temperatureDiffusionCoefficient >
            std::numeric_limits<double>::epsilon())
        {
            const ScalarGrid2Ptr temp = GetTemperature();
            const std::shared_ptr<CellCenteredScalarGrid2> temp0 =
                std::dynamic_pointer_cast<CellCenteredScalarGrid2>(
                    temp->Clone());

            GetDiffusionSolver()->Solve(
                *temp0, m_temperatureDiffusionCoefficient,
                timeIntervalInSeconds, temp.get(), *GetColliderSDF());
            ExtrapolateIntoCollider(temp.get());
        }
    }

    ScalarGrid2Ptr den = GetSmokeDensity();
    den->ParallelForEachDataPointIndex([&den, this](size_t i, size_t j) {
        (*den)(i, j) *= 1.0 - m_smokeDecayFactor;
    });
    ScalarGrid2Ptr temp = GetTemperature();
    temp->ParallelForEachDataPointIndex([&temp, this](size_t i, size_t j) {
        (*temp)(i, j) *= 1.0 - m_temperatureDecayFactor;
    });
}

void GridSmokeSolver2::ComputeBuoyancyForce(double timeIntervalInSeconds)
{
    const GridSystemData2Ptr grids = GetGridSystemData();
    FaceCenteredGrid2Ptr vel = grids->Velocity();

    Vector2D up{ 0, 1 };
    if (GetGravity().LengthSquared() > std::numeric_limits<double>::epsilon())
    {
        up = -GetGravity().Normalized();
    }

    if (std::abs(m_buoyancySmokeDensityFactor) >
            std::numeric_limits<double>::epsilon() ||
        std::abs(m_buoyancyTemperatureFactor) >
            std::numeric_limits<double>::epsilon())
    {
        ScalarGrid2Ptr den = GetSmokeDensity();
        ScalarGrid2Ptr temp = GetTemperature();

        double tAmb = 0.0;
        temp->ForEachCellIndex(
            [&tAmb, &temp](size_t i, size_t j) { tAmb += (*temp)(i, j); });

        tAmb /=
            static_cast<double>(temp->Resolution().x * temp->Resolution().y);

        ArrayView2<double> u = vel->UView();
        ArrayView2<double> v = vel->VView();
        auto uPos = vel->UPosition();
        auto vPos = vel->VPosition();

        if (std::abs(up.x) > std::numeric_limits<double>::epsilon())
        {
            vel->ParallelForEachUIndex([&uPos, this, &den, &temp, &tAmb, &u,
                                        &timeIntervalInSeconds,
                                        &up](const Vector2UZ& idx) {
                const Vector2D pt = uPos(idx);
                const double fBuoy =
                    m_buoyancySmokeDensityFactor * den->Sample(pt) +
                    m_buoyancyTemperatureFactor * (temp->Sample(pt) - tAmb);
                u(idx) += timeIntervalInSeconds * fBuoy * up.x;
            });
        }

        if (std::abs(up.y) > std::numeric_limits<double>::epsilon())
        {
            vel->ParallelForEachVIndex([&vPos, this, &den, &temp, &tAmb, &v,
                                        &timeIntervalInSeconds,
                                        &up](const Vector2UZ& idx) {
                const Vector2D pt = vPos(idx);
                const double fBuoy =
                    m_buoyancySmokeDensityFactor * den->Sample(pt) +
                    m_buoyancyTemperatureFactor * (temp->Sample(pt) - tAmb);
                v(idx) += timeIntervalInSeconds * fBuoy * up.y;
            });
        }

        ApplyBoundaryCondition();
    }
}

GridSmokeSolver2::Builder GridSmokeSolver2::GetBuilder()
{
    return Builder{};
}

GridSmokeSolver2 GridSmokeSolver2::Builder::Build() const
{
    return GridSmokeSolver2{ m_resolution, GetGridSpacing(), m_gridOrigin };
}

GridSmokeSolver2Ptr GridSmokeSolver2::Builder::MakeShared() const
{
    return std::make_shared<GridSmokeSolver2>(m_resolution, GetGridSpacing(),
                                              m_gridOrigin);
}
}  // namespace CubbyFlow