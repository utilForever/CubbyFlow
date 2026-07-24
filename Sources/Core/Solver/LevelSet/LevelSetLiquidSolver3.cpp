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
#include <Core/Solver/LevelSet/ENOLevelSetSolver3.hpp>
#include <Core/Solver/LevelSet/FMMLevelSetSolver3.hpp>
#include <Core/Solver/LevelSet/LevelSetLiquidSolver3.hpp>
#include <Core/Utils/LevelSetUtils.hpp>
#include <Core/Utils/Logging.hpp>
#include <Core/Utils/Timer.hpp>

namespace CubbyFlow
{
LevelSetLiquidSolver3::LevelSetLiquidSolver3()
    : LevelSetLiquidSolver3{ { 1, 1, 1 }, { 1, 1, 1 }, { 0, 0, 0 } }
{
    // Do nothing
}

LevelSetLiquidSolver3::LevelSetLiquidSolver3(const Vector3UZ& resolution,
                                             const Vector3D& gridSpacing,
                                             const Vector3D& gridOrigin)
    : GridFluidSolver3{ resolution, gridSpacing, gridOrigin }
{
    GridSystemData3Ptr grids = GetGridSystemData();
    m_signedDistanceFieldId = grids->AddAdvectableScalarData(
        std::make_shared<CellCenteredScalarGrid3::Builder>(),
        std::numeric_limits<double>::max());
    m_levelSetSolver = std::make_shared<ENOLevelSetSolver3>();
}

ScalarGrid3Ptr LevelSetLiquidSolver3::GetSignedDistanceField() const
{
    return GetGridSystemData()->AdvectableScalarDataAt(m_signedDistanceFieldId);
}

LevelSetSolver3Ptr LevelSetLiquidSolver3::GetLevelSetSolver() const
{
    return m_levelSetSolver;
}

void LevelSetLiquidSolver3::SetLevelSetSolver(
    const LevelSetSolver3Ptr& newSolver)
{
    m_levelSetSolver = newSolver;
}

void LevelSetLiquidSolver3::SetMinReinitializeDistance(double distance)
{
    m_minReinitializeDistance = distance;
}

void LevelSetLiquidSolver3::SetIsGlobalCompensationEnabled(bool isEnabled)
{
    m_isGlobalCompensationEnabled = isEnabled;
}

double LevelSetLiquidSolver3::ComputeVolume() const
{
    ScalarGrid3Ptr sdf = GetSignedDistanceField();
    const Vector3D gridSpacing = sdf->GridSpacing();
    const double cellVolume = gridSpacing.x * gridSpacing.y * gridSpacing.z;
    const double h = gridSpacing.Max();

    double volume = 0.0;
    sdf->ForEachDataPointIndex([&](size_t i, size_t j, size_t k) {
        volume += 1.0 - SmearedHeavisideSDF((*sdf)(i, j, k) / h);
    });
    volume *= cellVolume;

    return volume;
}

void LevelSetLiquidSolver3::OnBeginAdvanceTimeStep(double timeIntervalInSeconds)
{
    UNUSED_VARIABLE(timeIntervalInSeconds);

    // Measure current volume
    m_lastKnownVolume = ComputeVolume();

    CUBBYFLOW_INFO << "Current volume: " << m_lastKnownVolume;
}

void LevelSetLiquidSolver3::OnEndAdvanceTimeStep(double timeIntervalInSeconds)
{
    const double currentCFL = GetCFL(timeIntervalInSeconds);

    const Timer timer;
    Reinitialize(currentCFL);
    CUBBYFLOW_INFO << "reinitializing level set field took "
                   << timer.DurationInSeconds() << " seconds";

    // Measure current volume
    double currentVol = ComputeVolume();
    const double volDiff = currentVol - m_lastKnownVolume;

    CUBBYFLOW_INFO << "Current volume: " << currentVol << " "
                   << "Volume diff: " << volDiff;

    if (m_isGlobalCompensationEnabled)
    {
        AddVolume(-volDiff);

        currentVol = ComputeVolume();
        CUBBYFLOW_INFO << "Volume after global compensation: " << currentVol;
    }
}

void LevelSetLiquidSolver3::ComputeAdvection(double timeIntervalInSeconds)
{
    const double currentCFL = GetCFL(timeIntervalInSeconds);

    const Timer timer;
    ExtrapolateVelocityToAir(currentCFL);
    CUBBYFLOW_INFO << "velocity extrapolation took "
                   << timer.DurationInSeconds() << " seconds";

    GridFluidSolver3::ComputeAdvection(timeIntervalInSeconds);
}

ScalarField3Ptr LevelSetLiquidSolver3::GetFluidSDF() const
{
    return GetSignedDistanceField();
}

void LevelSetLiquidSolver3::Reinitialize(double currentCfl)
{
    if (m_levelSetSolver != nullptr)
    {
        const ScalarGrid3Ptr sdf = GetSignedDistanceField();
        const std::shared_ptr<ScalarGrid3> sdf0 = sdf->Clone();

        const Vector3D gridSpacing = sdf->GridSpacing();
        const double h = gridSpacing.Max();
        const double maxReinitDist =
            std::max(2.0 * currentCfl, m_minReinitializeDistance) * h;

        CUBBYFLOW_INFO << "Max reinitialize distance: " << maxReinitDist;

        m_levelSetSolver->Reinitialize(*sdf0, maxReinitDist, sdf.get());
        ExtrapolateIntoCollider(sdf.get());
    }
}

void LevelSetLiquidSolver3::ExtrapolateVelocityToAir(double currentCFL)
{
    ScalarGrid3Ptr sdf = GetSignedDistanceField();
    FaceCenteredGrid3Ptr vel = GetGridSystemData()->Velocity();

    ArrayView3<double> u = vel->UView();
    ArrayView3<double> v = vel->VView();
    ArrayView3<double> w = vel->WView();

    GridDataPositionFunc<3> uPos = vel->UPosition();
    GridDataPositionFunc<3> vPos = vel->VPosition();
    GridDataPositionFunc<3> wPos = vel->WPosition();

    Array3<char> uMarker{ u.Size() };
    Array3<char> vMarker{ v.Size() };
    Array3<char> wMarker{ w.Size() };

    ParallelForEachIndex(uMarker.Size(), [&](size_t i, size_t j, size_t k) {
        if (IsInsideSDF(sdf->Sample(uPos(i, j, k))))
        {
            uMarker(i, j, k) = 1;
        }
        else
        {
            uMarker(i, j, k) = 0;
            u(i, j, k) = 0.0;
        }
    });

    ParallelForEachIndex(vMarker.Size(), [&](size_t i, size_t j, size_t k) {
        if (IsInsideSDF(sdf->Sample(vPos(i, j, k))))
        {
            vMarker(i, j, k) = 1;
        }
        else
        {
            vMarker(i, j, k) = 0;
            v(i, j, k) = 0.0;
        }
    });

    ParallelForEachIndex(wMarker.Size(), [&](size_t i, size_t j, size_t k) {
        if (IsInsideSDF(sdf->Sample(wPos(i, j, k))))
        {
            wMarker(i, j, k) = 1;
        }
        else
        {
            wMarker(i, j, k) = 0;
            w(i, j, k) = 0.0;
        }
    });

    const Vector3D gridSpacing = sdf->GridSpacing();
    const double h = gridSpacing.Max();
    const double maxDist =
        std::max(2.0 * currentCFL, m_minReinitializeDistance) * h;

    CUBBYFLOW_INFO << "Max velocity extrapolation distance: " << maxDist;

    FMMLevelSetSolver3 fmmSolver;
    fmmSolver.Extrapolate(*vel, *sdf, maxDist, vel.get());

    ApplyBoundaryCondition();
}

void LevelSetLiquidSolver3::AddVolume(double volDiff)
{
    ScalarGrid3Ptr sdf = GetSignedDistanceField();
    const Vector3D gridSpacing = sdf->GridSpacing();
    const double cellVolume = gridSpacing.x * gridSpacing.y * gridSpacing.z;
    const double h = gridSpacing.Max();

    double volume0 = 0.0;
    double volume1 = 0.0;
    sdf->ForEachDataPointIndex([&](size_t i, size_t j, size_t k) {
        volume0 += 1.0 - SmearedHeavisideSDF((*sdf)(i, j, k) / h);
        volume1 += 1.0 - SmearedHeavisideSDF((*sdf)(i, j, k) / h + 1.0);
    });
    volume0 *= cellVolume;
    volume1 *= cellVolume;

    const double dVdh = (volume1 - volume0) / h;

    if (std::abs(dVdh) > 0.0)
    {
        double dist = volDiff / dVdh;

        sdf->ParallelForEachDataPointIndex(
            [&](size_t i, size_t j, size_t k) { (*sdf)(i, j, k) += dist; });
    }
}

LevelSetLiquidSolver3::Builder LevelSetLiquidSolver3::GetBuilder()
{
    return Builder{};
}

LevelSetLiquidSolver3 LevelSetLiquidSolver3::Builder::Build() const
{
    return LevelSetLiquidSolver3{ m_resolution, GetGridSpacing(),
                                  m_gridOrigin };
}

LevelSetLiquidSolver3Ptr LevelSetLiquidSolver3::Builder::MakeShared() const
{
    return std::make_shared<LevelSetLiquidSolver3>(
        m_resolution, GetGridSpacing(), m_gridOrigin);
}
}  // namespace CubbyFlow