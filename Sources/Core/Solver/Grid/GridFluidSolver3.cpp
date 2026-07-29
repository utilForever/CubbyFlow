// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Array/ArrayUtils.hpp>
#include <Core/Solver/Advection/CubicSemiLagrangian3.hpp>
#include <Core/Solver/Grid/GridBackwardEulerDiffusionSolver3.hpp>
#include <Core/Solver/Grid/GridFluidSolver3.hpp>
#include <Core/Solver/Grid/GridFractionalSinglePhasePressureSolver3.hpp>
#include <Core/Utils/LevelSetUtils.hpp>
#include <Core/Utils/Logging.hpp>
#include <Core/Utils/Timer.hpp>

namespace CubbyFlow
{
GridFluidSolver3::GridFluidSolver3()
    : GridFluidSolver3{ { 1, 1, 1 }, { 1, 1, 1 }, { 0, 0, 0 } }
{
    // Do nothing
}

GridFluidSolver3::GridFluidSolver3(const Vector3UZ& resolution,
                                   const Vector3D& gridSpacing,
                                   const Vector3D& gridOrigin)
{
    m_grids = std::make_shared<GridSystemData3>();
    m_grids->Resize(resolution, gridSpacing, gridOrigin);

    SetAdvectionSolver(std::make_shared<CubicSemiLagrangian3>());
    SetDiffusionSolver(std::make_shared<GridBackwardEulerDiffusionSolver3>());
    SetPressureSolver(
        std::make_shared<GridFractionalSinglePhasePressureSolver3>());
    SetIsUsingFixedSubTimeSteps(false);
}

const Vector3D& GridFluidSolver3::GetGravity() const
{
    return m_gravity;
}

void GridFluidSolver3::SetGravity(const Vector3D& newGravity)
{
    m_gravity = newGravity;
}

double GridFluidSolver3::GetViscosityCoefficient() const
{
    return m_viscosityCoefficient;
}

void GridFluidSolver3::SetViscosityCoefficient(double newValue)
{
    m_viscosityCoefficient = std::max(newValue, 0.0);
}

double GridFluidSolver3::GetCFL(double timeIntervalInSeconds) const
{
    FaceCenteredGrid3Ptr vel = m_grids->Velocity();
    double maxVel = 0.0;

    vel->ForEachCellIndex([&vel, &timeIntervalInSeconds, this, &maxVel](
                              size_t i, size_t j, size_t k) {
        const Vector3D v =
            vel->ValueAtCellCenter(i, j, k) + timeIntervalInSeconds * m_gravity;
        maxVel = std::max(maxVel, v.Max());
    });

    const Vector3D gridSpacing = m_grids->GridSpacing();
    const double minGridSize = gridSpacing.Min();

    return maxVel * timeIntervalInSeconds / minGridSize;
}

double GridFluidSolver3::GetMaxCFL() const
{
    return m_maxCFL;
}

void GridFluidSolver3::SetMaxCFL(double newCFL)
{
    m_maxCFL = std::max(newCFL, std::numeric_limits<double>::epsilon());
}

bool GridFluidSolver3::GetUseCompressedLinearSystem() const
{
    return m_useCompressedLinearSys;
}

void GridFluidSolver3::SetUseCompressedLinearSystem(bool isOn)
{
    m_useCompressedLinearSys = isOn;
}

const AdvectionSolver3Ptr& GridFluidSolver3::GetAdvectionSolver() const
{
    return m_advectionSolver;
}

void GridFluidSolver3::SetAdvectionSolver(const AdvectionSolver3Ptr& newSolver)
{
    m_advectionSolver = newSolver;
}

const GridDiffusionSolver3Ptr& GridFluidSolver3::GetDiffusionSolver() const
{
    return m_diffusionSolver;
}

void GridFluidSolver3::SetDiffusionSolver(
    const GridDiffusionSolver3Ptr& newSolver)
{
    m_diffusionSolver = newSolver;
}

const GridPressureSolver3Ptr& GridFluidSolver3::GetPressureSolver() const
{
    return m_pressureSolver;
}

void GridFluidSolver3::SetPressureSolver(
    const GridPressureSolver3Ptr& newSolver)
{
    m_pressureSolver = newSolver;
    if (m_pressureSolver != nullptr)
    {
        m_boundaryConditionSolver =
            m_pressureSolver->SuggestedBoundaryConditionSolver();

        // Apply domain boundary flag
        m_boundaryConditionSolver->SetClosedDomainBoundaryFlag(
            m_closedDomainBoundaryFlag);
    }
}

int GridFluidSolver3::GetClosedDomainBoundaryFlag() const
{
    return m_closedDomainBoundaryFlag;
}

void GridFluidSolver3::SetClosedDomainBoundaryFlag(int flag)
{
    m_closedDomainBoundaryFlag = flag;
    m_boundaryConditionSolver->SetClosedDomainBoundaryFlag(
        m_closedDomainBoundaryFlag);
}

const GridSystemData3Ptr& GridFluidSolver3::GetGridSystemData() const
{
    return m_grids;
}

void GridFluidSolver3::ResizeGrid(const Vector3UZ& newSize,
                                  const Vector3D& newGridSpacing,
                                  const Vector3D& newGridOrigin) const
{
    m_grids->Resize(newSize, newGridSpacing, newGridOrigin);
}

Vector3UZ GridFluidSolver3::GetResolution() const
{
    return m_grids->Resolution();
}

Vector3D GridFluidSolver3::GetGridSpacing() const
{
    return m_grids->GridSpacing();
}

Vector3D GridFluidSolver3::GetGridOrigin() const
{
    return m_grids->Origin();
}

const FaceCenteredGrid3Ptr& GridFluidSolver3::GetVelocity() const
{
    return m_grids->Velocity();
}

const Collider3Ptr& GridFluidSolver3::GetCollider() const
{
    return m_collider;
}

void GridFluidSolver3::SetCollider(const Collider3Ptr& newCollider)
{
    m_collider = newCollider;
}

const GridEmitter3Ptr& GridFluidSolver3::GetEmitter() const
{
    return m_emitter;
}

void GridFluidSolver3::SetEmitter(const GridEmitter3Ptr& newEmitter)
{
    m_emitter = newEmitter;
}

void GridFluidSolver3::OnInitialize()
{
    // When initializing the solver, update the collider and emitter state as
    // well since they also affects the initial condition of the simulation.
    Timer timer;
    UpdateCollider(0.0);
    CUBBYFLOW_INFO << "Update collider took " << timer.DurationInSeconds()
                   << " seconds";

    timer.Reset();
    UpdateEmitter(0.0);
    CUBBYFLOW_INFO << "Update emitter took " << timer.DurationInSeconds()
                   << " seconds";
}

void GridFluidSolver3::OnAdvanceTimeStep(double timeIntervalInSeconds)
{
    // The minimum grid resolution is 1x1.
    if (m_grids->Resolution().x == 0 || m_grids->Resolution().y == 0 ||
        m_grids->Resolution().z == 0)
    {
        CUBBYFLOW_WARN << "Empty grid. Skipping the simulation.";
        return;
    }

    BeginAdvanceTimeStep(timeIntervalInSeconds);

    Timer timer;
    ComputeExternalForces(timeIntervalInSeconds);
    CUBBYFLOW_INFO << "Computing external force took "
                   << timer.DurationInSeconds() << " seconds";

    timer.Reset();
    ComputeViscosity(timeIntervalInSeconds);
    CUBBYFLOW_INFO << "Computing viscosity force took "
                   << timer.DurationInSeconds() << " seconds";

    timer.Reset();
    ComputePressure(timeIntervalInSeconds);
    CUBBYFLOW_INFO << "Computing pressure force took "
                   << timer.DurationInSeconds() << " seconds";

    timer.Reset();
    ComputeAdvection(timeIntervalInSeconds);
    CUBBYFLOW_INFO << "Computing advection force took "
                   << timer.DurationInSeconds() << " seconds";

    EndAdvanceTimeStep(timeIntervalInSeconds);
}

unsigned int GridFluidSolver3::GetNumberOfSubTimeSteps(
    double timeIntervalInSeconds) const
{
    const double currentCFL = GetCFL(timeIntervalInSeconds);
    return static_cast<unsigned int>(
        std::max(std::ceil(currentCFL / m_maxCFL), 1.0));
}

void GridFluidSolver3::OnBeginAdvanceTimeStep(double timeIntervalInSeconds)
{
    UNUSED_VARIABLE(timeIntervalInSeconds);
}

void GridFluidSolver3::OnEndAdvanceTimeStep(double timeIntervalInSeconds)
{
    UNUSED_VARIABLE(timeIntervalInSeconds);
}

void GridFluidSolver3::ComputeExternalForces(double timeIntervalInSeconds)
{
    ComputeGravity(timeIntervalInSeconds);
}

void GridFluidSolver3::ComputeViscosity(double timeIntervalInSeconds)
{
    if (m_diffusionSolver != nullptr &&
        m_viscosityCoefficient > std::numeric_limits<double>::epsilon())
    {
        const FaceCenteredGrid3Ptr vel = GetVelocity();
        const std::shared_ptr<FaceCenteredGrid3> vel0 =
            std::dynamic_pointer_cast<FaceCenteredGrid3>(vel->Clone());

        m_diffusionSolver->Solve(*vel0, m_viscosityCoefficient,
                                 timeIntervalInSeconds, vel.get(),
                                 *GetColliderSDF(), *GetFluidSDF());
        ApplyBoundaryCondition();
    }
}

void GridFluidSolver3::ComputePressure(double timeIntervalInSeconds)
{
    if (m_pressureSolver != nullptr)
    {
        const FaceCenteredGrid3Ptr vel = GetVelocity();
        const std::shared_ptr<FaceCenteredGrid3> vel0 =
            std::dynamic_pointer_cast<FaceCenteredGrid3>(vel->Clone());

        m_pressureSolver->Solve(*vel0, timeIntervalInSeconds, vel.get(),
                                *GetColliderSDF(), *GetColliderVelocityField(),
                                *GetFluidSDF(), m_useCompressedLinearSys);
        ApplyBoundaryCondition();
    }
}

void GridFluidSolver3::ComputeAdvection(double timeIntervalInSeconds)
{
    const FaceCenteredGrid3Ptr vel = GetVelocity();

    if (m_advectionSolver != nullptr)
    {
        // Solve advections for custom scalar fields.
        size_t n = m_grids->NumberOfAdvectableScalarData();

        for (size_t i = 0; i < n; ++i)
        {
            ScalarGrid3Ptr grid = m_grids->AdvectableScalarDataAt(i);
            std::shared_ptr<ScalarGrid3> grid0 = grid->Clone();

            m_advectionSolver->Advect(*grid0, *vel, timeIntervalInSeconds,
                                      grid.get(), *GetColliderSDF());
            ExtrapolateIntoCollider(grid.get());
        }

        // Solve advections for custom vector fields.
        n = m_grids->NumberOfAdvectableVectorData();
        const size_t velIdx = m_grids->VelocityIndex();

        for (size_t i = 0; i < n; ++i)
        {
            // Handle velocity layer separately.
            if (i == velIdx)
            {
                continue;
            }

            VectorGrid3Ptr grid = m_grids->AdvectableVectorDataAt(i);
            std::shared_ptr<VectorGrid3> grid0 = grid->Clone();

            std::shared_ptr<CollocatedVectorGrid3> collocated =
                std::dynamic_pointer_cast<CollocatedVectorGrid3>(grid);
            std::shared_ptr<CollocatedVectorGrid3> collocated0 =
                std::dynamic_pointer_cast<CollocatedVectorGrid3>(grid0);

            if (collocated != nullptr)
            {
                m_advectionSolver->Advect(*collocated0, *vel,
                                          timeIntervalInSeconds,
                                          collocated.get(), *GetColliderSDF());
                ExtrapolateIntoCollider(collocated.get());
                continue;
            }

            std::shared_ptr<FaceCenteredGrid3> faceCentered =
                std::dynamic_pointer_cast<FaceCenteredGrid3>(grid);
            std::shared_ptr<FaceCenteredGrid3> faceCentered0 =
                std::dynamic_pointer_cast<FaceCenteredGrid3>(grid0);

            if (faceCentered != nullptr && faceCentered0 != nullptr)
            {
                m_advectionSolver->Advect(
                    *faceCentered0, *vel, timeIntervalInSeconds,
                    faceCentered.get(), *GetColliderSDF());
                ExtrapolateIntoCollider(faceCentered.get());
            }
        }

        // Solve velocity advection
        const std::shared_ptr<FaceCenteredGrid3> vel0 =
            std::dynamic_pointer_cast<FaceCenteredGrid3>(vel->Clone());

        m_advectionSolver->Advect(*vel0, *vel0, timeIntervalInSeconds,
                                  vel.get(), *GetColliderSDF());
        ApplyBoundaryCondition();
    }
}

ScalarField3Ptr GridFluidSolver3::GetFluidSDF() const
{
    return std::make_shared<ConstantScalarField3>(
        -std::numeric_limits<double>::max());
}

void GridFluidSolver3::ComputeGravity(double timeIntervalInSeconds)
{
    if (m_gravity.LengthSquared() > std::numeric_limits<double>::epsilon())
    {
        FaceCenteredGrid3Ptr vel = m_grids->Velocity();
        ArrayView3<double> u = vel->UView();
        ArrayView3<double> v = vel->VView();
        ArrayView3<double> w = vel->WView();

        if (std::abs(m_gravity.x) > std::numeric_limits<double>::epsilon())
        {
            vel->ForEachUIndex(
                [&u, &timeIntervalInSeconds, this](const Vector3UZ& idx) {
                    u(idx) += timeIntervalInSeconds * m_gravity.x;
                });
        }

        if (std::abs(m_gravity.y) > std::numeric_limits<double>::epsilon())
        {
            vel->ForEachVIndex(
                [&v, &timeIntervalInSeconds, this](const Vector3UZ& idx) {
                    v(idx) += timeIntervalInSeconds * m_gravity.y;
                });
        }

        if (std::abs(m_gravity.z) > std::numeric_limits<double>::epsilon())
        {
            vel->ForEachWIndex(
                [&w, &timeIntervalInSeconds, this](const Vector3UZ& idx) {
                    w(idx) += timeIntervalInSeconds * m_gravity.z;
                });
        }

        ApplyBoundaryCondition();
    }
}

void GridFluidSolver3::ApplyBoundaryCondition() const
{
    const FaceCenteredGrid3Ptr vel = m_grids->Velocity();

    if (vel != nullptr && m_boundaryConditionSolver != nullptr)
    {
        const auto depth = static_cast<unsigned int>(std::ceil(m_maxCFL));
        m_boundaryConditionSolver->ConstrainVelocity(vel.get(), depth);
    }
}

void GridFluidSolver3::ExtrapolateIntoCollider(ScalarGrid3* grid)
{
    Array3<char> marker(grid->DataSize());
    GridDataPositionFunc<3> pos = grid->DataPosition();

    ParallelForEachIndex(
        marker.Size(), [this, &pos, &marker](size_t i, size_t j, size_t k) {
            if (IsInsideSDF(GetColliderSDF()->Sample(pos(i, j, k))))
            {
                marker(i, j, k) = 0;
            }
            else
            {
                marker(i, j, k) = 1;
            }
        });

    const auto depth = static_cast<unsigned int>(std::ceil(m_maxCFL));
    ExtrapolateToRegion(grid->DataView(), marker, depth, grid->DataView());
}

void GridFluidSolver3::ExtrapolateIntoCollider(CollocatedVectorGrid3* grid)
{
    Array3<char> marker(grid->DataSize());
    GridDataPositionFunc<3> pos = grid->DataPosition();

    ParallelForEachIndex(
        marker.Size(), [this, &pos, &marker](size_t i, size_t j, size_t k) {
            if (IsInsideSDF(GetColliderSDF()->Sample(pos(i, j, k))))
            {
                marker(i, j, k) = 0;
            }
            else
            {
                marker(i, j, k) = 1;
            }
        });

    const auto depth = static_cast<unsigned int>(std::ceil(m_maxCFL));
    ExtrapolateToRegion(grid->DataView(), marker, depth, grid->DataView());
}

void GridFluidSolver3::ExtrapolateIntoCollider(FaceCenteredGrid3* grid)
{
    const ArrayView3<double> u = grid->UView();
    const ArrayView3<double> v = grid->VView();
    const ArrayView3<double> w = grid->WView();

    GridDataPositionFunc<3> uPos = grid->UPosition();
    GridDataPositionFunc<3> vPos = grid->VPosition();
    GridDataPositionFunc<3> wPos = grid->WPosition();

    Array3<char> uMarker{ u.Size() };
    Array3<char> vMarker{ v.Size() };
    Array3<char> wMarker{ w.Size() };

    ParallelForEachIndex(
        uMarker.Size(), [this, &uPos, &uMarker](size_t i, size_t j, size_t k) {
            if (IsInsideSDF(GetColliderSDF()->Sample(uPos(i, j, k))))
            {
                uMarker(i, j, k) = 0;
            }
            else
            {
                uMarker(i, j, k) = 1;
            }
        });

    ParallelForEachIndex(
        vMarker.Size(), [this, &vPos, &vMarker](size_t i, size_t j, size_t k) {
            if (IsInsideSDF(GetColliderSDF()->Sample(vPos(i, j, k))))
            {
                vMarker(i, j, k) = 0;
            }
            else
            {
                vMarker(i, j, k) = 1;
            }
        });

    ParallelForEachIndex(
        wMarker.Size(), [this, &wPos, &wMarker](size_t i, size_t j, size_t k) {
            if (IsInsideSDF(GetColliderSDF()->Sample(wPos(i, j, k))))
            {
                wMarker(i, j, k) = 0;
            }
            else
            {
                wMarker(i, j, k) = 1;
            }
        });

    const auto depth = static_cast<unsigned int>(std::ceil(m_maxCFL));
    ExtrapolateToRegion(grid->UView(), uMarker, depth, u);
    ExtrapolateToRegion(grid->VView(), vMarker, depth, v);
    ExtrapolateToRegion(grid->WView(), wMarker, depth, w);
}

ScalarField3Ptr GridFluidSolver3::GetColliderSDF() const
{
    return m_boundaryConditionSolver->GetColliderSDF();
}

VectorField3Ptr GridFluidSolver3::GetColliderVelocityField() const
{
    return m_boundaryConditionSolver->GetColliderVelocityField();
}

void GridFluidSolver3::BeginAdvanceTimeStep(double timeIntervalInSeconds)
{
    // Update collider and emitter
    Timer timer;
    UpdateCollider(timeIntervalInSeconds);
    CUBBYFLOW_INFO << "Update collider took " << timer.DurationInSeconds()
                   << " seconds";

    timer.Reset();
    UpdateEmitter(timeIntervalInSeconds);
    CUBBYFLOW_INFO << "Update emitter took " << timer.DurationInSeconds()
                   << " seconds";

    // Update boundary condition solver
    if (m_boundaryConditionSolver != nullptr)
    {
        m_boundaryConditionSolver->UpdateCollider(
            m_collider, m_grids->Resolution(), m_grids->GridSpacing(),
            m_grids->Origin());
    }

    // Apply boundary condition to the velocity field in case the field got
    // updated externally.
    ApplyBoundaryCondition();

    // Invoke callback
    OnBeginAdvanceTimeStep(timeIntervalInSeconds);
}

void GridFluidSolver3::EndAdvanceTimeStep(double timeIntervalInSeconds)
{
    // Invoke callback
    OnEndAdvanceTimeStep(timeIntervalInSeconds);
}

void GridFluidSolver3::UpdateCollider(double timeIntervalInSeconds) const
{
    if (m_collider != nullptr)
    {
        m_collider->Update(GetCurrentTimeInSeconds(), timeIntervalInSeconds);
    }
}

void GridFluidSolver3::UpdateEmitter(double timeIntervalInSeconds) const
{
    if (m_emitter != nullptr)
    {
        m_emitter->Update(GetCurrentTimeInSeconds(), timeIntervalInSeconds);
    }
}

GridFluidSolver3::Builder GridFluidSolver3::GetBuilder()
{
    return Builder{};
}
}  // namespace CubbyFlow