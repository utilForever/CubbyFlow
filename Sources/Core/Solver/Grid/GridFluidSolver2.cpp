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
#include <Core/Solver/Advection/CubicSemiLagrangian2.hpp>
#include <Core/Solver/Grid/GridBackwardEulerDiffusionSolver2.hpp>
#include <Core/Solver/Grid/GridFluidSolver2.hpp>
#include <Core/Solver/Grid/GridFractionalSinglePhasePressureSolver2.hpp>
#include <Core/Utils/LevelSetUtils.hpp>
#include <Core/Utils/Logging.hpp>
#include <Core/Utils/Timer.hpp>

namespace CubbyFlow
{
GridFluidSolver2::GridFluidSolver2()
    : GridFluidSolver2{ { 1, 1 }, { 1, 1 }, { 0, 0 } }
{
    // Do nothing
}

GridFluidSolver2::GridFluidSolver2(const Vector2UZ& resolution,
                                   const Vector2D& gridSpacing,
                                   const Vector2D& gridOrigin)
{
    m_grids = std::make_shared<GridSystemData2>();
    m_grids->Resize(resolution, gridSpacing, gridOrigin);

    SetAdvectionSolver(std::make_shared<CubicSemiLagrangian2>());
    SetDiffusionSolver(std::make_shared<GridBackwardEulerDiffusionSolver2>());
    SetPressureSolver(
        std::make_shared<GridFractionalSinglePhasePressureSolver2>());
    SetIsUsingFixedSubTimeSteps(false);
}

const Vector2D& GridFluidSolver2::GetGravity() const
{
    return m_gravity;
}

void GridFluidSolver2::SetGravity(const Vector2D& newGravity)
{
    m_gravity = newGravity;
}

double GridFluidSolver2::GetViscosityCoefficient() const
{
    return m_viscosityCoefficient;
}

void GridFluidSolver2::SetViscosityCoefficient(double newValue)
{
    m_viscosityCoefficient = std::max(newValue, 0.0);
}

double GridFluidSolver2::GetCFL(double timeIntervalInSeconds) const
{
    FaceCenteredGrid2Ptr vel = m_grids->Velocity();
    double maxVel = 0.0;

    vel->ForEachCellIndex([&vel, &timeIntervalInSeconds, this, &maxVel](
                              size_t i, size_t j) {
        const Vector2D v =
            vel->ValueAtCellCenter(i, j) + timeIntervalInSeconds * m_gravity;
        maxVel = std::max(maxVel, v.x);
        maxVel = std::max(maxVel, v.y);
    });

    const Vector2D gridSpacing = m_grids->GridSpacing();
    const double minGridSize = std::min(gridSpacing.x, gridSpacing.y);

    return maxVel * timeIntervalInSeconds / minGridSize;
}

double GridFluidSolver2::GetMaxCFL() const
{
    return m_maxCFL;
}

void GridFluidSolver2::SetMaxCFL(double newCFL)
{
    m_maxCFL = std::max(newCFL, std::numeric_limits<double>::epsilon());
}

bool GridFluidSolver2::GetUseCompressedLinearSystem() const
{
    return m_useCompressedLinearSys;
}

void GridFluidSolver2::SetUseCompressedLinearSystem(bool isOn)
{
    m_useCompressedLinearSys = isOn;
}

const AdvectionSolver2Ptr& GridFluidSolver2::GetAdvectionSolver() const
{
    return m_advectionSolver;
}

void GridFluidSolver2::SetAdvectionSolver(const AdvectionSolver2Ptr& newSolver)
{
    m_advectionSolver = newSolver;
}

const GridDiffusionSolver2Ptr& GridFluidSolver2::GetDiffusionSolver() const
{
    return m_diffusionSolver;
}

void GridFluidSolver2::SetDiffusionSolver(
    const GridDiffusionSolver2Ptr& newSolver)
{
    m_diffusionSolver = newSolver;
}

const GridPressureSolver2Ptr& GridFluidSolver2::GetPressureSolver() const
{
    return m_pressureSolver;
}

void GridFluidSolver2::SetPressureSolver(
    const GridPressureSolver2Ptr& newSolver)
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

int GridFluidSolver2::GetClosedDomainBoundaryFlag() const
{
    return m_closedDomainBoundaryFlag;
}

void GridFluidSolver2::SetClosedDomainBoundaryFlag(int flag)
{
    m_closedDomainBoundaryFlag = flag;
    m_boundaryConditionSolver->SetClosedDomainBoundaryFlag(
        m_closedDomainBoundaryFlag);
}

const GridSystemData2Ptr& GridFluidSolver2::GetGridSystemData() const
{
    return m_grids;
}

void GridFluidSolver2::ResizeGrid(const Vector2UZ& newSize,
                                  const Vector2D& newGridSpacing,
                                  const Vector2D& newGridOrigin) const
{
    m_grids->Resize(newSize, newGridSpacing, newGridOrigin);
}

Vector2UZ GridFluidSolver2::GetResolution() const
{
    return m_grids->Resolution();
}

Vector2D GridFluidSolver2::GetGridSpacing() const
{
    return m_grids->GridSpacing();
}

Vector2D GridFluidSolver2::GetGridOrigin() const
{
    return m_grids->Origin();
}

const FaceCenteredGrid2Ptr& GridFluidSolver2::GetVelocity() const
{
    return m_grids->Velocity();
}

const Collider2Ptr& GridFluidSolver2::GetCollider() const
{
    return m_collider;
}

void GridFluidSolver2::SetCollider(const Collider2Ptr& newCollider)
{
    m_collider = newCollider;
}

const GridEmitter2Ptr& GridFluidSolver2::GetEmitter() const
{
    return m_emitter;
}

void GridFluidSolver2::SetEmitter(const GridEmitter2Ptr& newEmitter)
{
    m_emitter = newEmitter;
}

void GridFluidSolver2::OnInitialize()
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

void GridFluidSolver2::OnAdvanceTimeStep(double timeIntervalInSeconds)
{
    // The minimum grid resolution is 1x1.
    if (m_grids->Resolution().x == 0 || m_grids->Resolution().y == 0)
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

unsigned int GridFluidSolver2::GetNumberOfSubTimeSteps(
    double timeIntervalInSeconds) const
{
    const double currentCFL = GetCFL(timeIntervalInSeconds);
    return static_cast<unsigned int>(
        std::max(std::ceil(currentCFL / m_maxCFL), 1.0));
}

void GridFluidSolver2::OnBeginAdvanceTimeStep(double timeIntervalInSeconds)
{
    UNUSED_VARIABLE(timeIntervalInSeconds);
}

void GridFluidSolver2::OnEndAdvanceTimeStep(double timeIntervalInSeconds)
{
    UNUSED_VARIABLE(timeIntervalInSeconds);
}

void GridFluidSolver2::ComputeExternalForces(double timeIntervalInSeconds)
{
    ComputeGravity(timeIntervalInSeconds);
}

void GridFluidSolver2::ComputeViscosity(double timeIntervalInSeconds)
{
    if (m_diffusionSolver != nullptr &&
        m_viscosityCoefficient > std::numeric_limits<double>::epsilon())
    {
        const FaceCenteredGrid2Ptr vel = GetVelocity();
        const std::shared_ptr<FaceCenteredGrid2> vel0 =
            std::dynamic_pointer_cast<FaceCenteredGrid2>(vel->Clone());

        m_diffusionSolver->Solve(*vel0, m_viscosityCoefficient,
                                 timeIntervalInSeconds, vel.get(),
                                 *GetColliderSDF(), *GetFluidSDF());
        ApplyBoundaryCondition();
    }
}

void GridFluidSolver2::ComputePressure(double timeIntervalInSeconds)
{
    if (m_pressureSolver != nullptr)
    {
        const FaceCenteredGrid2Ptr vel = GetVelocity();
        const std::shared_ptr<FaceCenteredGrid2> vel0 =
            std::dynamic_pointer_cast<FaceCenteredGrid2>(vel->Clone());

        m_pressureSolver->Solve(*vel0, timeIntervalInSeconds, vel.get(),
                                *GetColliderSDF(), *GetColliderVelocityField(),
                                *GetFluidSDF(), m_useCompressedLinearSys);
        ApplyBoundaryCondition();
    }
}

void GridFluidSolver2::ComputeAdvection(double timeIntervalInSeconds)
{
    const FaceCenteredGrid2Ptr vel = GetVelocity();

    if (m_advectionSolver != nullptr)
    {
        // Solve advections for custom scalar fields
        size_t n = m_grids->NumberOfAdvectableScalarData();

        for (size_t i = 0; i < n; ++i)
        {
            ScalarGrid2Ptr grid = m_grids->AdvectableScalarDataAt(i);
            std::shared_ptr<ScalarGrid2> grid0 = grid->Clone();

            m_advectionSolver->Advect(*grid0, *vel, timeIntervalInSeconds,
                                      grid.get(), *GetColliderSDF());
            ExtrapolateIntoCollider(grid.get());
        }

        // Solve advections for custom vector fields
        n = m_grids->NumberOfAdvectableVectorData();
        const size_t velIdx = m_grids->VelocityIndex();

        for (size_t i = 0; i < n; ++i)
        {
            // Handle velocity layer separately.
            if (i == velIdx)
            {
                continue;
            }

            VectorGrid2Ptr grid = m_grids->AdvectableVectorDataAt(i);
            std::shared_ptr<VectorGrid2> grid0 = grid->Clone();

            std::shared_ptr<CollocatedVectorGrid2> collocated =
                std::dynamic_pointer_cast<CollocatedVectorGrid2>(grid);
            std::shared_ptr<CollocatedVectorGrid2> collocated0 =
                std::dynamic_pointer_cast<CollocatedVectorGrid2>(grid0);

            if (collocated != nullptr)
            {
                m_advectionSolver->Advect(*collocated0, *vel,
                                          timeIntervalInSeconds,
                                          collocated.get(), *GetColliderSDF());
                ExtrapolateIntoCollider(collocated.get());
                continue;
            }

            std::shared_ptr<FaceCenteredGrid2> faceCentered =
                std::dynamic_pointer_cast<FaceCenteredGrid2>(grid);
            std::shared_ptr<FaceCenteredGrid2> faceCentered0 =
                std::dynamic_pointer_cast<FaceCenteredGrid2>(grid0);

            if (faceCentered != nullptr && faceCentered0 != nullptr)
            {
                m_advectionSolver->Advect(
                    *faceCentered0, *vel, timeIntervalInSeconds,
                    faceCentered.get(), *GetColliderSDF());
                ExtrapolateIntoCollider(faceCentered.get());
            }
        }

        // Solve velocity advection
        const std::shared_ptr<FaceCenteredGrid2> vel0 =
            std::dynamic_pointer_cast<FaceCenteredGrid2>(vel->Clone());

        m_advectionSolver->Advect(*vel0, *vel0, timeIntervalInSeconds,
                                  vel.get(), *GetColliderSDF());
        ApplyBoundaryCondition();
    }
}

ScalarField2Ptr GridFluidSolver2::GetFluidSDF() const
{
    return std::make_shared<ConstantScalarField2>(
        -std::numeric_limits<double>::max());
}

void GridFluidSolver2::ComputeGravity(double timeIntervalInSeconds)
{
    if (m_gravity.LengthSquared() > std::numeric_limits<double>::epsilon())
    {
        FaceCenteredGrid2Ptr vel = m_grids->Velocity();
        ArrayView2<double> u = vel->UView();
        ArrayView2<double> v = vel->VView();

        if (std::abs(m_gravity.x) > std::numeric_limits<double>::epsilon())
        {
            vel->ForEachUIndex(
                [&u, &timeIntervalInSeconds, this](const Vector2UZ& idx) {
                    u(idx) += timeIntervalInSeconds * m_gravity.x;
                });
        }

        if (std::abs(m_gravity.y) > std::numeric_limits<double>::epsilon())
        {
            vel->ForEachVIndex(
                [&v, &timeIntervalInSeconds, this](const Vector2UZ& idx) {
                    v(idx) += timeIntervalInSeconds * m_gravity.y;
                });
        }

        ApplyBoundaryCondition();
    }
}

void GridFluidSolver2::ApplyBoundaryCondition() const
{
    const FaceCenteredGrid2Ptr vel = m_grids->Velocity();

    if (vel != nullptr && m_boundaryConditionSolver != nullptr)
    {
        const auto depth = static_cast<unsigned int>(std::ceil(m_maxCFL));
        m_boundaryConditionSolver->ConstrainVelocity(vel.get(), depth);
    }
}

void GridFluidSolver2::ExtrapolateIntoCollider(ScalarGrid2* grid)
{
    Array2<char> marker(grid->DataSize());
    GridDataPositionFunc<2> pos = grid->DataPosition();

    ParallelForEachIndex(
        marker.Size(), [this, &pos, &marker](size_t i, size_t j) {
            if (IsInsideSDF(GetColliderSDF()->Sample(pos(i, j))))
            {
                marker(i, j) = 0;
            }
            else
            {
                marker(i, j) = 1;
            }
        });

    const auto depth = static_cast<unsigned int>(std::ceil(m_maxCFL));
    ExtrapolateToRegion(grid->DataView(), marker, depth, grid->DataView());
}

void GridFluidSolver2::ExtrapolateIntoCollider(CollocatedVectorGrid2* grid)
{
    Array2<char> marker(grid->DataSize());
    GridDataPositionFunc<2> pos = grid->DataPosition();

    ParallelForEachIndex(
        marker.Size(), [this, &pos, &marker](size_t i, size_t j) {
            if (IsInsideSDF(GetColliderSDF()->Sample(pos(i, j))))
            {
                marker(i, j) = 0;
            }
            else
            {
                marker(i, j) = 1;
            }
        });

    const auto depth = static_cast<unsigned int>(std::ceil(m_maxCFL));
    ExtrapolateToRegion(grid->DataView(), marker, depth, grid->DataView());
}

void GridFluidSolver2::ExtrapolateIntoCollider(FaceCenteredGrid2* grid)
{
    const ArrayView2<double> u = grid->UView();
    const ArrayView2<double> v = grid->VView();
    GridDataPositionFunc<2> uPos = grid->UPosition();
    GridDataPositionFunc<2> vPos = grid->VPosition();

    Array2<char> uMarker{ u.Size() };
    Array2<char> vMarker{ v.Size() };

    ParallelForEachIndex(
        uMarker.Size(), [this, &uPos, &uMarker](size_t i, size_t j) {
            if (IsInsideSDF(GetColliderSDF()->Sample(uPos(i, j))))
            {
                uMarker(i, j) = 0;
            }
            else
            {
                uMarker(i, j) = 1;
            }
        });

    ParallelForEachIndex(
        vMarker.Size(), [this, &vPos, &vMarker](size_t i, size_t j) {
            if (IsInsideSDF(GetColliderSDF()->Sample(vPos(i, j))))
            {
                vMarker(i, j) = 0;
            }
            else
            {
                vMarker(i, j) = 1;
            }
        });

    const auto depth = static_cast<unsigned int>(std::ceil(m_maxCFL));
    ExtrapolateToRegion(grid->UView(), uMarker, depth, u);
    ExtrapolateToRegion(grid->VView(), vMarker, depth, v);
}

ScalarField2Ptr GridFluidSolver2::GetColliderSDF() const
{
    return m_boundaryConditionSolver->GetColliderSDF();
}

VectorField2Ptr GridFluidSolver2::GetColliderVelocityField() const
{
    return m_boundaryConditionSolver->GetColliderVelocityField();
}

void GridFluidSolver2::BeginAdvanceTimeStep(double timeIntervalInSeconds)
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

void GridFluidSolver2::EndAdvanceTimeStep(double timeIntervalInSeconds)
{
    // Invoke callback
    OnEndAdvanceTimeStep(timeIntervalInSeconds);
}

void GridFluidSolver2::UpdateCollider(double timeIntervalInSeconds) const
{
    if (m_collider != nullptr)
    {
        m_collider->Update(GetCurrentTimeInSeconds(), timeIntervalInSeconds);
    }
}

void GridFluidSolver2::UpdateEmitter(double timeIntervalInSeconds) const
{
    if (m_emitter != nullptr)
    {
        m_emitter->Update(GetCurrentTimeInSeconds(), timeIntervalInSeconds);
    }
}

GridFluidSolver2::Builder GridFluidSolver2::GetBuilder()
{
    return Builder{};
}
}  // namespace CubbyFlow