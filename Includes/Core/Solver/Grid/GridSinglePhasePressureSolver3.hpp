// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_SINGLE_PHASE_PRESSURE_SOLVER3_HPP
#define CUBBYFLOW_SINGLE_PHASE_PRESSURE_SOLVER3_HPP

#include <Core/FDM/FDMMGLinearSystem3.hpp>
#include <Core/Solver/FDM/FDMLinearSystemSolver3.hpp>
#include <Core/Solver/FDM/FDMMGSolver3.hpp>
#include <Core/Solver/Grid/GridPressureSolver3.hpp>

namespace CubbyFlow
{
//!
//! \brief 3-D single-phase pressure solver.
//!
//! This class implements 3-D single-phase pressure solver. This solver encodes
//! the boundaries like Lego blocks -- if a grid cell center is inside or
//! outside the boundaries, it is either marked as occupied or not.
//! In addition, this class solves single-phase flow, solving the pressure for
//! selected fluid region only and treat other area as an atmosphere region.
//! Thus, the pressure outside the fluid will be set to a constant value and
//! velocity field won't be altered. This solver also computes the fluid
//! boundary in block-like manner; If a grid cell is inside or outside the
//! fluid, it is marked as either fluid or atmosphere. Thus, this solver in
//! general, does not compute sub-grid structure.
//!
class GridSinglePhasePressureSolver3 : public GridPressureSolver3
{
 public:
    //! Default constructor.
    GridSinglePhasePressureSolver3();

    //! Deleted copy constructor.
    GridSinglePhasePressureSolver3(const GridSinglePhasePressureSolver3&) =
        delete;

    //! Deleted move constructor.
    GridSinglePhasePressureSolver3(GridSinglePhasePressureSolver3&&) noexcept =
        delete;

    //! Default virtual destructor.
    ~GridSinglePhasePressureSolver3() override = default;

    //! Deleted copy assignment operator.
    GridSinglePhasePressureSolver3& operator=(
        const GridSinglePhasePressureSolver3&) = delete;

    //! Deleted move assignment operator.
    GridSinglePhasePressureSolver3& operator=(
        GridSinglePhasePressureSolver3&&) noexcept = delete;

    //!
    //! \brief Solves the pressure term and apply it to the velocity field.
    //!
    //! This function takes input velocity field and outputs pressure-applied
    //! velocity field. It also accepts extra arguments such as \p boundarySDF
    //! and \p fluidSDF that represent signed-distance representation of the
    //! boundary and fluid area. The negative region of \p boundarySDF means
    //! it is occupied by solid object. Also, the positive / negative area of
    //! the \p fluidSDF means it is occupied by fluid / atmosphere. If not
    //! specified, constant scalar field with std::numeric_limits<double>::max()
    //! will be used for \p boundarySDF meaning that no boundary at all.
    //! Similarly, a constant field with -std::numeric_limits<double>::max()
    //! will be used for \p fluidSDF which means it's fully occupied with fluid
    //! without any atmosphere.
    //!
    //! \param[in]     input                 The input velocity field.
    //! \param[in]     timeIntervalInSeconds The time interval for the sim.
    //! \param[in,out] output                The output velocity field.
    //! \param[in]     boundarySDF           The SDF of the boundary.
    //! \param[in]     boundaryVelocity      The velocity of the boundary.
    //! \param[in]     fluidSDF              The SDF of the fluid/atmosphere.
    //! \param[in]     useCompressed         True if it uses compressed system.
    //!
    void Solve(const FaceCenteredGrid3& input, double timeIntervalInSeconds,
               FaceCenteredGrid3* output,
               const ScalarField3& boundarySDF =
                   ConstantScalarField3{ std::numeric_limits<double>::max() },
               const VectorField3& boundaryVelocity =
                   ConstantVectorField3{ { 0, 0, 0 } },
               const ScalarField3& fluidSDF =
                   ConstantScalarField3{ -std::numeric_limits<double>::max() },
               bool useCompressed = false) override;

    //!
    //! \brief Returns the best boundary condition solver for this solver.
    //!
    //! This function returns the best boundary condition solver that works well
    //! with this pressure solver. Depending on the pressure solver
    //! implementation, different boundary condition solver might be used. For
    //! this particular class, an instance of
    //! GridBlockedBoundaryConditionSolver3 will be returned since this pressure
    //! solver encodes boundaries like pixelated Lego blocks.
    //!
    [[nodiscard]] GridBoundaryConditionSolver3Ptr
    SuggestedBoundaryConditionSolver() const override;

    //! Returns the linear system solver.
    [[nodiscard]] const FDMLinearSystemSolver3Ptr& GetLinearSystemSolver()
        const;

    //! Sets the linear system solver.
    void SetLinearSystemSolver(const FDMLinearSystemSolver3Ptr& solver);

    //! Returns the pressure field.
    [[nodiscard]] const FDMVector3& GetPressure() const;

 private:
    template <typename PositionFunc>
    void BuildMarkers(const Vector3UZ& size, const PositionFunc& pos,
                      const ScalarField3& boundarySDF,
                      const ScalarField3& fluidSDF);

    void DecompressSolution();

    virtual void BuildSystem(const FaceCenteredGrid3& input,
                             bool useCompressed);

    virtual void ApplyPressureGradient(const FaceCenteredGrid3& input,
                                       FaceCenteredGrid3* output);

    FDMLinearSystem3 m_system;
    FDMCompressedLinearSystem3 m_compSystem;
    FDMLinearSystemSolver3Ptr m_systemSolver;

    FDMMGLinearSystem3 m_mgSystem;
    FDMMGSolver3Ptr m_mgSystemSolver;

    std::vector<Array3<char>> m_markers;
};

//! Shared pointer type for the GridSinglePhasePressureSolver3.
using GridSinglePhasePressureSolver3Ptr =
    std::shared_ptr<GridSinglePhasePressureSolver3>;
}  // namespace CubbyFlow

#endif
