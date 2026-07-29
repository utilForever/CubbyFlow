// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_GRID_FORWARD_EULER_DIFFUSION_SOLVER2_HPP
#define CUBBYFLOW_GRID_FORWARD_EULER_DIFFUSION_SOLVER2_HPP

#include <Core/Solver/Grid/GridDiffusionSolver2.hpp>

namespace CubbyFlow
{
//!
//! \brief 2-D grid-based forward Euler diffusion solver.
//!
//! This class implements 2-D grid-based forward Euler diffusion solver using
//! second-order central differencing spatially. Since the method is relying on
//! explicit time-integration (i.e. forward Euler), the diffusion coefficient is
//! limited by the time interval and grid spacing such as:
//! \f$\mu < \frac{h}{8\Delta t} \f$ where \f$\mu\f$, \f$h\f$, and
//! \f$\Delta t\f$ are the diffusion coefficient, grid spacing, and time
//! interval, respectively.
//!
class GridForwardEulerDiffusionSolver2 final : public GridDiffusionSolver2
{
 public:
    //! Default constructor.
    GridForwardEulerDiffusionSolver2() = default;

    //! Deleted copy constructor.
    GridForwardEulerDiffusionSolver2(const GridForwardEulerDiffusionSolver2&) =
        delete;

    //! Deleted move constructor.
    GridForwardEulerDiffusionSolver2(
        GridForwardEulerDiffusionSolver2&&) noexcept = delete;

    //! Default virtual destructor.
    ~GridForwardEulerDiffusionSolver2() override = default;

    //! Deleted copy assignment operator.
    GridForwardEulerDiffusionSolver2& operator=(
        const GridForwardEulerDiffusionSolver2&) = delete;

    //! Deleted move assignment operator.
    GridForwardEulerDiffusionSolver2& operator=(
        GridForwardEulerDiffusionSolver2&&) noexcept = delete;

    //!
    //! Solves diffusion equation for a scalar field.
    //!
    //! \param source Input scalar field.
    //! \param diffusionCoefficient Amount of diffusion.
    //! \param timeIntervalInSeconds Small time-interval that diffusion occur.
    //! \param dest Output scalar field.
    //! \param boundarySDF Shape of the solid boundary that is empty by default.
    //! \param fluidSDF Shape of the fluid boundary that is full by default.
    //!
    void Solve(const ScalarGrid2& source, double diffusionCoefficient,
               double timeIntervalInSeconds, ScalarGrid2* dest,
               const ScalarField2& boundarySDF =
                   ConstantScalarField2{ std::numeric_limits<double>::max() },
               const ScalarField2& fluidSDF = ConstantScalarField2{
                   -std::numeric_limits<double>::max() }) override;

    //!
    //! Solves diffusion equation for a collocated vector field.
    //!
    //! \param source Input collocated vector field.
    //! \param diffusionCoefficient Amount of diffusion.
    //! \param timeIntervalInSeconds Small time-interval that diffusion occur.
    //! \param dest Output collocated vector field.
    //! \param boundarySDF Shape of the solid boundary that is empty by default.
    //! \param fluidSDF Shape of the fluid boundary that is full by default.
    //!
    void Solve(const CollocatedVectorGrid2& source, double diffusionCoefficient,
               double timeIntervalInSeconds, CollocatedVectorGrid2* dest,
               const ScalarField2& boundarySDF =
                   ConstantScalarField2{ std::numeric_limits<double>::max() },
               const ScalarField2& fluidSDF = ConstantScalarField2{
                   -std::numeric_limits<double>::max() }) override;

    //!
    //! Solves diffusion equation for a face-centered vector field.
    //!
    //! \param source Input face-centered vector field.
    //! \param diffusionCoefficient Amount of diffusion.
    //! \param timeIntervalInSeconds Small time-interval that diffusion occur.
    //! \param dest Output face-centered vector field.
    //! \param boundarySDF Shape of the solid boundary that is empty by default.
    //! \param fluidSDF Shape of the fluid boundary that is full by default.
    //!
    void Solve(const FaceCenteredGrid2& source, double diffusionCoefficient,
               double timeIntervalInSeconds, FaceCenteredGrid2* dest,
               const ScalarField2& boundarySDF =
                   ConstantScalarField2{ std::numeric_limits<double>::max() },
               const ScalarField2& fluidSDF = ConstantScalarField2{
                   -std::numeric_limits<double>::max() }) override;

 private:
    template <typename PositionFunc>
    void BuildMarkers(const Vector2UZ& size, const PositionFunc& pos,
                      const ScalarField2& boundarySDF,
                      const ScalarField2& fluidSDF);

    Array2<char> m_markers;
};

//! Shared pointer type for the GridForwardEulerDiffusionSolver2.
using GridForwardEulerDiffusionSolver2Ptr =
    std::shared_ptr<GridForwardEulerDiffusionSolver2>;
}  // namespace CubbyFlow

#endif
