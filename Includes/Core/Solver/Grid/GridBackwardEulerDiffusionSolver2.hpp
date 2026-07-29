// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_GRID_BACKWARD_EULER_DIFFUSION_SOLVER2_HPP
#define CUBBYFLOW_GRID_BACKWARD_EULER_DIFFUSION_SOLVER2_HPP

#include <Core/Solver/FDM/FDMLinearSystemSolver2.hpp>
#include <Core/Solver/Grid/GridDiffusionSolver2.hpp>

namespace CubbyFlow
{
//!
//! \brief 2-D grid-based backward Euler diffusion solver.
//!
//! This class implements 2-D grid-based forward Euler diffusion solver using
//! second-order central differencing spatially. Since the method is following
//! the implicit time-integration (i.e. backward Euler), larger time interval or
//! diffusion coefficient can be used without breaking the result. Note, higher
//! values for those parameters will still impact the accuracy of the result.
//! To solve the backward Euler method, a linear system solver is used and
//! incomplete Cholesky conjugate gradient method is used by default.
//!
class GridBackwardEulerDiffusionSolver2 final : public GridDiffusionSolver2
{
 public:
    enum class BoundaryType
    {
        Dirichlet,
        Neumann
    };

    //! Constructs the solver with given boundary type.
    explicit GridBackwardEulerDiffusionSolver2(
        BoundaryType boundaryType = BoundaryType::Neumann);

    //! Deleted copy constructor.
    GridBackwardEulerDiffusionSolver2(
        const GridBackwardEulerDiffusionSolver2&) = delete;

    //! Deleted move constructor.
    GridBackwardEulerDiffusionSolver2(
        GridBackwardEulerDiffusionSolver2&&) noexcept = delete;

    //! Default virtual destructor.
    ~GridBackwardEulerDiffusionSolver2() override = default;

    //! Deleted copy assignment operator.
    GridBackwardEulerDiffusionSolver2& operator=(
        const GridBackwardEulerDiffusionSolver2&) = delete;

    //! Deleted move assignment operator.
    GridBackwardEulerDiffusionSolver2& operator=(
        GridBackwardEulerDiffusionSolver2&&) noexcept = delete;

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

    //! Sets the linear system solver for this diffusion solver.
    void SetLinearSystemSolver(const FDMLinearSystemSolver2Ptr& solver);

 private:
    template <typename PositionFunc>
    void BuildMarkers(const Vector2UZ& size, const PositionFunc& pos,
                      const ScalarField2& boundarySDF,
                      const ScalarField2& fluidSDF);

    void BuildMatrix(const Vector2UZ& size, const Vector2D& c);

    void BuildVectors(const ConstArrayView2<double>& f, const Vector2D& c);

    void BuildVectors(const ConstArrayView2<Vector2D>& f, const Vector2D& c,
                      size_t component);

    BoundaryType m_boundaryType;
    FDMLinearSystem2 m_system;
    FDMLinearSystemSolver2Ptr m_systemSolver;
    Array2<char> m_markers;
};

//! Shared pointer type for the GridBackwardEulerDiffusionSolver2.
using GridBackwardEulerDiffusionSolver2Ptr =
    std::shared_ptr<GridBackwardEulerDiffusionSolver2>;
}  // namespace CubbyFlow

#endif
