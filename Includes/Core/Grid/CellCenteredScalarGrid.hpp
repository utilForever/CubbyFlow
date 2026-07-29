// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_CELL_CENTERED_SCALAR_GRID_HPP
#define CUBBYFLOW_CELL_CENTERED_SCALAR_GRID_HPP

#include <Core/Grid/ScalarGrid.hpp>

namespace CubbyFlow
{
//!
//! \brief N-D Cell-centered scalar grid structure.
//!
//! This class represents N-D cell-centered scalar grid which extends
//! ScalarGrid. As its name suggests, the class defines the data point at the
//! center of a grid cell. Thus, the dimension of data points are equal to the
//! dimension of the cells.
//!
template <size_t N>
class CellCenteredScalarGrid final : public ScalarGrid<N>
{
 public:
    CUBBYFLOW_GRID_TYPE_NAME(CellCenteredScalarGrid, N)

    class Builder;

    using ScalarGrid<N>::Resize;
    using ScalarGrid<N>::Resolution;
    using ScalarGrid<N>::Origin;
    using ScalarGrid<N>::GridSpacing;

    //! Constructs zero-sized grid.
    CellCenteredScalarGrid() = default;

    //! Constructs a grid with given resolution, grid spacing, origin and
    //! initial value.
    explicit CellCenteredScalarGrid(
        const Vector<size_t, N>& resolution,
        const Vector<double, N>& gridSpacing =
            Vector<double, N>::MakeConstant(1.0),
        const Vector<double, N>& origin = Vector<double, N>{},
        double initialValue = 0.0);

    //! Default virtual destructor.
    ~CellCenteredScalarGrid() override = default;

    //! Copy constructor.
    CellCenteredScalarGrid(const CellCenteredScalarGrid& other);

    //! Move constructor.
    CellCenteredScalarGrid(CellCenteredScalarGrid&& other) noexcept;

    //! Copy assignment operator.
    CellCenteredScalarGrid& operator=(const CellCenteredScalarGrid& other);

    //! Move assignment operator.
    CellCenteredScalarGrid& operator=(CellCenteredScalarGrid&& other) noexcept;

    //! Returns the actual data point size.
    [[nodiscard]] Vector<size_t, N> DataSize() const override;

    //! Returns data position for the grid point at (0, 0, ...).
    //! Note that this is different from origin() since origin() returns
    //! the lower corner point of the bounding box.
    [[nodiscard]] Vector<double, N> DataOrigin() const override;

    //!
    //! \brief Swaps the contents with the given \p other grid.
    //!
    //! This function swaps the contents of the grid instance with the given
    //! grid object \p other only if \p other has the same type with this grid.
    //!
    void Swap(Grid<N>* other) override;

    //! Sets the contents with the given \p other grid.
    void Set(const CellCenteredScalarGrid& other);

    //! Returns the copy of the grid instance.
    [[nodiscard]] std::shared_ptr<ScalarGrid<N>> Clone() const override;

    //! Returns builder fox CellCenteredScalarGrid.
    static Builder GetBuilder();

 protected:
    using ScalarGrid<N>::SwapScalarGrid;
    using ScalarGrid<N>::SetScalarGrid;
};

//! 2-D CellCenteredScalarGrid type.
using CellCenteredScalarGrid2 = CellCenteredScalarGrid<2>;

//! 3-D CellCenteredScalarGrid type.
using CellCenteredScalarGrid3 = CellCenteredScalarGrid<3>;

//! Shared pointer for the CellCenteredScalarGrid2 type.
using CellCenteredScalarGrid2Ptr = std::shared_ptr<CellCenteredScalarGrid2>;

//! Shared pointer for the CellCenteredScalarGrid3 type.
using CellCenteredScalarGrid3Ptr = std::shared_ptr<CellCenteredScalarGrid3>;

//!
//! \brief Front-end to create CellCenteredScalarGrid objects step by step.
//!
template <size_t N>
class CellCenteredScalarGrid<N>::Builder final : public ScalarGridBuilder<N>
{
 public:
    //! Returns builder with resolution.
    Builder& WithResolution(const Vector<size_t, N>& resolution);

    //! Returns builder with grid spacing.
    Builder& WithGridSpacing(const Vector<double, N>& gridSpacing);

    //! Returns builder with grid origin.
    Builder& WithOrigin(const Vector<double, N>& gridOrigin);

    //! Returns builder with initial value.
    Builder& WithInitialValue(double initialVal);

    //! Builds CellCenteredScalarGrid instance.
    CellCenteredScalarGrid<N> Build() const;

    //! Builds shared pointer of CellCenteredScalarGrid instance.
    std::shared_ptr<CellCenteredScalarGrid<N>> MakeShared() const;

    //!
    //! \brief Builds shared pointer of CellCenteredScalarGrid instance.
    //!
    //! This is an overriding function that implements ScalarGridBuilder2.
    //!
    [[nodiscard]] std::shared_ptr<ScalarGrid<N>> Build(
        const Vector<size_t, N>& resolution,
        const Vector<double, N>& gridSpacing,
        const Vector<double, N>& gridOrigin, double initialVal) const override;

 private:
    Vector<size_t, N> m_resolution = Vector<size_t, N>::MakeConstant(1);
    Vector<double, N> m_gridSpacing = Vector<double, N>::MakeConstant(1.0);
    Vector<double, N> m_gridOrigin = Vector<double, N>{};
    double m_initialVal = 0.0;
};
}  // namespace CubbyFlow

#endif