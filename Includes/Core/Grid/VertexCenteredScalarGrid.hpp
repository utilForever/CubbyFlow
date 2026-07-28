// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_VERTEX_CENTERED_SCALAR_GRID_HPP
#define CUBBYFLOW_VERTEX_CENTERED_SCALAR_GRID_HPP

#include <Core/Grid/ScalarGrid.hpp>

namespace CubbyFlow
{
//!
//! \brief N-D Vertex-centered scalar grid structure.
//!
//! This class represents N-D vertex-centered scalar grid which extends
//! ScalarGrid. As its name suggests, the class defines the data point at the
//! grid vertices (corners). Thus, A x B x ... grid resolution will have (A+1) x
//! (B+1) x ... data points.
//!
template <size_t N>
class VertexCenteredScalarGrid final : public ScalarGrid<N>
{
 public:
    CUBBYFLOW_GRID_TYPE_NAME(VertexCenteredScalarGrid, N)

    class Builder;

    using ScalarGrid<N>::Resize;
    using ScalarGrid<N>::Resolution;
    using ScalarGrid<N>::Origin;

    //! Constructs zero-sized grid.
    VertexCenteredScalarGrid() = default;

    //! Constructs a grid with given resolution, grid spacing, origin and
    //! initial value.
    explicit VertexCenteredScalarGrid(
        const Vector<size_t, N>& resolution,
        const Vector<double, N>& gridSpacing =
            Vector<double, N>::MakeConstant(1),
        const Vector<double, N>& origin = Vector<double, N>{},
        double initialValue = 0.0);

    //! Default virtual destructor.
    ~VertexCenteredScalarGrid() override = default;

    //! Copy constructor.
    VertexCenteredScalarGrid(const VertexCenteredScalarGrid& other);

    //! Move constructor.
    VertexCenteredScalarGrid(VertexCenteredScalarGrid&& other) noexcept;

    //! Copy assignment operator.
    VertexCenteredScalarGrid<N>& operator=(
        const VertexCenteredScalarGrid& other);

    //! Move assignment operator.
    VertexCenteredScalarGrid<N>& operator=(
        VertexCenteredScalarGrid&& other) noexcept;

    //! Returns the actual data point size.
    [[nodiscard]] Vector<size_t, N> DataSize() const override;

    //! Returns data position for the grid point at (0, 0, ...).
    //! Note that this is different from origin() since origin() returns
    //! the lower corner point of the bounding box.
    [[nodiscard]] Vector<double, N> DataOrigin() const override;

    //! Returns the copy of the grid instance.
    [[nodiscard]] std::shared_ptr<ScalarGrid<N>> Clone() const override;

    //!
    //! \brief Swaps the contents with the given \p other grid.
    //!
    //! This function swaps the contents of the grid instance with the given
    //! grid object \p other only if \p other has the same type with this grid.
    //!
    void Swap(Grid<N>* other) override;

    //! Sets the contents with the given \p other grid.
    void Set(const VertexCenteredScalarGrid<N>& other);

    //! Returns builder fox VertexCenteredScalarGrid<N>.
    static Builder GetBuilder();

 protected:
    using ScalarGrid<N>::SwapScalarGrid;
    using ScalarGrid<N>::SetScalarGrid;
};

//! 2-D VertexCenteredScalarGrid type.
using VertexCenteredScalarGrid2 = VertexCenteredScalarGrid<2>;

//! 3-D VertexCenteredScalarGrid type.
using VertexCenteredScalarGrid3 = VertexCenteredScalarGrid<3>;

//! Shared pointer for the VertexCenteredScalarGrid2 type.
using VertexCenteredScalarGrid2Ptr = std::shared_ptr<VertexCenteredScalarGrid2>;

//! Shared pointer for the VertexCenteredScalarGrid3 type.
using VertexCenteredScalarGrid3Ptr = std::shared_ptr<VertexCenteredScalarGrid3>;

//!
//! \brief Front-end to create VertexCenteredScalarGrid objects step by step.
//!
template <size_t N>
class VertexCenteredScalarGrid<N>::Builder final : public ScalarGridBuilder<N>
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

    //! Builds VertexCenteredScalarGrid<N> instance.
    VertexCenteredScalarGrid<N> Build() const;

    //! Builds shared pointer of VertexCenteredScalarGrid<N> instance.
    std::shared_ptr<VertexCenteredScalarGrid<N>> MakeShared() const;

    //!
    //! \brief Builds shared pointer of VertexCenteredScalarGrid<N> instance.
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
    Vector<double, N> m_gridOrigin;
    double m_initialVal = 0.0;
};
}  // namespace CubbyFlow

#endif