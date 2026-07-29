// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_VERTEX_CENTERED_VECTOR_GRID_HPP
#define CUBBYFLOW_VERTEX_CENTERED_VECTOR_GRID_HPP

#include <Core/Grid/CollocatedVectorGrid.hpp>

namespace CubbyFlow
{
//!
//! \brief N-D Vertex-centered vector grid structure.
//!
//! This class represents N-D vertex-centered vector grid which extends
//! CollocatedVectorGrid. As its name suggests, the class defines the data
//! point at the grid vertices (corners). Thus, A x B x ... grid resolution will
//! have (A+1) x (B+1) x ... data points.
//!
template <size_t N>
class VertexCenteredVectorGrid final : public CollocatedVectorGrid<N>
{
 public:
    CUBBYFLOW_GRID_TYPE_NAME(VertexCenteredVectorGrid, N)

    class Builder;

    using CollocatedVectorGrid<N>::Resize;
    using CollocatedVectorGrid<N>::Resolution;
    using CollocatedVectorGrid<N>::Origin;
    using CollocatedVectorGrid<N>::GridSpacing;
    using CollocatedVectorGrid<N>::DataView;
    using CollocatedVectorGrid<N>::DataPosition;

    //! Constructs zero-sized grid.
    VertexCenteredVectorGrid() = default;

    //! Constructs a grid with given resolution, grid spacing, origin and
    //! initial value.
    explicit VertexCenteredVectorGrid(
        const Vector<size_t, N>& resolution,
        const Vector<double, N>& gridSpacing =
            Vector<double, N>::MakeConstant(1),
        const Vector<double, N>& origin = Vector<double, N>{},
        const Vector<double, N>& initialValue = Vector<double, N>{});

    //! Default virtual destructor.
    ~VertexCenteredVectorGrid() override = default;

    //! Copy constructor.
    VertexCenteredVectorGrid(const VertexCenteredVectorGrid& other);

    //! Move constructor.
    VertexCenteredVectorGrid(VertexCenteredVectorGrid&& other) noexcept;

    //! Copy assignment operator.
    VertexCenteredVectorGrid& operator=(const VertexCenteredVectorGrid& other);

    //! Move assignment operator.
    VertexCenteredVectorGrid& operator=(
        VertexCenteredVectorGrid&& other) noexcept;

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
    void Set(const VertexCenteredVectorGrid& other);

    //! Fills the grid with given value.
    void Fill(const Vector<double, N>& value,
              ExecutionPolicy policy = ExecutionPolicy::Parallel) override;

    //! Fills the grid with given function.
    void Fill(
        const std::function<Vector<double, N>(const Vector<double, N>&)>& func,
        ExecutionPolicy policy = ExecutionPolicy::Parallel) override;

    //! Returns the copy of the grid instance.
    [[nodiscard]] std::shared_ptr<VectorGrid<N>> Clone() const override;

    //! Returns builder fox VertexCenteredVectorGrid.
    static Builder GetBuilder();

 protected:
    using CollocatedVectorGrid<N>::SwapCollocatedVectorGrid;
    using CollocatedVectorGrid<N>::SetCollocatedVectorGrid;
};

//! 2-D VertexCenteredVectorGrid type.
using VertexCenteredVectorGrid2 = VertexCenteredVectorGrid<2>;

//! 3-D VertexCenteredVectorGrid type.
using VertexCenteredVectorGrid3 = VertexCenteredVectorGrid<3>;

//! Shared pointer for the VertexCenteredVectorGrid2 type.
using VertexCenteredVectorGrid2Ptr = std::shared_ptr<VertexCenteredVectorGrid2>;

//! Shared pointer for the VertexCenteredVectorGrid3 type.
using VertexCenteredVectorGrid3Ptr = std::shared_ptr<VertexCenteredVectorGrid3>;

//! A grid builder class that returns N-D vertex-centered vector grid.
template <size_t N>
class VertexCenteredVectorGrid<N>::Builder final : public VectorGridBuilder<N>
{
 public:
    //! Returns builder with resolution.
    Builder& WithResolution(const Vector<size_t, N>& resolution);

    //! Returns builder with grid spacing.
    Builder& WithGridSpacing(const Vector<double, N>& gridSpacing);

    //! Returns builder with grid origin.
    Builder& WithOrigin(const Vector<double, N>& gridOrigin);

    //! Returns builder with initial value.
    Builder& WithInitialValue(const Vector<double, N>& initialVal);

    //! Builds VertexCenteredVectorGrid instance.
    VertexCenteredVectorGrid Build() const;

    //! Builds shared pointer of VertexCenteredVectorGrid instance.
    std::shared_ptr<VertexCenteredVectorGrid<N>> MakeShared() const;

    //!
    //! \brief Builds shared pointer of VertexCenteredVectorGrid instance.
    //!
    //! This is an overriding function that implements VectorGridBuilder2.
    //!
    [[nodiscard]] std::shared_ptr<VectorGrid<N>> Build(
        const Vector<size_t, N>& resolution,
        const Vector<double, N>& gridSpacing,
        const Vector<double, N>& gridOrigin,
        const Vector<double, N>& initialVal) const override;

 private:
    Vector<size_t, N> m_resolution = Vector<size_t, N>::MakeConstant(1);
    Vector<double, N> m_gridSpacing = Vector<double, N>::MakeConstant(1.0);
    Vector<double, N> m_gridOrigin;
    Vector<double, N> m_initialVal;
};
}  // namespace CubbyFlow

#endif