// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_FACE_CENTERED_GRID_HPP
#define CUBBYFLOW_FACE_CENTERED_GRID_HPP

#include <Core/Array/Array.hpp>
#include <Core/Array/ArraySamplers.hpp>
#include <Core/Grid/VectorGrid.hpp>

#include <array>

namespace CubbyFlow
{
//!
//! \brief N-D face-centered (a.k.a MAC or staggered) grid.
//!
//! This class implements face-centered grid which is also known as
//! marker-and-cell (MAC) or staggered grid. This vector grid stores each vector
//! component at face center. Thus, u, v (and w for 3-D) components are not
//! collocated.
//!
template <size_t N>
class FaceCenteredGrid final : public VectorGrid<N>
{
 public:
    CUBBYFLOW_GRID_TYPE_NAME(FaceCenteredGrid, N)

    class Builder;

    using VectorGrid<N>::Resize;
    using VectorGrid<N>::Resolution;
    using VectorGrid<N>::GridSpacing;

    //! Read-write scalar data view type.
    using ScalarDataView = ArrayView<double, N>;

    //! Read-only scalar data view type.
    using ConstScalarDataView = ArrayView<const double, N>;

    //! Constructs empty grid.
    FaceCenteredGrid();

    //! Resizes the grid using given parameters.
    explicit FaceCenteredGrid(
        const Vector<size_t, N>& resolution,
        const Vector<double, N>& gridSpacing =
            Vector<double, N>::MakeConstant(1.0),
        const Vector<double, N>& origin = Vector<double, N>{},
        const Vector<double, N>& initialValue = Vector<double, N>{});

    //! Default virtual destructor.
    ~FaceCenteredGrid() override = default;

    //! Copy constructor.
    FaceCenteredGrid(const FaceCenteredGrid& other);

    //! Move constructor.
    FaceCenteredGrid(FaceCenteredGrid&& other) noexcept;

    //! Copy assignment operator.
    FaceCenteredGrid& operator=(const FaceCenteredGrid& other);

    //! Move assignment operator.
    FaceCenteredGrid& operator=(FaceCenteredGrid&& other) noexcept;

    //!
    //! \brief Swaps the contents with the given \p other grid.
    //!
    //! This function swaps the contents of the grid instance with the given
    //! grid object \p other only if \p other has the same type with this grid.
    //!
    void Swap(Grid<N>* other) override;

    //! Sets the contents with the given \p other grid.
    void Set(const FaceCenteredGrid& other);

    //! Returns u-value at given data point.
    [[nodiscard]] double& U(const Vector<size_t, N>& idx);

    //! Returns u-value at given data point.
    template <typename... Indices>
    [[nodiscard]] double& U(size_t i, Indices... indices)
    {
        return U(Vector<size_t, N>(i, indices...));
    }

    //! Returns u-value at given data point.
    [[nodiscard]] const double& U(const Vector<size_t, N>& idx) const;

    //! Returns u-value at given data point.
    template <typename... Indices>
    [[nodiscard]] const double& U(size_t i, Indices... indices) const
    {
        return U(Vector<size_t, N>(i, indices...));
    }

    //! Returns v-value at given data point.
    [[nodiscard]] double& V(const Vector<size_t, N>& idx);

    //! Returns v-value at given data point.
    template <typename... Indices>
    [[nodiscard]] double& V(size_t i, Indices... indices)
    {
        return V(Vector<size_t, N>(i, indices...));
    }

    //! Returns v-value at given data point.
    [[nodiscard]] const double& V(const Vector<size_t, N>& idx) const;

    //! Returns v-value at given data point.
    template <typename... Indices>
    [[nodiscard]] const double& V(size_t i, Indices... indices) const
    {
        return V(Vector<size_t, N>(i, indices...));
    }

    //! Returns w-value at given data point.
    template <size_t M = N>
    [[nodiscard]] std::enable_if_t<M == 3, double&> W(
        const Vector<size_t, N>& idx)
    {
        return m_data[2](idx);
    }

    //! Returns w-value at given data point.
    template <size_t M = N, typename... Indices>
    [[nodiscard]] std::enable_if_t<M == 3, double&> W(size_t i,
                                                      Indices... indices)
    {
        return W(Vector<size_t, N>(i, indices...));
    }

    //! Returns w-value at given data point.
    template <size_t M = N>
    std::enable_if_t<M == 3, const double&> W(
        const Vector<size_t, N>& idx) const
    {
        return m_data[2](idx);
    }

    //! Returns w-value at given data point.
    template <size_t M = N, typename... Indices>
    [[nodiscard]] std::enable_if_t<M == 3, const double&> W(
        size_t i, Indices... indices) const
    {
        return W(Vector<size_t, N>(i, indices...));
    }

    //! Returns interpolated value at cell center.
    [[nodiscard]] Vector<double, N> ValueAtCellCenter(
        const Vector<size_t, N>& idx) const;

    //! Returns interpolated value at cell center.
    template <typename... Indices>
    [[nodiscard]] Vector<double, N> ValueAtCellCenter(size_t i,
                                                      Indices... indices) const
    {
        return ValueAtCellCenter(Vector<size_t, N>(i, indices...));
    }

    //! Returns divergence at cell-center location.
    [[nodiscard]] double DivergenceAtCellCenter(
        const Vector<size_t, N>& idx) const;

    //! Returns divergence at cell-center location.
    template <typename... Indices>
    [[nodiscard]] double DivergenceAtCellCenter(size_t i,
                                                Indices... indices) const
    {
        return DivergenceAtCellCenter(Vector<size_t, N>(i, indices...));
    }

    //! Returns curl at cell-center location.
    [[nodiscard]] typename GetCurl<N>::Type CurlAtCellCenter(
        const Vector<size_t, N>& idx) const;

    //! Returns curl at cell-center location.
    template <typename... Indices>
    [[nodiscard]] typename GetCurl<N>::Type CurlAtCellCenter(
        size_t i, Indices... indices) const
    {
        return CurlAtCellCenter(Vector<size_t, N>(i, indices...));
    }

    //! Returns u data view.
    [[nodiscard]] ScalarDataView UView();

    //! Returns read-only u data view.
    [[nodiscard]] ConstScalarDataView UView() const;

    //! Returns v data view.
    [[nodiscard]] ScalarDataView VView();

    //! Returns read-only v data view.
    [[nodiscard]] ConstScalarDataView VView() const;

    //! Returns w data view.
    template <size_t M = N>
    [[nodiscard]] std::enable_if_t<M == 3, ScalarDataView> WView()
    {
        return DataView(2);
    }

    //! Returns read-only w data view.
    template <size_t M = N>
    [[nodiscard]] std::enable_if_t<M == 3, ConstScalarDataView> WView() const
    {
        return DataView(2);
    }

    //! Returns i-th data view.
    [[nodiscard]] ScalarDataView DataView(size_t i);

    //! Returns read-only i-th data view.
    [[nodiscard]] ConstScalarDataView DataView(size_t i) const;

    //! Returns function object that maps u data point to its actual position.
    [[nodiscard]] GridDataPositionFunc<N> UPosition() const;

    //! Returns function object that maps v data point to its actual position.
    [[nodiscard]] GridDataPositionFunc<N> VPosition() const;

    //! Returns function object that maps w data point to its actual position.
    template <size_t M = N>
    [[nodiscard]] std::enable_if_t<M == 3, GridDataPositionFunc<N>> WPosition()
        const
    {
        return DataPosition(2);
    }

    //! Returns function object that maps data point to its actual position.
    [[nodiscard]] GridDataPositionFunc<N> DataPosition(size_t i) const;

    //! Returns data size of the u component.
    [[nodiscard]] Vector<size_t, N> USize() const;

    //! Returns data size of the v component.
    [[nodiscard]] Vector<size_t, N> VSize() const;

    //! Returns data size of the w component.
    template <size_t M = N>
    [[nodiscard]] std::enable_if_t<M == 3, Vector<size_t, N>> WSize() const
    {
        return DataSize(2);
    }

    //! Returns data size of the i-th component.
    [[nodiscard]] Vector<size_t, N> DataSize(size_t i) const;

    //!
    //! \brief Returns u-data position for the grid point at (0, 0, ...).
    //!
    //! Note that this is different from origin() since origin() returns
    //! the lower corner point of the bounding box.
    //!
    [[nodiscard]] Vector<double, N> UOrigin() const;

    //!
    //! \brief Returns v-data position for the grid point at (0, 0, ...).
    //!
    //! Note that this is different from origin() since origin() returns
    //! the lower corner point of the bounding box.
    //!
    [[nodiscard]] Vector<double, N> VOrigin() const;

    //!
    //! \brief Returns w-data position for the grid point at (0, 0, ...).
    //!
    //! Note that this is different from origin() since origin() returns
    //! the lower corner point of the bounding box.
    //!
    template <size_t M = N>
    [[nodiscard]] std::enable_if_t<M == 3, Vector<double, N>> WOrigin() const
    {
        return DataOrigin(2);
    }

    //!
    //! \brief Returns i-th data position for the grid point at (0, 0, ...).
    //!
    //! Note that this is different from origin() since origin() returns
    //! the lower corner point of the bounding box.
    //!
    [[nodiscard]] Vector<double, N> DataOrigin(size_t i) const;

    //! Fills the grid with given value.
    void Fill(const Vector<double, N>& value,
              ExecutionPolicy policy = ExecutionPolicy::Parallel) override;

    //! Fills the grid with given function.
    void Fill(
        const std::function<Vector<double, N>(const Vector<double, N>&)>& func,
        ExecutionPolicy policy = ExecutionPolicy::Parallel) override;

    //! Returns the copy of the grid instance.
    [[nodiscard]] std::shared_ptr<VectorGrid<N>> Clone() const override;

    //!
    //! \brief Invokes the given function \p func for each u-data point.
    //!
    //! This function invokes the given function object \p func for each u-data
    //! point in serial manner. The input parameters are i, j (and k for 3-D)
    //! indices of a u-data point. The order of execution is i-first, j-next.
    //!
    void ForEachUIndex(
        const std::function<void(const Vector<size_t, N>&)>& func) const;

    //!
    //! \brief Invokes the given function \p func for each u-data point in
    //! parallel.
    //!
    //! This function invokes the given function object \p func for each u-data
    //! point in parallel manner. The input parameters are i, j (and k for 3-D)
    //! indices of a u-data point. The order of execution can be arbitrary since
    //! it's multi-threaded.
    //!
    void ParallelForEachUIndex(
        const std::function<void(const Vector<size_t, N>&)>& func) const;

    //!
    //! \brief Invokes the given function \p func for each v-data point.
    //!
    //! This function invokes the given function object \p func for each v-data
    //! point in serial manner. The input parameters are i, j (and k for 3-D)
    //! indices of a v-data point. The order of execution is i-first, j-next.
    //!
    void ForEachVIndex(
        const std::function<void(const Vector<size_t, N>&)>& func) const;

    //!
    //! \brief Invokes the given function \p func for each v-data point in
    //! parallel.
    //!
    //! This function invokes the given function object \p func for each v-data
    //! point in parallel manner. The input parameters are i, j (and k for 3-D)
    //! indices of a v-data point. The order of execution can be arbitrary since
    //! it's multi-threaded.
    //!
    void ParallelForEachVIndex(
        const std::function<void(const Vector<size_t, N>&)>& func) const;

    //!
    //! \brief Invokes the given function \p func for each w-data point.
    //!
    //! This function invokes the given function object \p func for each w-data
    //! point in serial manner. The input parameters are i, j (and k for 3-D)
    //! indices of a w-data point. The order of execution is i-first, j-next.
    //!
    template <size_t M = N>
    std::enable_if_t<M == 3, void> ForEachWIndex(
        const std::function<void(const Vector<size_t, N>&)>& func) const
    {
        ForEachIndex(DataSize(2), GetUnroll<void, N>::Unroll(func));
    }

    //!
    //! \brief Invokes the given function \p func for each w-data point
    //! parallelly.
    //!
    //! This function invokes the given function object \p func for each w-data
    //! point in parallel manner. The input parameters are i, j (and k for 3-D)
    //! indices of a w-data point. The order of execution can be arbitrary since
    //! it's multi-threaded.
    //!
    template <size_t M = N>
    std::enable_if_t<M == 3, void> ParallelForEachWIndex(
        const std::function<void(const Vector<size_t, N>&)>& func) const
    {
        ParallelForEachIndex(DataSize(2), GetUnroll<void, N>::Unroll(func));
    }

    //! Returns sampled value at given position \p x.
    [[nodiscard]] Vector<double, N> Sample(
        const Vector<double, N>& x) const override;

    //! Returns divergence at given position \p x.
    [[nodiscard]] double Divergence(const Vector<double, N>& x) const override;

    //! Returns curl at given position \p x.
    [[nodiscard]] typename GetCurl<N>::Type Curl(
        const Vector<double, N>& x) const override;

    //!
    //! \brief Returns the sampler function.
    //!
    //! This function returns the data sampler function object. The sampling
    //! function is linear.
    //!
    [[nodiscard]] std::function<Vector<double, N>(const Vector<double, N>&)>
    Sampler() const override;

    //! Returns builder fox FaceCenteredGrid.
    static Builder GetBuilder();

 protected:
    using VectorGrid<N>::SwapGrid;
    using VectorGrid<N>::SetGrid;

    void OnResize(const Vector<size_t, N>& resolution,
                  const Vector<double, N>& gridSpacing,
                  const Vector<double, N>& origin,
                  const Vector<double, N>& initialValue) final;

    //! Fetches the data into a continuous linear array.
    void GetData(Array1<double>& data) const override;

    //! Sets the data from a continuous linear array.
    void SetData(const ConstArrayView1<double>& data) override;

 private:
    std::array<Array<double, N>, N> m_data;
    std::array<Vector<double, N>, N> m_dataOrigins;
    std::array<LinearArraySampler<double, N>, N> m_linearSamplers;
    std::function<Vector<double, N>(const Vector<double, N>&)> m_sampler;

    void ResetSampler();
};

//! 2-D FaceCenteredGrid type.
using FaceCenteredGrid2 = FaceCenteredGrid<2>;

//! 3-D FaceCenteredGrid type.
using FaceCenteredGrid3 = FaceCenteredGrid<3>;

//! Shared pointer type for the FaceCenteredGrid2.
using FaceCenteredGrid2Ptr = std::shared_ptr<FaceCenteredGrid2>;

//! Shared pointer type for the FaceCenteredGrid3.
using FaceCenteredGrid3Ptr = std::shared_ptr<FaceCenteredGrid3>;

//!
//! \brief Front-end to create FaceCenteredGrid objects step by step.
//!
template <size_t N>
class FaceCenteredGrid<N>::Builder final : public VectorGridBuilder<N>
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

    //! Builds FaceCenteredGrid instance.
    FaceCenteredGrid Build() const;

    //! Builds shared pointer of FaceCenteredGrid instance.
    std::shared_ptr<FaceCenteredGrid> MakeShared() const;

    //!
    //! \brief Builds shared pointer of FaceCenteredGrid instance.
    //!
    //! This is an overriding function that implements VectorGridBuilder2.
    //!
    std::shared_ptr<VectorGrid<N>> Build(
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