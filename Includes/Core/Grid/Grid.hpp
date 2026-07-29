// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_GRID_HPP
#define CUBBYFLOW_GRID_HPP

#include <Core/Geometry/BoundingBox.hpp>
#include <Core/Matrix/Matrix.hpp>
#include <Core/Utils/Serialization.hpp>

#include <string>

namespace CubbyFlow
{
template <size_t N>
class GridDataPositionFunc final
{
 public:
    using RawFunctionType =
        std::function<Vector<double, N>(const Vector<size_t, N>&)>;

    explicit GridDataPositionFunc(const RawFunctionType& func) : m_func(func)
    {
        // Do nothing
    }

    template <typename... Indices>
    Vector<double, N> operator()(size_t i, Indices... indices) const
    {
        return (*this)(Vector<size_t, N>(i, indices...));
    }

    Vector<double, N> operator()(const Vector<size_t, N>& idx) const
    {
        return m_func(idx);
    }

 private:
    RawFunctionType m_func;
};

//!
//! \brief Abstract base class for N-D cartesian grid structure.
//!
//! This class represents N-D cartesian grid structure. This class is an
//! abstract base class and does not store any data. The class only stores the
//! shape of the grid. The grid structure is axis-aligned and can have different
//! grid spacing per axis.
//!
template <size_t N>
class Grid : public Serializable
{
 public:
    //! Constructs an empty grid.
    Grid() = default;

    //! Default virtual destructor.
    ~Grid() override = default;

    //! Copy constructor.
    Grid(const Grid& other);

    //! Move constructor.
    Grid(Grid&& other) noexcept;

    //! Copy assignment operator.
    Grid& operator=(const Grid& other);

    //! Move assignment operator.
    Grid& operator=(Grid&& other) noexcept;

    //! Returns the type name of derived grid.
    [[nodiscard]] virtual std::string TypeName() const = 0;

    //! Returns the grid resolution.
    [[nodiscard]] const Vector<size_t, N>& Resolution() const;

    //! Returns the grid origin.
    [[nodiscard]] const Vector<double, N>& Origin() const;

    //! Returns the grid spacing.
    [[nodiscard]] const Vector<double, N>& GridSpacing() const;

    //! Returns the bounding box of the grid.
    [[nodiscard]] const BoundingBox<double, N>& GetBoundingBox() const;

    //! Returns the function that maps grid index to the cell-center position.
    [[nodiscard]] GridDataPositionFunc<N> CellCenterPosition() const;

    //!
    //! \brief Invokes the given function \p func for each grid cell.
    //!
    //! This function invokes the given function object \p func for each grid
    //! cell in serial manner. The input parameters are i and j indices of a
    //! grid cell. The order of execution is i-first, j-last.
    //!
    void ForEachCellIndex(
        const std::function<void(const Vector<size_t, N>&)>& func) const;

    template <size_t M = N>
    std::enable_if_t<M == 2, void> ForEachCellIndex(
        const std::function<void(size_t, size_t)>& func) const
    {
        ForEachCellIndex([&func](const Vector2UZ& idx) { func(idx.x, idx.y); });
    }

    template <size_t M = N>
    std::enable_if_t<M == 3, void> ForEachCellIndex(
        const std::function<void(size_t, size_t, size_t)>& func) const
    {
        ForEachCellIndex(
            [&func](const Vector3UZ& idx) { func(idx.x, idx.y, idx.z); });
    }

    //!
    //! \brief Invokes the given function \p func for each grid cell in
    //! parallel.
    //!
    //! This function invokes the given function object \p func for each grid
    //! cell in parallel manner. The input parameters are i and j indices of a
    //! grid cell. The order of execution can be arbitrary since it's
    //! multi-threaded.
    //!
    void ParallelForEachCellIndex(
        const std::function<void(const Vector<size_t, N>&)>& func) const;

    template <size_t M = N>
    std::enable_if_t<M == 2, void> ParallelForEachCellIndex(
        const std::function<void(size_t, size_t)>& func) const
    {
        ParallelForEachCellIndex(
            [&func](const Vector2UZ& idx) { func(idx.x, idx.y); });
    }

    template <size_t M = N>
    std::enable_if_t<M == 3, void> ParallelForEachCellIndex(
        const std::function<void(size_t, size_t, size_t)>& func) const
    {
        ParallelForEachCellIndex(
            [&func](const Vector3UZ& idx) { func(idx.x, idx.y, idx.z); });
    }

    //! Returns true if resolution, grid-spacing and origin are same.
    [[nodiscard]] bool HasSameShape(const Grid& other) const;

    //! Swaps the data with other grid.
    virtual void Swap(Grid* other) = 0;

 protected:
    //! Sets the size parameters including the resolution, grid spacing, and
    //! origin.
    void SetSizeParameters(const Vector<size_t, N>& resolution,
                           const Vector<double, N>& gridSpacing,
                           const Vector<double, N>& origin);

    //! Swaps the size parameters with given grid \p other.
    void SwapGrid(Grid* other);

    //! Sets the size parameters with given grid \p other.
    void SetGrid(const Grid& other);

    //! Fetches the data into a continuous linear array.
    virtual void GetData(Array1<double>& data) const = 0;

    //! Sets the data from a continuous linear array.
    virtual void SetData(const ConstArrayView1<double>& data) = 0;

 private:
    Vector<size_t, N> m_resolution;
    Vector<double, N> m_gridSpacing = Vector<double, N>::MakeConstant(1);
    Vector<double, N> m_origin;
    BoundingBox<double, N> m_boundingBox =
        BoundingBox<double, N>{ Vector<double, N>{}, Vector<double, N>{} };
};

//! 2-D Grid type.
using Grid2 = Grid<2>;

//! 3-D Grid type.
using Grid3 = Grid<3>;

//! Shared pointer type for Grid.
using Grid2Ptr = std::shared_ptr<Grid2>;

//! Shared pointer type for Grid3.
using Grid3Ptr = std::shared_ptr<Grid3>;

#define CUBBYFLOW_GRID_TYPE_NAME(DerivedClassName, N)   \
    [[nodiscard]] std::string TypeName() const override \
    {                                                   \
        return #DerivedClassName + std::to_string(N);   \
    }
}  // namespace CubbyFlow

#endif