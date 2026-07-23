// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_POINT_PARALLEL_HASH_GRID_SEARCHER_HPP
#define CUBBYFLOW_POINT_PARALLEL_HASH_GRID_SEARCHER_HPP

#include <Core/Matrix/Matrix.hpp>
#include <Core/Searcher/PointNeighborSearcher.hpp>

namespace CubbyFlow
{
//!
//! \brief Parallel version of hash grid-based N-D point searcher.
//!
//! This class implements parallel version of N-D point searcher by using hash
//! grid for its internal acceleration data structure. Each point is recorded to
//! its corresponding bucket where the hashing function is N-D grid mapping.
//!
template <size_t N>
class PointParallelHashGridSearcher final : public PointNeighborSearcher<N>
{
 public:
    CUBBYFLOW_NEIGHBOR_SEARCHER_TYPE_NAME(PointParallelHashGridSearcher, N)

    class Builder;

    using typename PointNeighborSearcher<N>::ForEachNearbyPointFunc;
    using PointNeighborSearcher<N>::Build;

    //!
    //! \brief      Constructs hash grid with given resolution and grid spacing.
    //!
    //! This constructor takes hash grid resolution and its grid spacing as
    //! its input parameters. The grid spacing must be 2x or greater than
    //! search radius.
    //!
    //! \param[in]  resolution  The resolution.
    //! \param[in]  gridSpacing The grid spacing.
    //!
    PointParallelHashGridSearcher(const Vector<size_t, N>& resolution,
                                  double gridSpacing);

    //! Default virtual destructor.
    ~PointParallelHashGridSearcher() override = default;

    //! Copy constructor.
    PointParallelHashGridSearcher(const PointParallelHashGridSearcher& other);

    //! Move constructor.
    PointParallelHashGridSearcher(
        PointParallelHashGridSearcher&& other) noexcept;

    //! Copy assignment operator.
    PointParallelHashGridSearcher& operator=(
        const PointParallelHashGridSearcher& other);

    //! Move assignment operator.
    PointParallelHashGridSearcher& operator=(
        PointParallelHashGridSearcher&& other) noexcept;

    //!
    //! \brief Builds internal acceleration structure for given points list.
    //!
    //! This function builds the hash grid for given points in parallel.
    //!
    //! \param[in]  points  The points to be added.
    //!
    void Build(const ConstArrayView1<Vector<double, N>>& points) override;

    //!
    //! \brief Builds internal acceleration structure for given points list and
    //!        max search radius.
    //!
    //! This function builds the hash grid for given points in parallel.
    //!
    //! \param[in]  points The points to be added.
    //! \param[in]  maxSearchRadius Max search radius.
    //!
    void Build(const ConstArrayView1<Vector<double, N>>& points,
               double maxSearchRadius) override;

    //!
    //! Invokes the callback function for each nearby point around the origin
    //! within given radius.
    //!
    //! \param[in]  origin   The origin position.
    //! \param[in]  radius   The search radius.
    //! \param[in]  callback The callback function.
    //!
    void ForEachNearbyPoint(
        const Vector<double, N>& origin, double radius,
        const ForEachNearbyPointFunc& callback) const override;

    //!
    //! Returns true if there are any nearby points for given origin within
    //! radius.
    //!
    //! \param[in]  origin The origin.
    //! \param[in]  radius The radius.
    //!
    //! \return     True if has nearby point, false otherwise.
    //!
    [[nodiscard]] bool HasNearbyPoint(const Vector<double, N>& origin,
                                      double radius) const override;

    //!
    //! \brief      Returns the hash key list.
    //!
    //! The hash key list maps sorted point index i to its hash key value.
    //! The sorting order is based on the key value itself.
    //!
    //! \return     The hash key list.
    //!
    [[nodiscard]] ConstArrayView1<size_t> Keys() const;

    //!
    //! \brief      Returns the start index table.
    //!
    //! The start index table maps the hash grid bucket index to starting index
    //! of the sorted point list. Assume the hash key list looks like:
    //!
    //! \code
    //! [5|8|8|10|10|10]
    //! \endcode
    //!
    //! Then startIndexTable and endIndexTable should be like:
    //!
    //! \code
    //! [.....|0|...|1|..|3|..]
    //! [.....|1|...|3|..|6|..]
    //!       ^5    ^8   ^10
    //! \endcode
    //!
    //! So that endIndexTable[i] - startIndexTable[i] is the number points
    //! in i-th table bucket.
    //!
    //! \return     The start index table.
    //!
    [[nodiscard]] ConstArrayView1<size_t> StartIndexTable() const;

    //!
    //! \brief      Returns the end index table.
    //!
    //! The end index table maps the hash grid bucket index to starting index
    //! of the sorted point list. Assume the hash key list looks like:
    //!
    //! \code
    //! [5|8|8|10|10|10]
    //! \endcode
    //!
    //! Then startIndexTable and endIndexTable should be like:
    //!
    //! \code
    //! [.....|0|...|1|..|3|..]
    //! [.....|1|...|3|..|6|..]
    //!       ^5    ^8   ^10
    //! \endcode
    //!
    //! So that endIndexTable[i] - startIndexTable[i] is the number points
    //! in i-th table bucket.
    //!
    //! \return     The end index table.
    //!
    [[nodiscard]] ConstArrayView1<size_t> EndIndexTable() const;

    //!
    //! \brief      Returns the sorted indices of the points.
    //!
    //! When the hash grid is built, it sorts the points in hash key order. But
    //! rather than sorting the original points, this class keeps the shuffled
    //! indices of the points. The list this function returns maps sorted index
    //! i to original index j.
    //!
    //! \return     The sorted indices of the points.
    //!
    [[nodiscard]] ConstArrayView1<size_t> SortedIndices() const;

    //!
    //! \brief      Creates a new instance of the object with same properties
    //!             than original.
    //!
    //! \return     Copy of this object.
    //!
    [[nodiscard]] std::shared_ptr<PointNeighborSearcher<N>> Clone()
        const override;

    //! Copy from the other instance.
    void Set(const PointParallelHashGridSearcher& other);

    //! Serializes the neighbor searcher into the buffer.
    void Serialize(std::vector<uint8_t>* buffer) const override;

    //! Deserializes the neighbor searcher from the buffer.
    void Deserialize(const std::vector<uint8_t>& buffer) override;

    //! Returns builder fox PointParallelHashGridSearcher.
    static Builder GetBuilder();

 private:
    friend class PointParallelHashGridSearcherTests;

    template <size_t M = N>
        requires(M == 2)
    static void Serialize(const PointParallelHashGridSearcher<2>& searcher,
                          std::vector<uint8_t>* buffer);

    template <size_t M = N>
        requires(M == 3)
    static void Serialize(const PointParallelHashGridSearcher<3>& searcher,
                          std::vector<uint8_t>* buffer);

    template <size_t M = N>
        requires(M == 2)
    static void Deserialize(const std::vector<uint8_t>& buffer,
                            PointParallelHashGridSearcher<2>& searcher);

    template <size_t M = N>
        requires(M == 3)
    static void Deserialize(const std::vector<uint8_t>& buffer,
                            PointParallelHashGridSearcher<3>& searcher);

    double m_gridSpacing = 1.0;
    Vector<ssize_t, N> m_resolution = Vector<ssize_t, N>::MakeConstant(1);
    Array1<Vector<double, N>> m_points;
    Array1<size_t> m_keys;
    Array1<size_t> m_startIndexTable;
    Array1<size_t> m_endIndexTable;
    Array1<size_t> m_sortedIndices;
};

//! 2-D PointParallelHashGridSearcher type.
using PointParallelHashGridSearcher2 = PointParallelHashGridSearcher<2>;

//! 3-D PointParallelHashGridSearcher type.
using PointParallelHashGridSearcher3 = PointParallelHashGridSearcher<3>;

//! Shared pointer for the PointParallelHashGridSearcher2 type.
using PointParallelHashGridSearcher2Ptr =
    std::shared_ptr<PointParallelHashGridSearcher2>;

//! Shared pointer for the PointParallelHashGridSearcher3 type.
using PointParallelHashGridSearcher3Ptr =
    std::shared_ptr<PointParallelHashGridSearcher3>;

//!
//! \brief Front-end to create PointParallelHashGridSearcher objects step by
//!        step.
//!
template <size_t N>
class PointParallelHashGridSearcher<N>::Builder final
    : public PointNeighborSearcherBuilder<N>
{
 public:
    //! Default constructor.
    Builder() = default;

    //! Default virtual destructor.
    ~Builder() override = default;

    //! Deleted copy constructor.
    Builder(const Builder& other) = delete;

    //! Deleted move constructor.
    Builder(Builder&& other) noexcept = delete;

    //! Deleted copy assignment operator.
    Builder& operator=(const Builder& other) = delete;

    //! Deleted move assignment operator.
    Builder& operator=(Builder&& other) noexcept = delete;

    //! Returns builder with resolution.
    Builder& WithResolution(const Vector<size_t, N>& resolution);

    //! Returns builder with grid spacing.
    Builder& WithGridSpacing(double gridSpacing);

    //! Builds PointParallelHashGridSearcher instance.
    PointParallelHashGridSearcher<N> Build() const;

    //! Builds shared pointer of PointParallelHashGridSearcher instance.
    std::shared_ptr<PointParallelHashGridSearcher<N>> MakeShared() const;

    //! Returns shared pointer of PointNeighborSearcher type.
    std::shared_ptr<PointNeighborSearcher<N>> BuildPointNeighborSearcher()
        const override;

 private:
    Vector<size_t, N> m_resolution = Vector<size_t, N>::MakeConstant(64);
    double m_gridSpacing = 1.0;
};
}  // namespace CubbyFlow

#endif
