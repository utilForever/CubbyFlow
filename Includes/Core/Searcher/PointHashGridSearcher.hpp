// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_POINT_HASH_GRID_SEARCHER_HPP
#define CUBBYFLOW_POINT_HASH_GRID_SEARCHER_HPP

#include <Core/Array/Array.hpp>
#include <Core/Matrix/Matrix.hpp>
#include <Core/Searcher/PointNeighborSearcher.hpp>
#include <Core/Utils/Macros.hpp>

namespace CubbyFlow
{
//!
//! \brief Hash grid-based N-D point searcher.
//!
//! This class implements N-D point searcher by using hash grid for its internal
//! acceleration data structure. Each point is recorded to its corresponding
//! bucket where the hashing function is N-D grid mapping.
//!
template <size_t N>
class PointHashGridSearcher final : public PointNeighborSearcher<N>
{
 public:
    CUBBYFLOW_NEIGHBOR_SEARCHER_TYPE_NAME(PointHashGridSearcher, N)

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
    PointHashGridSearcher(const Vector<size_t, N>& resolution,
                          double gridSpacing);

    //! Default virtual destructor.
    ~PointHashGridSearcher() override = default;

    //! Copy constructor.
    PointHashGridSearcher(const PointHashGridSearcher& other);

    //! Move constructor.
    PointHashGridSearcher(PointHashGridSearcher&& other) noexcept;

    //! Copy assignment operator.
    PointHashGridSearcher& operator=(const PointHashGridSearcher& other);

    //! Move assignment operator.
    PointHashGridSearcher& operator=(PointHashGridSearcher&& other) noexcept;

    //! Builds internal acceleration structure for given points list and max
    //! search radius.
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
    //! \brief      Adds a single point to the hash grid.
    //!
    //! This function adds a single point to the hash grid for future queries.
    //! It can be used for a hash grid that is already built by calling function
    //! PointHashGridSearcher::build.
    //!
    //! \param[in]  point The point to be added.
    //!
    void Add(const Vector<double, N>& point);

    //!
    //! \brief      Returns the internal bucket.
    //!
    //! A bucket is a list of point indices that has same hash value. This
    //! function returns the (immutable) internal bucket structure.
    //!
    //! \return     List of buckets.
    //!
    [[nodiscard]] const Array1<Array1<size_t>>& Buckets() const;

    //!
    //! \brief      Creates a new instance of the object with same properties
    //!             than original.
    //!
    //! \return     Copy of this object.
    //!
    [[nodiscard]] std::shared_ptr<PointNeighborSearcher<N>> Clone()
        const override;

    //! Copy from the other instance.
    void Set(const PointHashGridSearcher& other);

    //! Serializes the neighbor searcher into the buffer.
    void Serialize(std::vector<uint8_t>* buffer) const override;

    //! Deserializes the neighbor searcher from the buffer.
    void Deserialize(const std::vector<uint8_t>& buffer) override;

    //! Returns builder fox PointHashGridSearcher.
    static Builder GetBuilder();

 private:
    template <size_t M = N>
    CUBBYFLOW_REQUIRES(M == 2)
    static void Serialize(const PointHashGridSearcher<2>& searcher,
                          std::vector<uint8_t>* buffer);

    template <size_t M = N>
    CUBBYFLOW_REQUIRES(M == 3)
    static void Serialize(const PointHashGridSearcher<3>& searcher,
                          std::vector<uint8_t>* buffer);

    template <size_t M = N>
    CUBBYFLOW_REQUIRES(M == 2)
    static void Deserialize(const std::vector<uint8_t>& buffer,
                            PointHashGridSearcher<2>& searcher);

    template <size_t M = N>
    CUBBYFLOW_REQUIRES(M == 3)
    static void Deserialize(const std::vector<uint8_t>& buffer,
                            PointHashGridSearcher<3>& searcher);

    double m_gridSpacing = 1.0;
    Vector<ssize_t, N> m_resolution = Vector<ssize_t, N>::MakeConstant(1);
    Array1<Vector<double, N>> m_points;
    Array1<Array1<size_t>> m_buckets;
};

//! 2-D PointHashGridSearcher type.
using PointHashGridSearcher2 = PointHashGridSearcher<2>;

//! 3-D PointHashGridSearcher type.
using PointHashGridSearcher3 = PointHashGridSearcher<3>;

//! Shared pointer for the PointHashGridSearcher2 type.
using PointHashGridSearcher2Ptr = std::shared_ptr<PointHashGridSearcher2>;

//! Shared pointer for the PointHashGridSearcher3 type.
using PointHashGridSearcher3Ptr = std::shared_ptr<PointHashGridSearcher3>;

//!
//! \brief Front-end to create PointHashGridSearcher objects step by step.
//!
template <size_t N>
class PointHashGridSearcher<N>::Builder final
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

    //! Builds PointHashGridSearcher instance.
    PointHashGridSearcher<N> Build() const;

    //! Builds shared pointer of PointHashGridSearcher instance.
    std::shared_ptr<PointHashGridSearcher<N>> MakeShared() const;

    //! Returns shared pointer of PointNeighborSearcher3 type.
    std::shared_ptr<PointNeighborSearcher<N>> BuildPointNeighborSearcher()
        const override;

 private:
    Vector<size_t, N> m_resolution = Vector<size_t, N>::MakeConstant(64);
    double m_gridSpacing = 1.0;
};
}  // namespace CubbyFlow

#endif
