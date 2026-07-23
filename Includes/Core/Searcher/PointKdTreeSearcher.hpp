// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_POINT_KDTREE_SEARCHER_HPP
#define CUBBYFLOW_POINT_KDTREE_SEARCHER_HPP

#include <Core/Geometry/KdTree.hpp>
#include <Core/Searcher/PointNeighborSearcher.hpp>

namespace CubbyFlow
{
//!
//! \brief KdTree-based N-D point searcher.
//!
//! This class implements N-D point searcher by using KdTree for its internal
//! acceleration data structure.
//!
template <size_t N>
class PointKdTreeSearcher final : public PointNeighborSearcher<N>
{
 public:
    CUBBYFLOW_NEIGHBOR_SEARCHER_TYPE_NAME(PointKdTreeSearcher, N)

    class Builder;

    using typename PointNeighborSearcher<N>::ForEachNearbyPointFunc;
    using PointNeighborSearcher<N>::Build;

    //! Constructs an empty kD-tree instance.
    PointKdTreeSearcher() = default;

    //! Default virtual destructor.
    ~PointKdTreeSearcher() override = default;

    //! Copy constructor.
    PointKdTreeSearcher(const PointKdTreeSearcher& other);

    //! Move constructor.
    PointKdTreeSearcher(PointKdTreeSearcher&& other) noexcept;

    //! Copy assignment operator.
    PointKdTreeSearcher& operator=(const PointKdTreeSearcher& other);

    //! Move assignment operator.
    PointKdTreeSearcher& operator=(PointKdTreeSearcher&& other) noexcept;

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
    //! \brief      Creates a new instance of the object with same properties
    //!             than original.
    //!
    //! \return     Copy of this object.
    //!
    [[nodiscard]] std::shared_ptr<PointNeighborSearcher<N>> Clone()
        const override;

    //! Copy from the other instance.
    void Set(const PointKdTreeSearcher& other);

    //! Serializes the neighbor searcher into the buffer.
    void Serialize(std::vector<uint8_t>* buffer) const override;

    //! Deserializes the neighbor searcher from the buffer.
    void Deserialize(const std::vector<uint8_t>& buffer) override;

    //! Returns builder fox PointKdTreeSearcher.
    static Builder GetBuilder();

 private:
    template <size_t M = N>
        requires(M == 2)
    static void Serialize(const PointKdTreeSearcher<2>& searcher,
                          std::vector<uint8_t>* buffer);

    template <size_t M = N>
        requires(M == 3)
    static void Serialize(const PointKdTreeSearcher<3>& searcher,
                          std::vector<uint8_t>* buffer);

    template <size_t M = N>
        requires(M == 2)
    static void Deserialize(const std::vector<uint8_t>& buffer,
                            PointKdTreeSearcher<2>& searcher);

    template <size_t M = N>
        requires(M == 3)
    static void Deserialize(const std::vector<uint8_t>& buffer,
                            PointKdTreeSearcher<3>& searcher);

    KdTree<double, N> m_tree;
};

//! 2-D PointKdTreeSearcher type.
using PointKdTreeSearcher2 = PointKdTreeSearcher<2>;

//! 3-D PointKdTreeSearcher type.
using PointKdTreeSearcher3 = PointKdTreeSearcher<3>;

//! Shared pointer for the PointKdTreeSearcher2 type.
using PointKdTreeSearcher2Ptr = std::shared_ptr<PointKdTreeSearcher2>;

//! Shared pointer for the PointKdTreeSearcher3 type.
using PointKdTreeSearcher3Ptr = std::shared_ptr<PointKdTreeSearcher3>;

//!
//! \brief Front-end to create PointKdTreeSearcher objects step by step.
//!
template <size_t N>
class PointKdTreeSearcher<N>::Builder final
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

    //! Builds PointKdTreeSearcher instance.
    PointKdTreeSearcher Build() const;

    //! Builds shared pointer of PointKdTreeSearcher instance.
    std::shared_ptr<PointKdTreeSearcher<N>> MakeShared() const;

    //! Returns shared pointer of PointNeighborSearcher3 type.
    std::shared_ptr<PointNeighborSearcher<N>> BuildPointNeighborSearcher()
        const override;
};
}  // namespace CubbyFlow

#endif
