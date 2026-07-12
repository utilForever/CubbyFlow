// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_CUDA_POINT_HASH_GRID_SEARCHER2_HPP
#define CUBBYFLOW_CUDA_POINT_HASH_GRID_SEARCHER2_HPP

#ifdef CUBBYFLOW_USE_CUDA

#include <Core/CUDA/CUDAArrayView.hpp>
#include <Core/Matrix/Matrix.hpp>

#if defined(__HIP__)
#include <Core/CUDA/cuda_to_hip.h>
#else
#include <cuda_runtime.h>
#endif

namespace CubbyFlow
{
//!
//! \brief Parallel version of hash grid-based 2-D point searcher.
//!
//! This class implements parallel version of 2-D point searcher by using hash
//! grid for its internal acceleration data structure. Each point is recorded to
//! its corresponding bucket where the hashing function is 2-D grid mapping.
//!
class CUDAPointHashGridSearcher2 final
{
 public:
    class Builder;

    class HashUtils
    {
     public:
        HashUtils() = default;

        inline CUBBYFLOW_CUDA_HOST_DEVICE HashUtils(float gridSpacing,
                                                    uint2 resolution);

        inline CUBBYFLOW_CUDA_HOST_DEVICE void GetNearbyKeys(
            float2 position, uint32_t* nearbyKeys) const;

        inline CUBBYFLOW_CUDA_HOST_DEVICE int2
        GetBucketIndex(float2 position) const;

        inline CUBBYFLOW_CUDA_HOST_DEVICE uint32_t
        GetHashKeyFromBucketIndex(int2 bucketIndex) const;

        inline CUBBYFLOW_CUDA_HOST_DEVICE uint32_t
        GetHashKeyFromPosition(float2 position) const;

     private:
        float m_gridSpacing = 1.0f;
        uint32_t m_dummy = 0;
        uint2 m_resolution = make_uint2(1, 1);
    };

    template <typename Callback>
    class ForEachNearbyPointFunc
    {
     public:
        inline CUBBYFLOW_CUDA_HOST_DEVICE ForEachNearbyPointFunc(
            float r, float gridSpacing, uint2 resolution, const uint32_t* sit,
            const uint32_t* eit, const uint32_t* si, const float2* p,
            const float2* o, Callback cb);

        template <typename Index>
        inline CUBBYFLOW_CUDA_HOST_DEVICE void operator()(Index idx);

     private:
        HashUtils m_hashUtils;
        float m_radius;
        const uint32_t* m_startIndexTable;
        const uint32_t* m_endIndexTable;
        const uint32_t* m_sortedIndices;
        const float2* m_points;
        const float2* m_origins;
        Callback m_callback;
    };

    //!
    //! \brief Constructs hash grid with given resolution and grid spacing.
    //!
    //! This constructor takes hash grid resolution and its grid spacing as
    //! its input parameters. The grid spacing must be 2x or greater than
    //! search radius.
    //!
    //! \param[in]  resolution  The resolution.
    //! \param[in]  gridSpacing The grid spacing.
    //!
    CUDAPointHashGridSearcher2(const uint2& resolution, float gridSpacing);

    //!
    //! \brief Constructs hash grid with given resolution and grid spacing.
    //!
    //! This constructor takes hash grid resolution and its grid spacing as
    //! its input parameters. The grid spacing must be 2x or greater than
    //! search radius.
    //!
    //! \param[in]  resolutionX The resolution x.
    //! \param[in]  resolutionY The resolution y.
    //! \param[in]  gridSpacing The grid spacing.
    //!
    CUDAPointHashGridSearcher2(uint32_t resolutionX, uint32_t resolutionY,
                               float gridSpacing);

    //! Copy constructor.
    CUDAPointHashGridSearcher2(const CUDAPointHashGridSearcher2& other);

    //! Move constructor.
    CUDAPointHashGridSearcher2(CUDAPointHashGridSearcher2&& other) noexcept;

    //! Default destructor.
    ~CUDAPointHashGridSearcher2() = default;

    //! Copy assignment operator.
    CUDAPointHashGridSearcher2& operator=(
        const CUDAPointHashGridSearcher2& other);

    //! Move assignment operator.
    CUDAPointHashGridSearcher2& operator=(
        CUDAPointHashGridSearcher2&& other) noexcept;

    //!
    //! \brief Builds internal acceleration structure for given points list.
    //!
    //! This function builds the hash grid for given points in parallel.
    //!
    //! \param[in]  points The points to be added.
    //!
    void Build(const ConstCUDAArrayView1<float2>& points);

    //!
    //! Invokes the callback function for each nearby point around the
    //! origin within given radius.
    //!
    //! \param[in]  origins  Array of the origin positions.
    //! \param[in]  radii    The search radius.
    //! \param[in]  callback The callback function.
    //!
    template <typename Callback>
    void ForEachNearbyPoint(const ConstCUDAArrayView1<float2>& origins,
                            float radius, Callback callback) const;

    float GridSpacing() const;

    Vector2UZ Resolution() const;

    ConstCUDAArrayView1<float2> SortedPoints() const;

    //!
    //! \brief Returns the hash key list.
    //!
    //! The hash key list maps sorted point index i to its hash key value.
    //! The sorting order is based on the key value itself.
    //!
    //! \return The hash key list.
    //!
    ConstCUDAArrayView1<uint32_t> Keys() const;

    //!
    //! \brief Returns the start index table.
    //!
    //! The start index table maps the hash grid bucket index to starting
    //! index of the sorted point list. Assume the hash key list looks like:
    //!
    //! \code
    //! [5|8|8|10|10|10]
    //! \endcode
    //!
    //! Then startIndexTable and endIndexTable should be like:
    //!
    //! \code
    //! [.....|0|...|1|..|2|..]
    //! [.....|1|...|2|..|6|..]
    //!       ^5    ^8   ^10
    //! \endcode
    //!
    //! So that endIndexTable[i] - startIndexTable[i] is the number points
    //! in i-th table bucket.
    //!
    //! \return The start index table.
    //!
    ConstCUDAArrayView1<uint32_t> StartIndexTable() const;

    //!
    //! \brief Returns the end index table.
    //!
    //! The end index table maps the hash grid bucket index to starting
    //! index of the sorted point list. Assume the hash key list looks like:
    //!
    //! \code
    //! [5|8|8|10|10|10]
    //! \endcode
    //!
    //! Then startIndexTable and endIndexTable should be like:
    //!
    //! \code
    //! [.....|0|...|1|..|2|..]
    //! [.....|1|...|2|..|6|..]
    //!       ^5    ^8   ^10
    //! \endcode
    //!
    //! So that endIndexTable[i] - startIndexTable[i] is the number points
    //! in i-th table bucket.
    //!
    //! \return The end index table.
    //!
    ConstCUDAArrayView1<uint32_t> EndIndexTable() const;

    //!
    //! \brief Returns the sorted indices of the points.
    //!
    //! When the hash grid is built, it sorts the points in hash key order.
    //! But rather than sorting the original points, this class keeps the
    //! shuffled indices of the points. The list this function returns maps
    //! sorted index i to original index j.
    //!
    //! \return The sorted indices of the points.
    //!
    ConstCUDAArrayView1<uint32_t> SortedIndices() const;

    //! Copy from the other instance.
    void Set(const CUDAPointHashGridSearcher2& other);

    //! Returns builder fox CUDAPointHashGridSearcher2.
    static Builder GetBuilder();

 private:
    float m_gridSpacing = 1.0f;
    uint1 m_dummy{};
    uint2 m_resolution = make_uint2(1, 1);
    CUDAArray1<float2> m_points;
    CUDAArray1<uint32_t> m_keys;
    CUDAArray1<uint32_t> m_startIndexTable;
    CUDAArray1<uint32_t> m_endIndexTable;
    CUDAArray1<uint32_t> m_sortedIndices;
};

//! Shared pointer for the CUDAPointHashGridSearcher2 type.
using CUDAPointHashGridSearcher2Ptr =
    std::shared_ptr<CUDAPointHashGridSearcher2>;

//!
//! \brief Front-end to create CUDAPointHashGridSearcher2 objects step by
//!        step.
//!
class CUDAPointHashGridSearcher2::Builder final
{
 public:
    //! Returns builder with resolution.
    Builder& WithResolution(const Vector2UZ& resolution);

    //! Returns builder with grid spacing.
    Builder& WithGridSpacing(float gridSpacing);

    //! Builds CUDAPointHashGridSearcher2 instance.
    CUDAPointHashGridSearcher2 Build() const;

    //! Builds shared pointer of CUDAPointHashGridSearcher2 instance.
    CUDAPointHashGridSearcher2Ptr MakeShared() const;

 private:
    Vector2UZ m_resolution{ 64, 64 };
    float m_gridSpacing = 1.0f;
};
}  // namespace CubbyFlow

#include <Core/CUDA/CUDAPointHashGridSearcher2-Impl.hpp>

#endif

#endif
