// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_CUDA_POINT_HASH_GRID_SEARCHER3_HPP
#define CUBBYFLOW_CUDA_POINT_HASH_GRID_SEARCHER3_HPP

#ifdef CUBBYFLOW_USE_CUDA

#include <Core/CUDA/CUDAArrayView.hpp>
#include <Core/Matrix/Matrix.hpp>

#include <cuda_runtime.h>

namespace CubbyFlow
{
//!
//! \brief Parallel version of hash grid-based 3-D point searcher.
//!
//! This class implements parallel version of 3-D point searcher by using hash
//! grid for its internal acceleration data structure. Each point is recorded to
//! its corresponding bucket where the hashing function is 3-D grid mapping.
//!
class CUDAPointHashGridSearcher3 final
{
 public:
    class Builder;

    class HashUtils
    {
     public:
        HashUtils() = default;

        inline CUBBYFLOW_CUDA_HOST_DEVICE HashUtils(float gridSpacing,
                                                    uint3 resolution);

        inline CUBBYFLOW_CUDA_HOST_DEVICE void GetNearbyKeys(
            float4 position, uint32_t* nearbyKeys) const;

        inline CUBBYFLOW_CUDA_HOST_DEVICE int3
        GetBucketIndex(float4 position) const;

        inline CUBBYFLOW_CUDA_HOST_DEVICE uint32_t
        GetHashKeyFromBucketIndex(int3 bucketIndex) const;

        inline CUBBYFLOW_CUDA_HOST_DEVICE uint32_t
        GetHashKeyFromPosition(float4 position) const;

     private:
        float m_gridSpacing = 1.0f;
        uint3 m_resolution = make_uint3(1, 1, 1);
    };

    template <typename Callback>
    class ForEachNearbyPointFunc
    {
     public:
        inline CUBBYFLOW_CUDA_HOST_DEVICE ForEachNearbyPointFunc(
            float r, float gridSpacing, uint3 resolution, const uint32_t* sit,
            const uint32_t* eit, const uint32_t* si, const float4* p,
            const float4* o, Callback cb);

        template <typename Index>
        inline CUBBYFLOW_CUDA_HOST_DEVICE void operator()(Index idx);

     private:
        HashUtils m_hashUtils;
        float m_radius;
        const uint32_t* m_startIndexTable;
        const uint32_t* m_endIndexTable;
        const uint32_t* m_sortedIndices;
        const float4* m_points;
        const float4* m_origins;
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
    CUDAPointHashGridSearcher3(const uint3& resolution, float gridSpacing);

    //!
    //! \brief Constructs hash grid with given resolution and grid spacing.
    //!
    //! This constructor takes hash grid resolution and its grid spacing as
    //! its input parameters. The grid spacing must be 2x or greater than
    //! search radius.
    //!
    //! \param[in]  resolutionX The resolution x.
    //! \param[in]  resolutionY The resolution y.
    //! \param[in]  resolutionZ The resolution z.
    //! \param[in]  gridSpacing The grid spacing.
    //!
    CUDAPointHashGridSearcher3(uint32_t resolutionX, uint32_t resolutionY,
                               uint32_t resolutionZ, float gridSpacing);

    //! Copy constructor.
    CUDAPointHashGridSearcher3(const CUDAPointHashGridSearcher3& other);

    //! Move constructor.
    CUDAPointHashGridSearcher3(CUDAPointHashGridSearcher3&& other) noexcept;

    //! Default destructor.
    ~CUDAPointHashGridSearcher3() = default;

    //! Copy assignment operator.
    CUDAPointHashGridSearcher3& operator=(
        const CUDAPointHashGridSearcher3& other);

    //! Move assignment operator.
    CUDAPointHashGridSearcher3& operator=(
        CUDAPointHashGridSearcher3&& other) noexcept;

    //!
    //! \brief Builds internal acceleration structure for given points list.
    //!
    //! This function builds the hash grid for given points in parallel.
    //!
    //! \param[in]  points The points to be added.
    //!
    void Build(const ConstCUDAArrayView1<float4>& points);

    //!
    //! Invokes the callback function for each nearby point around the
    //! origin within given radius.
    //!
    //! \param[in]  origins  Array of the origin positions.
    //! \param[in]  radii    The search radius.
    //! \param[in]  callback The callback function.
    //!
    template <typename Callback>
    void ForEachNearbyPoint(const ConstCUDAArrayView1<float4>& origins,
                            float radius, Callback callback) const;

    float GridSpacing() const;

    Vector3UZ Resolution() const;

    ConstCUDAArrayView1<float4> SortedPoints() const;

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
    //! [.....|0|...|1|..|3|..]
    //! [.....|1|...|3|..|6|..]
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
    //! [.....|0|...|1|..|3|..]
    //! [.....|1|...|3|..|6|..]
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
    void Set(const CUDAPointHashGridSearcher3& other);

    //! Returns builder fox CUDAPointHashGridSearcher3.
    static Builder GetBuilder();

 private:
    float m_gridSpacing = 1.0f;
    uint3 m_resolution = make_uint3(1, 1, 1);
    CUDAArray1<float4> m_points;
    CUDAArray1<uint32_t> m_keys;
    CUDAArray1<uint32_t> m_startIndexTable;
    CUDAArray1<uint32_t> m_endIndexTable;
    CUDAArray1<uint32_t> m_sortedIndices;
};

//! Shared pointer for the CUDAPointHashGridSearcher3 type.
using CUDAPointHashGridSearcher3Ptr =
    std::shared_ptr<CUDAPointHashGridSearcher3>;

//!
//! \brief Front-end to create CUDAPointHashGridSearcher3 objects step by
//!        step.
//!
class CUDAPointHashGridSearcher3::Builder final
{
 public:
    //! Returns builder with resolution.
    Builder& WithResolution(const Vector3UZ& resolution);

    //! Returns builder with grid spacing.
    Builder& WithGridSpacing(float gridSpacing);

    //! Builds CUDAPointHashGridSearcher3 instance.
    CUDAPointHashGridSearcher3 Build() const;

    //! Builds shared pointer of CUDAPointHashGridSearcher3 instance.
    CUDAPointHashGridSearcher3Ptr MakeShared() const;

 private:
    float m_gridSpacing = 1.0f;
    Vector3UZ m_resolution{ 64, 64, 64 };
};
}  // namespace CubbyFlow

#include <Core/CUDA/CUDAPointHashGridSearcher3-Impl.hpp>

#endif

#endif
