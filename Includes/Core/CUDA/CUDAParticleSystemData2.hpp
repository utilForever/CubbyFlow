// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_CUDA_PARTICLE_SYSTEM_DATA2_HPP
#define CUBBYFLOW_CUDA_PARTICLE_SYSTEM_DATA2_HPP

#ifdef CUBBYFLOW_USE_CUDA

#include <Core/Array/ArrayView.hpp>
#include <Core/CUDA/CUDAArray.hpp>
#include <Core/CUDA/CUDAPointHashGridSearcher2.hpp>
#include <Core/Matrix/Matrix.hpp>

#if defined(__HIP__)
#include <Core/CUDA/cuda_to_hip.h>
#else
#include <cuda_runtime.h>
#endif

namespace CubbyFlow
{
class CUDAParticleSystemData2
{
 public:
    //! Scalar int data chunk.
    using IntData = CUDAArray1<int>;

    //! Scalar float data chunk.
    using FloatData = CUDAArray1<float>;

    //! Vector data chunk.
    using VectorData = CUDAArray1<float2>;

    //! Default constructor.
    CUDAParticleSystemData2();

    //! Constructs particle system data with given number of particles.
    explicit CUDAParticleSystemData2(size_t numberOfParticles);

    //! Default virtual destructor.
    virtual ~CUDAParticleSystemData2() = default;

    //! Copy constructor.
    CUDAParticleSystemData2(const CUDAParticleSystemData2& other);

    //! Move constructor.
    CUDAParticleSystemData2(CUDAParticleSystemData2&& other) noexcept;

    //! Copy assignment operator.
    CUDAParticleSystemData2& operator=(const CUDAParticleSystemData2& other);

    //! Move assignment operator.
    CUDAParticleSystemData2& operator=(
        CUDAParticleSystemData2&& other) noexcept;

    //!
    //! \brief Resizes the number of particles of the container.
    //!
    //! \param[in]  newNumberOfParticles    New number of particles.
    //!
    void Resize(size_t newNumberOfParticles);

    //! Returns the number of particles.
    size_t NumberOfParticles() const;

    //!
    //! \brief Adds a int data layer and returns its index.
    //!
    //! \param[in] initialVal  Initial value of the new int data.
    //!
    size_t AddIntData(int initialVal = 0);

    //!
    //! \brief Adds a float data layer and returns its index.
    //!
    //! \param[in] initialVal  Initial value of the new float data.
    //!
    size_t AddFloatData(float initialVal = 0.0f);

    //!
    //! \brief Adds a vector data layer and returns its index.
    //!
    //! \param[in] initialVal  Initial value of the new vector data.
    //!
    size_t AddVectorData(const Vector2F& initialVal = Vector2F{});

    //! Returns the number of int data layers.
    size_t NumberOfIntData() const;

    //! Returns the number of float data layers.
    size_t NumberOfFloatData() const;

    //! Returns the number of vector data layers.
    size_t NumberOfVectorData() const;

    //! Returns the position array (immutable).
    ConstCUDAArrayView1<float2> Positions() const;

    //! Returns the position array (mutable).
    CUDAArrayView1<float2> Positions();

    //! Returns the velocity array (immutable).
    ConstCUDAArrayView1<float2> Velocities() const;

    //! Returns the velocity array (mutable).
    CUDAArrayView1<float2> Velocities();

    //! Returns custom int data layer at given index (immutable).
    ConstCUDAArrayView1<int> IntDataAt(size_t idx) const;

    //! Returns custom int data layer at given index (mutable).
    CUDAArrayView1<int> IntDataAt(size_t idx);

    //! Returns custom float data layer at given index (immutable).
    ConstCUDAArrayView1<float> FloatDataAt(size_t idx) const;

    //! Returns custom float data layer at given index (mutable).
    CUDAArrayView1<float> FloatDataAt(size_t idx);

    //! Returns custom vector data layer at given index (immutable).
    ConstCUDAArrayView1<float2> VectorDataAt(size_t idx) const;

    //! Returns custom vector data layer at given index (mutable).
    CUDAArrayView1<float2> VectorDataAt(size_t idx);

    //!
    //! \brief Adds a particle to the data structure.
    //!
    //! \param[in]  newPosition The new position.
    //! \param[in]  newVelocity The new velocity.
    //!
    void AddParticle(const Vector2F& newPosition,
                     const Vector2F& newVelocity = Vector2F{});

    //!
    //! \brief Adds particles to the data structure (CPU).
    //!
    //! \param[in]  newPositions  The new positions.
    //! \param[in]  newVelocities The new velocities.
    //!
    void AddParticles(
        ConstArrayView1<Vector2F> newPositions,
        ConstArrayView1<Vector2F> newVelocities = ArrayView1<Vector2F>{});

    //!
    //! \brief Adds particles to the data structure (GPU).
    //!
    //! \param[in]  newPositions  The new positions.
    //! \param[in]  newVelocities The new velocities.
    //!
    void AddParticles(
        ConstCUDAArrayView1<float2> newPositions,
        ConstCUDAArrayView1<float2> newVelocities = CUDAArrayView1<float2>{});

    //! Returns neighbor starts.
    ConstCUDAArrayView1<uint32_t> NeighborStarts() const;

    //! Returns neighbor ends.
    ConstCUDAArrayView1<uint32_t> NeighborEnds() const;

    //!
    //! \brief Returns neighbor searcher.
    //!
    //! \return Current neighbor searcher.
    //!
    const CUDAPointHashGridSearcher2* NeighborSearcher() const;

    //!
    //! \brief Returns neighbor lists.
    //!
    //! \return Neighbor lists.
    //!
    ConstCUDAArrayView1<uint32_t> NeighborLists() const;

    //! Builds neighbor searcher with given search radius.
    void BuildNeighborSearcher(float maxSearchRadius);

    //! Builds neighbor lists with given search radius.
    void BuildNeighborLists(float maxSearchRadius);

    //! Copies from other particle system data.
    void Set(const CUDAParticleSystemData2& other);

 protected:
    CUDAPointHashGridSearcher2Ptr m_neighborSearcher;
    CUDAArray1<uint32_t> m_neighborStarts;
    CUDAArray1<uint32_t> m_neighborEnds;
    CUDAArray1<uint32_t> m_neighborLists;

 private:
    size_t m_numberOfParticles = 0;
    size_t m_positionIdx;
    size_t m_velocityIdx;

    std::vector<IntData> m_intDataList;
    std::vector<FloatData> m_floatDataList;
    std::vector<VectorData> m_vectorDataList;
};

//! Shared pointer type of CUDAParticleSystemData2.
using CUDAParticleSystemData2Ptr = std::shared_ptr<CUDAParticleSystemData2>;
}  // namespace CubbyFlow

#endif

#endif