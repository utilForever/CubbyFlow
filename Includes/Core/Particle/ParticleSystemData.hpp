// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_PARTICLE_SYSTEM_DATA_HPP
#define CUBBYFLOW_PARTICLE_SYSTEM_DATA_HPP

#include <Core/Array/Array.hpp>
#include <Core/Searcher/PointNeighborSearcher.hpp>
#include <Core/Utils/Serialization.hpp>

#ifndef CUBBYFLOW_DOXYGEN

namespace flatbuffers
{
class FlatBufferBuilder;
template <typename T>
struct Offset;
}  // namespace flatbuffers

namespace CubbyFlow
{
namespace fbs
{
struct ParticleSystemData2;
struct ParticleSystemData3;
}  // namespace fbs
}  // namespace CubbyFlow

#endif

namespace CubbyFlow
{
//!
//! \brief      N-D particle system data.
//!
//! This class is the key data structure for storing particle system data. A
//! single particle has position, velocity, and force attributes by default. But
//! it can also have additional custom scalar or vector attributes.
//!
template <size_t N>
class ParticleSystemData : public Serializable
{
 public:
    //! Scalar data chunk.
    using ScalarData = Array1<double>;

    //! Vector data chunk.
    using VectorData = Array1<Vector<double, N>>;

    //! Default constructor.
    ParticleSystemData();

    //! Constructs particle system data with given number of particles.
    explicit ParticleSystemData(size_t numberOfParticles);

    //! Default virtual destructor.
    ~ParticleSystemData() override = default;

    //! Copy constructor.
    ParticleSystemData(const ParticleSystemData& other);

    //! Move constructor.
    ParticleSystemData(ParticleSystemData&& other) noexcept;

    //! Copy assignment operator.
    ParticleSystemData& operator=(const ParticleSystemData& other);

    //! Move assignment operator.
    ParticleSystemData& operator=(ParticleSystemData&& other) noexcept;

    //!
    //! \brief      Resizes the number of particles of the container.
    //!
    //! This function will resize internal containers to store newly given
    //! number of particles including custom data layers. However, this will
    //! invalidate neighbor searcher and neighbor lists. It is users
    //! responsibility to call ParticleSystemData::BuildNeighborSearcher and
    //! ParticleSystemData::BuildNeighborLists to refresh those data.
    //!
    //! \param[in]  newNumberOfParticles    New number of particles.
    //!
    virtual void Resize(size_t newNumberOfParticles);

    //! Returns the number of particles.
    [[nodiscard]] size_t NumberOfParticles() const;

    //!
    //! \brief      Adds a scalar data layer and returns its index.
    //!
    //! This function adds a new scalar data layer to the system. It can be used
    //! for adding a scalar attribute, such as temperature, to the particles.
    //!
    //! \param[in] initialVal  Initial value of the new scalar data.
    //!
    [[nodiscard]] size_t AddScalarData(double initialVal = 0.0);

    //!
    //! \brief      Adds a vector data layer and returns its index.
    //!
    //! This function adds a new vector data layer to the system. It can be used
    //! for adding a vector attribute, such as vortex, to the particles.
    //!
    //! \param[in] initialVal  Initial value of the new vector data.
    //!
    [[nodiscard]] size_t AddVectorData(
        const Vector<double, N>& initialVal = Vector<double, N>{});

    //! Returns the radius of the particles.
    [[nodiscard]] double Radius() const;

    //! Sets the radius of the particles.
    virtual void SetRadius(double newRadius);

    //! Returns the mass of the particles.
    [[nodiscard]] double Mass() const;

    //! Sets the mass of the particles.
    virtual void SetMass(double newMass);

    //! Returns the position array (immutable).
    [[nodiscard]] ConstArrayView1<Vector<double, N>> Positions() const;

    //! Returns the position array (mutable).
    [[nodiscard]] ArrayView1<Vector<double, N>> Positions();

    //! Returns the velocity array (immutable).
    [[nodiscard]] ConstArrayView1<Vector<double, N>> Velocities() const;

    //! Returns the velocity array (mutable).
    [[nodiscard]] ArrayView1<Vector<double, N>> Velocities();

    //! Returns the force array (immutable).
    [[nodiscard]] ConstArrayView1<Vector<double, N>> Forces() const;

    //! Returns the force array (mutable).
    [[nodiscard]] ArrayView1<Vector<double, N>> Forces();

    //! Returns custom scalar data layer at given index (immutable).
    [[nodiscard]] ConstArrayView1<double> ScalarDataAt(size_t idx) const;

    //! Returns custom scalar data layer at given index (mutable).
    [[nodiscard]] ArrayView1<double> ScalarDataAt(size_t idx);

    //! Returns custom vector data layer at given index (immutable).
    [[nodiscard]] ConstArrayView1<Vector<double, N>> VectorDataAt(
        size_t idx) const;

    //! Returns custom vector data layer at given index (mutable).
    [[nodiscard]] ArrayView1<Vector<double, N>> VectorDataAt(size_t idx);

    //!
    //! \brief      Adds a particle to the data structure.
    //!
    //! This function will add a single particle to the data structure. For
    //! custom data layers, zeros will be assigned for new particles.
    //! However, this will invalidate neighbor searcher and neighbor lists. It
    //! is users responsibility to call
    //! ParticleSystemData::BuildNeighborSearcher and
    //! ParticleSystemData::BuildNeighborLists to refresh those data.
    //!
    //! \param[in]  newPosition The new position.
    //! \param[in]  newVelocity The new velocity.
    //! \param[in]  newForce    The new force.
    //!
    void AddParticle(const Vector<double, N>& newPosition,
                     const Vector<double, N>& newVelocity = Vector<double, N>(),
                     const Vector<double, N>& newForce = Vector<double, N>());

    //!
    //! \brief      Adds particles to the data structure.
    //!
    //! This function will add particles to the data structure. For custom data
    //! layers, zeros will be assigned for new particles. However, this will
    //! invalidate neighbor searcher and neighbor lists. It is users
    //! responsibility to call ParticleSystemData::BuildNeighborSearcher and
    //! ParticleSystemData::BuildNeighborLists to refresh those data.
    //!
    //! \param[in]  newPositions  The new positions.
    //! \param[in]  newVelocities The new velocities.
    //! \param[in]  newForces     The new forces.
    //!
    void AddParticles(const ConstArrayView1<Vector<double, N>>& newPositions,
                      const ConstArrayView1<Vector<double, N>>& newVelocities =
                          ConstArrayView1<Vector<double, N>>(),
                      const ConstArrayView1<Vector<double, N>>& newForces =
                          ConstArrayView1<Vector<double, N>>());

    //!
    //! \brief      Returns neighbor searcher.
    //!
    //! This function returns currently set neighbor searcher object. By
    //! default, PointParallelHashGridSearcher2 is used.
    //!
    //! \return     Current neighbor searcher.
    //!
    [[nodiscard]] const std::shared_ptr<PointNeighborSearcher<N>>&
    NeighborSearcher() const;

    //! Sets neighbor searcher.
    void SetNeighborSearcher(
        const std::shared_ptr<PointNeighborSearcher<N>>& newNeighborSearcher);

    //!
    //! \brief      Returns neighbor lists.
    //!
    //! This function returns neighbor lists which is available after calling
    //! PointParallelHashGridSearcher2::BuildNeighborLists. Each list stores
    //! indices of the neighbors.
    //!
    //! \return     Neighbor lists.
    //!
    [[nodiscard]] const Array1<Array1<size_t>>& NeighborLists() const;

    //! Builds neighbor searcher with given search radius.
    void BuildNeighborSearcher(double maxSearchRadius);

    //! Builds neighbor lists with given search radius.
    void BuildNeighborLists(double maxSearchRadius);

    //! Serializes this particle system data to the buffer.
    void Serialize(std::vector<uint8_t>* buffer) const override;

    //! Deserializes this particle system data from the buffer.
    void Deserialize(const std::vector<uint8_t>& buffer) override;

    //! Copies from other particle system data.
    void Set(const ParticleSystemData& other);

 protected:
    template <size_t M = N>
    static std::enable_if_t<M == 2, void> Serialize(
        const ParticleSystemData<2>& particles,
        flatbuffers::FlatBufferBuilder* builder,
        flatbuffers::Offset<fbs::ParticleSystemData2>* fbsParticleSystemData);

    template <size_t M = N>
    static std::enable_if_t<M == 3, void> Serialize(
        const ParticleSystemData<3>& particles,
        flatbuffers::FlatBufferBuilder* builder,
        flatbuffers::Offset<fbs::ParticleSystemData3>* fbsParticleSystemData);

    template <size_t M = N>
    static std::enable_if_t<M == 2, void> Deserialize(
        const fbs::ParticleSystemData2* fbsParticleSystemData,
        ParticleSystemData<2>& particles);

    template <size_t M = N>
    static std::enable_if_t<M == 3, void> Deserialize(
        const fbs::ParticleSystemData3* fbsParticleSystemData,
        ParticleSystemData<3>& particles);

 private:
    double m_radius = 1e-3;
    double m_mass = 1e-3;
    size_t m_numberOfParticles = 0;
    size_t m_positionIdx = 0;
    size_t m_velocityIdx = 0;
    size_t m_forceIdx = 0;

    Array1<ScalarData> m_scalarDataList;
    Array1<VectorData> m_vectorDataList;

    std::shared_ptr<PointNeighborSearcher<N>> m_neighborSearcher;
    Array1<Array1<size_t>> m_neighborLists;
};

//! 2-D ParticleSystemData type.
using ParticleSystemData2 = ParticleSystemData<2>;

//! 3-D ParticleSystemData type.
using ParticleSystemData3 = ParticleSystemData<3>;

//! Shared pointer type of ParticleSystemData2.
using ParticleSystemData2Ptr = std::shared_ptr<ParticleSystemData2>;

//! Shared pointer type of ParticleSystemData3.
using ParticleSystemData3Ptr = std::shared_ptr<ParticleSystemData3>;
}  // namespace CubbyFlow

#endif
