// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_SPH_SYSTEM_DATA_HPP
#define CUBBYFLOW_SPH_SYSTEM_DATA_HPP

#include <Core/Particle/ParticleSystemData.hpp>

namespace CubbyFlow
{
//!
//! \brief      N-D SPH particle system data.
//!
//! This class extends ParticleSystemData2 to specialize the data model for SPH.
//! It includes density and pressure array as a default particle attribute, and
//! it also contains SPH utilities such as interpolation operator.
//!
template <size_t N>
class SPHSystemData : public ParticleSystemData<N>
{
 public:
    using Base = ParticleSystemData<N>;
    using Base::AddScalarData;
    using Base::Mass;
    using Base::NeighborLists;
    using Base::NeighborSearcher;
    using Base::NumberOfParticles;
    using Base::Positions;
    using Base::ScalarDataAt;
    using Base::Serialize;
    using Base::Set;

    //! Constructs empty SPH system.
    SPHSystemData();

    //! Constructs SPH system data with given number of particles.
    explicit SPHSystemData(size_t numberOfParticles);

    //! Default virtual destructor.
    ~SPHSystemData() override = default;

    //! Copy constructor.
    SPHSystemData(const SPHSystemData& other);

    //! Move constructor.
    SPHSystemData(SPHSystemData&& other) noexcept;

    //! Copy assignment operator.
    SPHSystemData& operator=(const SPHSystemData& other);

    //! Move assignment operator.
    SPHSystemData& operator=(SPHSystemData&& other) noexcept;

    //!
    //! \brief      Sets the radius.
    //!
    //! Sets the radius of the particle system. The radius will be interpreted
    //! as target spacing.
    //!
    void SetRadius(double newRadius) override;

    //!
    //! \brief      Sets the mass of a particle.
    //!
    //! Setting the mass of a particle will change the target density.
    //!
    //! \param[in]  newMass The new mass.
    //!
    void SetMass(double newMass) override;

    //! Returns the density array view (immutable).
    [[nodiscard]] ConstArrayView1<double> Densities() const;

    //! Returns the density array view (mutable).
    [[nodiscard]] ArrayView1<double> Densities();

    //! Returns the pressure array view (immutable).
    [[nodiscard]] ConstArrayView1<double> Pressures() const;

    //! Returns the pressure array view (mutable).
    [[nodiscard]] ArrayView1<double> Pressures();

    //!
    //! \brief Updates the density array with the latest particle positions.
    //!
    //! This function updates the density array by recalculating each particle's
    //! latest nearby particles' position.
    //!
    //! \warning You must update the neighbor searcher
    //! (SPHSystemData::BuildNeighborSearcher) before calling this function.
    //!
    void UpdateDensities();

    //! Returns the target density of this particle system.
    [[nodiscard]] double TargetDensity() const;

    //! Sets the target density of this particle system.
    void SetTargetDensity(double targetDensity);

    //! Returns the target particle spacing in meters.
    [[nodiscard]] double TargetSpacing() const;

    //!
    //! \brief Sets the target particle spacing in meters.
    //!
    //! Once this function is called, hash grid and density should be
    //! updated using UpdateHashGrid() and UpdateDensities()).
    //!
    void SetTargetSpacing(double spacing);

    //!
    //! \brief Returns the relative kernel radius.
    //!
    //! Returns the relative kernel radius compared to the target particle
    //! spacing (i.e. kernel radius / target spacing).
    //!
    [[nodiscard]] double RelativeKernelRadius() const;

    //!
    //! \brief Sets the relative kernel radius.
    //!
    //! Sets the relative kernel radius compared to the target particle
    //! spacing (i.e. kernel radius / target spacing).
    //! Once this function is called, hash grid and density should
    //! be updated using UpdateHashGrid() and UpdateDensities()).
    //!
    void SetRelativeKernelRadius(double relativeRadius);

    //! Returns the kernel radius in meters unit.
    [[nodiscard]] double KernelRadius() const;

    //!
    //! \brief Sets the absolute kernel radius.
    //!
    //! Sets the absolute kernel radius compared to the target particle
    //! spacing (i.e. relative kernel radius * target spacing).
    //! Once this function is called, hash grid and density should
    //! be updated using UpdateHashGrid() and UpdateDensities()).
    //!
    void SetKernelRadius(double kernelRadius);

    //! Returns sum of kernel function evaluation for each nearby particle.
    [[nodiscard]] double SumOfKernelNearby(
        const Vector<double, N>& position) const;

    //!
    //! \brief Returns interpolated value at given origin point.
    //!
    //! Returns interpolated scalar data from the given position using
    //! standard SPH weighted average. The data array should match the
    //! particle layout. For example, density or pressure arrays can be
    //! used.
    //!
    //! \warning You must update the neighbor searcher
    //! (SPHSystemData::BuildNeighborSearcher) before calling this function.
    //!
    [[nodiscard]] double Interpolate(
        const Vector<double, N>& origin,
        const ConstArrayView1<double>& values) const;

    //!
    //! \brief Returns interpolated vector value at given origin point.
    //!
    //! Returns interpolated vector data from the given position using
    //! standard SPH weighted average. The data array should match the
    //! particle layout. For example, velocity or acceleration arrays can be
    //! used.
    //!
    //! \warning You must update the neighbor searcher
    //! (SPHSystemData::BuildNeighborSearcher) before calling this function.
    //!
    [[nodiscard]] Vector<double, N> Interpolate(
        const Vector<double, N>& origin,
        const ConstArrayView1<Vector<double, N>>& values) const;

    //!
    //! Returns the gradient of the given values at i-th particle.
    //!
    //! \warning You must update the neighbor lists
    //! (SPHSystemData::BuildNeighborLists) before calling this function.
    //!
    [[nodiscard]] Vector<double, N> GradientAt(
        size_t i, const ConstArrayView1<double>& values) const;

    //!
    //! Returns the laplacian of the given values at i-th particle.
    //!
    //! \warning You must update the neighbor lists
    //! (SPHSystemData::BuildNeighborLists) before calling this function.
    //!
    [[nodiscard]] double LaplacianAt(
        size_t i, const ConstArrayView1<double>& values) const;

    //!
    //! Returns the laplacian of the given values at i-th particle.
    //!
    //! \warning You must update the neighbor lists
    //! (SPHSystemData::BuildNeighborLists) before calling this function.
    //!
    [[nodiscard]] Vector<double, N> LaplacianAt(
        size_t i, const ConstArrayView1<Vector<double, N>>& values) const;

    //! Builds neighbor searcher with kernel radius.
    void BuildNeighborSearcher();

    //! Builds neighbor lists with kernel radius.
    void BuildNeighborLists();

    //! Serializes this SPH system data to the buffer.
    void Serialize(std::vector<uint8_t>* buffer) const override;

    //! Deserializes this SPH system data from the buffer.
    void Deserialize(const std::vector<uint8_t>& buffer) override;

    //! Copies from other SPH system data.
    void Set(const SPHSystemData& other);

 private:
    //! Target density of this particle system in kg/m^2.
    double m_targetDensity = WATER_DENSITY;

    //! Target spacing of this particle system in meters.
    double m_targetSpacing = 0.1;

    //! Relative radius of SPH kernel.
    //! SPH kernel radius divided by target spacing.
    double m_kernelRadiusOverTargetSpacing = 1.8;

    //! SPH kernel radius in meters.
    double m_kernelRadius = 0.1;

    size_t m_pressureIdx = 0;

    size_t m_densityIdx = 0;

    //! Computes the mass based on the target density and spacing.
    void ComputeMass();
};

//! 2-D SPHSystemData type.
using SPHSystemData2 = SPHSystemData<2>;

//! 3-D SPHSystemData type.
using SPHSystemData3 = SPHSystemData<3>;

//! Shared pointer for the SPHSystemData2 type.
using SPHSystemData2Ptr = std::shared_ptr<SPHSystemData2>;

//! Shared pointer for the SPHSystemData3 type.
using SPHSystemData3Ptr = std::shared_ptr<SPHSystemData3>;
}  // namespace CubbyFlow

#endif
