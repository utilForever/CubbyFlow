// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_MPM_SYSTEM_DATA_HPP
#define CUBBYFLOW_MPM_SYSTEM_DATA_HPP

#include <Core/Grid/VertexCenteredScalarGrid.hpp>
#include <Core/Grid/VertexCenteredVectorGrid.hpp>
#include <Core/Particle/MPM/SnowConstitutiveModel.hpp>
#include <Core/Particle/ParticleSystemData.hpp>

#include <array>

namespace CubbyFlow
{
//!
//! \brief Tensor-product cubic B-spline interpolation kernel.
//!
template <size_t N>
class CubicBSplineKernel final
{
 public:
    static_assert(N == 2 || N == 3, "MPM supports only 2-D and 3-D.");

    static constexpr size_t STENCIL_SIZE = N == 2 ? 16 : 64;

    struct Entry
    {
        Vector<ssize_t, N> index;
        double weight = 0.0;
        Vector<double, N> gradient;
    };

    using Stencil = std::array<Entry, STENCIL_SIZE>;

    [[nodiscard]] static double Weight(double x);

    [[nodiscard]] static double Gradient(double x);

    [[nodiscard]] static Stencil GetStencil(
        const Vector<double, N>& position, const Vector<double, N>& gridSpacing,
        const Vector<double, N>& dataOrigin);

 private:
    static void GetStencilCoordinates(const Vector<double, N>& position,
                                      const Vector<double, N>& gridSpacing,
                                      const Vector<double, N>& dataOrigin,
                                      Vector<double, N>* normalized,
                                      Vector<ssize_t, N>* firstIndex);

    [[nodiscard]] static Entry GetStencilEntry(
        const Vector<size_t, N>& offset, const Vector<double, N>& normalized,
        const Vector<ssize_t, N>& firstIndex,
        const Vector<double, N>& gridSpacing);
};

//!
//! \brief N-D material point method particle and grid state.
//!
//! \note Serialization preserves only the inherited particle-system state;
//! MPM-specific particle and grid state is reset after deserialization.
//!
template <size_t N>
class MPMSystemData final : public ParticleSystemData<N>
{
 public:
    static_assert(N == 2 || N == 3, "MPM supports only 2-D and 3-D.");

    using Base = ParticleSystemData<N>;
    using DeformationState = SnowDeformationState<N>;

    //! Constructs MPM state with a vertex-centered background grid.
    explicit MPMSystemData(
        const Vector<size_t, N>& resolution =
            Vector<size_t, N>::MakeConstant(1),
        const Vector<double, N>& gridSpacing =
            Vector<double, N>::MakeConstant(1.0),
        const Vector<double, N>& gridOrigin = Vector<double, N>{},
        size_t numberOfParticles = 0);

    //! Resizes particle state, initializing new MPM attributes.
    void Resize(size_t newNumberOfParticles) override;

    //! Deserializes inherited particle state and resets MPM-specific state.
    void Deserialize(const std::vector<uint8_t>& buffer) override;

    //! Copies inherited particle state and resets MPM-specific state.
    void Set(const ParticleSystemData<N>& other) override;

    //! Resizes the background grid without changing particle state.
    void ResizeGrid(const Vector<size_t, N>& resolution,
                    const Vector<double, N>& gridSpacing,
                    const Vector<double, N>& gridOrigin);

    //! Returns per-particle masses.
    [[nodiscard]] ConstArrayView1<double> ParticleMasses() const;

    //! Returns per-particle masses.
    [[nodiscard]] ArrayView1<double> ParticleMasses();

    //! Returns per-particle initial volumes.
    [[nodiscard]] ConstArrayView1<double> InitialVolumes() const;

    //! Returns per-particle initial volumes.
    [[nodiscard]] ArrayView1<double> InitialVolumes();

    //! Returns per-particle deformation states.
    [[nodiscard]] ConstArrayView1<DeformationState> DeformationStates() const;

    //! Returns per-particle deformation states.
    [[nodiscard]] ArrayView1<DeformationState> DeformationStates();

    //! Returns grid mass.
    [[nodiscard]] const VertexCenteredScalarGrid<N>& GridMass() const;

    //! Returns grid mass.
    [[nodiscard]] VertexCenteredScalarGrid<N>& GridMass();

    //! Returns current grid velocities.
    [[nodiscard]] const VertexCenteredVectorGrid<N>& GridVelocities() const;

    //! Returns current grid velocities.
    [[nodiscard]] VertexCenteredVectorGrid<N>& GridVelocities();

    //! Returns grid velocities before the grid update.
    [[nodiscard]] const VertexCenteredVectorGrid<N>&
    GridVelocitiesBeforeUpdate() const;

    //! Returns grid velocities before the grid update.
    [[nodiscard]] VertexCenteredVectorGrid<N>& GridVelocitiesBeforeUpdate();

    //! Returns the FLIP fraction used for grid-to-particle transfer.
    [[nodiscard]] double FLIPBlendingFactor() const;

    //! Sets the FLIP fraction used for grid-to-particle transfer.
    void SetFLIPBlendingFactor(double factor);

    //! Transfers particle mass and momentum to the background grid.
    //! Stencil nodes outside the finite grid are clamped to its boundary.
    void TransferFromParticlesToGrid();

    //! Transfers updated background-grid velocities to particles.
    //! Stencil nodes outside the finite grid are clamped to its boundary.
    void TransferFromGridToParticles();

 private:
    [[nodiscard]] static Vector<size_t, N> ClampIndex(
        const Vector<ssize_t, N>& index, const Vector<size_t, N>& dataSize);

    [[nodiscard]] static bool IsFinite(const Vector<double, N>& value);

    static void ValidateGridParameters(const Vector<size_t, N>& resolution,
                                       const Vector<double, N>& gridSpacing,
                                       const Vector<double, N>& gridOrigin);

    void ValidateGridState() const;

    void ResetMPMState();

    Array1<double> m_particleMasses;
    Array1<double> m_initialVolumes;
    Array1<DeformationState> m_deformationStates;
    VertexCenteredScalarGrid<N> m_gridMass;
    VertexCenteredVectorGrid<N> m_gridVelocities;
    VertexCenteredVectorGrid<N> m_gridVelocitiesBeforeUpdate;
    double m_flipBlendingFactor = 0.95;
};

//! 2-D material point method system data.
using MPMSystemData2 = MPMSystemData<2>;

//! 3-D material point method system data.
using MPMSystemData3 = MPMSystemData<3>;

//! Shared pointer type of MPMSystemData2.
using MPMSystemData2Ptr = std::shared_ptr<MPMSystemData2>;

//! Shared pointer type of MPMSystemData3.
using MPMSystemData3Ptr = std::shared_ptr<MPMSystemData3>;
}  // namespace CubbyFlow

#include <Core/Particle/MPM/MPMSystemData-Impl.hpp>

#endif
