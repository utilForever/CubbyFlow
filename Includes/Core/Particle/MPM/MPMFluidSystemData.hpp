// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_MPM_FLUID_SYSTEM_DATA_HPP
#define CUBBYFLOW_MPM_FLUID_SYSTEM_DATA_HPP

#include <Core/Particle/MPM/MPMSystemData.hpp>

namespace CubbyFlow
{
//!
//! \brief N-D weakly compressible MPM fluid transfer state.
//!
//! Stores the particle volume ratio and velocity gradient required by a fluid
//! constitutive model without carrying snow deformation or plasticity state.
//!
template <size_t N>
class MPMFluidSystemData final : public MPMTransferSystemData<N>
{
 public:
    static_assert(N == 2 || N == 3, "MPM supports only 2-D and 3-D.");

    using Base = MPMTransferSystemData<N>;
    using MatrixType = Matrix<double, N, N>;

    //! Constructs fluid MPM state with a vertex-centered background grid.
    explicit MPMFluidSystemData(
        const Vector<size_t, N>& resolution =
            Vector<size_t, N>::MakeConstant(1),
        const Vector<double, N>& gridSpacing =
            Vector<double, N>::MakeConstant(1.0),
        const Vector<double, N>& gridOrigin = Vector<double, N>{},
        size_t numberOfParticles = 0);

    //! Resizes particle state, initializing new fluid attributes.
    void Resize(size_t newNumberOfParticles) override;

    //! Deserializes inherited particle state and resets fluid state.
    void Deserialize(const std::vector<uint8_t>& buffer) override;

    //! Copies inherited particle state and resets fluid state.
    void Set(const ParticleSystemData<N>& other) override;

    //! Returns current-to-reference particle volume ratios.
    [[nodiscard]] ConstArrayView1<double> VolumeRatios() const;

    //! Returns current-to-reference particle volume ratios.
    [[nodiscard]] ArrayView1<double> VolumeRatios();

    //! Returns particle velocity gradients interpolated from the grid.
    [[nodiscard]] ConstArrayView1<MatrixType> VelocityGradients() const;

    //! Returns particle velocity gradients interpolated from the grid.
    [[nodiscard]] ArrayView1<MatrixType> VelocityGradients();

    //!
    //! Transfers updated grid velocities and advances fluid kinematic state.
    //!
    //! The volume ratio is updated by the determinant of the incremental
    //! deformation gradient, I + dt * velocityGradient.
    //!
    void TransferFromGridToParticles(double timeStepInSeconds);

 private:
    void ResetFluidState();

    Array1<double> m_volumeRatios;
    Array1<MatrixType> m_velocityGradients;
};

//! 2-D weakly compressible MPM fluid system data.
using MPMFluidSystemData2 = MPMFluidSystemData<2>;

//! 3-D weakly compressible MPM fluid system data.
using MPMFluidSystemData3 = MPMFluidSystemData<3>;

//! Shared pointer type of MPMFluidSystemData2.
using MPMFluidSystemData2Ptr = std::shared_ptr<MPMFluidSystemData2>;

//! Shared pointer type of MPMFluidSystemData3.
using MPMFluidSystemData3Ptr = std::shared_ptr<MPMFluidSystemData3>;
}  // namespace CubbyFlow

#include <Core/Particle/MPM/MPMFluidSystemData-Impl.hpp>

#endif
