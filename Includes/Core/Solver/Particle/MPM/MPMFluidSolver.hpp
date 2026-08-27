// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_MPM_FLUID_SOLVER_HPP
#define CUBBYFLOW_MPM_FLUID_SOLVER_HPP

#include <Core/Particle/MPM/MPMFluidConstitutiveModel.hpp>
#include <Core/Particle/MPM/MPMFluidSystemData.hpp>
#include <Core/Solver/Particle/ParticleSystemSolver2.hpp>
#include <Core/Solver/Particle/ParticleSystemSolver3.hpp>
#include <Core/Utils/Constants.hpp>

#include <memory>
#include <type_traits>

namespace CubbyFlow
{
//!
//! \brief N-D explicit weakly compressible material point method solver.
//!
//! Uses the Tait equation of state with existing MPM particle-grid transfers
//! and particle-solver collision lifecycle.
//!
template <size_t N>
class MPMFluidSolver : public std::conditional_t<N == 2, ParticleSystemSolver2,
                                                 ParticleSystemSolver3>
{
 public:
    static_assert(N == 2 || N == 3,
                  "MPM fluid solver supports only 2-D and 3-D.");

    using Base = std::conditional_t<N == 2, ParticleSystemSolver2,
                                    ParticleSystemSolver3>;
    using MatrixType = Matrix<double, N, N>;
    using VectorType = Vector<double, N>;
    using SizeType = Vector<size_t, N>;

    class Builder;

    //! Constructs an empty weakly compressible MPM fluid solver.
    explicit MPMFluidSolver(
        const SizeType& resolution = SizeType::MakeConstant(32),
        const VectorType& gridSpacing = VectorType::MakeConstant(1.0),
        const VectorType& gridOrigin = VectorType{}, double radius = 1e-3,
        double mass = 1e-3, double targetDensity = WATER_DENSITY,
        double speedOfSound = 100.0, double eosExponent = 7.0,
        double negativePressureScale = 0.0);

    //! Returns the owned MPM fluid particle and grid state.
    [[nodiscard]] std::shared_ptr<MPMFluidSystemData<N>> GetMPMSystemData()
        const;

    //! Returns the immutable weakly compressible constitutive model.
    [[nodiscard]] const MPMFluidConstitutiveModel<N>& GetConstitutiveModel()
        const;

    //! Returns the scale applied to adaptive time-step limits.
    [[nodiscard]] double GetTimeStepLimitScale() const;

    //! Sets the adaptive time-step scale in `(0, 1]`.
    void SetTimeStepLimitScale(double newScale);

    //! Returns a builder for MPMFluidSolver.
    [[nodiscard]] static Builder GetBuilder();

 protected:
    //! Initializes particle-grid state before the first adaptive step query.
    void OnInitialize() override;

    //! Returns the number of adaptive sub-time-steps.
    [[nodiscard]] unsigned int GetNumberOfSubTimeSteps(
        double timeIntervalInSeconds) const override;

    //! Fluid forces are integrated on the background grid.
    void AccumulateForces(double timeStepInSeconds) override;

    //! Advances particle-grid fluid state before base particle integration.
    void OnBeginAdvanceTimeStep(double timeStepInSeconds) override;

 private:
    [[nodiscard]] static SizeType ClampIndex(const Vector<ssize_t, N>& index,
                                             const SizeType& dataSize);

    void InitializeReferenceVolumes();

    void UpdateGridVelocities(double timeStepInSeconds);

    std::shared_ptr<MPMFluidSystemData<N>> m_mpmSystemData;
    MPMFluidConstitutiveModel<N> m_constitutiveModel;
    double m_timeStepLimitScale = 0.9;
};

//! Front-end to create MPMFluidSolver objects step by step.
template <size_t N>
class MPMFluidSolver<N>::Builder final
{
 public:
    [[nodiscard]] Builder& WithResolution(const SizeType& resolution);

    [[nodiscard]] Builder& WithGridSpacing(const VectorType& gridSpacing);

    [[nodiscard]] Builder& WithOrigin(const VectorType& gridOrigin);

    [[nodiscard]] Builder& WithRadius(double radius);

    [[nodiscard]] Builder& WithMass(double mass);

    [[nodiscard]] Builder& WithTargetDensity(double targetDensity);

    [[nodiscard]] Builder& WithSpeedOfSound(double speedOfSound);

    [[nodiscard]] Builder& WithEosExponent(double eosExponent);

    [[nodiscard]] Builder& WithNegativePressureScale(
        double negativePressureScale);

    [[nodiscard]] MPMFluidSolver Build() const;

    [[nodiscard]] std::shared_ptr<MPMFluidSolver> MakeShared() const;

 private:
    SizeType m_resolution = SizeType::MakeConstant(32);
    VectorType m_gridSpacing = VectorType::MakeConstant(1.0);
    VectorType m_gridOrigin;
    double m_radius = 1e-3;
    double m_mass = 1e-3;
    double m_targetDensity = WATER_DENSITY;
    double m_speedOfSound = 100.0;
    double m_eosExponent = 7.0;
    double m_negativePressureScale = 0.0;
};

using MPMFluidSolver2 = MPMFluidSolver<2>;
using MPMFluidSolver3 = MPMFluidSolver<3>;
using MPMFluidSolver2Ptr = std::shared_ptr<MPMFluidSolver2>;
using MPMFluidSolver3Ptr = std::shared_ptr<MPMFluidSolver3>;
}  // namespace CubbyFlow

#include <Core/Solver/Particle/MPM/MPMFluidSolver-Impl.hpp>

#endif
