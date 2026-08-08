// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_SNOW_MPM_SOLVER_HPP
#define CUBBYFLOW_SNOW_MPM_SOLVER_HPP

#include <Core/Particle/MPM/MPMSystemData.hpp>
#include <Core/Particle/MPM/SnowConstitutiveModel.hpp>
#include <Core/Solver/Particle/ParticleSystemSolver2.hpp>
#include <Core/Solver/Particle/ParticleSystemSolver3.hpp>
#include <Core/Utils/Constants.hpp>

#include <memory>
#include <type_traits>

namespace CubbyFlow
{
//!
//! \brief N-D material point method solver for snow.
//!
//! Uses fixed-corotated elastoplastic snow response with the existing MPM
//! particle-grid transfers and particle-solver collision lifecycle.
//!
template <size_t N>
class SnowMPMSolver : public std::conditional_t<N == 2, ParticleSystemSolver2,
                                                ParticleSystemSolver3>
{
 public:
    static_assert(N == 2 || N == 3, "Snow MPM supports only 2-D and 3-D.");

    using Base = std::conditional_t<N == 2, ParticleSystemSolver2,
                                    ParticleSystemSolver3>;
    using MatrixType = Matrix<double, N, N>;
    using VectorType = Vector<double, N>;
    using SizeType = Vector<size_t, N>;

    class Builder;

    //! Constructs an empty snow MPM solver.
    explicit SnowMPMSolver(
        const SizeType& resolution = SizeType::MakeConstant(32),
        const VectorType& gridSpacing = VectorType::MakeConstant(1.0),
        const VectorType& gridOrigin = VectorType{}, double radius = 1e-3,
        double mass = 1e-3);

    //! Returns the owned MPM particle and grid state.
    [[nodiscard]] std::shared_ptr<MPMSystemData<N>> GetMPMSystemData() const;

    //! Returns the scale applied to adaptive time-step limits.
    [[nodiscard]] double GetTimeStepLimitScale() const;

    //! Sets the adaptive time-step scale in `(0, 1]`.
    void SetTimeStepLimitScale(double newScale);

    //! Returns the closed domain boundary flag.
    [[nodiscard]] int GetClosedDomainBoundaryFlag() const;

    //! Sets the closed domain boundary flag.
    void SetClosedDomainBoundaryFlag(int flag);

    //! Returns a builder for SnowMPMSolver.
    [[nodiscard]] static Builder GetBuilder();

 protected:
    //! Initializes particle-grid state before the first adaptive step query.
    void OnInitialize() override;

    //! Returns the number of adaptive sub-time-steps.
    [[nodiscard]] unsigned int GetNumberOfSubTimeSteps(
        double timeIntervalInSeconds) const override;

    //! Snow forces are integrated on the background grid.
    void AccumulateForces(double timeStepInSeconds) override;

    //! Advances particle-grid snow state before base particle integration.
    void OnBeginAdvanceTimeStep(double timeStepInSeconds) override;

    //! Projects particles back into selected closed domain boundaries.
    void OnEndAdvanceTimeStep(double timeStepInSeconds) override;

 private:
    [[nodiscard]] static SizeType ClampIndex(const Vector<ssize_t, N>& index,
                                             const SizeType& dataSize);

    void InitializeReferenceVolumes();

    void UpdateGridVelocities(double timeStepInSeconds);

    void ConstrainGridVelocities();

    [[nodiscard]] MatrixType ComputeVelocityGradient(
        size_t particleIndex) const;

    [[nodiscard]] double ComputeMaxVelocityGradient() const;

    void UpdateDeformation(double timeStepInSeconds);

    void ConstrainParticlesToDomain();

    std::shared_ptr<MPMSystemData<N>> m_mpmSystemData;
    SnowConstitutiveModel<N> m_constitutiveModel;
    double m_timeStepLimitScale = 0.9;
    double m_maxVelocityGradient = 0.0;
    int m_closedDomainBoundaryFlag = DIRECTION_ALL;
};

//! Front-end to create SnowMPMSolver objects step by step.
template <size_t N>
class SnowMPMSolver<N>::Builder final
{
 public:
    [[nodiscard]] Builder& WithResolution(const SizeType& resolution);

    [[nodiscard]] Builder& WithGridSpacing(const VectorType& gridSpacing);

    [[nodiscard]] Builder& WithOrigin(const VectorType& gridOrigin);

    [[nodiscard]] Builder& WithRadius(double radius);

    [[nodiscard]] Builder& WithMass(double mass);

    [[nodiscard]] SnowMPMSolver Build() const;

    [[nodiscard]] std::shared_ptr<SnowMPMSolver> MakeShared() const;

 private:
    SizeType m_resolution = SizeType::MakeConstant(32);
    VectorType m_gridSpacing = VectorType::MakeConstant(1.0);
    VectorType m_gridOrigin;
    double m_radius = 1e-3;
    double m_mass = 1e-3;
};

using SnowMPMSolver2 = SnowMPMSolver<2>;
using SnowMPMSolver3 = SnowMPMSolver<3>;
using SnowMPMSolver2Ptr = std::shared_ptr<SnowMPMSolver2>;
using SnowMPMSolver3Ptr = std::shared_ptr<SnowMPMSolver3>;
}  // namespace CubbyFlow

#include <Core/Solver/Particle/MPM/SnowMPMSolver-Impl.hpp>

#endif
