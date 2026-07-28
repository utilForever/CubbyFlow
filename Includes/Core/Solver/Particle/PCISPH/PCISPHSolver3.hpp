// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_PCISPH_SOLVER3_HPP
#define CUBBYFLOW_PCISPH_SOLVER3_HPP

#include <Core/Solver/Particle/SPH/SPHSolver3.hpp>

namespace CubbyFlow
{
//!
//! \brief 3-D PCISPH solver.
//!
//! This class implements 3-D predictive-corrective SPH solver. The main
//! pressure solver is based on Solenthaler and Pajarola's 2009 SIGGRAPH paper.
//!
//! \see Solenthaler and Pajarola, Predictive-corrective incompressible SPH,
//!      ACM transactions on graphics (TOG). Vol. 28. No. 3. ACM, 2009.
//!
class PCISPHSolver3 : public SPHSolver3
{
 public:
    class Builder;

    //! Constructs a solver with empty particle set.
    PCISPHSolver3();

    //! Constructs a solver with target density, spacing, and relative kernel
    //! radius.
    PCISPHSolver3(double targetDensity, double targetSpacing,
                  double relativeKernelRadius);

    //! Deleted copy constructor.
    PCISPHSolver3(const PCISPHSolver3&) = delete;

    //! Deleted move constructor.
    PCISPHSolver3(PCISPHSolver3&&) noexcept = delete;

    //! Default virtual destructor.
    ~PCISPHSolver3() override = default;

    //! Deleted copy assignment operator.
    PCISPHSolver3& operator=(const PCISPHSolver3&) = delete;

    //! Deleted move assignment operator.
    PCISPHSolver3& operator=(PCISPHSolver3&&) noexcept = delete;

    //! Returns max allowed density error ratio.
    [[nodiscard]] double GetMaxDensityErrorRatio() const;

    //!
    //! \brief Sets max allowed density error ratio.
    //!
    //! This function sets the max allowed density error ratio during the PCISPH
    //! iteration. Default is 0.01 (1%). The input value should be positive.
    //!
    void SetMaxDensityErrorRatio(double ratio);

    //! Returns max number of iterations.
    [[nodiscard]] unsigned int GetMaxNumberOfIterations() const;

    //!
    //! \brief Sets max number of PCISPH iterations.
    //!
    //! This function sets the max number of PCISPH iterations. Default is 5.
    //!
    void SetMaxNumberOfIterations(unsigned int n);

    //! Returns builder fox PCISPHSolver3.
    [[nodiscard]] static Builder GetBuilder();

 protected:
    using SPHSolver3::AccumulatePressureForce;

    //! Accumulates the pressure force to the forces array in the particle
    //! system.
    void AccumulatePressureForce(double timeIntervalInSeconds) override;

    //! Performs pre-processing step before the simulation.
    void OnBeginAdvanceTimeStep(double timeStepInSeconds) override;

 private:
    [[nodiscard]] double ComputeDelta(double timeStepInSeconds) const;
    [[nodiscard]] double ComputeBeta(double timeStepInSeconds) const;

    double m_maxDensityErrorRatio = 0.01;
    unsigned int m_maxNumberOfIterations = 5;

    ParticleSystemData3::VectorData m_tempPositions;
    ParticleSystemData3::VectorData m_tempVelocities;
    ParticleSystemData3::VectorData m_pressureForces;
    ParticleSystemData3::ScalarData m_densityErrors;
};

//! Shared pointer type for the PCISPHSolver3.
using PCISPHSolver3Ptr = std::shared_ptr<PCISPHSolver3>;

//!
//! \brief Front-end to create PCISPHSolver3 objects step by step.
//!
class PCISPHSolver3::Builder final : public SPHSolverBuilderBase3<Builder>
{
 public:
    //! Builds PCISPHSolver3.
    [[nodiscard]] PCISPHSolver3 Build() const;

    //! Builds shared pointer of PCISPHSolver3 instance.
    [[nodiscard]] PCISPHSolver3Ptr MakeShared() const;
};
}  // namespace CubbyFlow

#endif
