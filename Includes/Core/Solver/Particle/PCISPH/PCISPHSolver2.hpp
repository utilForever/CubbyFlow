// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_PCISPH_SOLVER2_HPP
#define CUBBYFLOW_PCISPH_SOLVER2_HPP

#include <Core/Solver/Particle/SPH/SPHSolver2.hpp>

namespace CubbyFlow
{
//!
//! \brief 2-D PCISPH solver.
//!
//! This class implements 2-D predictive-corrective SPH solver. The main
//! pressure solver is based on Solenthaler and Pajarola's 2009 SIGGRAPH paper.
//!
//! \see Solenthaler and Pajarola, Predictive-corrective incompressible SPH,
//!      ACM transactions on graphics (TOG). Vol. 28. No. 3. ACM, 2009.
//!
class PCISPHSolver2 : public SPHSolver2
{
 public:
    class Builder;

    //! Constructs a solver with empty particle set.
    PCISPHSolver2();

    //! Constructs a solver with target density, spacing, and relative kernel
    //! radius.
    PCISPHSolver2(double targetDensity, double targetSpacing,
                  double relativeKernelRadius);

    //! Deleted copy constructor.
    PCISPHSolver2(const PCISPHSolver2&) = delete;

    //! Deleted move constructor.
    PCISPHSolver2(PCISPHSolver2&&) noexcept = delete;

    //! Default virtual destructor.
    ~PCISPHSolver2() override = default;

    //! Deleted copy assignment operator.
    PCISPHSolver2& operator=(const PCISPHSolver2&) = delete;

    //! Deleted move assignment operator.
    PCISPHSolver2& operator=(PCISPHSolver2&&) noexcept = delete;

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

    //! Returns builder fox PCISPHSolver2.
    [[nodiscard]] static Builder GetBuilder();

 protected:
    using SPHSolver2::AccumulatePressureForce;

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

    ParticleSystemData2::VectorData m_tempPositions;
    ParticleSystemData2::VectorData m_tempVelocities;
    ParticleSystemData2::VectorData m_pressureForces;
    ParticleSystemData2::ScalarData m_densityErrors;
};

//! Shared pointer type for the PCISPHSolver2.
using PCISPHSolver2Ptr = std::shared_ptr<PCISPHSolver2>;

//!
//! \brief Front-end to create PCISPHSolver2 objects step by step.
//!
class PCISPHSolver2::Builder final : public SPHSolverBuilderBase2<Builder>
{
 public:
    //! Builds PCISPHSolver2.
    [[nodiscard]] PCISPHSolver2 Build() const;

    //! Builds shared pointer of PCISPHSolver2 instance.
    [[nodiscard]] PCISPHSolver2Ptr MakeShared() const;
};
}  // namespace CubbyFlow

#endif
