// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_FDM_MGPCG_SOLVER3_HPP
#define CUBBYFLOW_FDM_MGPCG_SOLVER3_HPP

#include <Core/Solver/FDM/FDMMGSolver3.hpp>

namespace CubbyFlow
{
//!
//! \brief 3-D finite difference-type linear system solver using Multigrid
//!        Preconditioned conjugate gradient (MGPCG).
//!
//! \see McAdams, Aleka, Eftychios Sifakis, and Joseph Teran.
//!      "A parallel multigrid Poisson solver for fluids simulation on large
//!      grids." Proceedings of the 2010 ACM SIGGRAPH/Eurographics Symposium on
//!      Computer Animation. Eurographics Association, 2010.
//!
class FDMMGPCGSolver3 final : public FDMMGSolver3
{
 public:
    //!
    //! Constructs the solver with given parameters.
    //!
    //! \param numberOfCGIter - Number of CG iterations.
    //! \param maxNumberOfLevels - Number of maximum MG levels.
    //! \param numberOfRestrictionIter - Number of restriction iterations.
    //! \param numberOfCorrectionIter - Number of correction iterations.
    //! \param numberOfCoarsestIter - Number of iterations at the coarsest grid.
    //! \param numberOfFinalIter - Number of final iterations.
    //! \param maxTolerance - Number of max residual tolerance.
    //! \param sorFactor - Factor of successive over-relaxation (SOR).
    //! \param useRedBlackOrdering - Flag to use red-black ordering.
    FDMMGPCGSolver3(unsigned int numberOfCGIter, size_t maxNumberOfLevels,
                    unsigned int numberOfRestrictionIter = 5,
                    unsigned int numberOfCorrectionIter = 5,
                    unsigned int numberOfCoarsestIter = 20,
                    unsigned int numberOfFinalIter = 20,
                    double maxTolerance = 1e-9, double sorFactor = 1.5,
                    bool useRedBlackOrdering = false);

    using FDMMGSolver3::Solve;

    //! Solves the given linear system.
    bool Solve(FDMMGLinearSystem3* system) override;

    //! Returns the max number of Jacobi iterations.
    [[nodiscard]] unsigned int GetMaxNumberOfIterations() const;

    //! Returns the last number of Jacobi iterations the solver made.
    [[nodiscard]] unsigned int GetLastNumberOfIterations() const;

    //! Returns the max residual tolerance for the Jacobi method.
    [[nodiscard]] double GetTolerance() const;

    //! Returns the last residual after the Jacobi iterations.
    [[nodiscard]] double GetLastResidual() const;

 private:
    struct Preconditioner final
    {
        void Build(FDMMGLinearSystem3* system, MGParameters<FDMBLAS3> mgParams);

        void Solve(const FDMVector3& b, FDMVector3* x) const;

        FDMMGLinearSystem3* system = nullptr;
        MGParameters<FDMBLAS3> mgParams;
    };

    unsigned int m_maxNumberOfIterations;
    unsigned int m_lastNumberOfIterations;
    double m_tolerance;
    double m_lastResidualNorm;

    FDMVector3 m_r;
    FDMVector3 m_d;
    FDMVector3 m_q;
    FDMVector3 m_s;
    Preconditioner m_precond;
};

//! Shared pointer type for the FDMMGPCGSolver3.
using FDMMGPCGSolver3Ptr = std::shared_ptr<FDMMGPCGSolver3>;
}  // namespace CubbyFlow

#endif
