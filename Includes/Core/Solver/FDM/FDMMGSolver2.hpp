// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_FDM_MG_SOLVER2_HPP
#define CUBBYFLOW_FDM_MG_SOLVER2_HPP

#include <Core/FDM/FDMMGLinearSystem2.hpp>
#include <Core/Solver/FDM/FDMLinearSystemSolver2.hpp>
#include <Core/Utils/MG.hpp>

namespace CubbyFlow
{
//! \brief 2-D finite difference-type linear system solver using Multigrid.
class FDMMGSolver2 : public FDMLinearSystemSolver2
{
 public:
    //! Constructs the solver with given parameters.
    explicit FDMMGSolver2(size_t maxNumberOfLevels,
                          unsigned int numberOfRestrictionIter = 5,
                          unsigned int numberOfCorrectionIter = 5,
                          unsigned int numberOfCoarsestIter = 20,
                          unsigned int numberOfFinalIter = 20,
                          double maxTolerance = 1e-9, double sorFactor = 1.5,
                          bool useRedBlackOrdering = false);

    //! Returns the Multigrid parameters.
    [[nodiscard]] const MGParameters<FDMBLAS2>& GetParams() const;

    //! Returns the SOR (Successive Over Relaxation) factor.
    [[nodiscard]] double GetSORFactor() const;

    //! Returns true if red-black ordering is enabled.
    [[nodiscard]] bool GetUseRedBlackOrdering() const;

    //! No-op. Multigrid-type solvers do not solve FDMLinearSystem2.
    bool Solve(FDMLinearSystem2* system) final;

    //! Solves Multigrid linear system.
    virtual bool Solve(FDMMGLinearSystem2* system);

 private:
    MGParameters<FDMBLAS2> m_mgParams;
    double m_sorFactor;
    bool m_useRedBlackOrdering;
};

//! Shared pointer type for the FDMMGSolver2.
using FDMMGSolver2Ptr = std::shared_ptr<FDMMGSolver2>;
}  // namespace CubbyFlow

#endif