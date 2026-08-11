// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_CG_IMPL_HPP
#define CUBBYFLOW_CG_IMPL_HPP

#include <Core/Math/MathUtils.hpp>

#include <limits>

namespace CubbyFlow
{
template <typename BLASType>
void CG(const typename BLASType::MatrixType& A,
        const typename BLASType::VectorType& b,
        unsigned int maxNumberOfIterations, double tolerance,
        typename BLASType::VectorType* x, typename BLASType::VectorType* r,
        typename BLASType::VectorType* d, typename BLASType::VectorType* q,
        typename BLASType::VectorType* s, unsigned int* lastNumberOfIterations,
        double* lastResidualNorm)
{
    using PrecondType = NullCGPreconditioner<BLASType>;
    PrecondType precond;

    PCG<BLASType, PrecondType>(A, b, maxNumberOfIterations, tolerance, &precond,
                               x, r, d, q, s, lastNumberOfIterations,
                               lastResidualNorm);
}

template <typename BLASType>
void CR(const typename BLASType::MatrixType& A,
        const typename BLASType::VectorType& b,
        unsigned int maxNumberOfIterations, double tolerance,
        typename BLASType::VectorType* x, typename BLASType::VectorType* r,
        typename BLASType::VectorType* d, typename BLASType::VectorType* q,
        typename BLASType::VectorType* s, unsigned int* lastNumberOfIterations,
        double* lastResidualNorm)
{
    BLASType::Residual(A, *x, b, r);
    BLASType::Set(*r, d);
    BLASType::MVM(A, *r, s);
    BLASType::Set(*s, q);

    double rho = BLASType::Dot(*r, *s);
    double residualNorm = BLASType::L2Norm(*r);
    unsigned int iter = 0;

    while (residualNorm > tolerance && iter < maxNumberOfIterations)
    {
        const double denominator = BLASType::Dot(*q, *q);

        if (!std::isfinite(rho) || !std::isfinite(denominator) ||
            denominator <= 0.0 || rho == 0.0)
        {
            residualNorm = std::numeric_limits<double>::infinity();
            break;
        }

        const double alpha = rho / denominator;

        BLASType::AXPlusY(alpha, *d, *x, x);
        BLASType::AXPlusY(-alpha, *q, *r, r);

        residualNorm = BLASType::L2Norm(*r);
        ++iter;

        if (!std::isfinite(residualNorm))
        {
            residualNorm = std::numeric_limits<double>::infinity();
            break;
        }

        if (residualNorm <= tolerance)
        {
            break;
        }

        BLASType::MVM(A, *r, s);

        const double rhoNew = BLASType::Dot(*r, *s);

        if (!std::isfinite(rhoNew))
        {
            residualNorm = std::numeric_limits<double>::infinity();
            break;
        }

        const double beta = rhoNew / rho;

        BLASType::AXPlusY(beta, *d, *r, d);
        BLASType::AXPlusY(beta, *q, *s, q);
        rho = rhoNew;
    }

    *lastNumberOfIterations = iter;
    *lastResidualNorm = residualNorm;
}

template <typename BLASType, typename PrecondType>
void PCG(const typename BLASType::MatrixType& A,
         const typename BLASType::VectorType& b,
         unsigned int maxNumberOfIterations, double tolerance, PrecondType* M,
         typename BLASType::VectorType* x, typename BLASType::VectorType* r,
         typename BLASType::VectorType* d, typename BLASType::VectorType* q,
         typename BLASType::VectorType* s, unsigned int* lastNumberOfIterations,
         double* lastResidualNorm)
{
    // Clear
    BLASType::Set(0, r);
    BLASType::Set(0, d);
    BLASType::Set(0, q);
    BLASType::Set(0, s);

    // r = b - Ax
    BLASType::Residual(A, *x, b, r);

    // d = M^-1r
    M->Solve(*r, d);

    // sigmaNew = r.d
    double sigmaNew = BLASType::Dot(*r, *d);

    unsigned int iter = 0;
    bool trigger = false;

    while (sigmaNew > Square(tolerance) && iter < maxNumberOfIterations)
    {
        // q = Ad
        BLASType::MVM(A, *d, q);

        // alpha = sigmaNew / d.q
        double alpha = sigmaNew / BLASType::Dot(*d, *q);

        // x = x + alpha * d
        BLASType::AXPlusY(alpha, *d, *x, x);

        // if i is divisible by 50...
        if (trigger || (iter % 50 == 0 && iter > 0))
        {
            // r = b - Ax
            BLASType::Residual(A, *x, b, r);
            trigger = false;
        }
        else
        {
            // r = r - alpha * q
            BLASType::AXPlusY(-alpha, *q, *r, r);
        }

        // s = M^-1r
        M->Solve(*r, s);

        // sigmaOld = sigmaNew
        const double sigmaOld = sigmaNew;

        // sigmaNew = r.s
        sigmaNew = BLASType::Dot(*r, *s);

        if (sigmaNew > sigmaOld)
        {
            trigger = true;
        }

        // beta = sigmaNew / sigmaOld
        double beta = sigmaNew / sigmaOld;

        // d = s + beta*d
        BLASType::AXPlusY(beta, *d, *s, d);

        ++iter;
    }

    *lastNumberOfIterations = iter;

    // std::fabs(sigmaNew) - Workaround for negative zero
    *lastResidualNorm = std::sqrt(std::fabs(sigmaNew));
}
}  // namespace CubbyFlow

#endif
