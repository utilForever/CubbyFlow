// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_CG_HPP
#define CUBBYFLOW_CG_HPP

#include <Core/Math/BLAS.hpp>

namespace CubbyFlow
{
//!
//! \brief No-op pre-conditioner for conjugate gradient.
//!
//! This pre-conditioner does nothing but simply copies the input vector to the
//! output vector. Thus, it can be considered as an identity matrix.
//!
template <typename BLASType>
struct NullCGPreconditioner final
{
    static void Build(const typename BLASType::MatrixType&)
    {
        // Do nothing
    }

    void Solve(const typename BLASType::VectorType& b,
               typename BLASType::VectorType* x)
    {
        BLASType::Set(b, x);
    }
};

//!
//! \brief Solves conjugate gradient.
//!
template <typename BLASType>
void CG(const typename BLASType::MatrixType& A,
        const typename BLASType::VectorType& b,
        unsigned int maxNumberOfIterations, double tolerance,
        typename BLASType::VectorType* x, typename BLASType::VectorType* r,
        typename BLASType::VectorType* d, typename BLASType::VectorType* q,
        typename BLASType::VectorType* s, unsigned int* lastNumberOfIterations,
        double* lastResidualNorm);

//!
//! \brief Solves a symmetric linear system with conjugate residual.
//!
template <typename BLASType>
void CR(const typename BLASType::MatrixType& A,
        const typename BLASType::VectorType& b,
        unsigned int maxNumberOfIterations, double tolerance,
        typename BLASType::VectorType* x, typename BLASType::VectorType* r,
        typename BLASType::VectorType* d, typename BLASType::VectorType* q,
        typename BLASType::VectorType* s, unsigned int* lastNumberOfIterations,
        double* lastResidualNorm);

//!
//! \brief Solves pre-conditioned conjugate gradient.
//!
template <typename BLASType, typename PrecondType>
void PCG(const typename BLASType::MatrixType& A,
         const typename BLASType::VectorType& b,
         unsigned int maxNumberOfIterations, double tolerance, PrecondType* M,
         typename BLASType::VectorType* x, typename BLASType::VectorType* r,
         typename BLASType::VectorType* d, typename BLASType::VectorType* q,
         typename BLASType::VectorType* s, unsigned int* lastNumberOfIterations,
         double* lastResidualNorm);
}  // namespace CubbyFlow

#include <Core/Math/CG-Impl.hpp>

#endif
