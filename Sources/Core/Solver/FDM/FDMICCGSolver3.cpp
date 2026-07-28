// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Math/CG.hpp>
#include <Core/Solver/FDM/FDMICCGSolver3.hpp>
#include <Core/Utils/Logging.hpp>

namespace CubbyFlow
{
void FDMICCGSolver3::Preconditioner::Build(const FDMMatrix3& matrix)
{
    const Vector3UZ size = matrix.Size();
    A = matrix.View();

    d.Resize(size, 0.0);
    y.Resize(size, 0.0);

    ForEachIndex(size, [&matrix, this](size_t i, size_t j, size_t k) {
        const double denom =
            matrix(i, j, k).center -
            ((i > 0) ? Square(matrix(i - 1, j, k).right) * d(i - 1, j, k)
                     : 0.0) -
            ((j > 0) ? Square(matrix(i, j - 1, k).up) * d(i, j - 1, k) : 0.0) -
            ((k > 0) ? Square(matrix(i, j, k - 1).front) * d(i, j, k - 1)
                     : 0.0);

        if (std::fabs(denom) > 0.0)
        {
            d(i, j, k) = 1.0 / denom;
        }
        else
        {
            d(i, j, k) = 0.0;
        }
    });
}

void FDMICCGSolver3::Preconditioner::Solve(const FDMVector3& b, FDMVector3* x)
{
    const Vector3UZ size = b.Size();
    const auto sx = static_cast<ssize_t>(size.x);
    const auto sy = static_cast<ssize_t>(size.y);
    const auto sz = static_cast<ssize_t>(size.z);

    ForEachIndex(size, [this, &b](size_t i, size_t j, size_t k) {
        y(i, j, k) = (b(i, j, k) -
                      ((i > 0) ? A(i - 1, j, k).right * y(i - 1, j, k) : 0.0) -
                      ((j > 0) ? A(i, j - 1, k).up * y(i, j - 1, k) : 0.0) -
                      ((k > 0) ? A(i, j, k - 1).front * y(i, j, k - 1) : 0.0)) *
                     d(i, j, k);
    });

    for (ssize_t k = sz - 1; k >= 0; --k)
    {
        for (ssize_t j = sy - 1; j >= 0; --j)
        {
            for (ssize_t i = sx - 1; i >= 0; --i)
            {
                (*x)(i, j, k) =
                    (y(i, j, k) -
                     ((i + 1 < sx) ? A(i, j, k).right * (*x)(i + 1, j, k)
                                   : 0.0) -
                     ((j + 1 < sy) ? A(i, j, k).up * (*x)(i, j + 1, k) : 0.0) -
                     ((k + 1 < sz) ? A(i, j, k).front * (*x)(i, j, k + 1)
                                   : 0.0)) *
                    d(i, j, k);
            }
        }
    }
}

void FDMICCGSolver3::PreconditionerCompressed::Build(const MatrixCSRD& matrix)
{
    const size_t size = matrix.GetCols();
    A = &matrix;

    d.Resize(size, 0.0);
    y.Resize(size, 0.0);

    const auto rp = A->RowPointersBegin();
    const auto ci = A->ColumnIndicesBegin();
    const auto nnz = A->NonZeroBegin();

    ForEachIndex(size, [&rp, &ci, &nnz, this](size_t i) {
        const size_t rowBegin = rp[i];
        const size_t rowEnd = rp[i + 1];

        double denom = 0.0;
        for (size_t jj = rowBegin; jj < rowEnd; ++jj)
        {
            const size_t j = ci[jj];

            if (j == i)
            {
                denom += nnz[jj];
            }
            else if (j < i)
            {
                denom -= Square(nnz[jj]) * d[j];
            }
        }

        if (std::fabs(denom) > 0.0)
        {
            d[i] = 1.0 / denom;
        }
        else
        {
            d[i] = 0.0;
        }
    });
}

void FDMICCGSolver3::PreconditionerCompressed::Solve(const VectorND& b,
                                                     VectorND* x)
{
    const auto size = static_cast<ssize_t>(b.GetRows());

    const auto rp = A->RowPointersBegin();
    const auto ci = A->ColumnIndicesBegin();
    const auto nnz = A->NonZeroBegin();

    ForEachIndex(b.GetRows(), [&rp, &b, &ci, &nnz, this](size_t i) {
        const size_t rowBegin = rp[i];
        const size_t rowEnd = rp[i + 1];

        double sum = b[i];
        for (size_t jj = rowBegin; jj < rowEnd; ++jj)
        {
            const size_t j = ci[jj];

            if (j < i)
            {
                sum -= nnz[jj] * y[j];
            }
        }

        y[i] = sum * d[i];
    });

    for (ssize_t i = size - 1; i >= 0; --i)
    {
        const size_t rowBegin = rp[i];
        const size_t rowEnd = rp[i + 1];

        double sum = y[i];
        for (size_t jj = rowBegin; jj < rowEnd; ++jj)
        {
            const auto j = static_cast<ssize_t>(ci[jj]);

            if (j > i)
            {
                sum -= nnz[jj] * (*x)[j];
            }
        }

        (*x)[i] = sum * d[i];
    }
}

FDMICCGSolver3::FDMICCGSolver3(unsigned int maxNumberOfIterations,
                               double tolerance)
    : m_maxNumberOfIterations{ maxNumberOfIterations },
      m_lastNumberOfIterations{ 0 },
      m_tolerance{ tolerance },
      m_lastResidualNorm{ std::numeric_limits<double>::max() }
{
    // Do nothing
}

bool FDMICCGSolver3::Solve(FDMLinearSystem3* system)
{
    FDMMatrix3& matrix = system->A;
    FDMVector3& solution = system->x;
    FDMVector3& rhs = system->b;

    ClearCompressedVectors();

    assert(matrix.Size() == rhs.Size());
    assert(matrix.Size() == solution.Size());

    const Vector3UZ size = matrix.Size();
    m_r.Resize(size);
    m_d.Resize(size);
    m_q.Resize(size);
    m_s.Resize(size);

    system->x.Fill(0.0);
    m_r.Fill(0.0);
    m_d.Fill(0.0);
    m_q.Fill(0.0);
    m_s.Fill(0.0);

    m_precond.Build(matrix);

    PCG<FDMBLAS3, Preconditioner>(matrix, rhs, m_maxNumberOfIterations,
                                  m_tolerance, &m_precond, &solution, &m_r,
                                  &m_d, &m_q, &m_s, &m_lastNumberOfIterations,
                                  &m_lastResidualNorm);

    CUBBYFLOW_INFO << "Residual norm after solving ICCG: " << m_lastResidualNorm
                   << " Number of ICCG iterations: "
                   << m_lastNumberOfIterations;

    return (m_lastResidualNorm <= m_tolerance) ||
           (m_lastNumberOfIterations < m_maxNumberOfIterations);
}

bool FDMICCGSolver3::SolveCompressed(FDMCompressedLinearSystem3* system)
{
    MatrixCSRD& matrix = system->A;
    VectorND& solution = system->x;
    VectorND& rhs = system->b;

    ClearUncompressedVectors();

    const size_t size = solution.GetRows();
    m_rComp.Resize(size);
    m_dComp.Resize(size);
    m_qComp.Resize(size);
    m_sComp.Resize(size);

    system->x.Fill(0.0);
    m_rComp.Fill(0.0);
    m_dComp.Fill(0.0);
    m_qComp.Fill(0.0);
    m_sComp.Fill(0.0);

    m_precondComp.Build(matrix);

    PCG<FDMCompressedBLAS3, PreconditionerCompressed>(
        matrix, rhs, m_maxNumberOfIterations, m_tolerance, &m_precondComp,
        &solution, &m_rComp, &m_dComp, &m_qComp, &m_sComp,
        &m_lastNumberOfIterations, &m_lastResidualNorm);

    CUBBYFLOW_INFO << "Residual after solving ICCG: " << m_lastResidualNorm
                   << " Number of ICCG iterations: "
                   << m_lastNumberOfIterations;

    return (m_lastResidualNorm <= m_tolerance) ||
           (m_lastNumberOfIterations < m_maxNumberOfIterations);
}

unsigned int FDMICCGSolver3::GetMaxNumberOfIterations() const
{
    return m_maxNumberOfIterations;
}

unsigned int FDMICCGSolver3::GetLastNumberOfIterations() const
{
    return m_lastNumberOfIterations;
}

double FDMICCGSolver3::GetTolerance() const
{
    return m_tolerance;
}

double FDMICCGSolver3::GetLastResidual() const
{
    return m_lastResidualNorm;
}

void FDMICCGSolver3::ClearUncompressedVectors()
{
    m_r.Clear();
    m_d.Clear();
    m_q.Clear();
    m_s.Clear();
}

void FDMICCGSolver3::ClearCompressedVectors()
{
    m_r.Clear();
    m_d.Clear();
    m_q.Clear();
    m_s.Clear();
}
}  // namespace CubbyFlow