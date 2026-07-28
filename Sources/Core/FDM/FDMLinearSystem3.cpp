// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/FDM/FDMLinearSystem3.hpp>
#include <Core/Math/MathUtils.hpp>
#include <Core/Utils/IterationUtils.hpp>

#include <cassert>

namespace CubbyFlow
{
void FDMLinearSystem3::Clear()
{
    A.Clear();
    x.Clear();
    b.Clear();
}

void FDMLinearSystem3::Resize(const Vector3UZ& size)
{
    A.Resize(size);
    x.Resize(size);
    b.Resize(size);
}

void FDMCompressedLinearSystem3::Clear()
{
    A.Clear();
    x.Clear();
    b.Clear();
}

void FDMBLAS3::Set(double s, FDMVector3* result)
{
    result->Fill(s);
}

void FDMBLAS3::Set(const FDMVector3& v, FDMVector3* result)
{
    result->CopyFrom(v);
}

void FDMBLAS3::Set(double s, FDMMatrix3* result)
{
    FDMMatrixRow3 row;
    row.center = row.right = row.up = row.front = s;
    result->Fill(row);
}

void FDMBLAS3::Set(const FDMMatrix3& m, FDMMatrix3* result)
{
    result->CopyFrom(m);
}

double FDMBLAS3::Dot(const FDMVector3& a, const FDMVector3& b)
{
    const Vector3UZ& size = a.Size();

    assert(size == b.Size());

    double result = 0.0;

    for (size_t k = 0; k < size.z; ++k)
    {
        for (size_t j = 0; j < size.y; ++j)
        {
            for (size_t i = 0; i < size.x; ++i)
            {
                result += a(i, j, k) * b(i, j, k);
            }
        }
    }

    return result;
}

void FDMBLAS3::AXPlusY(double a, const FDMVector3& x, const FDMVector3& y,
                       FDMVector3* result)
{
    const Vector3UZ& size = x.Size();

    assert(x.Size() == y.Size());
    assert(x.Size() == result->Size());

    ParallelForEachIndex(size,
                         [&result, &a, &x, &y](size_t i, size_t j, size_t k) {
                             (*result)(i, j, k) = a * x(i, j, k) + y(i, j, k);
                         });
}

void FDMBLAS3::MVM(const FDMMatrix3& m, const FDMVector3& v, FDMVector3* result)
{
    const Vector3UZ& size = m.Size();

    assert(size == v.Size());
    assert(size == result->Size());

    ParallelForEachIndex(
        size, [&result, &m, &v, &size](size_t i, size_t j, size_t k) {
            (*result)(i, j, k) =
                m(i, j, k).center * v(i, j, k) +
                ((i > 0) ? m(i - 1, j, k).right * v(i - 1, j, k) : 0.0) +
                ((i + 1 < size.x) ? m(i, j, k).right * v(i + 1, j, k) : 0.0) +
                ((j > 0) ? m(i, j - 1, k).up * v(i, j - 1, k) : 0.0) +
                ((j + 1 < size.y) ? m(i, j, k).up * v(i, j + 1, k) : 0.0) +
                ((k > 0) ? m(i, j, k - 1).front * v(i, j, k - 1) : 0.0) +
                ((k + 1 < size.z) ? m(i, j, k).front * v(i, j, k + 1) : 0.0);
        });
}

void FDMBLAS3::Residual(const FDMMatrix3& a, const FDMVector3& x,
                        const FDMVector3& b, FDMVector3* result)
{
    const Vector3UZ& size = a.Size();

    assert(size == x.Size());
    assert(size == b.Size());
    assert(size == result->Size());

    ParallelForEachIndex(
        size, [&result, &b, &a, &x, &size](size_t i, size_t j, size_t k) {
            (*result)(i, j, k) =
                b(i, j, k) - a(i, j, k).center * x(i, j, k) -
                ((i > 0) ? a(i - 1, j, k).right * x(i - 1, j, k) : 0.0) -
                ((i + 1 < size.x) ? a(i, j, k).right * x(i + 1, j, k) : 0.0) -
                ((j > 0) ? a(i, j - 1, k).up * x(i, j - 1, k) : 0.0) -
                ((j + 1 < size.y) ? a(i, j, k).up * x(i, j + 1, k) : 0.0) -
                ((k > 0) ? a(i, j, k - 1).front * x(i, j, k - 1) : 0.0) -
                ((k + 1 < size.z) ? a(i, j, k).front * x(i, j, k + 1) : 0.0);
        });
}

double FDMBLAS3::L2Norm(const FDMVector3& v)
{
    return std::sqrt(Dot(v, v));
}

double FDMBLAS3::LInfNorm(const FDMVector3& v)
{
    const Vector3UZ& size = v.Size();
    double result = 0.0;

    for (size_t k = 0; k < size.z; ++k)
    {
        for (size_t j = 0; j < size.y; ++j)
        {
            for (size_t i = 0; i < size.x; ++i)
            {
                result = AbsMax(result, v(i, j, k));
            }
        }
    }

    return std::fabs(result);
}

void FDMCompressedBLAS3::Set(double s, VectorND* result)
{
    result->Fill(s);
}

void FDMCompressedBLAS3::Set(const VectorND& v, VectorND* result)
{
    result->CopyFrom(v);
}

void FDMCompressedBLAS3::Set(double s, MatrixCSRD* result)
{
    result->Set(s);
}

void FDMCompressedBLAS3::Set(const MatrixCSRD& m, MatrixCSRD* result)
{
    result->Set(m);
}

double FDMCompressedBLAS3::Dot(const VectorND& a, const VectorND& b)
{
    return a.Dot(b);
}

void FDMCompressedBLAS3::AXPlusY(double a, const VectorND& x, const VectorND& y,
                                 VectorND* result)
{
    *result = a * x + y;
}

void FDMCompressedBLAS3::MVM(const MatrixCSRD& m, const VectorND& v,
                             VectorND* result)
{
    const auto rp = m.RowPointersBegin();
    const auto ci = m.ColumnIndicesBegin();
    const auto nnz = m.NonZeroBegin();

    ParallelForEachIndex(v.GetRows(), [&rp, &ci, &nnz, &v, &result](size_t i) {
        const size_t rowBegin = rp[i];
        const size_t rowEnd = rp[i + 1];

        double sum = 0.0;

        for (size_t jj = rowBegin; jj < rowEnd; ++jj)
        {
            const size_t j = ci[jj];
            sum += nnz[jj] * v[j];
        }

        (*result)[i] = sum;
    });
}

void FDMCompressedBLAS3::Residual(const MatrixCSRD& a, const VectorND& x,
                                  const VectorND& b, VectorND* result)
{
    const auto rp = a.RowPointersBegin();
    const auto ci = a.ColumnIndicesBegin();
    const auto nnz = a.NonZeroBegin();

    ParallelForEachIndex(x.GetRows(),
                         [&rp, &ci, &nnz, &x, &result, &b](size_t i) {
                             const size_t rowBegin = rp[i];
                             const size_t rowEnd = rp[i + 1];

                             double sum = 0.0;

                             for (size_t jj = rowBegin; jj < rowEnd; ++jj)
                             {
                                 const size_t j = ci[jj];
                                 sum += nnz[jj] * x[j];
                             }

                             (*result)[i] = b[i] - sum;
                         });
}

double FDMCompressedBLAS3::L2Norm(const VectorND& v)
{
    return std::sqrt(v.Dot(v));
}

double FDMCompressedBLAS3::LInfNorm(const VectorND& v)
{
    return std::fabs(v.AbsMax());
}
}  // namespace CubbyFlow