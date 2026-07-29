// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Solver/FDM/FDMICCGSolver3.hpp>
#include <Core/Solver/Grid/GridBackwardEulerDiffusionSolver3.hpp>
#include <Core/Utils/LevelSetUtils.hpp>

namespace CubbyFlow
{
const char FLUID = 0;
const char AIR = 1;
const char BOUNDARY = 2;

namespace
{
struct MatrixRowData3
{
    const Vector3UZ& size;
    const Vector3D& c;
    const Array3<char>& markers;
    bool isDirichlet;
};

bool AddsCenter(char marker, bool isDirichlet)
{
    return marker == FLUID || (isDirichlet && marker != AIR);
}

void AddXTerms(size_t i, size_t j, size_t k, const MatrixRowData3& data,
               FDMMatrixRow3* row)
{
    if (i + 1 < data.size.x)
    {
        const char marker = data.markers(i + 1, j, k);

        if (AddsCenter(marker, data.isDirichlet))
        {
            row->center += data.c.x;
        }

        if (marker == FLUID)
        {
            row->right -= data.c.x;
        }
    }

    if (i > 0 && AddsCenter(data.markers(i - 1, j, k), data.isDirichlet))
    {
        row->center += data.c.x;
    }
}

void AddYTerms(size_t i, size_t j, size_t k, const MatrixRowData3& data,
               FDMMatrixRow3* row)
{
    if (j + 1 < data.size.y)
    {
        const char marker = data.markers(i, j + 1, k);

        if (AddsCenter(marker, data.isDirichlet))
        {
            row->center += data.c.y;
        }

        if (marker == FLUID)
        {
            row->up -= data.c.y;
        }
    }

    if (j > 0 && AddsCenter(data.markers(i, j - 1, k), data.isDirichlet))
    {
        row->center += data.c.y;
    }
}

void AddZTerms(size_t i, size_t j, size_t k, const MatrixRowData3& data,
               FDMMatrixRow3* row)
{
    if (k + 1 < data.size.z)
    {
        const char marker = data.markers(i, j, k + 1);

        if (AddsCenter(marker, data.isDirichlet))
        {
            row->center += data.c.z;
        }

        if (marker == FLUID)
        {
            row->front -= data.c.z;
        }
    }

    if (k > 0 && AddsCenter(data.markers(i, j, k - 1), data.isDirichlet))
    {
        row->center += data.c.z;
    }
}

FDMMatrixRow3 BuildMatrixRow(size_t i, size_t j, size_t k,
                             const MatrixRowData3& data)
{
    FDMMatrixRow3 row{};
    row.center = 1.0;

    if (data.markers(i, j, k) != FLUID)
    {
        return row;
    }

    AddXTerms(i, j, k, data, &row);
    AddYTerms(i, j, k, data, &row);
    AddZTerms(i, j, k, data, &row);

    return row;
}

template <typename ValueAt>
struct RHSData3
{
    const Vector3UZ& size;
    const Vector3D& c;
    const Array3<char>& markers;
    bool isDirichlet;
    const ValueAt& valueAt;
};

template <typename ValueAt>
double BoundaryContribution(size_t i, size_t j, size_t k, bool isValid,
                            double coefficient, const RHSData3<ValueAt>& data)
{
    if (!isValid || data.markers(i, j, k) != BOUNDARY)
    {
        return 0.0;
    }

    return coefficient * data.valueAt(i, j, k);
}

template <typename ValueAt>
double BuildRHS(size_t i, size_t j, size_t k, const RHSData3<ValueAt>& data)
{
    double result = data.valueAt(i, j, k);

    if (!data.isDirichlet || data.markers(i, j, k) != FLUID)
    {
        return result;
    }

    result +=
        BoundaryContribution(i + 1, j, k, i + 1 < data.size.x, data.c.x, data);
    result +=
        BoundaryContribution(i > 0 ? i - 1 : i, j, k, i > 0, data.c.x, data);
    result +=
        BoundaryContribution(i, j + 1, k, j + 1 < data.size.y, data.c.y, data);
    result +=
        BoundaryContribution(i, j > 0 ? j - 1 : j, k, j > 0, data.c.y, data);
    result +=
        BoundaryContribution(i, j, k + 1, k + 1 < data.size.z, data.c.z, data);
    result +=
        BoundaryContribution(i, j, k > 0 ? k - 1 : k, k > 0, data.c.z, data);

    return result;
}
}  // namespace

GridBackwardEulerDiffusionSolver3::GridBackwardEulerDiffusionSolver3(
    BoundaryType boundaryType)
    : m_boundaryType(boundaryType)
{
    m_systemSolver = std::make_shared<FDMICCGSolver3>(
        100, std::numeric_limits<double>::epsilon());
}

void GridBackwardEulerDiffusionSolver3::Solve(const ScalarGrid3& source,
                                              double diffusionCoefficient,
                                              double timeIntervalInSeconds,
                                              ScalarGrid3* dest,
                                              const ScalarField3& boundarySDF,
                                              const ScalarField3& fluidSDF)
{
    const GridDataPositionFunc<3> pos = source.DataPosition();
    const Vector3D& h = source.GridSpacing();
    const Vector3D c =
        timeIntervalInSeconds * diffusionCoefficient / ElemMul(h, h);

    BuildMarkers(source.DataSize(), pos, boundarySDF, fluidSDF);
    BuildMatrix(source.DataSize(), c);
    BuildVectors(source.DataView(), c);

    if (m_systemSolver != nullptr)
    {
        // Solve the system
        m_systemSolver->Solve(&m_system);

        // Assign the solution
        source.ParallelForEachDataPointIndex(
            [&dest, this](size_t i, size_t j, size_t k) {
                (*dest)(i, j, k) = m_system.x(i, j, k);
            });
    }
}

void GridBackwardEulerDiffusionSolver3::Solve(
    const CollocatedVectorGrid3& source, double diffusionCoefficient,
    double timeIntervalInSeconds, CollocatedVectorGrid3* dest,
    const ScalarField3& boundarySDF, const ScalarField3& fluidSDF)
{
    const GridDataPositionFunc<3> pos = source.DataPosition();
    const Vector3D& h = source.GridSpacing();
    const Vector3D c =
        timeIntervalInSeconds * diffusionCoefficient / ElemMul(h, h);

    BuildMarkers(source.DataSize(), pos, boundarySDF, fluidSDF);
    BuildMatrix(source.DataSize(), c);

    // u
    BuildVectors(source.DataView(), c, 0);

    if (m_systemSolver != nullptr)
    {
        // Solve the system
        m_systemSolver->Solve(&m_system);

        // Assign the solution
        source.ParallelForEachDataPointIndex(
            [&dest, this](const Vector3UZ& idx) {
                (*dest)(idx).x = m_system.x(idx);
            });
    }

    // v
    BuildVectors(source.DataView(), c, 1);

    if (m_systemSolver != nullptr)
    {
        // Solve the system
        m_systemSolver->Solve(&m_system);

        // Assign the solution
        source.ParallelForEachDataPointIndex(
            [&dest, this](const Vector3UZ& idx) {
                (*dest)(idx).y = m_system.x(idx);
            });
    }

    // w
    BuildVectors(source.DataView(), c, 2);

    if (m_systemSolver != nullptr)
    {
        // Solve the system
        m_systemSolver->Solve(&m_system);

        // Assign the solution
        source.ParallelForEachDataPointIndex(
            [&dest, this](const Vector3UZ& idx) {
                (*dest)(idx).z = m_system.x(idx);
            });
    }
}

void GridBackwardEulerDiffusionSolver3::Solve(const FaceCenteredGrid3& source,
                                              double diffusionCoefficient,
                                              double timeIntervalInSeconds,
                                              FaceCenteredGrid3* dest,
                                              const ScalarField3& boundarySDF,
                                              const ScalarField3& fluidSDF)
{
    const Vector3D& h = source.GridSpacing();
    const Vector3D c =
        timeIntervalInSeconds * diffusionCoefficient / ElemMul(h, h);

    // u
    const auto uPos = source.UPosition();
    BuildMarkers(source.USize(), uPos, boundarySDF, fluidSDF);
    BuildMatrix(source.USize(), c);
    BuildVectors(source.UView(), c);

    if (m_systemSolver != nullptr)
    {
        // Solve the system
        m_systemSolver->Solve(&m_system);

        // Assign the solution
        source.ParallelForEachUIndex([&dest, this](const Vector3UZ& idx) {
            dest->U(idx) = m_system.x(idx);
        });
    }

    // v
    const auto vPos = source.VPosition();
    BuildMarkers(source.VSize(), vPos, boundarySDF, fluidSDF);
    BuildMatrix(source.VSize(), c);
    BuildVectors(source.VView(), c);

    if (m_systemSolver != nullptr)
    {
        // Solve the system
        m_systemSolver->Solve(&m_system);

        // Assign the solution
        source.ParallelForEachVIndex([&dest, this](const Vector3UZ& idx) {
            dest->V(idx) = m_system.x(idx);
        });
    }

    // w
    const auto wPos = source.WPosition();
    BuildMarkers(source.WSize(), wPos, boundarySDF, fluidSDF);
    BuildMatrix(source.WSize(), c);
    BuildVectors(source.WView(), c);

    if (m_systemSolver != nullptr)
    {
        // Solve the system
        m_systemSolver->Solve(&m_system);

        // Assign the solution
        source.ParallelForEachWIndex([&dest, this](const Vector3UZ& idx) {
            dest->W(idx) = m_system.x(idx);
        });
    }
}

void GridBackwardEulerDiffusionSolver3::SetLinearSystemSolver(
    const FDMLinearSystemSolver3Ptr& Solver)
{
    m_systemSolver = Solver;
}

template <typename PositionFunc>
void GridBackwardEulerDiffusionSolver3::BuildMarkers(
    const Vector3UZ& size, const PositionFunc& pos,
    const ScalarField3& boundarySDF, const ScalarField3& fluidSDF)
{
    m_markers.Resize(size);

    ParallelForEachIndex(
        m_markers.Size(),
        [&boundarySDF, &pos, this, &fluidSDF](size_t i, size_t j, size_t k) {
            if (IsInsideSDF(boundarySDF.Sample(pos(i, j, k))))
            {
                m_markers(i, j, k) = BOUNDARY;
            }
            else if (IsInsideSDF(fluidSDF.Sample(pos(i, j, k))))
            {
                m_markers(i, j, k) = FLUID;
            }
            else
            {
                m_markers(i, j, k) = AIR;
            }
        });
}

void GridBackwardEulerDiffusionSolver3::BuildMatrix(const Vector3UZ& size,
                                                    const Vector3D& c)
{
    m_system.A.Resize(size);

    bool isBoundaryType = (m_boundaryType == BoundaryType::Dirichlet);
    const MatrixRowData3 data{ size, c, m_markers, isBoundaryType };

    // Build linear system
    ParallelForEachIndex(
        m_system.A.Size(), [this, &data](size_t i, size_t j, size_t k) {
            m_system.A(i, j, k) = BuildMatrixRow(i, j, k, data);
        });
}

void GridBackwardEulerDiffusionSolver3::BuildVectors(
    const ConstArrayView3<double>& f, const Vector3D& c)
{
    Vector3UZ size = f.Size();

    m_system.x.Resize(size, 0.0);
    m_system.b.Resize(size, 0.0);

    const auto valueAt = [&f](size_t x, size_t y, size_t z) {
        return f(x, y, z);
    };
    const RHSData3<decltype(valueAt)> data{
        size, c, m_markers, m_boundaryType == BoundaryType::Dirichlet, valueAt
    };

    // Build linear system
    ParallelForEachIndex(m_system.x.Size(),
                         [this, &f, &data](size_t i, size_t j, size_t k) {
                             m_system.x(i, j, k) = f(i, j, k);
                             m_system.b(i, j, k) = BuildRHS(i, j, k, data);
                         });
}

void GridBackwardEulerDiffusionSolver3::BuildVectors(
    const ConstArrayView3<Vector3D>& f, const Vector3D& c, size_t component)
{
    Vector3UZ size = f.Size();

    m_system.x.Resize(size, 0.0);
    m_system.b.Resize(size, 0.0);

    const auto valueAt = [&f, component](size_t x, size_t y, size_t z) {
        return f(x, y, z)[component];
    };
    const RHSData3<decltype(valueAt)> data{
        size, c, m_markers, m_boundaryType == BoundaryType::Dirichlet, valueAt
    };

    // Build linear system
    ParallelForEachIndex(m_system.x.Size(), [this, &f, component, &data](
                                                size_t i, size_t j, size_t k) {
        m_system.x(i, j, k) = f(i, j, k)[component];
        m_system.b(i, j, k) = BuildRHS(i, j, k, data);
    });
}
}  // namespace CubbyFlow
