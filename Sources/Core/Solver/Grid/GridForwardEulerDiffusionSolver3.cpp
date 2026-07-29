// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/FDM/FDMUtils.hpp>
#include <Core/Solver/Grid/GridForwardEulerDiffusionSolver3.hpp>
#include <Core/Utils/LevelSetUtils.hpp>

namespace CubbyFlow
{
static const char FLUID = 0;
static const char AIR = 1;
static const char BOUNDARY = 2;

template <typename T>
T Laplacian(const ConstArrayView3<T>& data, const Array3<char>& marker,
            const Vector3D& GridSpacing, size_t i, size_t j, size_t k)
{
    const T center = data(i, j, k);
    const Vector3UZ& ds = data.Size();

    assert(i < ds.x && j < ds.y && k < ds.z);

    T dLeft = T{};
    T dRight = T{};
    T dDown = T{};
    T dUp = T{};
    T dBack = T{};
    T dFront = T{};

    if (i > 0 && marker(i - 1, j, k) == FLUID)
    {
        dLeft = center - data(i - 1, j, k);
    }
    if (i + 1 < ds.x && marker(i + 1, j, k) == FLUID)
    {
        dRight = data(i + 1, j, k) - center;
    }

    if (j > 0 && marker(i, j - 1, k) == FLUID)
    {
        dDown = center - data(i, j - 1, k);
    }
    if (j + 1 < ds.y && marker(i, j + 1, k) == FLUID)
    {
        dUp = data(i, j + 1, k) - center;
    }

    if (k > 0 && marker(i, j, k - 1) == FLUID)
    {
        dBack = center - data(i, j, k - 1);
    }
    if (k + 1 < ds.z && marker(i, j, k + 1) == FLUID)
    {
        dFront = data(i, j, k + 1) - center;
    }

    return (dRight - dLeft) / Square(GridSpacing.x) +
           (dUp - dDown) / Square(GridSpacing.y) +
           (dFront - dBack) / Square(GridSpacing.z);
}

void GridForwardEulerDiffusionSolver3::Solve(const ScalarGrid3& source,
                                             double diffusionCoefficient,
                                             double timeIntervalInSeconds,
                                             ScalarGrid3* dest,
                                             const ScalarField3& boundarySDF,
                                             const ScalarField3& fluidSDF)
{
    ConstArrayView3<double> src = source.DataView();
    Vector3D h = source.GridSpacing();
    const GridDataPositionFunc<3> pos = source.DataPosition();

    BuildMarkers(source.Resolution(), pos, boundarySDF, fluidSDF);

    source.ParallelForEachDataPointIndex(
        [this, &dest, &source, &diffusionCoefficient, &timeIntervalInSeconds,
         &src, &h](size_t i, size_t j, size_t k) {
            if (m_markers(i, j, k) == FLUID)
            {
                (*dest)(i, j, k) =
                    source(i, j, k) + diffusionCoefficient *
                                          timeIntervalInSeconds *
                                          Laplacian(src, m_markers, h, i, j, k);
            }
            else
            {
                (*dest)(i, j, k) = source(i, j, k);
            }
        });
}

void GridForwardEulerDiffusionSolver3::Solve(
    const CollocatedVectorGrid3& source, double diffusionCoefficient,
    double timeIntervalInSeconds, CollocatedVectorGrid3* dest,
    const ScalarField3& boundarySDF, const ScalarField3& fluidSDF)
{
    ConstArrayView3<Vector3<double>> src = source.DataView();
    Vector3D h = source.GridSpacing();
    const GridDataPositionFunc<3> pos = source.DataPosition();

    BuildMarkers(source.Resolution(), pos, boundarySDF, fluidSDF);

    source.ParallelForEachDataPointIndex(
        [this, &dest, &src, &diffusionCoefficient, &timeIntervalInSeconds, &h,
         &source](const Vector3UZ& idx) {
            if (m_markers(idx) == FLUID)
            {
                (*dest)(idx) =
                    src(idx) +
                    diffusionCoefficient * timeIntervalInSeconds *
                        Laplacian(src, m_markers, h, idx.x, idx.y, idx.z);
            }
            else
            {
                (*dest)(idx) = source(idx);
            }
        });
}

void GridForwardEulerDiffusionSolver3::Solve(const FaceCenteredGrid3& source,
                                             double diffusionCoefficient,
                                             double timeIntervalInSeconds,
                                             FaceCenteredGrid3* dest,
                                             const ScalarField3& boundarySDF,
                                             const ScalarField3& fluidSDF)
{
    ConstArrayView3<double> uSrc = source.UView();
    ConstArrayView3<double> vSrc = source.VView();
    ConstArrayView3<double> wSrc = source.WView();
    ArrayView3<double> u = dest->UView();
    ArrayView3<double> v = dest->VView();
    ArrayView3<double> w = dest->WView();
    GridDataPositionFunc<3> uPos = source.UPosition();
    GridDataPositionFunc<3> vPos = source.VPosition();
    GridDataPositionFunc<3> wPos = source.WPosition();
    Vector3D h = source.GridSpacing();

    BuildMarkers(source.USize(), uPos, boundarySDF, fluidSDF);

    source.ParallelForEachUIndex([&boundarySDF, &uPos, &u, &uSrc,
                                  &diffusionCoefficient, &timeIntervalInSeconds,
                                  &h](const Vector3UZ& idx) {
        if (!IsInsideSDF(boundarySDF.Sample(uPos(idx))))
        {
            u(idx) = uSrc(idx) + diffusionCoefficient * timeIntervalInSeconds *
                                     Laplacian3(uSrc, h, idx.x, idx.y, idx.z);
        }
    });

    BuildMarkers(source.VSize(), vPos, boundarySDF, fluidSDF);

    source.ParallelForEachVIndex([&boundarySDF, &vPos, &v, &vSrc,
                                  &diffusionCoefficient, &timeIntervalInSeconds,
                                  &h](const Vector3UZ& idx) {
        if (!IsInsideSDF(boundarySDF.Sample(vPos(idx))))
        {
            v(idx) = vSrc(idx) + diffusionCoefficient * timeIntervalInSeconds *
                                     Laplacian3(vSrc, h, idx.x, idx.y, idx.z);
        }
    });

    BuildMarkers(source.WSize(), wPos, boundarySDF, fluidSDF);

    source.ParallelForEachWIndex([&boundarySDF, &wPos, &w, &wSrc,
                                  &diffusionCoefficient, &timeIntervalInSeconds,
                                  &h](const Vector3UZ& idx) {
        if (!IsInsideSDF(boundarySDF.Sample(wPos(idx))))
        {
            w(idx) = wSrc(idx) + diffusionCoefficient * timeIntervalInSeconds *
                                     Laplacian3(wSrc, h, idx.x, idx.y, idx.z);
        }
    });
}

template <typename PositionFunc>
void GridForwardEulerDiffusionSolver3::BuildMarkers(
    const Vector3UZ& size, const PositionFunc& pos,
    const ScalarField3& boundarySDF, const ScalarField3& fluidSDF)
{
    m_markers.Resize(size);

    ForEachIndex(m_markers.Size(), [&boundarySDF, &pos, this, &fluidSDF](
                                       size_t i, size_t j, size_t k) {
        if (IsInsideSDF(boundarySDF.Sample(pos(Vector3UZ{ i, j, k }))))
        {
            m_markers(i, j, k) = BOUNDARY;
        }
        else if (IsInsideSDF(fluidSDF.Sample(pos(Vector3UZ{ i, j, k }))))
        {
            m_markers(i, j, k) = FLUID;
        }
        else
        {
            m_markers(i, j, k) = AIR;
        }
    });
}
}  // namespace CubbyFlow
