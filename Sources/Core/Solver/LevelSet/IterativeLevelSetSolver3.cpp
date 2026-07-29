// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Array/ArrayUtils.hpp>
#include <Core/FDM/FDMUtils.hpp>
#include <Core/Solver/LevelSet/IterativeLevelSetSolver3.hpp>
#include <Core/Utils/Logging.hpp>

namespace CubbyFlow
{
double ReinitializedValue(double value, double sign, double dtau,
                          const std::array<double, 2>& dx,
                          const std::array<double, 2>& dy,
                          const std::array<double, 2>& dz)
{
    const double positiveGradient =
        std::sqrt(Square(std::max(dx[0], 0.0)) + Square(std::min(dx[1], 0.0)) +
                  Square(std::max(dy[0], 0.0)) + Square(std::min(dy[1], 0.0)) +
                  Square(std::max(dz[0], 0.0)) + Square(std::min(dz[1], 0.0)));
    const double negativeGradient =
        std::sqrt(Square(std::min(dx[0], 0.0)) + Square(std::max(dx[1], 0.0)) +
                  Square(std::min(dy[0], 0.0)) + Square(std::max(dy[1], 0.0)) +
                  Square(std::min(dz[0], 0.0)) + Square(std::max(dz[1], 0.0)));

    return value - dtau * std::max(sign, 0.0) * (positiveGradient - 1.0) -
           dtau * std::min(sign, 0.0) * (negativeGradient - 1.0);
}

double ExtrapolatedValue(double value, double dtau, const Vector3D& gradient,
                         const std::array<double, 2>& dx,
                         const std::array<double, 2>& dy,
                         const std::array<double, 2>& dz)
{
    return value - dtau * (std::max(gradient.x, 0.0) * dx[0] +
                           std::min(gradient.x, 0.0) * dx[1] +
                           std::max(gradient.y, 0.0) * dy[0] +
                           std::min(gradient.y, 0.0) * dy[1] +
                           std::max(gradient.z, 0.0) * dz[0] +
                           std::min(gradient.z, 0.0) * dz[1]);
}

void IterativeLevelSetSolver3::Reinitialize(const ScalarGrid3& inputSDF,
                                            double maxDistance,
                                            ScalarGrid3* outputSDF)
{
    const Vector3UZ size = inputSDF.DataSize();
    const Vector3D gridSpacing = inputSDF.GridSpacing();

    if (!inputSDF.HasSameShape(*outputSDF))
    {
        throw std::invalid_argument{
            "inputSDF and outputSDF have not same shape."
        };
    }

    ArrayView3<double> outputAcc = outputSDF->DataView();

    const double dtau = PseudoTimeStep(inputSDF.DataView(), gridSpacing);
    const unsigned int numberOfIterations =
        DistanceToNumberOfIterations(maxDistance, dtau);

    Copy(inputSDF.DataView(), outputAcc);

    Array3<double> temp{ size };
    ArrayView3<double> tempAcc = temp.View();

    CUBBYFLOW_INFO << "Reinitializing with pseudoTimeStep: " << dtau
                   << " numberOfIterations: " << numberOfIterations;

    for (unsigned int n = 0; n < numberOfIterations; ++n)
    {
        inputSDF.ParallelForEachDataPointIndex(
            [&outputAcc, &gridSpacing, this, &dtau, &tempAcc](
                size_t i, size_t j, size_t k) {
                const double s = Sign(outputAcc, gridSpacing, i, j, k);

                std::array<double, 2> dx{}, dy{}, dz{};

                GetDerivatives(outputAcc, gridSpacing, i, j, k, &dx, &dy, &dz);
                tempAcc(i, j, k) =
                    ReinitializedValue(outputAcc(i, j, k), s, dtau, dx, dy, dz);
            });

        std::swap(tempAcc, outputAcc);
    }

    const ArrayView3<double> outputSDFAcc = outputSDF->DataView();
    Copy(outputAcc, outputSDFAcc);
}

void IterativeLevelSetSolver3::Extrapolate(const ScalarGrid3& input,
                                           const ScalarField3& sdf,
                                           double maxDistance,
                                           ScalarGrid3* output)
{
    if (!input.HasSameShape(*output))
    {
        throw std::invalid_argument{
            "inputSDF and outputSDF have not same shape."
        };
    }

    Array3<double> sdfGrid{ input.DataSize() };
    GridDataPositionFunc<3> pos = input.DataPosition();
    ParallelForEachIndex(sdfGrid.Size(),
                         [&sdfGrid, &sdf, &pos](size_t i, size_t j, size_t k) {
                             sdfGrid(i, j, k) = sdf.Sample(pos(i, j, k));
                         });

    Extrapolate(input.DataView(), sdfGrid.View(), input.GridSpacing(),
                maxDistance, output->DataView());
}

void IterativeLevelSetSolver3::Extrapolate(const CollocatedVectorGrid3& input,
                                           const ScalarField3& sdf,
                                           double maxDistance,
                                           CollocatedVectorGrid3* output)
{
    if (!input.HasSameShape(*output))
    {
        throw std::invalid_argument{
            "inputSDF and outputSDF have not same shape."
        };
    }

    Array3<double> sdfGrid{ input.DataSize() };
    GridDataPositionFunc<3> pos = input.DataPosition();
    ParallelForEachIndex(sdfGrid.Size(),
                         [&sdfGrid, &sdf, &pos](size_t i, size_t j, size_t k) {
                             sdfGrid(i, j, k) = sdf.Sample(pos(i, j, k));
                         });

    const Vector3D gridSpacing = input.GridSpacing();

    Array3<double> u{ input.DataSize() };
    Array3<double> u0{ input.DataSize() };
    Array3<double> v{ input.DataSize() };
    Array3<double> v0{ input.DataSize() };
    Array3<double> w{ input.DataSize() };
    Array3<double> w0{ input.DataSize() };

    input.ParallelForEachDataPointIndex(
        [&u, &input, &v, &w](size_t i, size_t j, size_t k) {
            u(i, j, k) = input(i, j, k).x;
            v(i, j, k) = input(i, j, k).y;
            w(i, j, k) = input(i, j, k).z;
        });

    Extrapolate(u, sdfGrid.View(), gridSpacing, maxDistance, u0);
    Extrapolate(v, sdfGrid.View(), gridSpacing, maxDistance, v0);
    Extrapolate(w, sdfGrid.View(), gridSpacing, maxDistance, w0);

    output->ParallelForEachDataPointIndex(
        [&output, &u, &v, &w](size_t i, size_t j, size_t k) {
            (*output)(i, j, k).x = u(i, j, k);
            (*output)(i, j, k).y = v(i, j, k);
            (*output)(i, j, k).z = w(i, j, k);
        });
}

void IterativeLevelSetSolver3::Extrapolate(const FaceCenteredGrid3& input,
                                           const ScalarField3& sdf,
                                           double maxDistance,
                                           FaceCenteredGrid3* output)
{
    if (!input.HasSameShape(*output))
    {
        throw std::invalid_argument{
            "inputSDF and outputSDF have not same shape."
        };
    }

    const Vector3D& gridSpacing = input.GridSpacing();

    const ConstArrayView3<double> u = input.UView();
    auto uPos = input.UPosition();
    Array3<double> sdfAtU{ u.Size() };
    input.ParallelForEachUIndex([&sdfAtU, &sdf, &uPos](const Vector3UZ& idx) {
        sdfAtU(idx) = sdf.Sample(uPos(idx));
    });

    Extrapolate(u, sdfAtU, gridSpacing, maxDistance, output->UView());

    const ConstArrayView3<double> v = input.VView();
    auto vPos = input.VPosition();
    Array3<double> sdfAtV{ v.Size() };
    input.ParallelForEachVIndex([&sdfAtV, &sdf, &vPos](const Vector3UZ& idx) {
        sdfAtV(idx) = sdf.Sample(vPos(idx));
    });

    Extrapolate(v, sdfAtV, gridSpacing, maxDistance, output->VView());

    const ConstArrayView3<double> w = input.WView();
    auto wPos = input.WPosition();
    Array3<double> sdfAtW{ w.Size() };
    input.ParallelForEachWIndex([&sdfAtW, &sdf, &wPos](const Vector3UZ& idx) {
        sdfAtW(idx) = sdf.Sample(wPos(idx));
    });

    Extrapolate(w, sdfAtW, gridSpacing, maxDistance, output->WView());
}

void IterativeLevelSetSolver3::Extrapolate(const ConstArrayView3<double>& input,
                                           const ConstArrayView3<double>& sdf,
                                           const Vector3D& gridSpacing,
                                           double maxDistance,
                                           ArrayView3<double> output)
{
    const Vector3UZ size = input.Size();

    ArrayView3<double> outputAcc{ output };

    const double dtau = PseudoTimeStep(sdf, gridSpacing);
    const unsigned int numberOfIterations =
        DistanceToNumberOfIterations(maxDistance, dtau);

    Copy(input, outputAcc);

    Array3<double> temp{ size };
    ArrayView3<double> tempAcc = temp.View();

    for (unsigned int n = 0; n < numberOfIterations; ++n)
    {
        ParallelFor(ZERO_SIZE, size.x, ZERO_SIZE, size.y, ZERO_SIZE, size.z,
                    [&sdf, &gridSpacing, this, &outputAcc, &tempAcc, &dtau](
                        size_t i, size_t j, size_t k) {
                        if (sdf(i, j, k) >= 0)
                        {
                            std::array<double, 2> dx{}, dy{}, dz{};
                            const Vector3D grad =
                                Gradient3(sdf, gridSpacing, i, j, k);

                            GetDerivatives(outputAcc, gridSpacing, i, j, k, &dx,
                                           &dy, &dz);
                            tempAcc(i, j, k) = ExtrapolatedValue(
                                outputAcc(i, j, k), dtau, grad, dx, dy, dz);
                        }
                        else
                        {
                            tempAcc(i, j, k) = outputAcc(i, j, k);
                        }
                    });

        std::swap(tempAcc, outputAcc);
    }

    Copy(outputAcc, output);
}

double IterativeLevelSetSolver3::GetMaxCFL() const
{
    return m_maxCFL;
}

void IterativeLevelSetSolver3::SetMaxCFL(double newMaxCFL)
{
    m_maxCFL = std::max(newMaxCFL, 0.0);
}

unsigned int IterativeLevelSetSolver3::DistanceToNumberOfIterations(
    double distance, double dtau)
{
    return static_cast<unsigned int>(std::ceil(distance / dtau));
}

double IterativeLevelSetSolver3::Sign(const ConstArrayView3<double>& sdf,
                                      const Vector3D& gridSpacing, size_t i,
                                      size_t j, size_t k)
{
    const double d = sdf(i, j, k);
    const double e = std::min({ gridSpacing.x, gridSpacing.y, gridSpacing.z });
    return d / std::sqrt(d * d + e * e);
}

double IterativeLevelSetSolver3::PseudoTimeStep(
    const ConstArrayView3<double>& sdf, const Vector3D& gridSpacing) const
{
    const Vector3UZ size = sdf.Size();

    const double h = std::max({ gridSpacing.x, gridSpacing.y, gridSpacing.z });

    double maxS = -std::numeric_limits<double>::max();
    double dtau = m_maxCFL * h;

    for (size_t k = 0; k < size.z; ++k)
    {
        for (size_t j = 0; j < size.y; ++j)
        {
            for (size_t i = 0; i < size.x; ++i)
            {
                double s = Sign(sdf, gridSpacing, i, j, k);
                maxS = std::max(s, maxS);
            }
        }
    }

    while (dtau * maxS / h > m_maxCFL)
    {
        dtau *= 0.5;
    }

    return dtau;
}
}  // namespace CubbyFlow
