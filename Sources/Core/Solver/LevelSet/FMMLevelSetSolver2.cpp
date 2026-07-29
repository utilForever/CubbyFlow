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
#include <Core/Solver/LevelSet/FMMLevelSetSolver2.hpp>
#include <Core/Utils/LevelSetUtils.hpp>

#include <algorithm>
#include <array>
#include <queue>

namespace CubbyFlow
{
static const char UNKNOWN = 0;
static const char KNOWN = 1;
static const char TRIAL = 2;

double BoundarySign(const ArrayView2<double>& sdf, const Vector2UZ& size,
                    size_t i, size_t j)
{
    const bool inside = IsInsideSDF(sdf(i, j));
    const auto crossesBoundary = [&inside, &sdf](bool valid, size_t x,
                                                 size_t y) {
        return valid && inside != IsInsideSDF(sdf(x, y));
    };
    const std::array neighbors{
        crossesBoundary(i > 0, i - 1, j),
        crossesBoundary(i + 1 < size.x, i + 1, j),
        crossesBoundary(j > 0, i, j - 1),
        crossesBoundary(j + 1 < size.y, i, j + 1),
    };

    if (std::find(neighbors.begin(), neighbors.end(), true) == neighbors.end())
    {
        return 0.0;
    }

    return inside ? -1.0 : 1.0;
}

bool HasKnownNeighbor(const Array2<char>& markers, const Vector2UZ& size,
                      size_t i, size_t j)
{
    const auto isKnown = [&markers](bool valid, size_t x, size_t y) {
        return valid && markers(x, y) == KNOWN;
    };
    const std::array neighbors{
        isKnown(i > 0, i - 1, j),
        isKnown(i + 1 < size.x, i + 1, j),
        isKnown(j > 0, i, j - 1),
        isKnown(j + 1 < size.y, i, j + 1),
    };

    return std::find(neighbors.begin(), neighbors.end(), true) !=
           neighbors.end();
}

// Find geometric solution near the boundary
inline double SolveQuadNearBoundary(const Array2<char>& markers,
                                    ArrayView2<double> output,
                                    const Vector2D& gridSpacing,
                                    const Vector2D& invGridSpacingSqr,
                                    double sign, size_t i, size_t j)
{
    UNUSED_VARIABLE(markers);
    UNUSED_VARIABLE(invGridSpacingSqr);

    const Vector2UZ size = output.Size();

    bool hasX = false;
    double phiX = std::numeric_limits<double>::max();

    if (i > 0)
    {
        if (IsInsideSDF(sign * output(i - 1, j)))
        {
            hasX = true;
            phiX = std::min(phiX, sign * output(i - 1, j));
        }
    }

    if (i + 1 < size.x)
    {
        if (IsInsideSDF(sign * output(i + 1, j)))
        {
            hasX = true;
            phiX = std::min(phiX, sign * output(i + 1, j));
        }
    }

    bool hasY = false;
    double phiY = std::numeric_limits<double>::max();

    if (j > 0)
    {
        if (IsInsideSDF(sign * output(i, j - 1)))
        {
            hasY = true;
            phiY = std::min(phiY, sign * output(i, j - 1));
        }
    }

    if (j + 1 < size.y)
    {
        if (IsInsideSDF(sign * output(i, j + 1)))
        {
            hasY = true;
            phiY = std::min(phiY, sign * output(i, j + 1));
        }
    }

    assert(hasX || hasY);

    const double distToBndX = gridSpacing.x * std::abs(output(i, j)) /
                              (std::abs(output(i, j)) + std::abs(phiX));
    const double distToBndY = gridSpacing.y * std::abs(output(i, j)) /
                              (std::abs(output(i, j)) + std::abs(phiY));

    double denomSqr = 0.0;

    if (hasX)
    {
        denomSqr += 1.0 / Square(distToBndX);
    }
    if (hasY)
    {
        denomSqr += 1.0 / Square(distToBndY);
    }

    const double solution = 1.0 / std::sqrt(denomSqr);

    return sign * solution;
}

inline double SolveQuad(const Array2<char>& markers, ArrayView2<double> output,
                        const Vector2D& gridSpacing,
                        const Vector2D& invGridSpacingSqr, size_t i, size_t j)
{
    const Vector2UZ size = output.Size();

    bool hasX = false;
    double phiX = std::numeric_limits<double>::max();

    if (i > 0)
    {
        if (markers(i - 1, j) == KNOWN)
        {
            hasX = true;
            phiX = std::min(phiX, output(i - 1, j));
        }
    }

    if (i + 1 < size.x)
    {
        if (markers(i + 1, j) == KNOWN)
        {
            hasX = true;
            phiX = std::min(phiX, output(i + 1, j));
        }
    }

    bool hasY = false;
    double phiY = std::numeric_limits<double>::max();

    if (j > 0)
    {
        if (markers(i, j - 1) == KNOWN)
        {
            hasY = true;
            phiY = std::min(phiY, output(i, j - 1));
        }
    }

    if (j + 1 < size.y)
    {
        if (markers(i, j + 1) == KNOWN)
        {
            hasY = true;
            phiY = std::min(phiY, output(i, j + 1));
        }
    }

    assert(hasX || hasY);

    double solution = 0.0;

    // Initial guess
    if (hasX)
    {
        solution = phiX + gridSpacing.x;
    }
    if (hasY)
    {
        solution = std::max(solution, phiY + gridSpacing.y);
    }

    // Solve quad
    double a = 0.0;
    double b = 0.0;
    double c = -1.0;

    if (hasX)
    {
        a += invGridSpacingSqr.x;
        b -= phiX * invGridSpacingSqr.x;
        c += Square(phiX) * invGridSpacingSqr.x;
    }
    if (hasY)
    {
        a += invGridSpacingSqr.y;
        b -= phiY * invGridSpacingSqr.y;
        c += Square(phiY) * invGridSpacingSqr.y;
    }

    const double det = b * b - a * c;

    if (det > 0.0)
    {
        solution = (-b + std::sqrt(det)) / a;
    }

    return solution;
}

void FMMLevelSetSolver2::Reinitialize(const ScalarGrid2& inputSDF,
                                      double maxDistance,
                                      ScalarGrid2* outputSDF)
{
    if (!inputSDF.HasSameShape(*outputSDF))
    {
        throw std::invalid_argument{
            "inputSDF and outputSDF have not same shape."
        };
    }

    Vector2UZ size = inputSDF.DataSize();
    Vector2D gridSpacing = inputSDF.GridSpacing();
    const Vector2D invGridSpacing = 1.0 / gridSpacing;
    Vector2D invGridSpacingSqr = ElemMul(invGridSpacing, invGridSpacing);
    Array2<char> markers{ size };

    auto output = outputSDF->DataView();

    ParallelForEachIndex(markers.Size(),
                         [&output, &inputSDF](size_t i, size_t j) {
                             output(i, j) = inputSDF(i, j);
                         });

    // Solve geometrically near the boundary
    ForEachIndex(markers.Size(), [&output, &size, &markers, &gridSpacing,
                                  &invGridSpacingSqr](size_t i, size_t j) {
        const double sign = BoundarySign(output, size, i, j);
        if (sign != 0.0)
        {
            output(i, j) = SolveQuadNearBoundary(markers, output, gridSpacing,
                                                 invGridSpacingSqr, sign, i, j);
        }
    });

    for (int sign = 0; sign < 2; ++sign)
    {
        // Build markers
        ParallelForEachIndex(markers.Size(),
                             [&output, &markers](size_t i, size_t j) {
                                 if (IsInsideSDF(output(i, j)))
                                 {
                                     markers(i, j) = KNOWN;
                                 }
                                 else
                                 {
                                     markers(i, j) = UNKNOWN;
                                 }
                             });

        auto compare = [&output](const Vector2UZ& a, const Vector2UZ& b) {
            return output(a.x, a.y) > output(b.x, b.y);
        };

        // Enqueue initial candidates
        std::priority_queue<Vector2UZ, std::vector<Vector2UZ>,
                            decltype(compare)>
            trial(compare);
        ForEachIndex(markers.Size(),
                     [&markers, &size, &trial](size_t i, size_t j) {
                         if (markers(i, j) != KNOWN &&
                             ((i > 0 && markers(i - 1, j) == KNOWN) ||
                              (i + 1 < size.x && markers(i + 1, j) == KNOWN) ||
                              (j > 0 && markers(i, j - 1) == KNOWN) ||
                              (j + 1 < size.y && markers(i, j + 1) == KNOWN)))
                         {
                             trial.push(Vector2UZ(i, j));
                             markers(i, j) = TRIAL;
                         }
                     });

        // Propagate
        while (!trial.empty())
        {
            const Vector2UZ idx = trial.top();
            trial.pop();

            const size_t i = idx.x;
            const size_t j = idx.y;

            markers(i, j) = KNOWN;
            output(i, j) = SolveQuad(markers, output, gridSpacing,
                                     invGridSpacingSqr, i, j);

            if (output(i, j) > maxDistance)
            {
                break;
            }

            if (i > 0)
            {
                if (markers(i - 1, j) == UNKNOWN)
                {
                    markers(i - 1, j) = TRIAL;
                    output(i - 1, j) = SolveQuad(markers, output, gridSpacing,
                                                 invGridSpacingSqr, i - 1, j);
                    trial.push(Vector2UZ{ i - 1, j });
                }
            }

            if (i + 1 < size.x)
            {
                if (markers(i + 1, j) == UNKNOWN)
                {
                    markers(i + 1, j) = TRIAL;
                    output(i + 1, j) = SolveQuad(markers, output, gridSpacing,
                                                 invGridSpacingSqr, i + 1, j);
                    trial.push(Vector2UZ{ i + 1, j });
                }
            }

            if (j > 0)
            {
                if (markers(i, j - 1) == UNKNOWN)
                {
                    markers(i, j - 1) = TRIAL;
                    output(i, j - 1) = SolveQuad(markers, output, gridSpacing,
                                                 invGridSpacingSqr, i, j - 1);
                    trial.push(Vector2UZ{ i, j - 1 });
                }
            }

            if (j + 1 < size.y)
            {
                if (markers(i, j + 1) == UNKNOWN)
                {
                    markers(i, j + 1) = TRIAL;
                    output(i, j + 1) = SolveQuad(markers, output, gridSpacing,
                                                 invGridSpacingSqr, i, j + 1);
                    trial.push(Vector2UZ{ i, j + 1 });
                }
            }
        }

        // Flip the sign
        ParallelForEachIndex(markers.Size(), [&output](size_t i, size_t j) {
            output(i, j) = -output(i, j);
        });
    }
}

void FMMLevelSetSolver2::Extrapolate(const ScalarGrid2& input,
                                     const ScalarField2& sdf,
                                     double maxDistance, ScalarGrid2* output)
{
    if (!input.HasSameShape(*output))
    {
        throw std::invalid_argument{ "input and output have not same shape." };
    }

    Array2<double> sdfGrid{ input.DataSize() };
    GridDataPositionFunc<2> pos = input.DataPosition();
    ParallelForEachIndex(sdfGrid.Size(),
                         [&sdfGrid, &sdf, &pos](size_t i, size_t j) {
                             sdfGrid(i, j) = sdf.Sample(pos(i, j));
                         });

    Extrapolate(input.DataView(), sdfGrid.View(), input.GridSpacing(),
                maxDistance, output->DataView());
}

void FMMLevelSetSolver2::Extrapolate(const CollocatedVectorGrid2& input,
                                     const ScalarField2& sdf,
                                     double maxDistance,
                                     CollocatedVectorGrid2* output)
{
    if (!input.HasSameShape(*output))
    {
        throw std::invalid_argument{ "input and output have not same shape." };
    }

    Array2<double> sdfGrid{ input.DataSize() };
    GridDataPositionFunc<2> pos = input.DataPosition();
    ParallelForEachIndex(sdfGrid.Size(),
                         [&sdfGrid, &sdf, &pos](size_t i, size_t j) {
                             sdfGrid(i, j) = sdf.Sample(pos(i, j));
                         });

    const Vector2D gridSpacing = input.GridSpacing();

    Array2<double> u{ input.DataSize() };
    Array2<double> u0{ input.DataSize() };
    Array2<double> v{ input.DataSize() };
    Array2<double> v0{ input.DataSize() };

    input.ParallelForEachDataPointIndex([&u, &input, &v](const Vector2UZ& idx) {
        u(idx) = input(idx).x;
        v(idx) = input(idx).y;
    });

    Extrapolate(u, sdfGrid.View(), gridSpacing, maxDistance, u0);
    Extrapolate(v, sdfGrid.View(), gridSpacing, maxDistance, v0);

    output->ParallelForEachDataPointIndex(
        [&output, &u, &v](const Vector2UZ& idx) {
            (*output)(idx).x = u(idx);
            (*output)(idx).y = v(idx);
        });
}

void FMMLevelSetSolver2::Extrapolate(const FaceCenteredGrid2& input,
                                     const ScalarField2& sdf,
                                     double maxDistance,
                                     FaceCenteredGrid2* output)
{
    if (!input.HasSameShape(*output))
    {
        throw std::invalid_argument{
            "inputSDF and outputSDF have not same shape."
        };
    }

    const Vector2D& gridSpacing = input.GridSpacing();

    const ConstArrayView2<double> u = input.UView();
    auto uPos = input.UPosition();
    Array2<double> sdfAtU{ u.Size() };
    input.ParallelForEachUIndex([&sdfAtU, &sdf, &uPos](const Vector2UZ& idx) {
        sdfAtU(idx) = sdf.Sample(uPos(idx));
    });

    Extrapolate(u, sdfAtU, gridSpacing, maxDistance, output->UView());

    const ConstArrayView2<double> v = input.VView();
    auto vPos = input.VPosition();
    Array2<double> sdfAtV{ v.Size() };
    input.ParallelForEachVIndex([&sdfAtV, &sdf, &vPos](const Vector2UZ& idx) {
        sdfAtV(idx) = sdf.Sample(vPos(idx));
    });

    Extrapolate(v, sdfAtV, gridSpacing, maxDistance, output->VView());
}

void FMMLevelSetSolver2::Extrapolate(const ConstArrayView2<double>& input,
                                     const ConstArrayView2<double>& sdf,
                                     const Vector2D& gridSpacing,
                                     double maxDistance,
                                     ArrayView2<double> output)
{
    Vector2UZ size = input.Size();
    const Vector2D invGridSpacing = 1.0 / gridSpacing;

    // Build markers
    Array2<char> markers{ size, UNKNOWN };
    ParallelForEachIndex(markers.Size(),
                         [&sdf, &markers, &output, &input](size_t i, size_t j) {
                             if (IsInsideSDF(sdf(i, j)))
                             {
                                 markers(i, j) = KNOWN;
                             }
                             output(i, j) = input(i, j);
                         });

    auto compare = [&sdf](const Vector2UZ& a, const Vector2UZ& b) {
        return sdf(a.x, a.y) > sdf(b.x, b.y);
    };

    // Enqueue initial candidates
    std::priority_queue<Vector2UZ, std::vector<Vector2UZ>, decltype(compare)>
        trial(compare);
    ForEachIndex(markers.Size(), [&markers, &trial, &size](size_t i, size_t j) {
        if (markers(i, j) != KNOWN && HasKnownNeighbor(markers, size, i, j))
        {
            trial.push(Vector2UZ{ i, j });
            markers(i, j) = TRIAL;
        }
    });

    // Propagate
    while (!trial.empty())
    {
        const Vector2UZ idx = trial.top();
        trial.pop();

        const size_t i = idx.x;
        const size_t j = idx.y;

        if (sdf(i, j) > maxDistance)
        {
            break;
        }

        Vector2D grad = Gradient2(sdf, gridSpacing, i, j).Normalized();

        double sum = 0.0;
        double count = 0.0;

        if (i > 0)
        {
            if (markers(i - 1, j) == KNOWN)
            {
                double weight = std::max(grad.x, 0.0) * invGridSpacing.x;

                // If gradient is zero, then just assign 1 to weight
                if (weight < std::numeric_limits<double>::epsilon())
                {
                    weight = 1.0;
                }

                sum += weight * output(i - 1, j);
                count += weight;
            }
            else if (markers(i - 1, j) == UNKNOWN)
            {
                markers(i - 1, j) = TRIAL;
                trial.push(Vector2UZ{ i - 1, j });
            }
        }

        if (i + 1 < size.x)
        {
            if (markers(i + 1, j) == KNOWN)
            {
                double weight = -std::min(grad.x, 0.0) * invGridSpacing.x;

                // If gradient is zero, then just assign 1 to weight
                if (weight < std::numeric_limits<double>::epsilon())
                {
                    weight = 1.0;
                }

                sum += weight * output(i + 1, j);
                count += weight;
            }
            else if (markers(i + 1, j) == UNKNOWN)
            {
                markers(i + 1, j) = TRIAL;
                trial.push(Vector2UZ{ i + 1, j });
            }
        }

        if (j > 0)
        {
            if (markers(i, j - 1) == KNOWN)
            {
                double weight = std::max(grad.y, 0.0) * invGridSpacing.y;

                // If gradient is zero, then just assign 1 to weight
                if (weight < std::numeric_limits<double>::epsilon())
                {
                    weight = 1.0;
                }

                sum += weight * output(i, j - 1);
                count += weight;
            }
            else if (markers(i, j - 1) == UNKNOWN)
            {
                markers(i, j - 1) = TRIAL;
                trial.push(Vector2UZ{ i, j - 1 });
            }
        }

        if (j + 1 < size.y)
        {
            if (markers(i, j + 1) == KNOWN)
            {
                double weight = -std::min(grad.y, 0.0) * invGridSpacing.y;

                // If gradient is zero, then just assign 1 to weight
                if (weight < std::numeric_limits<double>::epsilon())
                {
                    weight = 1.0;
                }

                sum += weight * output(i, j + 1);
                count += weight;
            }
            else if (markers(i, j + 1) == UNKNOWN)
            {
                markers(i, j + 1) = TRIAL;
                trial.push(Vector2UZ{ i, j + 1 });
            }
        }

        assert(count > 0.0);

        output(i, j) = sum / count;
        markers(i, j) = KNOWN;
    }
}
}  // namespace CubbyFlow
