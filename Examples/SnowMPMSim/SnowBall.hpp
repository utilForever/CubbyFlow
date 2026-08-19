// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_EXAMPLES_SNOW_MPM_SIM_SNOW_BALL_HPP
#define CUBBYFLOW_EXAMPLES_SNOW_MPM_SIM_SNOW_BALL_HPP

#include <Core/Array/Array.hpp>
#include <Core/Matrix/Matrix.hpp>
#include <Core/Solver/Particle/MPM/SnowMPMSolver.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <stdexcept>

namespace CubbyFlow
{
struct PaperSnowBall
{
    Array1<Vector3D> positions;
    Array1<double> massScales;
    Array1<double> hardeningScales;
};

inline void ConfigurePaperSemiImplicit(SnowMPMSolver3& solver,
                                       unsigned int requestedSubSteps)
{
    solver.SetIsUsingSemiImplicit(true);
    solver.SetIsUsingFixedSubTimeSteps(requestedSubSteps > 0);
    if (requestedSubSteps > 0)
    {
        solver.SetNumberOfFixedSubTimeSteps(requestedSubSteps);
    }
    solver.SetMaxNumberOfIterations(200);
    solver.SetTolerance(1e-3);
}

inline PaperSnowBall GeneratePaperSnowBall(const Vector3D& center,
                                           double radius, double gridSpacing,
                                           size_t particlesPerCell,
                                           uint32_t seed)
{
    if (radius <= 0.0 || gridSpacing <= 0.0 || particlesPerCell == 0)
    {
        throw std::invalid_argument("Invalid snowball sampling parameters.");
    }

    PaperSnowBall result;
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    const Vector3D lower = center - radius;
    const Vector3D upper = center + radius;

    const auto minX = static_cast<int64_t>(std::floor(lower.x / gridSpacing));
    const auto minY = static_cast<int64_t>(std::floor(lower.y / gridSpacing));
    const auto minZ = static_cast<int64_t>(std::floor(lower.z / gridSpacing));
    const auto maxX = static_cast<int64_t>(std::floor(upper.x / gridSpacing));
    const auto maxY = static_cast<int64_t>(std::floor(upper.y / gridSpacing));
    const auto maxZ = static_cast<int64_t>(std::floor(upper.z / gridSpacing));

    for (int64_t k = minZ; k <= maxZ; ++k)
    {
        for (int64_t j = minY; j <= maxY; ++j)
        {
            for (int64_t i = minX; i <= maxX; ++i)
            {
                for (size_t n = 0; n < particlesPerCell; ++n)
                {
                    const Vector3D position{
                        (static_cast<double>(i) + unit(rng)) * gridSpacing,
                        (static_cast<double>(j) + unit(rng)) * gridSpacing,
                        (static_cast<double>(k) + unit(rng)) * gridSpacing
                    };
                    const double normalizedRadius =
                        position.DistanceTo(center) / radius;

                    if (normalizedRadius > 1.0)
                    {
                        continue;
                    }

                    const bool isOuter = normalizedRadius >= 0.75;
                    const double noise = 0.8 + 0.4 * unit(rng);
                    result.positions.Append(position);
                    result.massScales.Append(isOuter ? 1.5 : 1.0);
                    result.hardeningScales.Append((isOuter ? 2.5 : 1.0) *
                                                  noise);
                }
            }
        }
    }

    return result;
}
}  // namespace CubbyFlow

#endif
