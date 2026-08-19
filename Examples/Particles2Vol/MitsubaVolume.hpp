// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_EXAMPLES_PARTICLES2VOL_MITSUBA_VOLUME_HPP
#define CUBBYFLOW_EXAMPLES_PARTICLES2VOL_MITSUBA_VOLUME_HPP

#include <Core/Array/Array.hpp>
#include <Core/Geometry/BoundingBox.hpp>
#include <Core/Particle/MPM/MPMSystemData.hpp>

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace CubbyFlow
{
struct MitsubaVolume
{
    Vector3UZ resolution;
    BoundingBox3D bounds;
    std::vector<float> density;
};

inline MitsubaVolume RasterizeMitsubaDensity(
    ConstArrayView1<Vector3D> positions, size_t cellResolution,
    size_t sourceResolution, double particlesPerCell,
    const BoundingBox3D& bounds)
{
    if (cellResolution == 0 || sourceResolution == 0 ||
        !std::isfinite(particlesPerCell) || particlesPerCell <= 0.0 ||
        bounds.IsEmpty())
    {
        throw std::invalid_argument("Invalid volume rasterization parameters.");
    }

    MitsubaVolume result;
    result.resolution = { cellResolution + 1, cellResolution + 1,
                          cellResolution + 1 };
    result.bounds = bounds;
    result.density.resize(result.resolution.x * result.resolution.y *
                          result.resolution.z);
    const Vector3D gridSpacing{ bounds.Width() / cellResolution,
                                bounds.Height() / cellResolution,
                                bounds.Depth() / cellResolution };
    const double volumeScale =
        std::pow(static_cast<double>(cellResolution) /
                     static_cast<double>(sourceResolution),
                 3.0) /
        particlesPerCell;

    for (const auto& position : positions)
    {
        const auto stencil = CubicBSplineKernel<3>::GetStencil(
            position, gridSpacing, bounds.lowerCorner);
        for (const auto& entry : stencil)
        {
            if (entry.index.x < 0 || entry.index.y < 0 || entry.index.z < 0 ||
                entry.index.x >= static_cast<ssize_t>(result.resolution.x) ||
                entry.index.y >= static_cast<ssize_t>(result.resolution.y) ||
                entry.index.z >= static_cast<ssize_t>(result.resolution.z))
            {
                continue;
            }

            const size_t x = static_cast<size_t>(entry.index.x);
            const size_t y = static_cast<size_t>(entry.index.y);
            const size_t z = static_cast<size_t>(entry.index.z);
            const size_t index =
                (z * result.resolution.y + y) * result.resolution.x + x;
            result.density[index] +=
                static_cast<float>(entry.weight * volumeScale);
        }
    }

    for (auto& density : result.density)
    {
        density = std::clamp(density, 0.0f, 1.0f);
    }
    return result;
}

inline void AppendUInt32(std::vector<uint8_t>* bytes, uint32_t value)
{
    for (size_t i = 0; i < 4; ++i)
    {
        bytes->push_back(static_cast<uint8_t>(value >> (8 * i)));
    }
}

inline void AppendFloat(std::vector<uint8_t>* bytes, float value)
{
    AppendUInt32(bytes, std::bit_cast<uint32_t>(value));
}

inline std::vector<uint8_t> EncodeMitsubaVolume(const MitsubaVolume& volume)
{
    constexpr size_t maxResolution =
        static_cast<size_t>(std::numeric_limits<uint32_t>::max());
    if (volume.resolution.x == 0 || volume.resolution.y == 0 ||
        volume.resolution.z == 0 || volume.resolution.x > maxResolution ||
        volume.resolution.y > maxResolution ||
        volume.resolution.z > maxResolution || volume.bounds.IsEmpty() ||
        volume.density.size() !=
            volume.resolution.x * volume.resolution.y * volume.resolution.z)
    {
        throw std::invalid_argument("Invalid Mitsuba volume.");
    }

    std::vector<uint8_t> bytes;
    bytes.reserve(48 + volume.density.size() * sizeof(float));
    bytes.insert(bytes.end(), { 'V', 'O', 'L', 3 });
    AppendUInt32(&bytes, 1);
    AppendUInt32(&bytes, static_cast<uint32_t>(volume.resolution.x));
    AppendUInt32(&bytes, static_cast<uint32_t>(volume.resolution.y));
    AppendUInt32(&bytes, static_cast<uint32_t>(volume.resolution.z));
    AppendUInt32(&bytes, 1);
    AppendFloat(&bytes, static_cast<float>(volume.bounds.lowerCorner.x));
    AppendFloat(&bytes, static_cast<float>(volume.bounds.lowerCorner.y));
    AppendFloat(&bytes, static_cast<float>(volume.bounds.lowerCorner.z));
    AppendFloat(&bytes, static_cast<float>(volume.bounds.upperCorner.x));
    AppendFloat(&bytes, static_cast<float>(volume.bounds.upperCorner.y));
    AppendFloat(&bytes, static_cast<float>(volume.bounds.upperCorner.z));
    for (float density : volume.density)
    {
        AppendFloat(&bytes, density);
    }
    return bytes;
}
}  // namespace CubbyFlow

#endif
