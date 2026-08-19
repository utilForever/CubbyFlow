#include "gtest/gtest.h"

#include "../../Examples/Particles2Vol/MitsubaVolume.hpp"

#include <algorithm>
#include <bit>
#include <cstdint>
#include <numeric>

using namespace CubbyFlow;

namespace
{
uint32_t ReadUInt32(const std::vector<uint8_t>& bytes, size_t offset)
{
    return static_cast<uint32_t>(bytes[offset]) |
           static_cast<uint32_t>(bytes[offset + 1]) << 8 |
           static_cast<uint32_t>(bytes[offset + 2]) << 16 |
           static_cast<uint32_t>(bytes[offset + 3]) << 24;
}
}  // namespace

TEST(MitsubaVolume, RasterizesWithMPMCubicKernel)
{
    const Array1<Vector3D> positions{ { 0.5, 0.5, 0.5 } };
    const auto volume = RasterizeMitsubaDensity(
        positions, 2, 2, 1.0, { { 0.0, 0.0, 0.0 }, { 1.0, 1.0, 1.0 } });

    EXPECT_EQ(volume.resolution, Vector3UZ(3, 3, 3));
    ASSERT_EQ(volume.density.size(), 27u);
    EXPECT_NEAR(
        std::accumulate(volume.density.begin(), volume.density.end(), 0.0), 1.0,
        1e-6);
    EXPECT_NEAR(*std::max_element(volume.density.begin(), volume.density.end()),
                8.0 / 27.0, 1e-6);
}

TEST(MitsubaVolume, PreservesSourceParticleVolume)
{
    const Array1<Vector3D> positions{ { 0.5, 0.5, 0.5 } };
    const auto volume = RasterizeMitsubaDensity(
        positions, 2, 4, 1.0, { { 0.0, 0.0, 0.0 }, { 1.0, 1.0, 1.0 } });

    EXPECT_NEAR(
        std::accumulate(volume.density.begin(), volume.density.end(), 0.0),
        0.125, 1e-6);
}

TEST(MitsubaVolume, EncodesVersion3FloatVolume)
{
    MitsubaVolume volume;
    volume.resolution = { 2, 1, 1 };
    volume.bounds = { { 0.0, 0.1, 0.2 }, { 1.0, 1.1, 1.2 } };
    volume.density = { 0.25f, 0.75f };

    const auto bytes = EncodeMitsubaVolume(volume);

    ASSERT_EQ(bytes.size(), 48u + 2u * sizeof(float));
    EXPECT_EQ(bytes[0], 'V');
    EXPECT_EQ(bytes[1], 'O');
    EXPECT_EQ(bytes[2], 'L');
    EXPECT_EQ(bytes[3], 3);
    EXPECT_EQ(ReadUInt32(bytes, 4), 1u);
    EXPECT_EQ(ReadUInt32(bytes, 8), 2u);
    EXPECT_EQ(ReadUInt32(bytes, 12), 1u);
    EXPECT_EQ(ReadUInt32(bytes, 16), 1u);
    EXPECT_EQ(ReadUInt32(bytes, 20), 1u);
    EXPECT_EQ(ReadUInt32(bytes, 48), std::bit_cast<uint32_t>(0.25f));
    EXPECT_EQ(ReadUInt32(bytes, 52), std::bit_cast<uint32_t>(0.75f));
}
