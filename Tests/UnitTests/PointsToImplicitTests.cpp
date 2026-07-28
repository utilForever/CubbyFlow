#include "gtest/gtest.h"

#include <Core/Grid/CellCenteredScalarGrid.hpp>
#include <Core/PointsToImplicit/AnisotropicPointsToImplicit2.hpp>
#include <Core/PointsToImplicit/AnisotropicPointsToImplicit3.hpp>
#include <Core/PointsToImplicit/SPHPointsToImplicit2.hpp>
#include <Core/PointsToImplicit/SPHPointsToImplicit3.hpp>
#include <Core/PointsToImplicit/SphericalPointsToImplicit2.hpp>
#include <Core/PointsToImplicit/SphericalPointsToImplicit3.hpp>
#include <Core/PointsToImplicit/ZhuBridsonPointsToImplicit2.hpp>
#include <Core/PointsToImplicit/ZhuBridsonPointsToImplicit3.hpp>

using namespace CubbyFlow;

namespace
{
template <typename Converter>
void ExpectConversion2(const Converter& converter)
{
    const Array1<Vector2D> points = {
        Vector2D{ 0.4, 0.4 }, Vector2D{ 0.5, 0.4 }, Vector2D{ 0.6, 0.4 },
        Vector2D{ 0.4, 0.5 }, Vector2D{ 0.5, 0.5 }, Vector2D{ 0.6, 0.5 },
        Vector2D{ 0.4, 0.6 }, Vector2D{ 0.5, 0.6 }, Vector2D{ 0.6, 0.6 }
    };
    CellCenteredScalarGrid2 grid({ 8, 8 }, { 0.125, 0.125 });

    converter.Convert(points.View(), &grid);

    EXPECT_LT(grid(3, 3), grid(0, 0));
}

template <typename Converter>
void ExpectConversion3(const Converter& converter)
{
    const Array1<Vector3D> points = {
        Vector3D{ 0.45, 0.45, 0.45 }, Vector3D{ 0.55, 0.45, 0.45 },
        Vector3D{ 0.45, 0.55, 0.45 }, Vector3D{ 0.55, 0.55, 0.45 },
        Vector3D{ 0.45, 0.45, 0.55 }, Vector3D{ 0.55, 0.45, 0.55 },
        Vector3D{ 0.45, 0.55, 0.55 }, Vector3D{ 0.55, 0.55, 0.55 }
    };
    CellCenteredScalarGrid3 grid({ 8, 8, 8 }, { 0.125, 0.125, 0.125 });

    converter.Convert(points.View(), &grid);

    EXPECT_LT(grid(3, 3, 3), grid(0, 0, 0));
}
}  // namespace

TEST(PointsToImplicit, Converts2)
{
    ExpectConversion2(AnisotropicPointsToImplicit2{ 0.4, 0.5, 0.5, 1 });
    ExpectConversion2(SPHPointsToImplicit2{ 0.4 });
    ExpectConversion2(SphericalPointsToImplicit2{ 0.2 });
    ExpectConversion2(ZhuBridsonPointsToImplicit2{ 0.4 });
}

TEST(PointsToImplicit, Converts3)
{
    ExpectConversion3(AnisotropicPointsToImplicit3{ 0.4, 0.5, 0.5, 1 });
    ExpectConversion3(SPHPointsToImplicit3{ 0.4 });
    ExpectConversion3(SphericalPointsToImplicit3{ 0.2 });
    ExpectConversion3(ZhuBridsonPointsToImplicit3{ 0.4 });
}
