// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/PointGenerator/BccLatticePointGenerator.hpp>

namespace CubbyFlow
{
void BccLatticePointGenerator::ForEachPoint(
    const BoundingBox3D& boundingBox, double spacing,
    const std::function<bool(const Vector3D&)>& callback) const
{
    const double halfSpacing = spacing / 2.0;
    const double boxWidth = boundingBox.Width();
    const double boxHeight = boundingBox.Height();
    const double boxDepth = boundingBox.Depth();

    Vector3D position;
    bool hasOffset = false;

    for (int k = 0; k * halfSpacing <= boxDepth; ++k)
    {
        position.z = k * halfSpacing + boundingBox.lowerCorner.z;

        const double offset = (hasOffset) ? halfSpacing : 0.0;

        for (int j = 0; j * spacing + offset <= boxHeight; ++j)
        {
            position.y = j * spacing + offset + boundingBox.lowerCorner.y;

            for (int i = 0; i * spacing + offset <= boxWidth; ++i)
            {
                position.x = i * spacing + offset + boundingBox.lowerCorner.x;

                if (!callback(position))
                {
                    return;
                }
            }
        }

        hasOffset = !hasOffset;
    }
}
}  // namespace CubbyFlow
