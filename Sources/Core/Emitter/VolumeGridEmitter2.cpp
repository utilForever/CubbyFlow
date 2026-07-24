// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Emitter/VolumeGridEmitter2.hpp>
#include <Core/Geometry/SurfaceToImplicit.hpp>
#include <Core/Grid/CollocatedVectorGrid.hpp>
#include <Core/Grid/FaceCenteredGrid.hpp>
#include <Core/Utils/LevelSetUtils.hpp>
#include <Core/Utils/Macros.hpp>

#include <cmath>
#include <utility>

namespace CubbyFlow
{
VolumeGridEmitter2::VolumeGridEmitter2(ImplicitSurface2Ptr sourceRegion,
                                       bool isOneShot)
    : m_sourceRegion(std::move(sourceRegion)), m_isOneShot(isOneShot)
{
    // Do nothing
}

void VolumeGridEmitter2::AddSignedDistanceTarget(
    const ScalarGrid2Ptr& scalarGridTarget)
{
    auto mapper = [](double sdf, const Vector2D&, double oldVal) {
        return std::min(oldVal, sdf);
    };

    AddTarget(scalarGridTarget, mapper);
}

void VolumeGridEmitter2::AddStepFunctionTarget(
    const ScalarGrid2Ptr& scalarGridTarget, double minValue, double maxValue)
{
    double smoothingWidth = scalarGridTarget->GridSpacing().Min();
    auto mapper = [minValue, maxValue, smoothingWidth](
                      double sdf, const Vector2D&, double oldVal) {
        const double step = 1.0 - SmearedHeavisideSDF(sdf / smoothingWidth);
        return std::max(oldVal, std::lerp(minValue, maxValue, step));
    };

    AddTarget(scalarGridTarget, mapper);
}

void VolumeGridEmitter2::AddTarget(const ScalarGrid2Ptr& scalarGridTarget,
                                   const ScalarMapper& customMapper)
{
    m_customScalarTargets.emplace_back(scalarGridTarget, customMapper);
}

void VolumeGridEmitter2::AddTarget(const VectorGrid2Ptr& vectorGridTarget,
                                   const VectorMapper& customMapper)
{
    m_customVectorTargets.emplace_back(vectorGridTarget, customMapper);
}

const ImplicitSurface2Ptr& VolumeGridEmitter2::GetSourceRegion() const
{
    return m_sourceRegion;
}

bool VolumeGridEmitter2::GetIsOneShot() const
{
    return m_isOneShot;
}

void VolumeGridEmitter2::OnUpdate(double currentTimeInSeconds,
                                  double timeIntervalInSeconds)
{
    UNUSED_VARIABLE(currentTimeInSeconds);
    UNUSED_VARIABLE(timeIntervalInSeconds);

    if (!GetIsEnabled())
    {
        return;
    }

    Emit();

    if (m_isOneShot)
    {
        SetIsEnabled(false);
    }

    m_hasEmitted = true;
}

void VolumeGridEmitter2::Emit()
{
    if (m_sourceRegion == nullptr)
    {
        return;
    }

    m_sourceRegion->UpdateQueryEngine();

    for (const auto& target : m_customScalarTargets)
    {
        const auto& grid = std::get<0>(target);
        const auto& mapper = std::get<1>(target);

        GridDataPositionFunc<2> pos = grid->DataPosition();
        grid->ParallelForEachDataPointIndex([&](size_t i, size_t j) {
            const Vector2D gx = pos(i, j);
            const double sdf = GetSourceRegion()->SignedDistance(gx);

            (*grid)(i, j) = mapper(sdf, gx, (*grid)(i, j));
        });
    }

    for (const auto& target : m_customVectorTargets)
    {
        const auto& grid = std::get<0>(target);
        const auto& mapper = std::get<1>(target);

        CollocatedVectorGrid2Ptr collocated =
            std::dynamic_pointer_cast<CollocatedVectorGrid2>(grid);
        if (collocated != nullptr)
        {
            GridDataPositionFunc<2> pos = collocated->DataPosition();
            collocated->ParallelForEachDataPointIndex([&](size_t i, size_t j) {
                const Vector2D gx = pos(i, j);
                const double sdf = GetSourceRegion()->SignedDistance(gx);

                if (IsInsideSDF(sdf))
                {
                    (*collocated)(i, j) = mapper(sdf, gx, (*collocated)(i, j));
                }
            });

            continue;
        }

        FaceCenteredGrid2Ptr faceCentered =
            std::dynamic_pointer_cast<FaceCenteredGrid2>(grid);
        if (faceCentered != nullptr)
        {
            auto uPos = faceCentered->UPosition();
            auto vPos = faceCentered->VPosition();

            faceCentered->ParallelForEachUIndex([&](const Vector2UZ& idx) {
                const Vector2D gx = uPos(idx);
                const double sdf = GetSourceRegion()->SignedDistance(gx);
                const Vector2D oldVal = faceCentered->Sample(gx);
                const Vector2D newVal = mapper(sdf, gx, oldVal);

                faceCentered->U(idx) = newVal.x;
            });

            faceCentered->ParallelForEachVIndex([&](const Vector2UZ& idx) {
                const Vector2D gx = vPos(idx);
                const double sdf = GetSourceRegion()->SignedDistance(gx);
                const Vector2D oldVal = faceCentered->Sample(gx);
                const Vector2D newVal = mapper(sdf, gx, oldVal);

                faceCentered->V(idx) = newVal.y;
            });
        }
    }
}

VolumeGridEmitter2::Builder VolumeGridEmitter2::GetBuilder()
{
    return Builder();
}

VolumeGridEmitter2::Builder& VolumeGridEmitter2::Builder::WithSourceRegion(
    const Surface2Ptr& sourceRegion)
{
    const auto implicit =
        std::dynamic_pointer_cast<ImplicitSurface2>(sourceRegion);

    m_sourceRegion = implicit
                         ? implicit
                         : std::make_shared<SurfaceToImplicit2>(sourceRegion);

    return *this;
}

VolumeGridEmitter2::Builder& VolumeGridEmitter2::Builder::WithIsOneShot(
    bool isOneShot)
{
    m_isOneShot = isOneShot;
    return *this;
}

VolumeGridEmitter2 VolumeGridEmitter2::Builder::Build() const
{
    return VolumeGridEmitter2(m_sourceRegion, m_isOneShot);
}

VolumeGridEmitter2Ptr VolumeGridEmitter2::Builder::MakeShared() const
{
    return std::make_shared<VolumeGridEmitter2>(m_sourceRegion, m_isOneShot);
}
}  // namespace CubbyFlow
