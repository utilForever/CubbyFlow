// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Emitter/VolumeGridEmitter3.hpp>
#include <Core/Geometry/SurfaceToImplicit.hpp>
#include <Core/Grid/CollocatedVectorGrid.hpp>
#include <Core/Grid/FaceCenteredGrid.hpp>
#include <Core/Utils/LevelSetUtils.hpp>
#include <Core/Utils/Macros.hpp>

#include <utility>

namespace CubbyFlow
{
VolumeGridEmitter3::VolumeGridEmitter3(ImplicitSurface3Ptr sourceRegion,
                                       bool isOneShot)
    : m_sourceRegion(std::move(sourceRegion)), m_isOneShot(isOneShot)
{
    // Do nothing
}

void VolumeGridEmitter3::AddSignedDistanceTarget(
    const ScalarGrid3Ptr& scalarGridTarget)
{
    auto mapper = [](double sdf, const Vector3D&, double oldVal) {
        return std::min(oldVal, sdf);
    };

    AddTarget(scalarGridTarget, mapper);
}

void VolumeGridEmitter3::AddStepFunctionTarget(
    const ScalarGrid3Ptr& scalarGridTarget, double minValue, double maxValue)
{
    double smoothingWidth = scalarGridTarget->GridSpacing().Min();
    auto mapper = [minValue, maxValue, smoothingWidth](
                      double sdf, const Vector3D&, double oldVal) {
        const double step = 1.0 - SmearedHeavisideSDF(sdf / smoothingWidth);
        return std::max(oldVal, (maxValue - minValue) * step + minValue);
    };

    AddTarget(scalarGridTarget, mapper);
}

void VolumeGridEmitter3::AddTarget(const ScalarGrid3Ptr& scalarGridTarget,
                                   const ScalarMapper& customMapper)
{
    m_customScalarTargets.emplace_back(scalarGridTarget, customMapper);
}

void VolumeGridEmitter3::AddTarget(const VectorGrid3Ptr& vectorGridTarget,
                                   const VectorMapper& customMapper)
{
    m_customVectorTargets.emplace_back(vectorGridTarget, customMapper);
}

const ImplicitSurface3Ptr& VolumeGridEmitter3::GetSourceRegion() const
{
    return m_sourceRegion;
}

bool VolumeGridEmitter3::GetIsOneShot() const
{
    return m_isOneShot;
}

void VolumeGridEmitter3::OnUpdate(double currentTimeInSeconds,
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

void VolumeGridEmitter3::Emit()
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

        GridDataPositionFunc<3> pos = grid->DataPosition();
        grid->ParallelForEachDataPointIndex([&](size_t i, size_t j, size_t k) {
            const Vector3D gx = pos(i, j, k);
            const double sdf = GetSourceRegion()->SignedDistance(gx);

            (*grid)(i, j, k) = mapper(sdf, gx, (*grid)(i, j, k));
        });
    }

    for (const auto& target : m_customVectorTargets)
    {
        const auto& grid = std::get<0>(target);
        const auto& mapper = std::get<1>(target);

        CollocatedVectorGrid3Ptr collocated =
            std::dynamic_pointer_cast<CollocatedVectorGrid3>(grid);
        if (collocated != nullptr)
        {
            GridDataPositionFunc<3> pos = collocated->DataPosition();
            collocated->ParallelForEachDataPointIndex(
                [&](size_t i, size_t j, size_t k) {
                    const Vector3D gx = pos(i, j, k);
                    const double sdf = GetSourceRegion()->SignedDistance(gx);

                    if (IsInsideSDF(sdf))
                    {
                        (*collocated)(i, j, k) =
                            mapper(sdf, gx, (*collocated)(i, j, k));
                    }
                });

            continue;
        }

        FaceCenteredGrid3Ptr faceCentered =
            std::dynamic_pointer_cast<FaceCenteredGrid3>(grid);
        if (faceCentered != nullptr)
        {
            auto uPos = faceCentered->UPosition();
            auto vPos = faceCentered->VPosition();
            auto wPos = faceCentered->WPosition();

            faceCentered->ParallelForEachUIndex([&](const Vector3UZ& idx) {
                const Vector3D gx = uPos(idx);
                const double sdf = GetSourceRegion()->SignedDistance(gx);
                const Vector3D oldVal = faceCentered->Sample(gx);
                const Vector3D newVal = mapper(sdf, gx, oldVal);

                faceCentered->U(idx) = newVal.x;
            });

            faceCentered->ParallelForEachVIndex([&](const Vector3UZ& idx) {
                const Vector3D gx = vPos(idx);
                const double sdf = GetSourceRegion()->SignedDistance(gx);
                const Vector3D oldVal = faceCentered->Sample(gx);
                const Vector3D newVal = mapper(sdf, gx, oldVal);

                faceCentered->V(idx) = newVal.y;
            });

            faceCentered->ParallelForEachWIndex([&](const Vector3UZ& idx) {
                const Vector3D gx = wPos(idx);
                const double sdf = GetSourceRegion()->SignedDistance(gx);
                const Vector3D oldVal = faceCentered->Sample(gx);
                const Vector3D newVal = mapper(sdf, gx, oldVal);

                faceCentered->W(idx) = newVal.z;
            });
        }
    }
}

VolumeGridEmitter3::Builder VolumeGridEmitter3::GetBuilder()
{
    return Builder();
}

VolumeGridEmitter3::Builder& VolumeGridEmitter3::Builder::WithSourceRegion(
    const Surface3Ptr& sourceRegion)
{
    const auto implicit =
        std::dynamic_pointer_cast<ImplicitSurface3>(sourceRegion);

    m_sourceRegion = implicit
                         ? implicit
                         : std::make_shared<SurfaceToImplicit3>(sourceRegion);

    return *this;
}

VolumeGridEmitter3::Builder& VolumeGridEmitter3::Builder::WithIsOneShot(
    bool isOneShot)
{
    m_isOneShot = isOneShot;
    return *this;
}

VolumeGridEmitter3 VolumeGridEmitter3::Builder::Build() const
{
    return VolumeGridEmitter3(m_sourceRegion, m_isOneShot);
}

VolumeGridEmitter3Ptr VolumeGridEmitter3::Builder::MakeShared() const
{
    return std::make_shared<VolumeGridEmitter3>(m_sourceRegion, m_isOneShot);
}
}  // namespace CubbyFlow