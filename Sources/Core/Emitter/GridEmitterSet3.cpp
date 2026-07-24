// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Emitter/GridEmitterSet3.hpp>

namespace CubbyFlow
{
GridEmitterSet3::GridEmitterSet3(const std::vector<GridEmitter3Ptr>& emitters)
{
    for (const auto& e : emitters)
    {
        AddEmitter(e);
    }
}

void GridEmitterSet3::AddEmitter(const GridEmitter3Ptr& emitter)
{
    m_emitters.push_back(emitter);
}

void GridEmitterSet3::OnUpdate(double currentTimeInSeconds,
                               double timeIntervalInSeconds)
{
    if (!GetIsEnabled())
    {
        return;
    }

    for (auto& emitter : m_emitters)
    {
        emitter->Update(currentTimeInSeconds, timeIntervalInSeconds);
    }
}

GridEmitterSet3::Builder GridEmitterSet3::GetBuilder()
{
    return Builder();
}

GridEmitterSet3::Builder& GridEmitterSet3::Builder::WithEmitters(
    const std::vector<GridEmitter3Ptr>& emitters)
{
    m_emitters = emitters;
    return *this;
}

GridEmitterSet3 GridEmitterSet3::Builder::Build() const
{
    return GridEmitterSet3(m_emitters);
}

GridEmitterSet3Ptr GridEmitterSet3::Builder::MakeShared() const
{
    return std::make_shared<GridEmitterSet3>(m_emitters);
}
}  // namespace CubbyFlow