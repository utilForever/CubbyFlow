// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Emitter/GridEmitterSet2.hpp>

namespace CubbyFlow
{
GridEmitterSet2::GridEmitterSet2(const std::vector<GridEmitter2Ptr>& emitters)
{
    for (const auto& e : emitters)
    {
        AddEmitter(e);
    }
}

void GridEmitterSet2::AddEmitter(const GridEmitter2Ptr& emitter)
{
    m_emitters.push_back(emitter);
}

void GridEmitterSet2::OnUpdate(double currentTimeInSeconds,
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

GridEmitterSet2::Builder GridEmitterSet2::GetBuilder()
{
    return Builder();
}

GridEmitterSet2::Builder& GridEmitterSet2::Builder::WithEmitters(
    const std::vector<GridEmitter2Ptr>& emitters)
{
    m_emitters = emitters;
    return *this;
}

GridEmitterSet2 GridEmitterSet2::Builder::Build() const
{
    return GridEmitterSet2(m_emitters);
}

GridEmitterSet2Ptr GridEmitterSet2::Builder::MakeShared() const
{
    return std::make_shared<GridEmitterSet2>(m_emitters);
}
}  // namespace CubbyFlow