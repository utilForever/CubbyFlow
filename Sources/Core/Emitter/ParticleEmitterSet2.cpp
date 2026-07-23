// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Emitter/ParticleEmitterSet2.hpp>

#include <utility>

namespace CubbyFlow
{
ParticleEmitterSet2::ParticleEmitterSet2(
    std::vector<ParticleEmitter2Ptr> emitters)
    : m_emitters(std::move(emitters))
{
    // Do nothing
}

void ParticleEmitterSet2::AddEmitter(const ParticleEmitter2Ptr& emitter)
{
    m_emitters.push_back(emitter);
}

void ParticleEmitterSet2::OnSetTarget(const ParticleSystemData2Ptr& particles)
{
    for (auto& emitter : m_emitters)
    {
        emitter->SetTarget(particles);
    }
}

void ParticleEmitterSet2::OnUpdate(double currentTimeInSeconds,
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

ParticleEmitterSet2::Builder ParticleEmitterSet2::GetBuilder()
{
    return Builder();
}

ParticleEmitterSet2::Builder& ParticleEmitterSet2::Builder::WithEmitters(
    const std::vector<ParticleEmitter2Ptr>& emitters)
{
    m_emitters = emitters;
    return *this;
}

ParticleEmitterSet2 ParticleEmitterSet2::Builder::Build() const
{
    return ParticleEmitterSet2(m_emitters);
}

ParticleEmitterSet2Ptr ParticleEmitterSet2::Builder::MakeShared() const
{
    return std::make_shared<ParticleEmitterSet2>(m_emitters);
}
}  // namespace CubbyFlow