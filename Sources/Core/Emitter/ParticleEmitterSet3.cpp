// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Emitter/ParticleEmitterSet3.hpp>

#include <utility>

namespace CubbyFlow
{
ParticleEmitterSet3::ParticleEmitterSet3(
    std::vector<ParticleEmitter3Ptr> emitters)
    : m_emitters(std::move(emitters))
{
    // Do nothing
}

void ParticleEmitterSet3::AddEmitter(const ParticleEmitter3Ptr& emitter)
{
    m_emitters.push_back(emitter);
}

void ParticleEmitterSet3::OnSetTarget(const ParticleSystemData3Ptr& particles)
{
    for (auto& emitter : m_emitters)
    {
        emitter->SetTarget(particles);
    }
}

void ParticleEmitterSet3::OnUpdate(double currentTimeInSeconds,
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

ParticleEmitterSet3::Builder ParticleEmitterSet3::GetBuilder()
{
    return Builder();
}

ParticleEmitterSet3::Builder& ParticleEmitterSet3::Builder::WithEmitters(
    const std::vector<ParticleEmitter3Ptr>& emitters)
{
    m_emitters = emitters;
    return *this;
}

ParticleEmitterSet3 ParticleEmitterSet3::Builder::Build() const
{
    return ParticleEmitterSet3(m_emitters);
}

ParticleEmitterSet3Ptr ParticleEmitterSet3::Builder::MakeShared() const
{
    return std::make_shared<ParticleEmitterSet3>(m_emitters);
}
}  // namespace CubbyFlow