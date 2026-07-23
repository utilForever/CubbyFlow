// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Emitter/PointParticleEmitter2.hpp>
#include <Core/Matrix/Matrix.hpp>

namespace CubbyFlow
{
PointParticleEmitter2::PointParticleEmitter2(
    const Vector2D& origin, const Vector2D& direction, double speed,
    double spreadAngleInDegrees, size_t maxNumOfNewParticlesPerSec,
    size_t maxNumOfParticles, uint32_t seed)
    : m_rng(seed),
      m_maxNumberOfNewParticlesPerSecond(maxNumOfNewParticlesPerSec),
      m_maxNumberOfParticles(maxNumOfParticles),
      m_origin(origin),
      m_direction(direction),
      m_speed(speed),
      m_spreadAngleInRadians(DegreesToRadians(spreadAngleInDegrees))
{
    // Do nothing
}

size_t PointParticleEmitter2::GetMaxNumberOfNewParticlesPerSecond() const
{
    return m_maxNumberOfNewParticlesPerSecond;
}

void PointParticleEmitter2::SetMaxNumberOfNewParticlesPerSecond(size_t rate)
{
    m_maxNumberOfNewParticlesPerSecond = rate;
}

size_t PointParticleEmitter2::GetMaxNumberOfParticles() const
{
    return m_maxNumberOfParticles;
}

void PointParticleEmitter2::SetMaxNumberOfParticles(size_t maxNumberOfParticles)
{
    m_maxNumberOfParticles = maxNumberOfParticles;
}

void PointParticleEmitter2::OnUpdate(double currentTimeInSeconds,
                                     double timeIntervalInSeconds)
{
    auto particles = GetTarget();

    if (particles == nullptr)
    {
        return;
    }

    if (m_numberOfEmittedParticles == 0)
    {
        m_firstFrameTimeInSeconds = currentTimeInSeconds;
    }

    const double elapsedTimeInSeconds =
        currentTimeInSeconds - m_firstFrameTimeInSeconds;

    auto newMaxTotalNumberOfEmittedParticles = static_cast<size_t>(
        std::ceil((elapsedTimeInSeconds + timeIntervalInSeconds) *
                  static_cast<double>(m_maxNumberOfNewParticlesPerSecond)));
    newMaxTotalNumberOfEmittedParticles =
        std::min(newMaxTotalNumberOfEmittedParticles, m_maxNumberOfParticles);
    const size_t maxNumberOfNewParticles =
        newMaxTotalNumberOfEmittedParticles - m_numberOfEmittedParticles;

    if (maxNumberOfNewParticles > 0)
    {
        Array1<Vector2D> candidatePositions;
        Array1<Vector2D> candidateVelocities;
        Array1<Vector2D> newPositions;
        Array1<Vector2D> newVelocities;

        Emit(&candidatePositions, &candidateVelocities,
             maxNumberOfNewParticles);

        newPositions.Append(candidatePositions);
        newVelocities.Append(candidateVelocities);

        particles->AddParticles(newPositions, newVelocities);

        m_numberOfEmittedParticles += newPositions.Length();
    }
}

void PointParticleEmitter2::Emit(Array1<Vector2D>* newPositions,
                                 Array1<Vector2D>* newVelocities,
                                 size_t maxNewNumberOfParticles)
{
    for (size_t i = 0; i < maxNewNumberOfParticles; ++i)
    {
        double newAngleInRadian = (Random() - 0.5) * m_spreadAngleInRadians;
        Matrix2x2D rotationMatrix =
            Matrix2x2D::MakeRotationMatrix(newAngleInRadian);

        newPositions->Append(m_origin);
        newVelocities->Append(m_speed * (rotationMatrix * m_direction));
    }
}

double PointParticleEmitter2::Random()
{
    std::uniform_real_distribution<> d{ 0.0, 1.0 };
    return d(m_rng);
}

PointParticleEmitter2::Builder PointParticleEmitter2::GetBuilder()
{
    return Builder();
}

PointParticleEmitter2::Builder& PointParticleEmitter2::Builder::WithOrigin(
    const Vector2D& origin)
{
    m_origin = origin;
    return *this;
}

PointParticleEmitter2::Builder& PointParticleEmitter2::Builder::WithDirection(
    const Vector2D& direction)
{
    m_direction = direction;
    return *this;
}

PointParticleEmitter2::Builder& PointParticleEmitter2::Builder::WithSpeed(
    double speed)
{
    m_speed = speed;
    return *this;
}

PointParticleEmitter2::Builder&
PointParticleEmitter2::Builder::WithSpreadAngleInDegrees(
    double spreadAngleInDegrees)
{
    m_spreadAngleInDegrees = spreadAngleInDegrees;
    return *this;
}

PointParticleEmitter2::Builder&
PointParticleEmitter2::Builder::WithMaxNumberOfNewParticlesPerSecond(
    size_t maxNumOfNewParticlesPerSec)
{
    m_maxNumberOfNewParticlesPerSecond = maxNumOfNewParticlesPerSec;
    return *this;
}

PointParticleEmitter2::Builder&
PointParticleEmitter2::Builder::WithMaxNumberOfParticles(
    size_t maxNumberOfParticles)
{
    m_maxNumberOfParticles = maxNumberOfParticles;
    return *this;
}

PointParticleEmitter2::Builder& PointParticleEmitter2::Builder::WithRandomSeed(
    uint32_t seed)
{
    m_seed = seed;
    return *this;
}

PointParticleEmitter2 PointParticleEmitter2::Builder::Build() const
{
    return PointParticleEmitter2(
        m_origin, m_direction, m_speed, m_spreadAngleInDegrees,
        m_maxNumberOfNewParticlesPerSecond, m_maxNumberOfParticles, m_seed);
}

PointParticleEmitter2Ptr PointParticleEmitter2::Builder::MakeShared() const
{
    return std::make_shared<PointParticleEmitter2>(
        m_origin, m_direction, m_speed, m_spreadAngleInDegrees,
        m_maxNumberOfNewParticlesPerSecond, m_maxNumberOfParticles, m_seed);
}
}  // namespace CubbyFlow