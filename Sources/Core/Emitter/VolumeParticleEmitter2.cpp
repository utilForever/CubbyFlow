// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Emitter/VolumeParticleEmitter2.hpp>
#include <Core/Geometry/SurfaceToImplicit.hpp>
#include <Core/Matrix/Matrix.hpp>
#include <Core/PointGenerator/TrianglePointGenerator.hpp>
#include <Core/Searcher/PointHashGridSearcher.hpp>
#include <Core/Utils/Logging.hpp>

#include <utility>

namespace CubbyFlow
{
static const size_t DEFAULT_HASH_GRID_RESOLUTION = 64;

static Vector2D Jitter(const Vector2D& point, double maxDistance, double random)
{
    const double angle = (random - 0.5) * (2 * PI_DOUBLE);
    return point +
           maxDistance * (Matrix2x2D::MakeRotationMatrix(angle) * Vector2D());
}

VolumeParticleEmitter2::VolumeParticleEmitter2(
    ImplicitSurface2Ptr implicitSurface, BoundingBox2D maxRegion,
    double spacing, const Vector2D& initialVel, const Vector2D& linearVel,
    double angularVel, size_t maxNumberOfParticles, double jitter,
    bool isOneShot, bool allowOverlapping, uint32_t seed)
    : m_rng(seed),
      m_implicitSurface(std::move(implicitSurface)),
      m_maxRegion(std::move(maxRegion)),
      m_spacing(spacing),
      m_initialVel(initialVel),
      m_linearVel(linearVel),
      m_angularVel(angularVel),
      m_maxNumberOfParticles(maxNumberOfParticles),
      m_jitter(jitter),
      m_isOneShot(isOneShot),
      m_allowOverlapping(allowOverlapping)
{
    m_pointsGen = std::make_shared<TrianglePointGenerator>();
}

void VolumeParticleEmitter2::OnUpdate(double currentTimeInSeconds,
                                      double timeIntervalInSeconds)
{
    UNUSED_VARIABLE(currentTimeInSeconds);
    UNUSED_VARIABLE(timeIntervalInSeconds);

    auto particles = GetTarget();

    if (particles == nullptr)
    {
        return;
    }

    if (!GetIsEnabled())
    {
        return;
    }

    Array1<Vector2D> newPositions;
    Array1<Vector2D> newVelocities;

    Emit(particles, &newPositions, &newVelocities);

    particles->AddParticles(newPositions, newVelocities);

    if (m_isOneShot)
    {
        SetIsEnabled(false);
    }
}

void VolumeParticleEmitter2::Emit(const ParticleSystemData2Ptr& particles,
                                  Array1<Vector2D>* newPositions,
                                  Array1<Vector2D>* newVelocities)
{
    if (m_implicitSurface == nullptr)
    {
        return;
    }

    m_implicitSurface->UpdateQueryEngine();

    BoundingBox2D region = m_maxRegion;
    if (m_implicitSurface->IsBounded())
    {
        const BoundingBox2D surfaceBBox = m_implicitSurface->GetBoundingBox();
        region.lowerCorner = Max(region.lowerCorner, surfaceBBox.lowerCorner);
        region.upperCorner = Min(region.upperCorner, surfaceBBox.upperCorner);
    }

    // Reserving more space for jittering
    const double j = GetJitter();
    const double maxJitterDist = 0.5 * j * m_spacing;
    size_t numNewParticles = 0;

    if (m_allowOverlapping || m_isOneShot)
    {
        m_pointsGen->ForEachPoint(
            region, m_spacing,
            [this, &maxJitterDist, &newPositions,
             &numNewParticles](const Vector2D& point) {
                const Vector2D candidate =
                    Jitter(point, maxJitterDist, Random());

                if (m_implicitSurface->SignedDistance(candidate) > 0.0)
                {
                    return true;
                }

                if (m_numberOfEmittedParticles >= m_maxNumberOfParticles)
                {
                    return false;
                }

                newPositions->Append(candidate);
                ++m_numberOfEmittedParticles;
                ++numNewParticles;

                return true;
            });
    }
    else
    {
        // Use serial hash grid searcher for continuous update.
        PointHashGridSearcher2 neighborSearcher(
            Vector2UZ(DEFAULT_HASH_GRID_RESOLUTION,
                      DEFAULT_HASH_GRID_RESOLUTION),
            2.0 * m_spacing);

        if (!m_allowOverlapping)
        {
            neighborSearcher.Build(particles->Positions());
        }

        m_pointsGen->ForEachPoint(
            region, m_spacing,
            [this, &maxJitterDist, &neighborSearcher, &newPositions,
             &numNewParticles](const Vector2D& point) {
                const Vector2D candidate =
                    Jitter(point, maxJitterDist, Random());

                if (!m_implicitSurface->IsInside(candidate) ||
                    neighborSearcher.HasNearbyPoint(candidate, m_spacing))
                {
                    return true;
                }

                if (m_numberOfEmittedParticles >= m_maxNumberOfParticles)
                {
                    return false;
                }

                newPositions->Append(candidate);
                neighborSearcher.Add(candidate);
                ++m_numberOfEmittedParticles;
                ++numNewParticles;

                return true;
            });
    }

    CUBBYFLOW_INFO << "Number of newly generated particles: "
                   << numNewParticles;
    CUBBYFLOW_INFO << "Number of total generated particles: "
                   << m_numberOfEmittedParticles;

    newVelocities->Resize(newPositions->Length());
    ParallelForEachIndex(newVelocities->Length(), [&newVelocities, this,
                                                   &newPositions](size_t i) {
        (*newVelocities)[i] = VelocityAt((*newPositions)[i]);
    });
}

void VolumeParticleEmitter2::SetPointGenerator(
    const PointGenerator2Ptr& newPointsGen)
{
    m_pointsGen = newPointsGen;
}

const ImplicitSurface2Ptr& VolumeParticleEmitter2::GetSurface() const
{
    return m_implicitSurface;
}

void VolumeParticleEmitter2::SetSurface(const ImplicitSurface2Ptr& newSurface)
{
    m_implicitSurface = newSurface;
}

const BoundingBox2D& VolumeParticleEmitter2::GetMaxRegion() const
{
    return m_maxRegion;
}

void VolumeParticleEmitter2::SetMaxRegion(const BoundingBox2D& newMaxRegion)
{
    m_maxRegion = newMaxRegion;
}

double VolumeParticleEmitter2::GetJitter() const
{
    return m_jitter;
}

void VolumeParticleEmitter2::SetJitter(double newJitter)
{
    m_jitter = std::clamp(newJitter, 0.0, 1.0);
}

bool VolumeParticleEmitter2::GetIsOneShot() const
{
    return m_isOneShot;
}

void VolumeParticleEmitter2::SetIsOneShot(bool newValue)
{
    m_isOneShot = newValue;
}

bool VolumeParticleEmitter2::GetAllowOverlapping() const
{
    return m_allowOverlapping;
}

void VolumeParticleEmitter2::SetAllowOverlapping(bool newValue)
{
    m_allowOverlapping = newValue;
}

size_t VolumeParticleEmitter2::GetMaxNumberOfParticles() const
{
    return m_maxNumberOfParticles;
}

void VolumeParticleEmitter2::SetMaxNumberOfParticles(
    size_t newMaxNumberOfParticles)
{
    m_maxNumberOfParticles = newMaxNumberOfParticles;
}

double VolumeParticleEmitter2::GetSpacing() const
{
    return m_spacing;
}

void VolumeParticleEmitter2::SetSpacing(double newSpacing)
{
    m_spacing = newSpacing;
}

Vector2D VolumeParticleEmitter2::GetInitialVelocity() const
{
    return m_initialVel;
}

void VolumeParticleEmitter2::SetInitialVelocity(const Vector2D& newInitialVel)
{
    m_initialVel = newInitialVel;
}

Vector2D VolumeParticleEmitter2::GetLinearVelocity() const
{
    return m_linearVel;
}

void VolumeParticleEmitter2::SetLinearVelocity(const Vector2D& newLinearVel)
{
    m_linearVel = newLinearVel;
}

double VolumeParticleEmitter2::GetAngularVelocity() const
{
    return m_angularVel;
}

void VolumeParticleEmitter2::SetAngularVelocity(double newAngularVel)
{
    m_angularVel = newAngularVel;
}

double VolumeParticleEmitter2::Random()
{
    std::uniform_real_distribution<> d{ 0.0, 1.0 };
    return d(m_rng);
}

Vector2D VolumeParticleEmitter2::VelocityAt(const Vector2D& point) const
{
    const Vector2D r = point - m_implicitSurface->transform.GetTranslation();
    return m_linearVel + m_angularVel * Vector2D{ -r.y, r.x } + m_initialVel;
}

VolumeParticleEmitter2::Builder VolumeParticleEmitter2::GetBuilder()
{
    return Builder();
}

VolumeParticleEmitter2::Builder&
VolumeParticleEmitter2::Builder::WithImplicitSurface(
    const ImplicitSurface2Ptr& implicitSurface)
{
    m_implicitSurface = implicitSurface;

    if (!m_isBoundSet)
    {
        m_maxRegion = m_implicitSurface->GetBoundingBox();
    }

    return *this;
}

VolumeParticleEmitter2::Builder& VolumeParticleEmitter2::Builder::WithSurface(
    const Surface2Ptr& surface)
{
    m_implicitSurface = std::make_shared<SurfaceToImplicit2>(surface);

    if (!m_isBoundSet)
    {
        m_maxRegion = surface->GetBoundingBox();
    }

    return *this;
}

VolumeParticleEmitter2::Builder& VolumeParticleEmitter2::Builder::WithMaxRegion(
    const BoundingBox2D& maxRegion)
{
    m_maxRegion = maxRegion;
    m_isBoundSet = true;
    return *this;
}

VolumeParticleEmitter2::Builder& VolumeParticleEmitter2::Builder::WithSpacing(
    double spacing)
{
    m_spacing = spacing;
    return *this;
}

VolumeParticleEmitter2::Builder&
VolumeParticleEmitter2::Builder::WithInitialVelocity(const Vector2D& initialVel)
{
    m_initialVel = initialVel;
    return *this;
}

VolumeParticleEmitter2::Builder&
VolumeParticleEmitter2::Builder::WithLinearVelocity(const Vector2D& linearVel)
{
    m_linearVel = linearVel;
    return *this;
}

VolumeParticleEmitter2::Builder&
VolumeParticleEmitter2::Builder::WithAngularVelocity(double angularVel)
{
    m_angularVel = angularVel;
    return *this;
}

VolumeParticleEmitter2::Builder&
VolumeParticleEmitter2::Builder::WithMaxNumberOfParticles(
    size_t maxNumberOfParticles)
{
    m_maxNumberOfParticles = maxNumberOfParticles;
    return *this;
}

VolumeParticleEmitter2::Builder& VolumeParticleEmitter2::Builder::WithJitter(
    double jitter)
{
    m_jitter = jitter;
    return *this;
}

VolumeParticleEmitter2::Builder& VolumeParticleEmitter2::Builder::WithIsOneShot(
    bool isOneShot)
{
    m_isOneShot = isOneShot;
    return *this;
}

VolumeParticleEmitter2::Builder&
VolumeParticleEmitter2::Builder::WithAllowOverlapping(bool allowOverlapping)
{
    m_allowOverlapping = allowOverlapping;
    return *this;
}

VolumeParticleEmitter2::Builder&
VolumeParticleEmitter2::Builder::WithRandomSeed(uint32_t seed)
{
    m_seed = seed;
    return *this;
}

VolumeParticleEmitter2 VolumeParticleEmitter2::Builder::Build() const
{
    return VolumeParticleEmitter2(m_implicitSurface, m_maxRegion, m_spacing,
                                  m_initialVel, m_linearVel, m_angularVel,
                                  m_maxNumberOfParticles, m_jitter, m_isOneShot,
                                  m_allowOverlapping, m_seed);
}

VolumeParticleEmitter2Ptr VolumeParticleEmitter2::Builder::MakeShared() const
{
    return std::make_shared<VolumeParticleEmitter2>(
        m_implicitSurface, m_maxRegion, m_spacing, m_initialVel, m_linearVel,
        m_angularVel, m_maxNumberOfParticles, m_jitter, m_isOneShot,
        m_allowOverlapping, m_seed);
}
}  // namespace CubbyFlow
