// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Emitter/VolumeParticleEmitter3.hpp>
#include <Core/Geometry/SurfaceToImplicit.hpp>
#include <Core/PointGenerator/BccLatticePointGenerator.hpp>
#include <Core/Searcher/PointHashGridSearcher.hpp>
#include <Core/Utils/Logging.hpp>
#include <Core/Utils/Samplers.hpp>

#include <utility>

namespace CubbyFlow
{
static const size_t DEFAULT_HASH_GRID_RESOLUTION = 64;

static Vector3D Jitter(const Vector3D& point, double maxDistance, double u1,
                       double u2)
{
    return point + maxDistance * UniformSampleSphere(u1, u2);
}

VolumeParticleEmitter3::VolumeParticleEmitter3(
    ImplicitSurface3Ptr implicitSurface, BoundingBox3D maxRegion,
    double spacing, const Vector3D& initialVel, const Vector3D& linearVel,
    const Vector3D& angularVel, size_t maxNumberOfParticles, double jitter,
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
    m_pointsGen = std::make_shared<BccLatticePointGenerator>();
}

void VolumeParticleEmitter3::OnUpdate(double currentTimeInSeconds,
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

    Array1<Vector3D> newPositions;
    Array1<Vector3D> newVelocities;

    Emit(particles, &newPositions, &newVelocities);

    particles->AddParticles(newPositions, newVelocities);

    if (m_isOneShot)
    {
        SetIsEnabled(false);
    }
}

void VolumeParticleEmitter3::Emit(const ParticleSystemData3Ptr& particles,
                                  Array1<Vector3D>* newPositions,
                                  Array1<Vector3D>* newVelocities)
{
    if (m_implicitSurface == nullptr)
    {
        return;
    }

    m_implicitSurface->UpdateQueryEngine();

    BoundingBox3D region = m_maxRegion;
    if (m_implicitSurface->IsBounded())
    {
        const BoundingBox3D surfaceBBox = m_implicitSurface->GetBoundingBox();
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
             &numNewParticles](const Vector3D& point) {
                const Vector3D candidate =
                    Jitter(point, maxJitterDist, Random(), Random());

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
        PointHashGridSearcher3 neighborSearcher(
            Vector3UZ(DEFAULT_HASH_GRID_RESOLUTION,
                      DEFAULT_HASH_GRID_RESOLUTION,
                      DEFAULT_HASH_GRID_RESOLUTION),
            2.0 * m_spacing);

        if (!m_allowOverlapping)
        {
            neighborSearcher.Build(particles->Positions());
        }

        m_pointsGen->ForEachPoint(
            region, m_spacing,
            [this, &maxJitterDist, &neighborSearcher, &newPositions,
             &numNewParticles](const Vector3D& point) {
                const Vector3D candidate =
                    Jitter(point, maxJitterDist, Random(), Random());

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

void VolumeParticleEmitter3::SetPointGenerator(
    const PointGenerator3Ptr& newPointsGen)
{
    m_pointsGen = newPointsGen;
}

const ImplicitSurface3Ptr& VolumeParticleEmitter3::GetSurface() const
{
    return m_implicitSurface;
}

void VolumeParticleEmitter3::SetSurface(const ImplicitSurface3Ptr& newSurface)
{
    m_implicitSurface = newSurface;
}

const BoundingBox3D& VolumeParticleEmitter3::GetMaxRegion() const
{
    return m_maxRegion;
}

void VolumeParticleEmitter3::SetMaxRegion(const BoundingBox3D& newMaxRegion)
{
    m_maxRegion = newMaxRegion;
}

double VolumeParticleEmitter3::GetJitter() const
{
    return m_jitter;
}

void VolumeParticleEmitter3::SetJitter(double newJitter)
{
    m_jitter = std::clamp(newJitter, 0.0, 1.0);
}

bool VolumeParticleEmitter3::GetIsOneShot() const
{
    return m_isOneShot;
}

void VolumeParticleEmitter3::SetIsOneShot(bool newValue)
{
    m_isOneShot = newValue;
}

bool VolumeParticleEmitter3::GetAllowOverlapping() const
{
    return m_allowOverlapping;
}

void VolumeParticleEmitter3::SetAllowOverlapping(bool newValue)
{
    m_allowOverlapping = newValue;
}

size_t VolumeParticleEmitter3::GetMaxNumberOfParticles() const
{
    return m_maxNumberOfParticles;
}

void VolumeParticleEmitter3::SetMaxNumberOfParticles(
    size_t newMaxNumberOfParticles)
{
    m_maxNumberOfParticles = newMaxNumberOfParticles;
}

double VolumeParticleEmitter3::GetSpacing() const
{
    return m_spacing;
}

void VolumeParticleEmitter3::SetSpacing(double newSpacing)
{
    m_spacing = newSpacing;
}

Vector3D VolumeParticleEmitter3::GetInitialVelocity() const
{
    return m_initialVel;
}

void VolumeParticleEmitter3::SetInitialVelocity(const Vector3D& newInitialVel)
{
    m_initialVel = newInitialVel;
}

Vector3D VolumeParticleEmitter3::GetLinearVelocity() const
{
    return m_linearVel;
}

void VolumeParticleEmitter3::SetLinearVelocity(const Vector3D& newLinearVel)
{
    m_linearVel = newLinearVel;
}

Vector3D VolumeParticleEmitter3::GetAngularVelocity() const
{
    return m_angularVel;
}

void VolumeParticleEmitter3::SetAngularVelocity(const Vector3D& newAngularVel)
{
    m_angularVel = newAngularVel;
}

double VolumeParticleEmitter3::Random()
{
    std::uniform_real_distribution<> d{ 0.0, 1.0 };
    return d(m_rng);
}

Vector3D VolumeParticleEmitter3::VelocityAt(const Vector3D& point) const
{
    const Vector3D r = point - m_implicitSurface->transform.GetTranslation();
    return m_linearVel + m_angularVel.Cross(r) + m_initialVel;
}

VolumeParticleEmitter3::Builder VolumeParticleEmitter3::GetBuilder()
{
    return Builder();
}

VolumeParticleEmitter3::Builder&
VolumeParticleEmitter3::Builder::WithImplicitSurface(
    const ImplicitSurface3Ptr& implicitSurface)
{
    m_implicitSurface = implicitSurface;

    if (!m_isBoundSet)
    {
        m_maxRegion = m_implicitSurface->GetBoundingBox();
    }

    return *this;
}

VolumeParticleEmitter3::Builder& VolumeParticleEmitter3::Builder::WithSurface(
    const Surface3Ptr& surface)
{
    m_implicitSurface = std::make_shared<SurfaceToImplicit3>(surface);

    if (!m_isBoundSet)
    {
        m_maxRegion = surface->GetBoundingBox();
    }

    return *this;
}

VolumeParticleEmitter3::Builder& VolumeParticleEmitter3::Builder::WithMaxRegion(
    const BoundingBox3D& maxRegion)
{
    m_maxRegion = maxRegion;
    m_isBoundSet = true;
    return *this;
}

VolumeParticleEmitter3::Builder& VolumeParticleEmitter3::Builder::WithSpacing(
    double spacing)
{
    m_spacing = spacing;
    return *this;
}

VolumeParticleEmitter3::Builder&
VolumeParticleEmitter3::Builder::WithInitialVelocity(const Vector3D& initialVel)
{
    m_initialVel = initialVel;
    return *this;
}

VolumeParticleEmitter3::Builder&
VolumeParticleEmitter3::Builder::WithLinearVelocity(const Vector3D& linearVel)
{
    m_linearVel = linearVel;
    return *this;
}

VolumeParticleEmitter3::Builder&
VolumeParticleEmitter3::Builder::WithAngularVelocity(const Vector3D& angularVel)
{
    m_angularVel = angularVel;
    return *this;
}

VolumeParticleEmitter3::Builder&
VolumeParticleEmitter3::Builder::WithMaxNumberOfParticles(
    size_t maxNumberOfParticles)
{
    m_maxNumberOfParticles = maxNumberOfParticles;
    return *this;
}

VolumeParticleEmitter3::Builder& VolumeParticleEmitter3::Builder::WithJitter(
    double jitter)
{
    m_jitter = jitter;
    return *this;
}

VolumeParticleEmitter3::Builder& VolumeParticleEmitter3::Builder::WithIsOneShot(
    bool isOneShot)
{
    m_isOneShot = isOneShot;
    return *this;
}

VolumeParticleEmitter3::Builder&
VolumeParticleEmitter3::Builder::WithAllowOverlapping(bool allowOverlapping)
{
    m_allowOverlapping = allowOverlapping;
    return *this;
}

VolumeParticleEmitter3::Builder&
VolumeParticleEmitter3::Builder::WithRandomSeed(uint32_t seed)
{
    m_seed = seed;
    return *this;
}

VolumeParticleEmitter3 VolumeParticleEmitter3::Builder::Build() const
{
    return VolumeParticleEmitter3(m_implicitSurface, m_maxRegion, m_spacing,
                                  m_initialVel, m_linearVel, m_angularVel,
                                  m_maxNumberOfParticles, m_jitter, m_isOneShot,
                                  m_allowOverlapping, m_seed);
}

VolumeParticleEmitter3Ptr VolumeParticleEmitter3::Builder::MakeShared() const
{
    return std::make_shared<VolumeParticleEmitter3>(
        m_implicitSurface, m_maxRegion, m_spacing, m_initialVel, m_linearVel,
        m_angularVel, m_maxNumberOfParticles, m_jitter, m_isOneShot,
        m_allowOverlapping, m_seed);
}
}  // namespace CubbyFlow
