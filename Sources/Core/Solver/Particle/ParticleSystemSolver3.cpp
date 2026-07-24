// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Array/ArrayUtils.hpp>
#include <Core/Field/ConstantVectorField.hpp>
#include <Core/Solver/Particle/ParticleSystemSolver3.hpp>
#include <Core/Utils/Logging.hpp>
#include <Core/Utils/Parallel.hpp>
#include <Core/Utils/Timer.hpp>

#include <algorithm>

namespace CubbyFlow
{
ParticleSystemSolver3::ParticleSystemSolver3()
    : ParticleSystemSolver3{ 1e-3, 1e-3 }
{
    // Do nothing
}

ParticleSystemSolver3::ParticleSystemSolver3(double radius, double mass)
{
    m_particleSystemData = std::make_shared<ParticleSystemData3>();
    m_particleSystemData->SetRadius(radius);
    m_particleSystemData->SetMass(mass);
    m_wind = std::make_shared<ConstantVectorField3>(Vector3D{});
}

double ParticleSystemSolver3::GetDragCoefficient() const
{
    return m_dragCoefficient;
}

void ParticleSystemSolver3::SetDragCoefficient(double newDragCoefficient)
{
    m_dragCoefficient = std::max(newDragCoefficient, 0.0);
}

double ParticleSystemSolver3::GetRestitutionCoefficient() const
{
    return m_restitutionCoefficient;
}

void ParticleSystemSolver3::SetRestitutionCoefficient(
    double newRestitutionCoefficient)
{
    m_restitutionCoefficient = std::clamp(newRestitutionCoefficient, 0.0, 1.0);
}

const Vector3D& ParticleSystemSolver3::GetGravity() const
{
    return m_gravity;
}

void ParticleSystemSolver3::SetGravity(const Vector3D& newGravity)
{
    m_gravity = newGravity;
}

const ParticleSystemData3Ptr& ParticleSystemSolver3::GetParticleSystemData()
    const
{
    return m_particleSystemData;
}

const Collider3Ptr& ParticleSystemSolver3::GetCollider() const
{
    return m_collider;
}

void ParticleSystemSolver3::SetCollider(const Collider3Ptr& newCollider)
{
    m_collider = newCollider;
}

const ParticleEmitter3Ptr& ParticleSystemSolver3::GetEmitter() const
{
    return m_emitter;
}

void ParticleSystemSolver3::SetEmitter(const ParticleEmitter3Ptr& newEmitter)
{
    m_emitter = newEmitter;
    newEmitter->SetTarget(m_particleSystemData);
}

const VectorField3Ptr& ParticleSystemSolver3::GetWind() const
{
    return m_wind;
}

void ParticleSystemSolver3::SetWind(const VectorField3Ptr& newWind)
{
    m_wind = newWind;
}

void ParticleSystemSolver3::OnInitialize()
{
    // When initializing the solver, update the collider and emitter state as
    // well since they also affects the initial condition of the simulation.
    Timer timer;
    UpdateCollider(0.0);
    CUBBYFLOW_INFO << "Update collider took " << timer.DurationInSeconds()
                   << " seconds";

    timer.Reset();
    UpdateEmitter(0.0);
    CUBBYFLOW_INFO << "Update emitter took " << timer.DurationInSeconds()
                   << " seconds";
}

void ParticleSystemSolver3::OnAdvanceTimeStep(double timeStepInSeconds)
{
    BeginAdvanceTimeStep(timeStepInSeconds);

    Timer timer;
    AccumulateForces(timeStepInSeconds);
    CUBBYFLOW_INFO << "Accumulating forces took " << timer.DurationInSeconds()
                   << " seconds";

    timer.Reset();
    TimeIntegration(timeStepInSeconds);
    CUBBYFLOW_INFO << "Time integration took " << timer.DurationInSeconds()
                   << " seconds";

    timer.Reset();
    ResolveCollision();
    CUBBYFLOW_INFO << "Resolving collision took " << timer.DurationInSeconds()
                   << " seconds";

    EndAdvanceTimeStep(timeStepInSeconds);
}

void ParticleSystemSolver3::AccumulateForces(double timeStepInSeconds)
{
    UNUSED_VARIABLE(timeStepInSeconds);

    // Add external forces
    AccumulateExternalForces();
}

void ParticleSystemSolver3::BeginAdvanceTimeStep(double timeStepInSeconds)
{
    // Clear forces
    ArrayView1<Vector3D> forces = m_particleSystemData->Forces();
    forces.Fill(Vector3D{});

    // Update collider and emitter
    Timer timer;
    UpdateCollider(timeStepInSeconds);
    CUBBYFLOW_INFO << "Update collider took " << timer.DurationInSeconds()
                   << " seconds";

    timer.Reset();
    UpdateEmitter(timeStepInSeconds);
    CUBBYFLOW_INFO << "Update emitter took " << timer.DurationInSeconds()
                   << " seconds";

    // Allocate buffers
    const size_t n = m_particleSystemData->NumberOfParticles();
    m_newPositions.Resize(n);
    m_newVelocities.Resize(n);

    OnBeginAdvanceTimeStep(timeStepInSeconds);
}

void ParticleSystemSolver3::EndAdvanceTimeStep(double timeStepInSeconds)
{
    // Update data
    const size_t n = m_particleSystemData->NumberOfParticles();
    ArrayView1<Vector3D> positions = m_particleSystemData->Positions();
    ArrayView1<Vector3D> velocities = m_particleSystemData->Velocities();

    ParallelFor(ZERO_SIZE, n, [&](size_t i) {
        positions[i] = m_newPositions[i];
        velocities[i] = m_newVelocities[i];
    });

    OnEndAdvanceTimeStep(timeStepInSeconds);
}

void ParticleSystemSolver3::OnBeginAdvanceTimeStep(double timeStepInSeconds)
{
    UNUSED_VARIABLE(timeStepInSeconds);
}

void ParticleSystemSolver3::OnEndAdvanceTimeStep(double timeStepInSeconds)
{
    UNUSED_VARIABLE(timeStepInSeconds);
}

void ParticleSystemSolver3::ResolveCollision()
{
    ResolveCollision(m_newPositions, m_newVelocities);
}

void ParticleSystemSolver3::ResolveCollision(ArrayView1<Vector3D> newPositions,
                                             ArrayView1<Vector3D> newVelocities)
{
    if (m_collider != nullptr)
    {
        const size_t numberOfParticles =
            m_particleSystemData->NumberOfParticles();
        const double radius = m_particleSystemData->Radius();

        ParallelFor(ZERO_SIZE, numberOfParticles, [&](size_t i) {
            m_collider->ResolveCollision(radius, m_restitutionCoefficient,
                                         &newPositions[i], &newVelocities[i]);
        });
    }
}

void ParticleSystemSolver3::SetParticleSystemData(
    const ParticleSystemData3Ptr& newParticles)
{
    m_particleSystemData = newParticles;
}

void ParticleSystemSolver3::AccumulateExternalForces()
{
    const size_t n = m_particleSystemData->NumberOfParticles();
    ArrayView1<Vector3D> forces = m_particleSystemData->Forces();
    ArrayView1<Vector3D> velocities = m_particleSystemData->Velocities();
    ArrayView1<Vector3D> positions = m_particleSystemData->Positions();
    const double mass = m_particleSystemData->Mass();

    ParallelFor(ZERO_SIZE, n, [&](size_t i) {
        // Gravity
        Vector3D force = mass * m_gravity;

        // Wind forces
        const Vector3D relativeVel =
            velocities[i] - m_wind->Sample(positions[i]);
        force += -m_dragCoefficient * relativeVel;

        forces[i] += force;
    });
}

void ParticleSystemSolver3::TimeIntegration(double timeStepInSeconds)
{
    const size_t n = m_particleSystemData->NumberOfParticles();
    ArrayView1<Vector3D> forces = m_particleSystemData->Forces();
    ArrayView1<Vector3D> velocities = m_particleSystemData->Velocities();
    ArrayView1<Vector3D> positions = m_particleSystemData->Positions();
    const double mass = m_particleSystemData->Mass();

    ParallelFor(ZERO_SIZE, n, [&](size_t i) {
        // Integrate velocity first
        Vector3D& newVelocity = m_newVelocities[i];
        newVelocity = velocities[i] + timeStepInSeconds * forces[i] / mass;

        // Integrate position.
        Vector3D& newPosition = m_newPositions[i];
        newPosition = positions[i] + timeStepInSeconds * newVelocity;
    });
}

void ParticleSystemSolver3::UpdateCollider(double timeStepInSeconds) const
{
    if (m_collider != nullptr)
    {
        m_collider->Update(GetCurrentTimeInSeconds(), timeStepInSeconds);
    }
}

void ParticleSystemSolver3::UpdateEmitter(double timeStepInSeconds) const
{
    if (m_emitter != nullptr)
    {
        m_emitter->Update(GetCurrentTimeInSeconds(), timeStepInSeconds);
    }
}

ParticleSystemSolver3::Builder ParticleSystemSolver3::GetBuilder()
{
    return Builder{};
}

ParticleSystemSolver3 ParticleSystemSolver3::Builder::Build() const
{
    return ParticleSystemSolver3{ m_radius, m_mass };
}

ParticleSystemSolver3Ptr ParticleSystemSolver3::Builder::MakeShared() const
{
    return std::make_shared<ParticleSystemSolver3>(m_radius, m_mass);
}
}  // namespace CubbyFlow