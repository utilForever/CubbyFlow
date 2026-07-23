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
#include <Core/Grid/CellCenteredScalarGrid.hpp>
#include <Core/Solver/Hybrid/PIC/PICSolver3.hpp>
#include <Core/Utils/Logging.hpp>
#include <Core/Utils/Timer.hpp>

namespace CubbyFlow
{
PICSolver3::PICSolver3() : PICSolver3{ { 1, 1, 1 }, { 1, 1, 1 }, { 0, 0, 0 } }
{
    // Do nothing
}

PICSolver3::PICSolver3(const Vector3UZ& resolution, const Vector3D& gridSpacing,
                       const Vector3D& gridOrigin)
    : GridFluidSolver3{ resolution, gridSpacing, gridOrigin }
{
    GridSystemData3Ptr grids = GetGridSystemData();
    m_signedDistanceFieldID = grids->AddScalarData(
        std::make_shared<CellCenteredScalarGrid3::Builder>(),
        std::numeric_limits<double>::max());
    m_particles = std::make_shared<ParticleSystemData3>();
}

ScalarGrid3Ptr PICSolver3::GetSignedDistanceField() const
{
    return GetGridSystemData()->ScalarDataAt(m_signedDistanceFieldID);
}

const ParticleSystemData3Ptr& PICSolver3::GetParticleSystemData() const
{
    return m_particles;
}

const ParticleEmitter3Ptr& PICSolver3::GetParticleEmitter() const
{
    return m_particleEmitter;
}

void PICSolver3::SetParticleEmitter(const ParticleEmitter3Ptr& newEmitter)
{
    m_particleEmitter = newEmitter;
    newEmitter->SetTarget(m_particles);
}

void PICSolver3::OnInitialize()
{
    GridFluidSolver3::OnInitialize();

    const Timer timer;
    UpdateParticleEmitter(0.0);
    CUBBYFLOW_INFO << "Update particle emitter took "
                   << timer.DurationInSeconds() << " seconds";
}

void PICSolver3::OnBeginAdvanceTimeStep(double timeIntervalInSeconds)
{
    CUBBYFLOW_INFO << "Number of PIC-type particles: "
                   << m_particles->NumberOfParticles();

    Timer timer;
    UpdateParticleEmitter(timeIntervalInSeconds);
    CUBBYFLOW_INFO << "Update particle emitter took "
                   << timer.DurationInSeconds() << " seconds";

    CUBBYFLOW_INFO << "Number of PIC-type particles: "
                   << m_particles->NumberOfParticles();

    timer.Reset();
    TransferFromParticlesToGrids();
    CUBBYFLOW_INFO << "TransferFromParticlesToGrids took "
                   << timer.DurationInSeconds() << " seconds";

    timer.Reset();
    BuildSignedDistanceField();
    CUBBYFLOW_INFO << "BuildSignedDistanceField took "
                   << timer.DurationInSeconds() << " seconds";

    timer.Reset();
    ExtrapolateVelocityToAir();
    CUBBYFLOW_INFO << "ExtrapolateGetVelocityToAir took "
                   << timer.DurationInSeconds() << " seconds";

    ApplyBoundaryCondition();
}

void PICSolver3::ComputeAdvection(double timeIntervalInSeconds)
{
    Timer timer;
    ExtrapolateVelocityToAir();
    CUBBYFLOW_INFO << "ExtrapolateGetVelocityToAir took "
                   << timer.DurationInSeconds() << " seconds";

    ApplyBoundaryCondition();

    timer.Reset();
    TransferFromGridsToParticles();
    CUBBYFLOW_INFO << "TransferFromGridsToParticles took "
                   << timer.DurationInSeconds() << " seconds";

    timer.Reset();
    MoveParticles(timeIntervalInSeconds);
    CUBBYFLOW_INFO << "MoveParticles took " << timer.DurationInSeconds()
                   << " seconds";
}

ScalarField3Ptr PICSolver3::GetFluidSDF() const
{
    return GetSignedDistanceField();
}

void PICSolver3::TransferFromParticlesToGrids()
{
    FaceCenteredGrid3Ptr flow = GetGridSystemData()->Velocity();
    ArrayView1<Vector3<double>> positions = m_particles->Positions();
    ArrayView1<Vector3<double>> velocities = m_particles->Velocities();
    size_t numberOfParticles = m_particles->NumberOfParticles();

    // Clear velocity to zero
    flow->Fill(Vector3D{});

    // Weighted-average velocity
    ArrayView3<double> u = flow->UView();
    ArrayView3<double> v = flow->VView();
    ArrayView3<double> w = flow->WView();
    Array3<double> uWeight{ u.Size() };
    Array3<double> vWeight{ v.Size() };
    Array3<double> wWeight{ w.Size() };
    m_uMarkers.Resize(u.Size());
    m_vMarkers.Resize(v.Size());
    m_wMarkers.Resize(w.Size());
    m_uMarkers.Fill(0);
    m_vMarkers.Fill(0);
    m_wMarkers.Fill(0);

    LinearArraySampler3<double> uSampler{ flow->UView(), flow->GridSpacing(),
                                          flow->UOrigin() };
    LinearArraySampler3<double> vSampler{ flow->VView(), flow->GridSpacing(),
                                          flow->VOrigin() };
    LinearArraySampler3<double> wSampler{ flow->WView(), flow->GridSpacing(),
                                          flow->WOrigin() };

    for (size_t i = 0; i < numberOfParticles; ++i)
    {
        std::array<Vector3UZ, 8> indices{};
        std::array<double, 8> weights{};

        uSampler.GetCoordinatesAndWeights(positions[i], indices, weights);
        for (int j = 0; j < 8; ++j)
        {
            u(indices[j]) += velocities[i].x * weights[j];
            uWeight(indices[j]) += weights[j];
            m_uMarkers(indices[j]) = 1;
        }

        vSampler.GetCoordinatesAndWeights(positions[i], indices, weights);
        for (int j = 0; j < 8; ++j)
        {
            v(indices[j]) += velocities[i].y * weights[j];
            vWeight(indices[j]) += weights[j];
            m_vMarkers(indices[j]) = 1;
        }

        wSampler.GetCoordinatesAndWeights(positions[i], indices, weights);
        for (int j = 0; j < 8; ++j)
        {
            w(indices[j]) += velocities[i].z * weights[j];
            wWeight(indices[j]) += weights[j];
            m_wMarkers(indices[j]) = 1;
        }
    }

    ParallelForEachIndex(uWeight.Size(), [&](size_t i, size_t j, size_t k) {
        if (uWeight(i, j, k) > 0.0)
        {
            u(i, j, k) /= uWeight(i, j, k);
        }
    });
    ParallelForEachIndex(vWeight.Size(), [&](size_t i, size_t j, size_t k) {
        if (vWeight(i, j, k) > 0.0)
        {
            v(i, j, k) /= vWeight(i, j, k);
        }
    });
    ParallelForEachIndex(wWeight.Size(), [&](size_t i, size_t j, size_t k) {
        if (wWeight(i, j, k) > 0.0)
        {
            w(i, j, k) /= wWeight(i, j, k);
        }
    });
}

void PICSolver3::TransferFromGridsToParticles()
{
    FaceCenteredGrid3Ptr flow = GetGridSystemData()->Velocity();
    ArrayView1<Vector3<double>> positions = m_particles->Positions();
    ArrayView1<Vector3<double>> velocities = m_particles->Velocities();
    const size_t numberOfParticles = m_particles->NumberOfParticles();

    ParallelFor(ZERO_SIZE, numberOfParticles,
                [&](size_t i) { velocities[i] = flow->Sample(positions[i]); });
}

void PICSolver3::MoveParticles(double timeIntervalInSeconds)
{
    FaceCenteredGrid3Ptr flow = GetGridSystemData()->Velocity();
    ArrayView1<Vector3<double>> positions = m_particles->Positions();
    ArrayView1<Vector3<double>> velocities = m_particles->Velocities();
    const size_t numberOfParticles = m_particles->NumberOfParticles();
    int domainBoundaryFlag = GetClosedDomainBoundaryFlag();
    BoundingBox3D boundingBox = flow->GetBoundingBox();

    ParallelFor(ZERO_SIZE, numberOfParticles, [&](size_t i) {
        Vector3D pt0 = positions[i];
        Vector3D pt1 = pt0;
        Vector3D vel = velocities[i];

        // Adaptive time-stepping
        const unsigned int numSubSteps =
            static_cast<unsigned int>(std::max(GetMaxCFL(), 1.0));
        const double dt = timeIntervalInSeconds / numSubSteps;
        for (unsigned int t = 0; t < numSubSteps; ++t)
        {
            Vector3D vel0 = flow->Sample(pt0);

            // Mid-point rule
            Vector3D midPt = pt0 + 0.5 * dt * vel0;
            Vector3D midVel = flow->Sample(midPt);
            pt1 = pt0 + dt * midVel;

            pt0 = pt1;
        }

        if ((domainBoundaryFlag & DIRECTION_LEFT) &&
            pt1.x <= boundingBox.lowerCorner.x)
        {
            pt1.x = boundingBox.lowerCorner.x;
            vel.x = 0.0;
        }
        if ((domainBoundaryFlag & DIRECTION_RIGHT) &&
            pt1.x >= boundingBox.upperCorner.x)
        {
            pt1.x = boundingBox.upperCorner.x;
            vel.x = 0.0;
        }
        if ((domainBoundaryFlag & DIRECTION_DOWN) &&
            pt1.y <= boundingBox.lowerCorner.y)
        {
            pt1.y = boundingBox.lowerCorner.y;
            vel.y = 0.0;
        }
        if ((domainBoundaryFlag & DIRECTION_UP) &&
            pt1.y >= boundingBox.upperCorner.y)
        {
            pt1.y = boundingBox.upperCorner.y;
            vel.y = 0.0;
        }
        if ((domainBoundaryFlag & DIRECTION_BACK) &&
            pt1.z <= boundingBox.lowerCorner.z)
        {
            pt1.z = boundingBox.lowerCorner.z;
            vel.z = 0.0;
        }
        if ((domainBoundaryFlag & DIRECTION_FRONT) &&
            pt1.z >= boundingBox.upperCorner.z)
        {
            pt1.z = boundingBox.upperCorner.z;
            vel.z = 0.0;
        }

        positions[i] = pt1;
        velocities[i] = vel;
    });

    Collider3Ptr col = GetCollider();
    if (col != nullptr)
    {
        ParallelFor(ZERO_SIZE, numberOfParticles, [&](size_t i) {
            col->ResolveCollision(0.0, 0.0, &positions[i], &velocities[i]);
        });
    }
}

void PICSolver3::ExtrapolateVelocityToAir()
{
    FaceCenteredGrid3Ptr vel = GetGridSystemData()->Velocity();
    const ArrayView3<double> u = vel->UView();
    const ArrayView3<double> v = vel->VView();
    const ArrayView3<double> w = vel->WView();

    const auto depth = static_cast<unsigned int>(std::ceil(GetMaxCFL()));
    ExtrapolateToRegion(vel->UView(), m_uMarkers, depth, u);
    ExtrapolateToRegion(vel->VView(), m_vMarkers, depth, v);
    ExtrapolateToRegion(vel->WView(), m_wMarkers, depth, w);
}

void PICSolver3::BuildSignedDistanceField()
{
    ScalarGrid3Ptr sdf = GetSignedDistanceField();
    GridDataPositionFunc<3> sdfPos = sdf->DataPosition();
    const double maxH = std::max(
        { sdf->GridSpacing().x, sdf->GridSpacing().y, sdf->GridSpacing().z });
    double radius = 1.2 * maxH / std::sqrt(2.0);
    double sdfBandRadius = 2.0 * radius;

    m_particles->BuildNeighborSearcher(2 * radius);
    PointNeighborSearcher3Ptr searcher = m_particles->NeighborSearcher();
    sdf->ParallelForEachDataPointIndex([&](size_t i, size_t j, size_t k) {
        Vector3D pt = sdfPos(i, j, k);
        double minDist = sdfBandRadius;

        searcher->ForEachNearbyPoint(
            pt, sdfBandRadius, [&](size_t, const Vector3D& x) {
                minDist = std::min(minDist, pt.DistanceTo(x));
            });
        (*sdf)(i, j, k) = minDist - radius;
    });

    ExtrapolateIntoCollider(sdf.get());
}

void PICSolver3::UpdateParticleEmitter(double timeIntervalInSeconds) const
{
    if (m_particleEmitter != nullptr)
    {
        m_particleEmitter->Update(GetCurrentTimeInSeconds(),
                                  timeIntervalInSeconds);
    }
}

PICSolver3::Builder PICSolver3::GetBuilder()
{
    return Builder{};
}

PICSolver3 PICSolver3::Builder::Build() const
{
    return PICSolver3{ m_resolution, GetGridSpacing(), m_gridOrigin };
}

PICSolver3Ptr PICSolver3::Builder::MakeShared() const
{
    return std::make_shared<PICSolver3>(m_resolution, GetGridSpacing(),
                                        m_gridOrigin);
}
}  // namespace CubbyFlow