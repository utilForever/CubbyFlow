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
#include <Core/Solver/Hybrid/PIC/PICSolver2.hpp>
#include <Core/Utils/Logging.hpp>
#include <Core/Utils/Timer.hpp>

namespace CubbyFlow
{
namespace
{
void MoveParticle(const FaceCenteredGrid2& flow, double timeIntervalInSeconds,
                  unsigned int numSubSteps, int domainBoundaryFlag,
                  const BoundingBox2D& boundingBox, Vector2D* position,
                  Vector2D* velocity)
{
    Vector2D pt = *position;
    const double dt = timeIntervalInSeconds / numSubSteps;
    for (unsigned int t = 0; t < numSubSteps; ++t)
    {
        const Vector2D midPt = pt + 0.5 * dt * flow.Sample(pt);
        pt += dt * flow.Sample(midPt);
    }

    if ((domainBoundaryFlag & DIRECTION_LEFT) &&
        pt.x <= boundingBox.lowerCorner.x)
    {
        pt.x = boundingBox.lowerCorner.x;
        velocity->x = 0.0;
    }
    if ((domainBoundaryFlag & DIRECTION_RIGHT) &&
        pt.x >= boundingBox.upperCorner.x)
    {
        pt.x = boundingBox.upperCorner.x;
        velocity->x = 0.0;
    }
    if ((domainBoundaryFlag & DIRECTION_DOWN) &&
        pt.y <= boundingBox.lowerCorner.y)
    {
        pt.y = boundingBox.lowerCorner.y;
        velocity->y = 0.0;
    }
    if ((domainBoundaryFlag & DIRECTION_UP) &&
        pt.y >= boundingBox.upperCorner.y)
    {
        pt.y = boundingBox.upperCorner.y;
        velocity->y = 0.0;
    }
    *position = pt;
}
}  // namespace

PICSolver2::PICSolver2() : PICSolver2{ { 1, 1 }, { 1, 1 }, { 0, 0 } }
{
    // Do nothing
}

PICSolver2::PICSolver2(const Vector2UZ& resolution, const Vector2D& gridSpacing,
                       const Vector2D& gridOrigin)
    : GridFluidSolver2{ resolution, gridSpacing, gridOrigin }
{
    GridSystemData2Ptr grids = GetGridSystemData();
    m_signedDistanceFieldID = grids->AddScalarData(
        std::make_shared<CellCenteredScalarGrid2::Builder>(),
        std::numeric_limits<double>::max());
    m_particles = std::make_shared<ParticleSystemData2>();
}

ScalarGrid2Ptr PICSolver2::GetSignedDistanceField() const
{
    return GetGridSystemData()->ScalarDataAt(m_signedDistanceFieldID);
}

const ParticleSystemData2Ptr& PICSolver2::GetParticleSystemData() const
{
    return m_particles;
}

const ParticleEmitter2Ptr& PICSolver2::GetParticleEmitter() const
{
    return m_particleEmitter;
}

void PICSolver2::SetParticleEmitter(const ParticleEmitter2Ptr& newEmitter)
{
    m_particleEmitter = newEmitter;
    newEmitter->SetTarget(m_particles);
}

void PICSolver2::OnInitialize()
{
    GridFluidSolver2::OnInitialize();

    const Timer timer;
    UpdateParticleEmitter(0.0);
    CUBBYFLOW_INFO << "Update particle emitter took "
                   << timer.DurationInSeconds() << " seconds";
}

void PICSolver2::OnBeginAdvanceTimeStep(double timeIntervalInSeconds)
{
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
    CUBBYFLOW_INFO << "ExtrapolateVelocityToAir took "
                   << timer.DurationInSeconds() << " seconds";

    ApplyBoundaryCondition();
}

void PICSolver2::ComputeAdvection(double timeIntervalInSeconds)
{
    Timer timer;
    ExtrapolateVelocityToAir();
    CUBBYFLOW_INFO << "ExtrapolateVelocityToAir took "
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

ScalarField2Ptr PICSolver2::GetFluidSDF() const
{
    return GetSignedDistanceField();
}

void PICSolver2::TransferFromParticlesToGrids()
{
    FaceCenteredGrid2Ptr flow = GetGridSystemData()->Velocity();
    ArrayView1<Vector2<double>> positions = m_particles->Positions();
    ArrayView1<Vector2<double>> velocities = m_particles->Velocities();
    const size_t numberOfParticles = m_particles->NumberOfParticles();

    // Clear velocity to zero
    flow->Fill(Vector2D{});

    // Weighted-average velocity
    ArrayView2<double> u = flow->UView();
    ArrayView2<double> v = flow->VView();
    Array2<double> uWeight{ u.Size() };
    Array2<double> vWeight{ v.Size() };
    m_uMarkers.Resize(u.Size());
    m_vMarkers.Resize(v.Size());
    m_uMarkers.Fill(0);
    m_vMarkers.Fill(0);

    const LinearArraySampler2<double> uSampler{ flow->UView(),
                                                flow->GridSpacing(),
                                                flow->UOrigin() };
    const LinearArraySampler2<double> vSampler{ flow->VView(),
                                                flow->GridSpacing(),
                                                flow->VOrigin() };

    for (size_t i = 0; i < numberOfParticles; ++i)
    {
        std::array<Vector2UZ, 4> indices{};
        std::array<double, 4> weights{};

        uSampler.GetCoordinatesAndWeights(positions[i], indices, weights);
        for (int j = 0; j < 4; ++j)
        {
            u(indices[j]) += velocities[i].x * weights[j];
            uWeight(indices[j]) += weights[j];
            m_uMarkers(indices[j]) = 1;
        }

        vSampler.GetCoordinatesAndWeights(positions[i], indices, weights);
        for (int j = 0; j < 4; ++j)
        {
            v(indices[j]) += velocities[i].y * weights[j];
            vWeight(indices[j]) += weights[j];
            m_vMarkers(indices[j]) = 1;
        }
    }

    ParallelForEachIndex(uWeight.Size(), [&uWeight, &u](size_t i, size_t j) {
        if (uWeight(i, j) > 0.0)
        {
            u(i, j) /= uWeight(i, j);
        }
    });
    ParallelForEachIndex(vWeight.Size(), [&vWeight, &v](size_t i, size_t j) {
        if (vWeight(i, j) > 0.0)
        {
            v(i, j) /= vWeight(i, j);
        }
    });
}

void PICSolver2::TransferFromGridsToParticles()
{
    FaceCenteredGrid2Ptr flow = GetGridSystemData()->Velocity();
    ArrayView1<Vector2<double>> positions = m_particles->Positions();
    ArrayView1<Vector2<double>> velocities = m_particles->Velocities();
    const size_t numberOfParticles = m_particles->NumberOfParticles();

    ParallelFor(ZERO_SIZE, numberOfParticles,
                [&velocities, &flow, &positions](size_t i) {
                    velocities[i] = flow->Sample(positions[i]);
                });
}

void PICSolver2::MoveParticles(double timeIntervalInSeconds)
{
    FaceCenteredGrid2Ptr flow = GetGridSystemData()->Velocity();
    ArrayView1<Vector2<double>> positions = m_particles->Positions();
    ArrayView1<Vector2<double>> velocities = m_particles->Velocities();
    const size_t numberOfParticles = m_particles->NumberOfParticles();
    int domainBoundaryFlag = GetClosedDomainBoundaryFlag();
    BoundingBox2D boundingBox = flow->GetBoundingBox();
    const auto numSubSteps =
        static_cast<unsigned int>(std::max(GetMaxCFL(), 1.0));

    ParallelFor(ZERO_SIZE, numberOfParticles,
                [&positions, &velocities, &timeIntervalInSeconds, &flow,
                 &numSubSteps, &domainBoundaryFlag, &boundingBox](size_t i) {
                    MoveParticle(*flow, timeIntervalInSeconds, numSubSteps,
                                 domainBoundaryFlag, boundingBox, &positions[i],
                                 &velocities[i]);
                });

    Collider2Ptr col = GetCollider();
    if (col != nullptr)
    {
        ParallelFor(ZERO_SIZE, numberOfParticles,
                    [&col, &positions, &velocities](size_t i) {
                        col->ResolveCollision(0.0, 0.0, &positions[i],
                                              &velocities[i]);
                    });
    }
}

void PICSolver2::ExtrapolateVelocityToAir()
{
    FaceCenteredGrid2Ptr vel = GetGridSystemData()->Velocity();
    const ArrayView2<double> u = vel->UView();
    const ArrayView2<double> v = vel->VView();

    const auto depth = static_cast<unsigned int>(std::ceil(GetMaxCFL()));
    ExtrapolateToRegion(vel->UView(), m_uMarkers, depth, u);
    ExtrapolateToRegion(vel->VView(), m_vMarkers, depth, v);
}

void PICSolver2::BuildSignedDistanceField()
{
    ScalarGrid2Ptr sdf = GetSignedDistanceField();
    GridDataPositionFunc<2> sdfPos = sdf->DataPosition();
    const double maxH = std::max(sdf->GridSpacing().x, sdf->GridSpacing().y);
    double radius = 1.2 * maxH / std::sqrt(2.0);

    m_particles->BuildNeighborSearcher(2 * radius);
    auto searcher = m_particles->NeighborSearcher();
    sdf->ParallelForEachDataPointIndex(
        [&sdfPos, &radius, &searcher, &sdf](size_t i, size_t j) {
            Vector2D pt = sdfPos(i, j);
            double minDist = 2.0 * radius;

            searcher->ForEachNearbyPoint(
                pt, 2.0 * radius, [&minDist, &pt](size_t, const Vector2D& x) {
                    minDist = std::min(minDist, pt.DistanceTo(x));
                });
            (*sdf)(i, j) = minDist - radius;
        });

    ExtrapolateIntoCollider(sdf.get());
}

void PICSolver2::UpdateParticleEmitter(double timeIntervalInSeconds) const
{
    if (m_particleEmitter != nullptr)
    {
        m_particleEmitter->Update(GetCurrentTimeInSeconds(),
                                  timeIntervalInSeconds);
    }
}

PICSolver2::Builder PICSolver2::GetBuilder()
{
    return Builder{};
}

PICSolver2 PICSolver2::Builder::Build() const
{
    return PICSolver2{ m_resolution, GetGridSpacing(), m_gridOrigin };
}

PICSolver2Ptr PICSolver2::Builder::MakeShared() const
{
    return std::make_shared<PICSolver2>(m_resolution, GetGridSpacing(),
                                        m_gridOrigin);
}
}  // namespace CubbyFlow
