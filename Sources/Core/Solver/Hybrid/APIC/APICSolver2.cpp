// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Solver/Hybrid/APIC/APICSolver2.hpp>

namespace CubbyFlow
{
namespace
{
Vector2D ClampPosition(const Vector2D& position, const BoundingBox2D& bbox,
                       const Vector2D& halfSpacing, size_t axis)
{
    Vector2D result = position;
    const size_t otherAxis = 1 - axis;

    result[otherAxis] = std::clamp(
        result[otherAxis], bbox.lowerCorner[otherAxis] + halfSpacing[otherAxis],
        bbox.upperCorner[otherAxis] - halfSpacing[otherAxis]);

    return result;
}

Vector2D SampleGradient(const LinearArraySampler2<double>& sampler,
                        const Vector2D& position,
                        const ArrayView2<double>& data)
{
    std::array<Vector2UZ, 4> indices{};
    std::array<Vector2D, 4> gradWeights{};

    sampler.GetCoordinatesAndGradientWeights(position, indices, gradWeights);

    Vector2D result;

    for (size_t j = 0; j < indices.size(); ++j)
    {
        result += gradWeights[j] * data(indices[j]);
    }

    return result;
}
}  // namespace

APICSolver2::APICSolver2() : APICSolver2{ { 1, 1 }, { 1, 1 }, { 0, 0 } }
{
    // Do nothing
}

APICSolver2::APICSolver2(const Vector2UZ& resolution,
                         const Vector2D& gridSpacing,
                         const Vector2D& gridOrigin)
    : PICSolver2{ resolution, gridSpacing, gridOrigin }
{
    // Do nothing
}

void APICSolver2::TransferFromParticlesToGrids()
{
    FaceCenteredGrid2Ptr flow = GetGridSystemData()->Velocity();
    const ParticleSystemData2Ptr particles = GetParticleSystemData();
    const ArrayView1<Vector2<double>> positions = particles->Positions();
    ArrayView1<Vector2<double>> velocities = particles->Velocities();
    const size_t numberOfParticles = particles->NumberOfParticles();
    const Vector2<double> hh = flow->GridSpacing() / 2.0;
    const BoundingBox2D& bbox = flow->GetBoundingBox();

    // Allocate buffers
    m_cX.Resize(numberOfParticles);
    m_cY.Resize(numberOfParticles);

    // Clear velocity to zero
    flow->Fill(Vector2D{});

    // Weighted-average velocity
    ArrayView2<double> u = flow->UView();
    ArrayView2<double> v = flow->VView();
    const auto uPos = flow->UPosition();
    const auto vPos = flow->VPosition();
    Array2<double> uWeight{ u.Size() };
    Array2<double> vWeight{ v.Size() };
    m_uMarkers.Resize(u.Size());
    m_vMarkers.Resize(v.Size());
    m_uMarkers.Fill(0);
    m_vMarkers.Fill(0);

    LinearArraySampler2<double> uSampler{ flow->UView(), flow->GridSpacing(),
                                          flow->UOrigin() };
    LinearArraySampler2<double> vSampler{ flow->VView(), flow->GridSpacing(),
                                          flow->VOrigin() };

    for (size_t i = 0; i < numberOfParticles; ++i)
    {
        std::array<Vector2UZ, 4> indices{};
        std::array<double, 4> weights{};

        Vector2<double> uPosClamped = positions[i];
        uPosClamped.y = std::clamp(uPosClamped.y, bbox.lowerCorner.y + hh.y,
                                   bbox.upperCorner.y - hh.y);
        uSampler.GetCoordinatesAndWeights(uPosClamped, indices, weights);

        for (int j = 0; j < 4; ++j)
        {
            Vector2D gridPos = uPos(indices[j]);
            double apicTerm = m_cX[i].Dot(gridPos - uPosClamped);

            u(indices[j]) += weights[j] * (velocities[i].x + apicTerm);
            uWeight(indices[j]) += weights[j];
            m_uMarkers(indices[j]) = 1;
        }

        Vector2<double> vPosClamped = positions[i];
        vPosClamped.x = std::clamp(vPosClamped.x, bbox.lowerCorner.x + hh.x,
                                   bbox.upperCorner.x - hh.x);
        vSampler.GetCoordinatesAndWeights(vPosClamped, indices, weights);

        for (int j = 0; j < 4; ++j)
        {
            Vector2D gridPos = vPos(indices[j]);
            double apicTerm = m_cY[i].Dot(gridPos - vPosClamped);

            v(indices[j]) += weights[j] * (velocities[i].y + apicTerm);
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

void APICSolver2::TransferFromGridsToParticles()
{
    const FaceCenteredGrid2Ptr flow = GetGridSystemData()->Velocity();
    ParticleSystemData2Ptr particles = GetParticleSystemData();
    ArrayView1<Vector2<double>> positions = particles->Positions();
    ArrayView1<Vector2<double>> velocities = particles->Velocities();
    const size_t numberOfParticles = particles->NumberOfParticles();
    const Vector2<double> hh = flow->GridSpacing() / 2.0;
    const BoundingBox2D& bbox = flow->GetBoundingBox();

    // Allocate buffers
    m_cX.Resize(numberOfParticles);
    m_cY.Resize(numberOfParticles);
    m_cX.Fill(Vector2D{});
    m_cY.Fill(Vector2D{});

    ArrayView2<double> u = flow->UView();
    ArrayView2<double> v = flow->VView();

    LinearArraySampler2<double> uSampler{ u, flow->GridSpacing(),
                                          flow->UOrigin() };
    LinearArraySampler2<double> vSampler{ v, flow->GridSpacing(),
                                          flow->VOrigin() };

    ParallelFor(ZERO_SIZE, numberOfParticles,
                [&velocities, &flow, &positions, &bbox, &hh, &uSampler, this,
                 &u, &vSampler, &v](size_t i) {
                    velocities[i] = flow->Sample(positions[i]);
                    m_cX[i] = SampleGradient(
                        uSampler, ClampPosition(positions[i], bbox, hh, 0), u);
                    m_cY[i] = SampleGradient(
                        vSampler, ClampPosition(positions[i], bbox, hh, 1), v);
                });
}

APICSolver2::Builder APICSolver2::GetBuilder()
{
    return Builder{};
}

APICSolver2 APICSolver2::Builder::Build() const
{
    return APICSolver2{ m_resolution, GetGridSpacing(), m_gridOrigin };
}

APICSolver2Ptr APICSolver2::Builder::MakeShared() const
{
    return std::make_shared<APICSolver2>(m_resolution, GetGridSpacing(),
                                         m_gridOrigin);
}
}  // namespace CubbyFlow
