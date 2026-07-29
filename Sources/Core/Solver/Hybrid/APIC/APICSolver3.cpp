// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Solver/Hybrid/APIC/APICSolver3.hpp>

namespace CubbyFlow
{
namespace
{
Vector3D ClampPosition(const Vector3D& position, const BoundingBox3D& bbox,
                       const Vector3D& halfSpacing, size_t axis)
{
    Vector3D result = position;

    for (size_t i = 0; i < 3; ++i)
    {
        if (i != axis)
        {
            result[i] =
                std::clamp(result[i], bbox.lowerCorner[i] + halfSpacing[i],
                           bbox.upperCorner[i] - halfSpacing[i]);
        }
    }

    return result;
}

Vector3D SampleGradient(const LinearArraySampler3<double>& sampler,
                        const Vector3D& position,
                        const ArrayView3<double>& data)
{
    std::array<Vector3UZ, 8> indices{};
    std::array<Vector3D, 8> gradWeights{};

    sampler.GetCoordinatesAndGradientWeights(position, indices, gradWeights);

    Vector3D result;

    for (size_t j = 0; j < indices.size(); ++j)
    {
        result += gradWeights[j] * data(indices[j]);
    }

    return result;
}
}  // namespace

APICSolver3::APICSolver3()
    : APICSolver3{ { 1, 1, 1 }, { 1, 1, 1 }, { 0, 0, 0 } }
{
    // Do nothing
}

APICSolver3::APICSolver3(const Vector3UZ& resolution,
                         const Vector3D& GridSpacing,
                         const Vector3D& gridOrigin)
    : PICSolver3{ resolution, GridSpacing, gridOrigin }
{
    // Do nothing
}

void APICSolver3::TransferFromParticlesToGrids()
{
    FaceCenteredGrid3Ptr flow = GetGridSystemData()->Velocity();
    const ParticleSystemData3Ptr particles = GetParticleSystemData();
    const ArrayView1<Vector3<double>> positions = particles->Positions();
    ArrayView1<Vector3<double>> velocities = particles->Velocities();
    const size_t numberOfParticles = particles->NumberOfParticles();
    const Vector3<double> hh = flow->GridSpacing() / 2.0;
    const BoundingBox3D& bbox = flow->GetBoundingBox();

    // Allocate buffers
    m_cX.Resize(numberOfParticles);
    m_cY.Resize(numberOfParticles);
    m_cZ.Resize(numberOfParticles);

    // Clear velocity to zero
    flow->Fill(Vector3D{});

    // Weighted-average velocity
    ArrayView3<double> u = flow->UView();
    ArrayView3<double> v = flow->VView();
    ArrayView3<double> w = flow->WView();
    const auto uPos = flow->UPosition();
    const auto vPos = flow->VPosition();
    const auto wPos = flow->WPosition();
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

        Vector3<double> uPosClamped = positions[i];
        uPosClamped.y = std::clamp(uPosClamped.y, bbox.lowerCorner.y + hh.y,
                                   bbox.upperCorner.y - hh.y);
        uPosClamped.z = std::clamp(uPosClamped.z, bbox.lowerCorner.z + hh.z,
                                   bbox.upperCorner.z - hh.z);
        uSampler.GetCoordinatesAndWeights(uPosClamped, indices, weights);

        for (int j = 0; j < 8; ++j)
        {
            Vector3D gridPos = uPos(indices[j]);
            double apicTerm = m_cX[i].Dot(gridPos - uPosClamped);

            u(indices[j]) += weights[j] * (velocities[i].x + apicTerm);
            uWeight(indices[j]) += weights[j];
            m_uMarkers(indices[j]) = 1;
        }

        Vector3<double> vPosClamped = positions[i];
        vPosClamped.x = std::clamp(vPosClamped.x, bbox.lowerCorner.x + hh.x,
                                   bbox.upperCorner.x - hh.x);
        vPosClamped.z = std::clamp(vPosClamped.z, bbox.lowerCorner.z + hh.z,
                                   bbox.upperCorner.z - hh.z);
        vSampler.GetCoordinatesAndWeights(vPosClamped, indices, weights);

        for (int j = 0; j < 8; ++j)
        {
            Vector3D gridPos = vPos(indices[j]);
            double apicTerm = m_cY[i].Dot(gridPos - vPosClamped);

            v(indices[j]) += weights[j] * (velocities[i].y + apicTerm);
            vWeight(indices[j]) += weights[j];
            m_vMarkers(indices[j]) = 1;
        }

        Vector3<double> wPosClamped = positions[i];
        wPosClamped.x = std::clamp(wPosClamped.x, bbox.lowerCorner.x + hh.x,
                                   bbox.upperCorner.x - hh.x);
        wPosClamped.y = std::clamp(wPosClamped.y, bbox.lowerCorner.y + hh.y,
                                   bbox.upperCorner.y - hh.y);
        wSampler.GetCoordinatesAndWeights(wPosClamped, indices, weights);

        for (int j = 0; j < 8; ++j)
        {
            Vector3D gridPos = wPos(indices[j]);
            double apicTerm = m_cZ[i].Dot(gridPos - wPosClamped);

            w(indices[j]) += weights[j] * (velocities[i].z + apicTerm);
            wWeight(indices[j]) += weights[j];
            m_wMarkers(indices[j]) = 1;
        }
    }

    ParallelForEachIndex(uWeight.Size(),
                         [&uWeight, &u](size_t i, size_t j, size_t k) {
                             if (uWeight(i, j, k) > 0.0)
                             {
                                 u(i, j, k) /= uWeight(i, j, k);
                             }
                         });
    ParallelForEachIndex(vWeight.Size(),
                         [&vWeight, &v](size_t i, size_t j, size_t k) {
                             if (vWeight(i, j, k) > 0.0)
                             {
                                 v(i, j, k) /= vWeight(i, j, k);
                             }
                         });
    ParallelForEachIndex(wWeight.Size(),
                         [&wWeight, &w](size_t i, size_t j, size_t k) {
                             if (wWeight(i, j, k) > 0.0)
                             {
                                 w(i, j, k) /= wWeight(i, j, k);
                             }
                         });
}

void APICSolver3::TransferFromGridsToParticles()
{
    FaceCenteredGrid3Ptr flow = GetGridSystemData()->Velocity();
    const ParticleSystemData3Ptr particles = GetParticleSystemData();
    const ArrayView1<Vector3<double>> positions = particles->Positions();
    ArrayView1<Vector3<double>> velocities = particles->Velocities();
    const size_t numberOfParticles = particles->NumberOfParticles();
    const Vector3<double> hh = flow->GridSpacing() / 2.0;
    const BoundingBox3D& bbox = flow->GetBoundingBox();

    // Allocate buffers
    m_cX.Resize(numberOfParticles);
    m_cY.Resize(numberOfParticles);
    m_cZ.Resize(numberOfParticles);
    m_cX.Fill(Vector3D{});
    m_cY.Fill(Vector3D{});
    m_cZ.Fill(Vector3D{});

    ArrayView3<double> u = flow->UView();
    ArrayView3<double> v = flow->VView();
    ArrayView3<double> w = flow->WView();

    LinearArraySampler3<double> uSampler{ u, flow->GridSpacing(),
                                          flow->UOrigin() };
    LinearArraySampler3<double> vSampler{ v, flow->GridSpacing(),
                                          flow->VOrigin() };
    LinearArraySampler3<double> wSampler{ w, flow->GridSpacing(),
                                          flow->WOrigin() };

    ParallelFor(ZERO_SIZE, numberOfParticles,
                [&velocities, &flow, &positions, &bbox, &hh, &uSampler, this,
                 &u, &vSampler, &v, &wSampler, &w](size_t i) {
                    velocities[i] = flow->Sample(positions[i]);
                    m_cX[i] = SampleGradient(
                        uSampler, ClampPosition(positions[i], bbox, hh, 0), u);
                    m_cY[i] = SampleGradient(
                        vSampler, ClampPosition(positions[i], bbox, hh, 1), v);
                    m_cZ[i] = SampleGradient(
                        wSampler, ClampPosition(positions[i], bbox, hh, 2), w);
                });
}

APICSolver3::Builder APICSolver3::GetBuilder()
{
    return Builder{};
}

APICSolver3 APICSolver3::Builder::Build() const
{
    return APICSolver3{ m_resolution, GetGridSpacing(), m_gridOrigin };
}

APICSolver3Ptr APICSolver3::Builder::MakeShared() const
{
    return std::make_shared<APICSolver3>(m_resolution, GetGridSpacing(),
                                         m_gridOrigin);
}
}  // namespace CubbyFlow
