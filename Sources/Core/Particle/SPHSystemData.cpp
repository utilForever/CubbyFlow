// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Geometry/BoundingBox.hpp>
#include <Core/Particle/SPHKernels.hpp>
#include <Core/Particle/SPHSystemData.hpp>
#include <Core/PointGenerator/BccLatticePointGenerator.hpp>
#include <Core/PointGenerator/TrianglePointGenerator.hpp>

#include <Flatbuffers/generated/SPHSystemData2_generated.h>
#include <Flatbuffers/generated/SPHSystemData3_generated.h>

namespace CubbyFlow
{
template <size_t N>
struct GetFlatbuffersSPHSystemData
{
    // Do nothing
};

template <>
struct GetFlatbuffersSPHSystemData<2>
{
    using Offset = flatbuffers::Offset<fbs::SPHSystemData2>;
    using BaseOffset = flatbuffers::Offset<fbs::ParticleSystemData2>;

    static const fbs::SPHSystemData2* GetSPHSystemData(const uint8_t* data)
    {
        return fbs::GetSPHSystemData2(data);
    }

    static Offset CreateSPHSystemData(flatbuffers::FlatBufferBuilder& fbb,
                                      BaseOffset base, double targetDensity,
                                      double targetSpacing,
                                      double kernelRadiusOverTargetSpacing,
                                      double kernelRadius, uint64_t pressureIdx,
                                      uint64_t densityIdx)
    {
        return CreateSPHSystemData2(fbb, base, targetDensity, targetSpacing,
                                    kernelRadiusOverTargetSpacing, kernelRadius,
                                    pressureIdx, densityIdx);
    }
};

template <>
struct GetFlatbuffersSPHSystemData<3>
{
    using Offset = flatbuffers::Offset<fbs::SPHSystemData3>;
    using BaseOffset = flatbuffers::Offset<fbs::ParticleSystemData3>;

    static const fbs::SPHSystemData3* GetSPHSystemData(const uint8_t* data)
    {
        return fbs::GetSPHSystemData3(data);
    }

    static Offset CreateSPHSystemData(flatbuffers::FlatBufferBuilder& fbb,
                                      BaseOffset base, double targetDensity,
                                      double targetSpacing,
                                      double kernelRadiusOverTargetSpacing,
                                      double kernelRadius, uint64_t pressureIdx,
                                      uint64_t densityIdx)
    {
        return CreateSPHSystemData3(fbb, base, targetDensity, targetSpacing,
                                    kernelRadiusOverTargetSpacing, kernelRadius,
                                    pressureIdx, densityIdx);
    }
};

template <size_t N>
SPHSystemData<N>::SPHSystemData() : SPHSystemData{ 0 }
{
    // Do nothing
}

template <size_t N>
SPHSystemData<N>::SPHSystemData(size_t numberOfParticles)
    : ParticleSystemData<N>{ numberOfParticles }
{
    m_densityIdx = AddScalarData();
    m_pressureIdx = AddScalarData();

    SetTargetSpacing(m_targetSpacing);
}

template <size_t N>
SPHSystemData<N>::SPHSystemData(const SPHSystemData& other) = default;

template <size_t N>
SPHSystemData<N>::SPHSystemData(SPHSystemData&& other) noexcept
    : ParticleSystemData<N>{ std::move(other) },
      m_targetDensity(std::exchange(other.m_targetDensity, WATER_DENSITY)),
      m_targetSpacing(std::exchange(other.m_targetSpacing, 0.1)),
      m_kernelRadiusOverTargetSpacing(
          std::exchange(other.m_kernelRadiusOverTargetSpacing, 1.8)),
      m_kernelRadius(std::exchange(other.m_kernelRadius, 0.1)),
      m_pressureIdx(std::exchange(other.m_pressureIdx, 0)),
      m_densityIdx(std::exchange(other.m_densityIdx, 0))
{
    // Do nothing
}

template <size_t N>
SPHSystemData<N>& SPHSystemData<N>::operator=(const SPHSystemData& other) =
    default;

template <size_t N>
SPHSystemData<N>& SPHSystemData<N>::operator=(SPHSystemData&& other) noexcept
{
    m_targetDensity = std::exchange(other.m_targetDensity, WATER_DENSITY);
    m_targetSpacing = std::exchange(other.m_targetSpacing, 0.1);
    m_kernelRadiusOverTargetSpacing =
        std::exchange(other.m_kernelRadiusOverTargetSpacing, 1.8);
    m_kernelRadius = std::exchange(other.m_kernelRadius, 0.1);
    m_pressureIdx = std::exchange(other.m_pressureIdx, 0);
    m_densityIdx = std::exchange(other.m_densityIdx, 0);
    ParticleSystemData<N>::operator=(std::move(other));
    return *this;
}

template <size_t N>
void SPHSystemData<N>::SetRadius(double newRadius)
{
    // Interpret it as setting target spacing.
    SetTargetSpacing(newRadius);
}

template <size_t N>
void SPHSystemData<N>::SetMass(double newMass)
{
    const double incRatio = newMass / Mass();
    m_targetDensity *= incRatio;

    ParticleSystemData<N>::SetMass(newMass);
}

template <size_t N>
ConstArrayView1<double> SPHSystemData<N>::Densities() const
{
    return ScalarDataAt(m_densityIdx);
}

template <size_t N>
ArrayView1<double> SPHSystemData<N>::Densities()
{
    return ScalarDataAt(m_densityIdx);
}

template <size_t N>
ConstArrayView1<double> SPHSystemData<N>::Pressures() const
{
    return ScalarDataAt(m_pressureIdx);
}

template <size_t N>
ArrayView1<double> SPHSystemData<N>::Pressures()
{
    return ScalarDataAt(m_pressureIdx);
}

template <size_t N>
void SPHSystemData<N>::UpdateDensities()
{
    auto p = Positions();
    ArrayView1<double> d = Densities();
    const double m = Mass();

    ParallelFor(ZERO_SIZE, NumberOfParticles(), [this, &p, &d, &m](size_t i) {
        const double sum = SumOfKernelNearby(p[i]);
        d[i] = m * sum;
    });
}

template <size_t N>
double SPHSystemData<N>::TargetDensity() const
{
    return m_targetDensity;
}

template <size_t N>
void SPHSystemData<N>::SetTargetDensity(double targetDensity)
{
    m_targetDensity = targetDensity;

    ComputeMass();
}

template <size_t N>
double SPHSystemData<N>::TargetSpacing() const
{
    return m_targetSpacing;
}

template <size_t N>
void SPHSystemData<N>::SetTargetSpacing(double spacing)
{
    ParticleSystemData<N>::SetRadius(spacing);

    m_targetSpacing = spacing;
    m_kernelRadius = m_kernelRadiusOverTargetSpacing * m_targetSpacing;

    ComputeMass();
}

template <size_t N>
double SPHSystemData<N>::RelativeKernelRadius() const
{
    return m_kernelRadiusOverTargetSpacing;
}

template <size_t N>
void SPHSystemData<N>::SetRelativeKernelRadius(double relativeRadius)
{
    m_kernelRadiusOverTargetSpacing = relativeRadius;
    m_kernelRadius = m_kernelRadiusOverTargetSpacing * m_targetSpacing;

    ComputeMass();
}

template <size_t N>
double SPHSystemData<N>::KernelRadius() const
{
    return m_kernelRadius;
}

template <size_t N>
void SPHSystemData<N>::SetKernelRadius(double kernelRadius)
{
    m_kernelRadius = kernelRadius;
    m_targetSpacing = kernelRadius / m_kernelRadiusOverTargetSpacing;

    ComputeMass();
}

template <size_t N>
double SPHSystemData<N>::SumOfKernelNearby(
    const Vector<double, N>& position) const
{
    double sum = 0.0;
    SPHStdKernel<N> kernel{ m_kernelRadius };

    NeighborSearcher()->ForEachNearbyPoint(
        position, m_kernelRadius,
        [&position, &sum, &kernel](size_t,
                                   const Vector<double, N>& neighborPosition) {
            double dist = position.DistanceTo(neighborPosition);
            sum += kernel(dist);
        });

    return sum;
}

template <size_t N>
double SPHSystemData<N>::Interpolate(
    const Vector<double, N>& origin,
    const ConstArrayView1<double>& values) const
{
    double sum = 0.0;
    ConstArrayView1<double> d = Densities();
    SPHStdKernel<N> kernel{ m_kernelRadius };
    const double m = Mass();

    NeighborSearcher()->ForEachNearbyPoint(
        origin, m_kernelRadius,
        [&origin, &m, &d, &kernel, &sum, &values](
            size_t i, const Vector<double, N>& neighborPosition) {
            double dist = origin.DistanceTo(neighborPosition);
            const double weight = m / d[i] * kernel(dist);
            sum += weight * values[i];
        });

    return sum;
}

template <size_t N>
Vector<double, N> SPHSystemData<N>::Interpolate(
    const Vector<double, N>& origin,
    const ConstArrayView1<Vector<double, N>>& values) const
{
    Vector<double, N> sum;
    ConstArrayView1<double> d = Densities();
    SPHStdKernel<N> kernel{ m_kernelRadius };
    const double m = Mass();

    NeighborSearcher()->ForEachNearbyPoint(
        origin, m_kernelRadius,
        [&origin, &m, &d, &kernel, &sum, &values](
            size_t i, const Vector<double, N>& neighborPosition) {
            double dist = origin.DistanceTo(neighborPosition);
            double weight = m / d[i] * kernel(dist);
            sum += weight * values[i];
        });

    return sum;
}

template <size_t N>
Vector<double, N> SPHSystemData<N>::GradientAt(
    size_t i, const ConstArrayView1<double>& values) const
{
    Vector<double, N> sum;
    auto p = Positions();
    ConstArrayView1<double> d = Densities();
    const Array1<size_t>& neighbors = NeighborLists()[i];
    Vector<double, N> origin = p[i];
    SPHSpikyKernel<N> kernel{ m_kernelRadius };
    const double m = Mass();

    for (size_t j : neighbors)
    {
        Vector<double, N> neighborPosition = p[j];

        if (const double dist = origin.DistanceTo(neighborPosition); dist > 0.0)
        {
            Vector<double, N> dir = (neighborPosition - origin) / dist;
            sum += d[i] * m *
                   (values[i] / Square(d[i]) + values[j] / Square(d[j])) *
                   kernel.Gradient(dist, dir);
        }
    }

    return sum;
}

template <size_t N>
double SPHSystemData<N>::LaplacianAt(
    size_t i, const ConstArrayView1<double>& values) const
{
    double sum = 0.0;
    auto p = Positions();
    ConstArrayView1<double> d = Densities();
    const Array1<size_t>& neighbors = NeighborLists()[i];
    Vector<double, N> origin = p[i];
    SPHSpikyKernel<N> kernel{ m_kernelRadius };
    const double m = Mass();

    for (size_t j : neighbors)
    {
        Vector<double, N> neighborPosition = p[j];
        double dist = origin.DistanceTo(neighborPosition);
        sum +=
            m * (values[j] - values[i]) / d[j] * kernel.SecondDerivative(dist);
    }

    return sum;
}

template <size_t N>
Vector<double, N> SPHSystemData<N>::LaplacianAt(
    size_t i, const ConstArrayView1<Vector<double, N>>& values) const
{
    Vector<double, N> sum;
    auto p = Positions();
    ConstArrayView1<double> d = Densities();
    const Array1<size_t>& neighbors = NeighborLists()[i];
    Vector<double, N> origin = p[i];
    SPHSpikyKernel<N> kernel{ m_kernelRadius };
    const double m = Mass();

    for (size_t j : neighbors)
    {
        Vector<double, N> neighborPosition = p[j];
        double dist = origin.DistanceTo(neighborPosition);
        sum +=
            m * (values[j] - values[i]) / d[j] * kernel.SecondDerivative(dist);
    }

    return sum;
}

template <size_t N>
void SPHSystemData<N>::BuildNeighborSearcher()
{
    ParticleSystemData<N>::BuildNeighborSearcher(m_kernelRadius);
}

template <size_t N>
void SPHSystemData<N>::BuildNeighborLists()
{
    ParticleSystemData<N>::BuildNeighborLists(m_kernelRadius);
}

template <size_t N>
struct GetPointGenerator
{
    // Do nothing
};

template <>
struct GetPointGenerator<2>
{
    using Type = TrianglePointGenerator;
};

template <>
struct GetPointGenerator<3>
{
    using Type = BccLatticePointGenerator;
};

template <size_t N>
void SPHSystemData<N>::ComputeMass()
{
    Array1<Vector<double, N>> points;
    typename GetPointGenerator<N>::Type pointsGenerator;
    BoundingBox<double, N> sampleBound{
        Vector<double, N>::MakeConstant(-1.5 * m_kernelRadius),
        Vector<double, N>::MakeConstant(1.5 * m_kernelRadius)
    };

    pointsGenerator.Generate(sampleBound, m_targetSpacing, &points);

    double maxNumberDensity = 0.0;
    SPHStdKernel<N> kernel{ m_kernelRadius };

    for (size_t i = 0; i < points.Length(); ++i)
    {
        const Vector<double, N>& point = points[i];
        double sum = 0.0;

        for (size_t j = 0; j < points.Length(); ++j)
        {
            const Vector<double, N>& neighborPoint = points[j];
            sum += kernel(neighborPoint.DistanceTo(point));
        }

        maxNumberDensity = std::max(maxNumberDensity, sum);
    }

    assert(maxNumberDensity > 0);

    double newMass = m_targetDensity / maxNumberDensity;

    ParticleSystemData<N>::SetMass(newMass);
}

template <size_t N>
void SPHSystemData<N>::Serialize(std::vector<uint8_t>* buffer) const
{
    flatbuffers::FlatBufferBuilder builder(1024);
    typename GetFlatbuffersSPHSystemData<N>::BaseOffset fbsParticleSystemData;

    ParticleSystemData<N>::Serialize(*this, &builder, &fbsParticleSystemData);

    auto fbsSPHSystemData = GetFlatbuffersSPHSystemData<N>::CreateSPHSystemData(
        builder, fbsParticleSystemData, m_targetDensity, m_targetSpacing,
        m_kernelRadiusOverTargetSpacing, m_kernelRadius, m_pressureIdx,
        m_densityIdx);

    builder.Finish(fbsSPHSystemData);

    uint8_t* buf = builder.GetBufferPointer();
    const size_t size = builder.GetSize();

    buffer->resize(size);
    memcpy(buffer->data(), buf, size);
}

template <size_t N>
void SPHSystemData<N>::Deserialize(const std::vector<uint8_t>& buffer)
{
    auto fbsSPHSystemData =
        GetFlatbuffersSPHSystemData<N>::GetSPHSystemData(buffer.data());

    auto base = fbsSPHSystemData->base();
    ParticleSystemData<N>::Deserialize(base, *this);

    // SPH specific
    m_targetDensity = fbsSPHSystemData->targetDensity();
    m_targetSpacing = fbsSPHSystemData->targetSpacing();
    m_kernelRadiusOverTargetSpacing =
        fbsSPHSystemData->kernelRadiusOverTargetSpacing();
    m_kernelRadius = fbsSPHSystemData->kernelRadius();
    m_pressureIdx = static_cast<size_t>(fbsSPHSystemData->pressureIdx());
    m_densityIdx = static_cast<size_t>(fbsSPHSystemData->densityIdx());
}

template <size_t N>
void SPHSystemData<N>::Set(const SPHSystemData& other)
{
    ParticleSystemData<N>::Set(other);

    m_targetDensity = other.m_targetDensity;
    m_targetSpacing = other.m_targetSpacing;
    m_kernelRadiusOverTargetSpacing = other.m_kernelRadiusOverTargetSpacing;
    m_kernelRadius = other.m_kernelRadius;
    m_densityIdx = other.m_densityIdx;
    m_pressureIdx = other.m_pressureIdx;
}

template class SPHSystemData<2>;

template class SPHSystemData<3>;
}  // namespace CubbyFlow
