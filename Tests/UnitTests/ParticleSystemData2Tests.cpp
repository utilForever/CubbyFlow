#include "gtest/gtest.h"

#include <Core/Particle/ParticleSystemData.hpp>

using namespace CubbyFlow;

TEST(ParticleSystemData2, Constructors)
{
    ParticleSystemData2 particleSystem;
    EXPECT_EQ(0u, particleSystem.NumberOfParticles());

    ParticleSystemData2 particleSystem2(100);
    EXPECT_EQ(100u, particleSystem2.NumberOfParticles());

    size_t a0 = particleSystem2.AddScalarData(2.0);
    size_t a1 = particleSystem2.AddScalarData(9.0);
    size_t a2 = particleSystem2.AddVectorData({ 1.0, -3.0 });

    ParticleSystemData2 particleSystem3(particleSystem2);
    EXPECT_EQ(100u, particleSystem3.NumberOfParticles());
    auto as0 = particleSystem3.ScalarDataAt(a0);
    for (size_t i = 0; i < 100; ++i)
    {
        EXPECT_DOUBLE_EQ(2.0, as0[i]);
    }

    auto as1 = particleSystem3.ScalarDataAt(a1);
    for (size_t i = 0; i < 100; ++i)
    {
        EXPECT_DOUBLE_EQ(9.0, as1[i]);
    }

    auto as2 = particleSystem3.VectorDataAt(a2);
    for (size_t i = 0; i < 100; ++i)
    {
        EXPECT_DOUBLE_EQ(1.0, as2[i].x);
        EXPECT_DOUBLE_EQ(-3.0, as2[i].y);
    }
}

TEST(ParticleSystemData2, CopyAssignmentAndSet)
{
    ParticleSystemData2 source(2);
    EXPECT_EQ(0u, source.AddScalarData());
    EXPECT_EQ(3u, source.AddVectorData());

    ParticleSystemData2 assigned(1);
    EXPECT_EQ(0u, assigned.AddScalarData());
    EXPECT_EQ(3u, assigned.AddVectorData());
    assigned = source;

    EXPECT_EQ(1u, assigned.AddScalarData());
    EXPECT_EQ(4u, assigned.AddVectorData());

    ParticleSystemData2 set(1);
    EXPECT_EQ(0u, set.AddScalarData());
    EXPECT_EQ(3u, set.AddVectorData());
    set.Set(source);

    EXPECT_EQ(1u, set.AddScalarData());
    EXPECT_EQ(4u, set.AddVectorData());
}

TEST(ParticleSystemData2, Resize)
{
    ParticleSystemData2 particleSystem;
    particleSystem.Resize(12);

    EXPECT_EQ(12u, particleSystem.NumberOfParticles());
}

TEST(ParticleSystemData2, AddScalarData)
{
    ParticleSystemData2 particleSystem;
    particleSystem.Resize(12);

    size_t a0 = particleSystem.AddScalarData(2.0);
    size_t a1 = particleSystem.AddScalarData(9.0);

    EXPECT_EQ(12u, particleSystem.NumberOfParticles());
    EXPECT_EQ(0u, a0);
    EXPECT_EQ(1u, a1);

    auto as0 = particleSystem.ScalarDataAt(a0);
    for (size_t i = 0; i < 12; ++i)
    {
        EXPECT_DOUBLE_EQ(2.0, as0[i]);
    }

    auto as1 = particleSystem.ScalarDataAt(a1);
    for (size_t i = 0; i < 12; ++i)
    {
        EXPECT_DOUBLE_EQ(9.0, as1[i]);
    }
}

TEST(ParticleSystemData2, AddVectorData)
{
    ParticleSystemData2 particleSystem;
    particleSystem.Resize(12);

    size_t a0 = particleSystem.AddVectorData(Vector2D(2.0, 4.0));
    size_t a1 = particleSystem.AddVectorData(Vector2D(9.0, -2.0));

    EXPECT_EQ(12u, particleSystem.NumberOfParticles());
    EXPECT_EQ(3u, a0);
    EXPECT_EQ(4u, a1);

    auto as0 = particleSystem.VectorDataAt(a0);
    for (size_t i = 0; i < 12; ++i)
    {
        EXPECT_EQ(Vector2D(2.0, 4.0), as0[i]);
    }

    auto as1 = particleSystem.VectorDataAt(a1);
    for (size_t i = 0; i < 12; ++i)
    {
        EXPECT_EQ(Vector2D(9.0, -2.0), as1[i]);
    }
}

TEST(ParticleSystemData2, AddParticles)
{
    ParticleSystemData2 particleSystem;
    particleSystem.Resize(12);

    particleSystem.AddParticles(
        Array1<Vector2D>({ Vector2D(1.0, 2.0), Vector2D(4.0, 5.0) }),
        Array1<Vector2D>({ Vector2D(7.0, 8.0), Vector2D(8.0, 7.0) }),
        Array1<Vector2D>({ Vector2D(5.0, 4.0), Vector2D(2.0, 1.0) }));

    EXPECT_EQ(14u, particleSystem.NumberOfParticles());
    auto p = particleSystem.Positions();
    auto v = particleSystem.Velocities();
    auto f = particleSystem.Forces();

    EXPECT_EQ(Vector2D(1.0, 2.0), p[12]);
    EXPECT_EQ(Vector2D(4.0, 5.0), p[13]);
    EXPECT_EQ(Vector2D(7.0, 8.0), v[12]);
    EXPECT_EQ(Vector2D(8.0, 7.0), v[13]);
    EXPECT_EQ(Vector2D(5.0, 4.0), f[12]);
    EXPECT_EQ(Vector2D(2.0, 1.0), f[13]);
}

TEST(ParticleSystemData2, AddParticlesException)
{
    ParticleSystemData2 particleSystem;
    particleSystem.Resize(12);

    EXPECT_ANY_THROW(particleSystem.AddParticles(
        Array1<Vector2D>({ Vector2D(1.0, 2.0), Vector2D(4.0, 5.0) }),
        Array1<Vector2D>({ Vector2D(7.0, 8.0) }),
        Array1<Vector2D>({ Vector2D(5.0, 4.0), Vector2D(2.0, 1.0) })));

    EXPECT_EQ(12u, particleSystem.NumberOfParticles());

    EXPECT_ANY_THROW(particleSystem.AddParticles(
        Array1<Vector2D>({ Vector2D(1.0, 2.0), Vector2D(4.0, 5.0) }),
        Array1<Vector2D>({ Vector2D(7.0, 8.0), Vector2D(2.0, 1.0) }),
        Array1<Vector2D>({ Vector2D(5.0, 4.0) })));

    EXPECT_EQ(12u, particleSystem.NumberOfParticles());
}

TEST(ParticleSystemData2, BuildNeighborSearcher)
{
    ParticleSystemData2 particleSystem;
    ParticleSystemData2::VectorData positions = {
        { 0.5, 0.7 }, { 0.1, 0.5 }, { 0.3, 0.1 }, { 0.2, 0.6 }, { 0.9, 0.7 },
        { 0.2, 0.5 }, { 0.5, 0.8 }, { 0.2, 0.3 }, { 0.9, 0.1 }, { 0.6, 0.8 },
        { 0.1, 0.7 }, { 0.4, 0.5 }, { 0.5, 0.9 }, { 0.7, 0.9 }, { 0.2, 0.8 },
        { 0.5, 0.5 }, { 0.4, 0.1 }, { 0.2, 0.4 }, { 0.1, 0.6 }, { 0.9, 0.8 }
    };
    particleSystem.AddParticles(positions);

    const double radius = 0.4;
    particleSystem.BuildNeighborSearcher(radius);

    auto neighborSearcher = particleSystem.NeighborSearcher();
    const Vector2D searchOrigin = { 0.1, 0.2 };
    std::vector<size_t> found;
    neighborSearcher->ForEachNearbyPoint(
        searchOrigin, radius,
        [&found](size_t i, const Vector2D&) { found.push_back(i); });

    for (size_t ii = 0; ii < positions.Length(); ++ii)
    {
        if (searchOrigin.DistanceTo(positions[ii]) <= radius)
        {
            EXPECT_TRUE(found.end() !=
                        std::find(found.begin(), found.end(), ii));
        }
    }
}

TEST(ParticleSystemData2, BuildNeighborLists)
{
    ParticleSystemData2 particleSystem;
    ParticleSystemData2::VectorData positions = {
        { 0.3, 0.5 }, { 0.6, 0.8 }, { 0.1, 0.8 }, { 0.7, 0.9 }, { 0.3, 0.2 },
        { 0.8, 0.3 }, { 0.8, 0.5 }, { 0.4, 0.9 }, { 0.8, 0.6 }, { 0.2, 0.9 },
        { 0.1, 0.2 }, { 0.6, 0.9 }, { 0.2, 0.2 }, { 0.5, 0.6 }, { 0.8, 0.4 },
        { 0.4, 0.2 }, { 0.2, 0.3 }, { 0.8, 0.6 }, { 0.2, 0.8 }, { 1.0, 0.5 }
    };
    particleSystem.AddParticles(positions);

    const double radius = 0.4;
    particleSystem.BuildNeighborSearcher(radius);
    particleSystem.BuildNeighborLists(radius);

    const auto& neighborLists = particleSystem.NeighborLists();
    EXPECT_EQ(positions.Length(), neighborLists.Length());

    for (size_t i = 0; i < neighborLists.Length(); ++i)
    {
        const auto& neighbors = neighborLists[i];

        for (size_t ii = 0; ii < positions.Length(); ++ii)
        {
            if (ii != i && positions[ii].DistanceTo(positions[i]) <= radius)
            {
                EXPECT_TRUE(neighbors.end() !=
                            std::find(neighbors.begin(), neighbors.end(), ii));
            }
        }
    }
}

TEST(ParticleSystemData2, Serialization)
{
    ParticleSystemData2 particleSystem;
    ParticleSystemData2::VectorData positions = {
        { 0.3, 0.5 }, { 0.6, 0.8 }, { 0.1, 0.8 }, { 0.7, 0.9 }, { 0.3, 0.2 },
        { 0.8, 0.3 }, { 0.8, 0.5 }, { 0.4, 0.9 }, { 0.8, 0.6 }, { 0.2, 0.9 },
        { 0.1, 0.2 }, { 0.6, 0.9 }, { 0.2, 0.2 }, { 0.5, 0.6 }, { 0.8, 0.4 },
        { 0.4, 0.2 }, { 0.2, 0.3 }, { 0.8, 0.6 }, { 0.2, 0.8 }, { 1.0, 0.5 }
    };
    particleSystem.AddParticles(positions);

    size_t a0 = particleSystem.AddScalarData(2.0);
    size_t a1 = particleSystem.AddScalarData(9.0);
    size_t a2 = particleSystem.AddVectorData({ 1.0, -3.0 });

    const double radius = 0.4;
    particleSystem.BuildNeighborSearcher(radius);
    particleSystem.BuildNeighborLists(radius);

    std::vector<uint8_t> buffer;
    particleSystem.Serialize(&buffer);

    ParticleSystemData2 particleSystem2;
    particleSystem2.Deserialize(buffer);

    EXPECT_EQ(positions.Length(), particleSystem2.NumberOfParticles());
    auto as0 = particleSystem2.ScalarDataAt(a0);
    for (size_t i = 0; i < positions.Length(); ++i)
    {
        EXPECT_DOUBLE_EQ(2.0, as0[i]);
    }

    auto as1 = particleSystem2.ScalarDataAt(a1);
    for (size_t i = 0; i < positions.Length(); ++i)
    {
        EXPECT_DOUBLE_EQ(9.0, as1[i]);
    }

    auto as2 = particleSystem2.VectorDataAt(a2);
    for (size_t i = 0; i < positions.Length(); ++i)
    {
        EXPECT_DOUBLE_EQ(1.0, as2[i].x);
        EXPECT_DOUBLE_EQ(-3.0, as2[i].y);
    }

    const auto& neighborLists = particleSystem.NeighborLists();
    const auto& neighborLists2 = particleSystem2.NeighborLists();
    EXPECT_EQ(neighborLists.Length(), neighborLists2.Length());

    for (size_t i = 0; i < neighborLists.Length(); ++i)
    {
        const auto& neighbors = neighborLists[i];
        const auto& neighbors2 = neighborLists2[i];
        EXPECT_EQ(neighbors.Length(), neighbors2.Length());

        for (size_t j = 0; j < neighbors.Length(); ++j)
        {
            EXPECT_EQ(neighbors[j], neighbors2[j]);
        }
    }
}
