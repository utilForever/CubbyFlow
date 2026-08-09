#include "gtest/gtest.h"

#include <Core/Geometry/Plane.hpp>
#include <Core/Geometry/RigidBodyCollider.hpp>
#include <Core/Solver/Particle/MPM/SnowMPMSolver.hpp>

#include <cmath>

using namespace CubbyFlow;

TEST(SnowMPMSolverManual, FrictionalColliderStateRemainsFinite)
{
    constexpr double radius = 0.01;
    SnowMPMSolver2 solver{ { 16, 16 }, { 0.1, 0.1 }, {}, radius, 1e-3 };
    solver.SetDragCoefficient(0.0);

    auto collider = std::make_shared<RigidBodyCollider2>(
        std::make_shared<Plane2>(Vector2D{ 0.0, 1.0 }, Vector2D{ 0.0, 0.2 }));
    collider->SetFrictionCoefficient(0.4);
    solver.SetCollider(collider);

    Array1<Vector2D> positions;
    Array1<Vector2D> velocities;
    for (size_t j = 0; j < 4; ++j)
    {
        for (size_t i = 0; i < 4; ++i)
        {
            positions.Append({ 0.6 + 0.1 * static_cast<double>(i),
                               0.8 + 0.1 * static_cast<double>(j) });
            velocities.Append({ 1.0, -6.0 });
        }
    }

    auto data = solver.GetMPMSystemData();
    data->AddParticles(positions, velocities);

    for (Frame frame{ 0, 1.0 / 120.0 }; frame.index < 12; ++frame)
    {
        solver.Update(frame);

        const auto particlePositions = data->Positions();
        const auto particleVelocities = data->Velocities();
        const auto volumes = data->InitialVolumes();
        const auto states = data->DeformationStates();
        for (size_t i = 0; i < data->NumberOfParticles(); ++i)
        {
            EXPECT_TRUE(std::isfinite(particlePositions[i].x));
            EXPECT_TRUE(std::isfinite(particlePositions[i].y));
            EXPECT_TRUE(std::isfinite(particleVelocities[i].x));
            EXPECT_TRUE(std::isfinite(particleVelocities[i].y));
            EXPECT_TRUE(std::isfinite(volumes[i]));
            EXPECT_GT(volumes[i], 0.0);
            EXPECT_GE(particlePositions[i].y, 0.2 + radius - 1e-9);

            for (size_t row = 0; row < 2; ++row)
            {
                for (size_t column = 0; column < 2; ++column)
                {
                    EXPECT_TRUE(std::isfinite(states[i].elastic(row, column)));
                    EXPECT_TRUE(std::isfinite(states[i].plastic(row, column)));
                }
            }
        }
    }
}
