#include "gtest/gtest.h"

#include <Core/Matrix/Matrix.hpp>

using namespace CubbyFlow;

TEST(Vector3, Constructors)
{
    Vector3F vec;
    EXPECT_FLOAT_EQ(0.f, vec.x);
    EXPECT_FLOAT_EQ(0.f, vec.y);
    EXPECT_FLOAT_EQ(0.f, vec.z);

    Vector3F vec2(5.f, 3.f, 8.f);
    EXPECT_FLOAT_EQ(5.f, vec2.x);
    EXPECT_FLOAT_EQ(3.f, vec2.y);
    EXPECT_FLOAT_EQ(8.f, vec2.z);

    Vector2F vec3(4.f, 7.f);
    Vector3F vec4(vec3, 9.f);
    EXPECT_FLOAT_EQ(4.f, vec4.x);
    EXPECT_FLOAT_EQ(7.f, vec4.y);
    EXPECT_FLOAT_EQ(9.f, vec4.z);

    Vector3F vec5 = { 7.f, 6.f, 1.f };
    EXPECT_FLOAT_EQ(7.f, vec5.x);
    EXPECT_FLOAT_EQ(6.f, vec5.y);
    EXPECT_FLOAT_EQ(1.f, vec5.z);

    Vector3F vec6(vec5);
    EXPECT_FLOAT_EQ(7.f, vec6.x);
    EXPECT_FLOAT_EQ(6.f, vec6.y);
    EXPECT_FLOAT_EQ(1.f, vec6.z);
}

TEST(Vector3, BasicSetterMethods)
{
    Vector3F vec(3.f, 9.f, 4.f);
    vec.Fill(0.f);
    EXPECT_FLOAT_EQ(0.f, vec.x);
    EXPECT_FLOAT_EQ(0.f, vec.y);
    EXPECT_FLOAT_EQ(0.f, vec.z);

    vec.Fill(4.f);
    EXPECT_FLOAT_EQ(4.f, vec.x);
    EXPECT_FLOAT_EQ(4.f, vec.y);
    EXPECT_FLOAT_EQ(4.f, vec.z);

    vec.Fill([](size_t i) -> float { return i * 5.f; });
    EXPECT_FLOAT_EQ(0.f, vec.x);
    EXPECT_FLOAT_EQ(5.f, vec.y);
    EXPECT_FLOAT_EQ(10.f, vec.z);

    vec.Fill([](size_t i, size_t j) -> float { return i + 8.f * (j + 1); });
    EXPECT_FLOAT_EQ(8.f, vec.x);
    EXPECT_FLOAT_EQ(9.f, vec.y);
    EXPECT_FLOAT_EQ(10.f, vec.z);

    Vector3F vec2{ 5.f, 3.f, 1.f };
    vec.Swap(vec2);
    EXPECT_FLOAT_EQ(5.f, vec.x);
    EXPECT_FLOAT_EQ(3.f, vec.y);
    EXPECT_FLOAT_EQ(1.f, vec.z);
    EXPECT_FLOAT_EQ(8.f, vec2.x);
    EXPECT_FLOAT_EQ(9.f, vec2.y);
    EXPECT_FLOAT_EQ(10.f, vec2.z);

    vec = Vector3F(4.f, 2.f, 8.f);
    vec.Normalize();
    float len = vec.x * vec.x + vec.y * vec.y + vec.z * vec.z;
    EXPECT_TRUE(fabsf(len - 1.f) < 1e-6);
}

TEST(Vector3, BasicGetterMethods)
{
    Vector3F vec(3.f, 7.f, -1.f), vec2(-3.f, -7.f, 1.f);

    float sum = vec.Sum();
    EXPECT_FLOAT_EQ(9.f, sum);

    float avg = vec.Avg();
    EXPECT_FLOAT_EQ(3.f, avg);

    float min = vec.Min();
    EXPECT_FLOAT_EQ(-1.f, min);

    float max = vec.Max();
    EXPECT_FLOAT_EQ(7.f, max);

    float absmin = vec2.AbsMin();
    EXPECT_FLOAT_EQ(1.f, absmin);

    float absmax = vec2.AbsMax();
    EXPECT_FLOAT_EQ(-7.f, absmax);

    size_t daxis = vec.DominantAxis();
    EXPECT_EQ(static_cast<size_t>(1), daxis);

    size_t saxis = vec.SubdominantAxis();
    EXPECT_EQ(static_cast<size_t>(2), saxis);

    float eps = 1e-6f;
    vec2 = vec.Normalized();
    float lenSqr = vec2.x * vec2.x + vec2.y * vec2.y + vec2.z * vec2.z;
    EXPECT_TRUE(lenSqr - 1.f < eps);

    vec2 *= 2.f;
    float len = vec2.Length();
    EXPECT_NEAR(2.f, len, eps);

    lenSqr = vec2.LengthSquared();
    EXPECT_NEAR(4.f, lenSqr, eps);
}

TEST(Vector3, BracketOperators)
{
    Vector3F vec(8.f, 9.f, 1.f);
    EXPECT_FLOAT_EQ(8.f, vec[0]);
    EXPECT_FLOAT_EQ(9.f, vec[1]);
    EXPECT_FLOAT_EQ(1.f, vec[2]);

    vec[0] = 7.f;
    vec[1] = 6.f;
    vec[2] = 5.f;
    EXPECT_FLOAT_EQ(7.f, vec.x);
    EXPECT_FLOAT_EQ(6.f, vec.y);
    EXPECT_FLOAT_EQ(5.f, vec.z);
}

TEST(Vector3, AssignmentOperators)
{
    Vector3F vec(5.f, 1.f, 0.f);
    Vector3F vec2(3.f, 3.f, 3.f);
    vec2 = vec;
    EXPECT_FLOAT_EQ(5.f, vec2.x);
    EXPECT_FLOAT_EQ(1.f, vec2.y);
    EXPECT_FLOAT_EQ(0.f, vec2.z);
}

TEST(Vector3, AugmentedOperators)
{
    Vector3F vec(3.f, 9.f, -2.f);
    vec += 4.f;
    EXPECT_FLOAT_EQ(7.f, vec.x);
    EXPECT_FLOAT_EQ(13.f, vec.y);
    EXPECT_FLOAT_EQ(2.f, vec.z);

    vec += Vector3F(-2.f, 1.f, 5.f);
    EXPECT_FLOAT_EQ(5.f, vec.x);
    EXPECT_FLOAT_EQ(14.f, vec.y);
    EXPECT_FLOAT_EQ(7.f, vec.z);

    vec -= 8.f;
    EXPECT_FLOAT_EQ(-3.f, vec.x);
    EXPECT_FLOAT_EQ(6.f, vec.y);
    EXPECT_FLOAT_EQ(-1.f, vec.z);

    vec -= Vector3F(-5.f, 3.f, -6.f);
    EXPECT_FLOAT_EQ(2.f, vec.x);
    EXPECT_FLOAT_EQ(3.f, vec.y);
    EXPECT_FLOAT_EQ(5.f, vec.z);

    vec *= 2.f;
    EXPECT_FLOAT_EQ(4.f, vec.x);
    EXPECT_FLOAT_EQ(6.f, vec.y);
    EXPECT_FLOAT_EQ(10.f, vec.z);

    ElemIMul(vec, Vector3F(3.f, -2.f, 0.4f));
    EXPECT_FLOAT_EQ(12.f, vec.x);
    EXPECT_FLOAT_EQ(-12.f, vec.y);
    EXPECT_FLOAT_EQ(4.f, vec.z);

    vec /= 4.f;
    EXPECT_FLOAT_EQ(3.f, vec.x);
    EXPECT_FLOAT_EQ(-3.f, vec.y);
    EXPECT_FLOAT_EQ(1.f, vec.z);

    ElemIDiv(vec, Vector3F(3.f, -1.f, 2.f));
    EXPECT_FLOAT_EQ(1.f, vec.x);
    EXPECT_FLOAT_EQ(3.f, vec.y);
    EXPECT_FLOAT_EQ(0.5f, vec.z);
}

TEST(Vector3, EqualOperatators)
{
    Vector3F vec, vec2(3.f, 7.f, 4.f), vec3(3.f, 5.f, 4.f), vec4(5.f, 1.f, 2.f);
    vec = vec2;
    EXPECT_EQ(vec, vec2);
    EXPECT_NE(vec, vec3);
    EXPECT_NE(vec, vec4);
}

TEST(Vector3, MinMaxFunctions)
{
    Vector3F vec(5.f, 1.f, 0.f);
    Vector3F vec2(3.f, 3.f, 3.f);
    Vector3F minVector = Min(vec, vec2);
    Vector3F maxVector = Max(vec, vec2);
    EXPECT_TRUE(minVector == Vector3F(3.f, 1.f, 0.f));
    EXPECT_TRUE(maxVector == Vector3F(5.f, 3.f, 3.f));
}

TEST(Vector3, ClampFunction)
{
    Vector3F vec(2.f, 4.f, 1.f), low(3.f, -1.f, 0.f), high(5.f, 2.f, 3.f);
    Vector3F clampedVec = Clamp(vec, low, high);
    EXPECT_TRUE(clampedVec == Vector3F(3.f, 2.f, 1.f));
}

TEST(Vector3, CeilFloorFunctions)
{
    Vector3F vec(2.2f, 4.7f, -0.2f);
    Vector3F ceilVec = Ceil(vec);
    EXPECT_TRUE(ceilVec == Vector3F(3.f, 5.f, 0.f));

    Vector3F floorVec = Floor(vec);
    EXPECT_TRUE(floorVec == Vector3F(2.f, 4.f, -1.f));
}

TEST(Vector3, BinaryOperators)
{
    Vector3F vec(3.f, 9.f, 4.f);
    vec = vec + 4.f;
    EXPECT_FLOAT_EQ(7.f, vec.x);
    EXPECT_FLOAT_EQ(13.f, vec.y);
    EXPECT_FLOAT_EQ(8.f, vec.z);

    vec = vec + Vector3F(-2.f, 1.f, 5.f);
    EXPECT_FLOAT_EQ(5.f, vec.x);
    EXPECT_FLOAT_EQ(14.f, vec.y);
    EXPECT_FLOAT_EQ(13.f, vec.z);

    vec = vec - 8.f;
    EXPECT_FLOAT_EQ(-3.f, vec.x);
    EXPECT_FLOAT_EQ(6.f, vec.y);
    EXPECT_FLOAT_EQ(5.f, vec.z);

    vec = vec - Vector3F(-5.f, 3.f, 12.f);
    EXPECT_FLOAT_EQ(2.f, vec.x);
    EXPECT_FLOAT_EQ(3.f, vec.y);
    EXPECT_FLOAT_EQ(-7.f, vec.z);

    vec = vec * 2.f;
    EXPECT_FLOAT_EQ(4.f, vec.x);
    EXPECT_FLOAT_EQ(6.f, vec.y);
    EXPECT_FLOAT_EQ(-14.f, vec.z);

    vec = ElemMul(vec, Vector3F(3.f, -2.f, 0.5f));
    EXPECT_FLOAT_EQ(12.f, vec.x);
    EXPECT_FLOAT_EQ(-12.f, vec.y);
    EXPECT_FLOAT_EQ(-7.f, vec.z);

    vec = vec / 4.f;
    EXPECT_FLOAT_EQ(3.f, vec.x);
    EXPECT_FLOAT_EQ(-3.f, vec.y);
    EXPECT_FLOAT_EQ(-1.75f, vec.z);

    ElemIDiv(vec, Vector3F(3.f, -1.f, 0.25f));
    EXPECT_FLOAT_EQ(1.f, vec.x);
    EXPECT_FLOAT_EQ(3.f, vec.y);
    EXPECT_FLOAT_EQ(-7.f, vec.z);

    Vector3D v = Vector3D(2.0, 1.0, 3.0).Normalized();
    Vector3D normal = Vector3D(1.0, 1.0, 1.0).Normalized();

    Vector3D reflected = v.Reflected(normal);
    Vector3D reflectedAnswer = Vector3D(-2.0, -3.0, -1.0).Normalized();
    EXPECT_NEAR(reflected.DistanceTo(reflectedAnswer), 0.0, 1e-9);

    Vector3D projected = v.Projected(normal);
    EXPECT_NEAR(projected.Dot(normal), 0.0, 1e-9);

    auto tangential = normal.Tangentials();
    EXPECT_NEAR(std::get<0>(tangential).Dot(normal), 0.0, 1e-9);
    EXPECT_NEAR(std::get<1>(tangential).Dot(normal), 0.0, 1e-9);
}
