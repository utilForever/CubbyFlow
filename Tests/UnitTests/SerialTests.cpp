#include "gtest/gtest.h"

#include <Core/Array/Array.hpp>
#include <Core/Utils/Constants.hpp>
#include <Core/Utils/Serial.hpp>

#include <random>

using namespace CubbyFlow;

TEST(Serial, Fill)
{
    size_t N = 100;
    std::vector<double> a(N);

    SerialFill(a.begin(), a.end(), 3.0);

    for (double val : a)
    {
        EXPECT_EQ(3.0, val);
    }
}

TEST(Serial, For)
{
    size_t N = 100;
    std::vector<double> a(N);

    for (size_t i = 0; i < N; ++i)
    {
        a[i] = static_cast<double>(i);
    }

    SerialFor(ZERO_SIZE, a.size(), [&a](size_t i) {
        double expected = static_cast<double>(i);
        EXPECT_DOUBLE_EQ(expected, a[i]);
    });
}

TEST(Serial, For2D)
{
    size_t nX = 100;
    size_t nY = 100;
    Array2<double> a(nX, nY);

    for (size_t j = 0; j < nY; ++j)
    {
        for (size_t i = 0; i < nX; ++i)
        {
            a(i, j) = static_cast<double>(i + j * nX);
        }
    }

    SerialFor(ZERO_SIZE, a.Width(), ZERO_SIZE, a.Height(),
              [&nX, &a](size_t i, size_t j) {
                  double expected = static_cast<double>(i + j * nX);
                  EXPECT_DOUBLE_EQ(expected, a(i, j));
              });
}

TEST(Serial, For3D)
{
    size_t nX = 100;
    size_t nY = 100;
    size_t nZ = 100;
    Array3<double> a(nX, nY, nZ);

    for (size_t k = 0; k < nZ; ++k)
    {
        for (size_t j = 0; j < nY; ++j)
        {
            for (size_t i = 0; i < nX; ++i)
            {
                a(i, j, k) = static_cast<double>(i + (j + k * nY) * nX);
            }
        }
    }

    SerialFor(ZERO_SIZE, a.Width(), ZERO_SIZE, a.Height(), ZERO_SIZE, a.Depth(),
              [&nY, &nX, &a](size_t i, size_t j, size_t k) {
                  double expected = static_cast<double>(i + (j + k * nY) * nX);
                  EXPECT_DOUBLE_EQ(expected, a(i, j, k));
              });
}

TEST(Serial, Sort)
{
    size_t N = 100;
    std::vector<double> a(N);

    std::mt19937 rng;
    std::uniform_real_distribution<> d(0.0, 1.0);

    for (size_t i = 0; i < N; ++i)
    {
        a[i] = d(rng);
    }

    SerialSort(a.begin(), a.end());

    for (size_t i = 0; i + 1 < a.size(); ++i)
    {
        EXPECT_LE(a[i], a[i + 1]) << i;
    }

    std::vector<double> b(N);
    for (size_t i = 0; i < N; ++i)
    {
        b[i] = d(rng);
    }

    std::vector<double> c = b;

    SerialSort(b.begin(), b.end());

    for (size_t i = 0; i + 1 < a.size(); ++i)
    {
        EXPECT_LE(b[i], b[i + 1]) << i;
    }

    std::vector<size_t> idx(N);
    for (size_t i = 0; i < N; ++i)
    {
        idx[i] = i;
    }

    SerialSort(idx.begin(), idx.end(),
               [&c](size_t x, size_t y) { return c[x] < c[y]; });

    for (size_t i = 0; i + 1 < a.size(); ++i)
    {
        EXPECT_LE(c[idx[i]], c[idx[i + 1]]);
    }
}