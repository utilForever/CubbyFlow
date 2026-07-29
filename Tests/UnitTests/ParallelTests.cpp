#include "gtest/gtest.h"

#include <Core/Array/Array.hpp>
#include <Core/Utils/Parallel.hpp>

#include <numeric>
#include <random>

using namespace CubbyFlow;

const unsigned int NUM_CORES = std::thread::hardware_concurrency();

TEST(Parallel, Async)
{
    auto future = Internal::Async([]() { return 42; });
    EXPECT_EQ(42, future.get());
}

TEST(Parallel, Fill)
{
    size_t N = std::max(20u, (3 * NUM_CORES) / 2);
    std::vector<double> a(N);

    ParallelFill(a.begin(), a.end(), 3.0);

    for (double val : a)
    {
        EXPECT_EQ(3.0, val);
    }
}

TEST(Parallel, For)
{
    size_t N = std::max(20u, (3 * NUM_CORES) / 2);
    std::vector<double> a(N);

    for (size_t i = 0; i < N; ++i)
    {
        a[i] = static_cast<double>(i);
    }

    ParallelFor(ZERO_SIZE, a.size(), [&a](size_t i) {
        double expected = static_cast<double>(i);
        EXPECT_DOUBLE_EQ(expected, a[i]);
    });
}

TEST(Parallel, RangeFor)
{
    size_t N = std::max(20u, (3 * NUM_CORES) / 2);
    std::vector<double> a(N);

    for (size_t i = 0; i < N; ++i)
    {
        a[i] = static_cast<double>(i);
    }

    ParallelRangeFor(ZERO_SIZE, a.size(), [&a](size_t iBegin, size_t iEnd) {
        for (size_t i = iBegin; i < iEnd; ++i)
        {
            double expected = static_cast<double>(i);
            EXPECT_DOUBLE_EQ(expected, a[i]);
        }
    });
}

TEST(Parallel, For2D)
{
    size_t nX = std::max(20u, (3 * NUM_CORES) / 2);
    size_t nY = std::max(30u, (3 * NUM_CORES) / 2);
    Array2<double> a(nX, nY);

    for (size_t j = 0; j < nY; ++j)
    {
        for (size_t i = 0; i < nX; ++i)
        {
            a(i, j) = static_cast<double>(i + j * nX);
        }
    }

    ParallelFor(ZERO_SIZE, a.Width(), ZERO_SIZE, a.Height(),
                [&nX, &a](size_t i, size_t j) {
                    double expected = static_cast<double>(i + j * nX);
                    EXPECT_DOUBLE_EQ(expected, a(i, j));
                });
}

TEST(Parallel, RangeFor2D)
{
    size_t nX = std::max(20u, (3 * NUM_CORES) / 2);
    size_t nY = std::max(30u, (3 * NUM_CORES) / 2);
    Array2<double> a(nX, nY);

    for (size_t j = 0; j < nY; ++j)
    {
        for (size_t i = 0; i < nX; ++i)
        {
            a(i, j) = static_cast<double>(i + j * nX);
        }
    }

    ParallelRangeFor(
        ZERO_SIZE, a.Width(), ZERO_SIZE, a.Height(),
        [&nX, &a](size_t iBegin, size_t iEnd, size_t jBegin, size_t jEnd) {
            for (size_t j = jBegin; j < jEnd; ++j)
            {
                for (size_t i = iBegin; i < iEnd; ++i)
                {
                    double expected = static_cast<double>(i + j * nX);
                    EXPECT_DOUBLE_EQ(expected, a(i, j));
                }
            }
        });
}

TEST(Parallel, For3D)
{
    size_t nX = std::max(20u, (3 * NUM_CORES) / 2);
    size_t nY = std::max(30u, (3 * NUM_CORES) / 2);
    size_t nZ = std::max(30u, (3 * NUM_CORES) / 2);
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

    ParallelFor(ZERO_SIZE, a.Width(), ZERO_SIZE, a.Height(), ZERO_SIZE,
                a.Depth(), [&nY, &nX, &a](size_t i, size_t j, size_t k) {
                    double expected =
                        static_cast<double>(i + (j + k * nY) * nX);
                    EXPECT_DOUBLE_EQ(expected, a(i, j, k));
                });
}

TEST(Parallel, RangeFor3D)
{
    size_t nX = std::max(20u, (3 * NUM_CORES) / 2);
    size_t nY = std::max(30u, (3 * NUM_CORES) / 2);
    size_t nZ = std::max(30u, (3 * NUM_CORES) / 2);
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

    ParallelRangeFor(
        ZERO_SIZE, a.Width(), ZERO_SIZE, a.Height(), ZERO_SIZE, a.Depth(),
        [&nY, &nX, &a](size_t iBegin, size_t iEnd, size_t jBegin, size_t jEnd,
                       size_t kBegin, size_t kEnd) {
            for (size_t k = kBegin; k < kEnd; ++k)
            {
                for (size_t j = jBegin; j < jEnd; ++j)
                {
                    for (size_t i = iBegin; i < iEnd; ++i)
                    {
                        double expected =
                            static_cast<double>(i + (j + k * nY) * nX);
                        EXPECT_DOUBLE_EQ(expected, a(i, j, k));
                    }
                }
            }
        });
}

TEST(Parallel, Sort)
{
    size_t N = std::max(20u, (3 * NUM_CORES) / 2);
    std::vector<double> a(N);

    std::mt19937 rng;
    std::uniform_real_distribution<> d(0.0, 1.0);

    for (size_t i = 0; i < N; ++i)
    {
        a[i] = d(rng);
    }

    ParallelSort(a.begin(), a.end());

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

    ParallelSort(b.begin(), b.end());

    for (size_t i = 0; i + 1 < a.size(); ++i)
    {
        EXPECT_LE(b[i], b[i + 1]) << i;
    }

    std::vector<size_t> idx(N);
    for (size_t i = 0; i < N; ++i)
    {
        idx[i] = i;
    }

    ParallelSort(idx.begin(), idx.end(),
                 [&c](size_t i1, size_t i2) { return c[i1] < c[i2]; });

    for (size_t i = 0; i + 1 < a.size(); ++i)
    {
        EXPECT_LE(c[idx[i]], c[idx[i + 1]]);
    }
}

TEST(Parallel, Reduce)
{
    size_t N = std::max(20u, (3 * NUM_CORES) / 2);
    std::vector<int> a(N);

    std::mt19937 rng;
    std::uniform_int_distribution<> d(0, 10000);

    for (size_t i = 0; i < N; ++i)
    {
        a[i] = d(rng);
    }

    int sum = ParallelReduce(
        ZERO_SIZE, a.size(), 0,
        [&a](size_t start, size_t end, int init) {
            int result = init;

            for (size_t i = start; i < end; ++i)
            {
                result += a[i];
            }

            return result;
        },
        std::plus<int>());

    int expected = std::accumulate(a.begin(), a.end(), 0);
    EXPECT_EQ(expected, sum);
}
