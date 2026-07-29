#include "gtest/gtest.h"

#include <Core/Array/Array.hpp>
#include <Core/Array/ArrayView.hpp>

using namespace CubbyFlow;

TEST(ArrayView1, Constructors)
{
    double data[5];
    for (int i = 0; i < 5; ++i)
    {
        data[i] = static_cast<double>(i);
    }

    ArrayView1<double> acc(data, 5);

    EXPECT_EQ(5u, acc.Length());
    EXPECT_EQ(data, acc.data());
}

TEST(ArrayView1, Iterators)
{
    Array1<float> arr1 = { 6.f, 4.f, 1.f, -5.f };
    auto acc = arr1.View();

    size_t i = 0;
    for (float& elem : acc)
    {
        EXPECT_FLOAT_EQ(acc[i], elem);
        ++i;
    }

    i = 0;
    for (const float& elem : acc)
    {
        EXPECT_FLOAT_EQ(acc[i], elem);
        ++i;
    }
}

TEST(ArrayView1, ForEachIndex)
{
    Array1<float> arr1 = { 6.f, 4.f, 1.f, -5.f };
    auto acc = arr1.View();

    size_t cnt = 0;
    ForEachIndex(acc.Size(), [&cnt](size_t i) {
        EXPECT_EQ(cnt, i);
        ++cnt;
    });
}

TEST(ArrayView1, ParallelForEachIndex)
{
    Array1<float> arr1(200);
    auto acc = arr1.View();

    ForEachIndex(acc.Size(), [&arr1](size_t i) {
        arr1[i] = 200.f - static_cast<float>(i);
    });

    ParallelForEachIndex(acc.Size(), [&arr1](size_t i) {
        float ans = 200.f - static_cast<float>(i);
        EXPECT_EQ(ans, arr1[i]);
    });
}

TEST(ConstArrayView1, Constructors)
{
    double data[5];
    for (int i = 0; i < 5; ++i)
    {
        data[i] = static_cast<double>(i);
    }

    // Construct with ArrayView1
    ArrayView1<double> acc(data, 5);
    ConstArrayView1<double> cacc(acc);

    EXPECT_EQ(5u, cacc.Length());
    EXPECT_EQ(data, cacc.data());
}

TEST(ConstArrayView1, Iterators)
{
    Array1<float> arr1 = { 6.f, 4.f, 1.f, -5.f };
    auto acc = arr1.View();

    size_t i = 0;
    for (const float& elem : acc)
    {
        EXPECT_FLOAT_EQ(acc[i], elem);
        ++i;
    }
}

TEST(ConstArrayView1, ForEachIndex)
{
    Array1<float> arr1 = { 6.f, 4.f, 1.f, -5.f };
    auto acc = arr1.View();

    size_t cnt = 0;
    ForEachIndex(acc.Size(), [&cnt](size_t i) {
        EXPECT_EQ(cnt, i);
        ++cnt;
    });
}

TEST(ConstArrayView1, ParallelForEachIndex)
{
    Array1<float> arr1(200);
    auto acc = arr1.View();

    ForEachIndex(acc.Size(), [&arr1](size_t i) {
        arr1[i] = 200.f - static_cast<float>(i);
    });

    ParallelForEachIndex(acc.Size(), [&arr1](size_t i) {
        float ans = 200.f - static_cast<float>(i);
        EXPECT_EQ(ans, arr1[i]);
    });
}
