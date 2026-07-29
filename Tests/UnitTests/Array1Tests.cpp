#include "gtest/gtest.h"

#include <Core/Array/Array.hpp>
#include <Core/Utils/Serialization.hpp>

using namespace CubbyFlow;

TEST(Array1, Constructors)
{
    Array1<float> arr1;
    EXPECT_EQ(0u, arr1.Length());

    Array1<float> arr2(9, 1.5f);
    EXPECT_EQ(9u, arr2.Length());
    for (size_t i = 0; i < 9; ++i)
    {
        EXPECT_FLOAT_EQ(1.5f, arr2[i]);
    }

    Array1<float> arr3({ 1.f, 2.f, 3.f, 4.f });
    EXPECT_EQ(4u, arr3.Length());
    for (size_t i = 0; i < 4; ++i)
    {
        EXPECT_FLOAT_EQ(static_cast<float>(i) + 1.f, arr3[i]);
    }

    Array1<float> arr4({ 1.f, 2.f, 3.f, 4.f });
    Array1<float> arr5(arr4);
    EXPECT_EQ(4u, arr5.Length());
    for (size_t i = 0; i < 4; ++i)
    {
        EXPECT_FLOAT_EQ(static_cast<float>(i) + 1.f, arr5[i]);
    }
}

TEST(Array1, SetMethods)
{
    Array1<float> arr1(12, -1.f);
    arr1.Fill(3.5f);
    for (float a : arr1)
    {
        EXPECT_EQ(3.5f, a);
    }

    Array1<float> arr2;
    arr1.CopyFrom(arr2);
    EXPECT_EQ(arr1.Length(), arr2.Length());
    for (size_t i = 0; i < arr2.Length(); ++i)
    {
        EXPECT_EQ(arr1[i], arr2[i]);
    }

    arr2 = { 2.f, 5.f, 9.f, -1.f };
    EXPECT_EQ(4u, arr2.Length());
    EXPECT_EQ(2.f, arr2[0]);
    EXPECT_EQ(5.f, arr2[1]);
    EXPECT_EQ(9.f, arr2[2]);
    EXPECT_EQ(-1.f, arr2[3]);
}

TEST(Array1, Clear)
{
    Array1<float> arr1 = { 2.f, 5.f, 9.f, -1.f };
    arr1.Clear();
    EXPECT_EQ(0u, arr1.Length());
}

TEST(Array1, ResizeMethod)
{
    Array1<float> arr;
    arr.Resize(9);
    EXPECT_EQ(9u, arr.Length());
    for (size_t i = 0; i < 9; ++i)
    {
        EXPECT_FLOAT_EQ(0.f, arr[i]);
    }

    arr.Resize(12, 4.f);
    EXPECT_EQ(12u, arr.Length());
    for (size_t i = 0; i < 8; ++i)
    {
        if (i < 9)
        {
            EXPECT_FLOAT_EQ(0.f, arr[i]);
        }
        else
        {
            EXPECT_FLOAT_EQ(4.f, arr[i]);
        }
    }
}

TEST(Array1, Iterators)
{
    Array1<float> arr1 = { 6.f, 4.f, 1.f, -5.f };

    size_t i = 0;
    for (float& elem : arr1)
    {
        EXPECT_FLOAT_EQ(arr1[i], elem);
        ++i;
    }

    i = 0;
    for (const float& elem : arr1)
    {
        EXPECT_FLOAT_EQ(arr1[i], elem);
        ++i;
    }
}

TEST(Array1, ForEach)
{
    Array1<float> arr1 = { 6.f, 4.f, 1.f, -5.f };
    size_t i = 0;
    std::for_each(arr1.begin(), arr1.end(), [&arr1, &i](float val) {
        EXPECT_FLOAT_EQ(arr1[i], val);
        ++i;
    });
}

TEST(Array1, ForEachIndex)
{
    Array1<float> arr1 = { 6.f, 4.f, 1.f, -5.f };
    size_t cnt = 0;
    ForEachIndex(arr1.Length(), [&cnt](size_t i) {
        EXPECT_EQ(cnt, i);
        ++cnt;
    });
}

TEST(Array1, ParallelForEachIndex)
{
    Array1<float> arr1(200);
    ForEachIndex(arr1.Length(), [&arr1](size_t i) {
        arr1[i] = static_cast<float>(200.f - i);
    });

    ParallelForEachIndex(arr1.Length(), [&arr1](size_t i) {
        float ans = static_cast<float>(200.f - i);
        EXPECT_EQ(ans, arr1[i]);
    });
}

TEST(Array1, DeserializeValidatesData)
{
    const std::vector<uint8_t> emptyBuffer;
    const std::vector<uint8_t> truncatedBuffer(1);
    Array1<double> invalidResult(1, 42.0);

    EXPECT_THROW(Deserialize(emptyBuffer, &invalidResult),
                 std::invalid_argument);
    EXPECT_THROW(Deserialize(truncatedBuffer, &invalidResult),
                 std::invalid_argument);
    ASSERT_EQ(1u, invalidResult.Length());
    EXPECT_DOUBLE_EQ(42.0, invalidResult[0]);

    std::vector<uint8_t> buffer;
    Serialize(static_cast<const uint8_t*>(nullptr), 0, &buffer);

    Array1<double> emptyResult(1, 42.0);
    Deserialize(buffer, &emptyResult);

    EXPECT_EQ(0u, emptyResult.Length());

    const uint8_t byte = 0;
    Serialize(&byte, sizeof(byte), &buffer);

    Array1<double> array(1, 42.0);

    EXPECT_THROW(Deserialize(buffer, &array), std::invalid_argument);
    ASSERT_EQ(1u, array.Length());
    EXPECT_DOUBLE_EQ(42.0, array[0]);
}
