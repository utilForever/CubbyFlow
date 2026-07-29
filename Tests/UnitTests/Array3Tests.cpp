#include "gtest/gtest.h"

#include <Core/Array/Array.hpp>
#include <Core/Utils/Serialization.hpp>

using namespace CubbyFlow;

TEST(Array3, Constructors)
{
    Array3<float> arr1;
    EXPECT_EQ(0u, arr1.Width());
    EXPECT_EQ(0u, arr1.Height());
    EXPECT_EQ(0u, arr1.Depth());

    Array3<float> arr2(Vector3UZ(3, 7, 4));
    EXPECT_EQ(3u, arr2.Width());
    EXPECT_EQ(7u, arr2.Height());
    EXPECT_EQ(4u, arr2.Depth());
    for (size_t i = 0; i < 84; ++i)
    {
        EXPECT_FLOAT_EQ(0.f, arr2[i]);
    }

    Array3<float> arr3(Vector3UZ(1, 9, 5), 1.5f);
    EXPECT_EQ(1u, arr3.Width());
    EXPECT_EQ(9u, arr3.Height());
    EXPECT_EQ(5u, arr3.Depth());
    for (size_t i = 0; i < 45; ++i)
    {
        EXPECT_FLOAT_EQ(1.5f, arr3[i]);
    }

    Array3<float> arr4(5, 2, 8);
    EXPECT_EQ(5u, arr4.Width());
    EXPECT_EQ(2u, arr4.Height());
    EXPECT_EQ(8u, arr4.Depth());
    for (size_t i = 0; i < 80; ++i)
    {
        EXPECT_FLOAT_EQ(0.f, arr4[i]);
    }

    Array3<float> arr5(3, 4, 2, 7.f);
    EXPECT_EQ(3u, arr5.Width());
    EXPECT_EQ(4u, arr5.Height());
    EXPECT_EQ(2u, arr5.Depth());
    for (size_t i = 0; i < 24; ++i)
    {
        EXPECT_FLOAT_EQ(7.f, arr5[i]);
    }

    Array3<float> arr6({ { { 1.f, 2.f, 3.f, 4.f },
                           { 5.f, 6.f, 7.f, 8.f },
                           { 9.f, 10.f, 11.f, 12.f } },
                         { { 13.f, 14.f, 15.f, 16.f },
                           { 17.f, 18.f, 19.f, 20.f },
                           { 21.f, 22.f, 23.f, 24.f } } });

    EXPECT_EQ(4u, arr6.Width());
    EXPECT_EQ(3u, arr6.Height());
    EXPECT_EQ(2u, arr6.Depth());
    for (size_t i = 0; i < 24; ++i)
    {
        EXPECT_FLOAT_EQ(static_cast<float>(i) + 1.f, arr6[i]);
    }

    Array3<float> arr7({ { { 1.f, 2.f, 3.f, 4.f },
                           { 5.f, 6.f, 7.f, 8.f },
                           { 9.f, 10.f, 11.f, 12.f } },
                         { { 13.f, 14.f, 15.f, 16.f },
                           { 17.f, 18.f, 19.f, 20.f },
                           { 21.f, 22.f, 23.f, 24.f } } });
    Array3<float> arr8(arr7);

    EXPECT_EQ(4u, arr8.Width());
    EXPECT_EQ(3u, arr8.Height());
    EXPECT_EQ(2u, arr8.Depth());
    for (size_t i = 0; i < 24; ++i)
    {
        EXPECT_FLOAT_EQ(static_cast<float>(i) + 1.f, arr8[i]);
    }
}

TEST(Array3, Clear)
{
    Array3<float> arr({ { { 1.f, 2.f, 3.f, 4.f },
                          { 5.f, 6.f, 7.f, 8.f },
                          { 9.f, 10.f, 11.f, 12.f } },
                        { { 13.f, 14.f, 15.f, 16.f },
                          { 17.f, 18.f, 19.f, 20.f },
                          { 21.f, 22.f, 23.f, 24.f } } });

    arr.Clear();
    EXPECT_EQ(0u, arr.Width());
    EXPECT_EQ(0u, arr.Height());
    EXPECT_EQ(0u, arr.Depth());
}

TEST(Array3, ResizeMethod)
{
    Array3<float> arr1;
    arr1.Resize(Vector3UZ(2, 9, 5));
    EXPECT_EQ(2u, arr1.Width());
    EXPECT_EQ(9u, arr1.Height());
    EXPECT_EQ(5u, arr1.Depth());
    for (size_t i = 0; i < 90; ++i)
    {
        EXPECT_FLOAT_EQ(0.f, arr1[i]);
    }

    arr1.Resize(Vector3UZ(8, 13, 7), 4.f);
    EXPECT_EQ(8u, arr1.Width());
    EXPECT_EQ(13u, arr1.Height());
    EXPECT_EQ(7u, arr1.Depth());
    for (size_t k = 0; k < 7; ++k)
    {
        for (size_t j = 0; j < 13; ++j)
        {
            for (size_t i = 0; i < 8; ++i)
            {
                if (i < 2 && j < 9 && k < 5)
                {
                    EXPECT_FLOAT_EQ(0.f, arr1(i, j, k));
                }
                else
                {
                    EXPECT_FLOAT_EQ(4.f, arr1(i, j, k));
                }
            }
        }
    }

    Array3<float> arr2;
    arr2.Resize(7, 6, 3);
    EXPECT_EQ(7u, arr2.Width());
    EXPECT_EQ(6u, arr2.Height());
    EXPECT_EQ(3u, arr2.Depth());
    for (size_t i = 0; i < 126; ++i)
    {
        EXPECT_FLOAT_EQ(0.f, arr2[i]);
    }

    arr2.Resize(1, 9, 4, 3.f);
    EXPECT_EQ(1u, arr2.Width());
    EXPECT_EQ(9u, arr2.Height());
    EXPECT_EQ(4u, arr2.Depth());
    for (size_t k = 0; k < 4; ++k)
    {
        for (size_t j = 0; j < 9; ++j)
        {
            for (size_t i = 0; i < 1; ++i)
            {
                if (j < 6 && k < 3)
                {
                    EXPECT_FLOAT_EQ(0.f, arr2(i, j, k));
                }
                else
                {
                    EXPECT_FLOAT_EQ(3.f, arr2(i, j, k));
                }
            }
        }
    }
}

TEST(Array3, Iterators)
{
    Array3<float> arr1({ { { 1.f, 2.f, 3.f, 4.f },
                           { 5.f, 6.f, 7.f, 8.f },
                           { 9.f, 10.f, 11.f, 12.f } },
                         { { 13.f, 14.f, 15.f, 16.f },
                           { 17.f, 18.f, 19.f, 20.f },
                           { 21.f, 22.f, 23.f, 24.f } } });

    float cnt = 1.f;
    for (float& elem : arr1)
    {
        EXPECT_FLOAT_EQ(cnt, elem);
        cnt += 1.f;
    }

    cnt = 1.f;
    for (const float& elem : arr1)
    {
        EXPECT_FLOAT_EQ(cnt, elem);
        cnt += 1.f;
    }
}

TEST(Array3, ForEach)
{
    Array3<float> arr1({ { { 1.f, 2.f, 3.f, 4.f },
                           { 5.f, 6.f, 7.f, 8.f },
                           { 9.f, 10.f, 11.f, 12.f } },
                         { { 13.f, 14.f, 15.f, 16.f },
                           { 17.f, 18.f, 19.f, 20.f },
                           { 21.f, 22.f, 23.f, 24.f } } });

    size_t i = 0;
    std::for_each(arr1.begin(), arr1.end(), [&arr1, &i](float val) {
        EXPECT_FLOAT_EQ(arr1[i], val);
        ++i;
    });
}

TEST(Array3, ForEachIndex)
{
    Array3<float> arr1({ { { 1.f, 2.f, 3.f, 4.f },
                           { 5.f, 6.f, 7.f, 8.f },
                           { 9.f, 10.f, 11.f, 12.f } },
                         { { 13.f, 14.f, 15.f, 16.f },
                           { 17.f, 18.f, 19.f, 20.f },
                           { 21.f, 22.f, 23.f, 24.f } } });

    ForEachIndex(arr1.Size(), [&arr1](size_t i, size_t j, size_t k) {
        size_t idx = i + (4 * (j + 3 * k)) + 1;
        EXPECT_FLOAT_EQ(static_cast<float>(idx), arr1(i, j, k));
    });
}

TEST(Array3, ParallelForEachIndex)
{
    Array3<float> arr1({ { { 1.f, 2.f, 3.f, 4.f },
                           { 5.f, 6.f, 7.f, 8.f },
                           { 9.f, 10.f, 11.f, 12.f } },
                         { { 13.f, 14.f, 15.f, 16.f },
                           { 17.f, 18.f, 19.f, 20.f },
                           { 21.f, 22.f, 23.f, 24.f } } });

    ParallelForEachIndex(arr1.Size(), [&arr1](size_t i, size_t j, size_t k) {
        size_t idx = i + (4 * (j + 3 * k)) + 1;
        EXPECT_FLOAT_EQ(static_cast<float>(idx), arr1(i, j, k));
    });
}

TEST(Array3, View)
{
    Array3<float> arr({ { { 1.f, 2.f, 3.f, 4.f },
                          { 5.f, 6.f, 7.f, 8.f },
                          { 9.f, 10.f, 11.f, 12.f } },
                        { { 13.f, 14.f, 15.f, 16.f },
                          { 17.f, 18.f, 19.f, 20.f },
                          { 21.f, 22.f, 23.f, 24.f } } });

    ArrayView3<float> arr2 = arr.View();
    EXPECT_EQ(4u, arr2.Width());
    EXPECT_EQ(3u, arr2.Height());

    for (size_t i = 0; i < 12; ++i)
    {
        EXPECT_FLOAT_EQ(static_cast<float>(i) + 1.f, arr2[i]);
    }
}
