// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/FDM/FDMMGLinearSystem2.hpp>
#include <Core/Utils/Parallel.hpp>

#include <array>

namespace CubbyFlow
{
namespace
{
constexpr std::array<double, 4> RESTRICTION_KERNEL = { 0.125, 0.375, 0.375,
                                                       0.125 };

void RestrictRange(const FDMVector2& finer, FDMVector2* coarser,
                   const Vector2UZ& size, size_t iBegin, size_t iEnd,
                   size_t jBegin, size_t jEnd)
{
    for (size_t j = jBegin; j < jEnd; ++j)
    {
        const std::array<size_t, 4> jIndices = { (j > 0) ? 2 * j - 1 : 2 * j,
                                                 2 * j, 2 * j + 1,
                                                 (j + 1 < size.y) ? 2 * j + 2
                                                                  : 2 * j + 1 };
        for (size_t i = iBegin; i < iEnd; ++i)
        {
            const std::array<size_t, 4> iIndices = {
                (i > 0) ? 2 * i - 1 : 2 * i, 2 * i, 2 * i + 1,
                (i + 1 < size.x) ? 2 * i + 2 : 2 * i + 1
            };
            double sum = 0.0;
            for (size_t y = 0; y < 4; ++y)
            {
                for (size_t x = 0; x < 4; ++x)
                {
                    sum += RESTRICTION_KERNEL[x] * RESTRICTION_KERNEL[y] *
                           finer(iIndices[x], jIndices[y]);
                }
            }
            (*coarser)(i, j) = sum;
        }
    }
}

void CorrectPoint(const FDMVector2& coarser, FDMVector2* finer,
                  const Vector2UZ& size, size_t i, size_t j)
{
    const size_t ci = i / 2;
    const size_t cj = j / 2;
    const std::array<size_t, 2> iIndices = {
        (i % 2 == 0 && i > 1) ? ci - 1 : ci,
        (i % 2 == 0 || i + 1 >= size.x) ? ci : ci + 1
    };
    const std::array<size_t, 2> jIndices = {
        (j % 2 == 0 && j > 1) ? cj - 1 : cj,
        (j % 2 == 0 || j + 1 >= size.y) ? cj : cj + 1
    };
    const std::array<double, 2> iWeights =
        (i % 2 == 0) ? std::array{ 0.25, 0.75 } : std::array{ 0.75, 0.25 };
    const std::array<double, 2> jWeights =
        (j % 2 == 0) ? std::array{ 0.25, 0.75 } : std::array{ 0.75, 0.25 };

    for (size_t y = 0; y < 2; ++y)
    {
        for (size_t x = 0; x < 2; ++x)
        {
            (*finer)(i, j) +=
                iWeights[x] * jWeights[y] * coarser(iIndices[x], jIndices[y]);
        }
    }
}
}  // namespace

void FDMMGLinearSystem2::Clear()
{
    A.levels.clear();
    x.levels.clear();
    b.levels.clear();
}

size_t FDMMGLinearSystem2::GetNumberOfLevels() const
{
    return A.levels.size();
}

void FDMMGLinearSystem2::ResizeWithCoarsest(const Vector2UZ& coarsestResolution,
                                            size_t numberOfLevels)
{
    FDMMGUtils2::ResizeArrayWithCoarsest(coarsestResolution, numberOfLevels,
                                         &A.levels);
    FDMMGUtils2::ResizeArrayWithCoarsest(coarsestResolution, numberOfLevels,
                                         &x.levels);
    FDMMGUtils2::ResizeArrayWithCoarsest(coarsestResolution, numberOfLevels,
                                         &b.levels);
}

void FDMMGLinearSystem2::ResizeWithFinest(const Vector2UZ& finestResolution,
                                          size_t maxNumberOfLevels)
{
    FDMMGUtils2::ResizeArrayWithFinest(finestResolution, maxNumberOfLevels,
                                       &A.levels);
    FDMMGUtils2::ResizeArrayWithFinest(finestResolution, maxNumberOfLevels,
                                       &x.levels);
    FDMMGUtils2::ResizeArrayWithFinest(finestResolution, maxNumberOfLevels,
                                       &b.levels);
}

void FDMMGUtils2::Restrict(const FDMVector2& finer, FDMVector2* coarser)
{
    assert(finer.Size().x == 2 * coarser->Size().x);
    assert(finer.Size().y == 2 * coarser->Size().y);

    // --*--|--*--|--*--|--*--
    //  1/8   3/8   3/8   1/8
    //           to
    // -----|-----*-----|-----
    const Vector2UZ n = coarser->Size();
    ParallelRangeFor(ZERO_SIZE, n.x, ZERO_SIZE, n.y,
                     [&n, &finer, &coarser](size_t iBegin, size_t iEnd,
                                            size_t jBegin, size_t jEnd) {
                         RestrictRange(finer, coarser, n, iBegin, iEnd, jBegin,
                                       jEnd);
                     });
}

void FDMMGUtils2::Correct(const FDMVector2& coarser, FDMVector2* finer)
{
    assert(finer->Size().x == 2 * coarser.Size().x);
    assert(finer->Size().y == 2 * coarser.Size().y);

    // -----|-----*-----|-----
    //           to
    //  1/4   3/4   3/4   1/4
    // --*--|--*--|--*--|--*--
    const Vector2UZ n = finer->Size();
    ParallelRangeFor(ZERO_SIZE, n.x, ZERO_SIZE, n.y,
                     [&n, &coarser, &finer](size_t iBegin, size_t iEnd,
                                            size_t jBegin, size_t jEnd) {
                         for (size_t j = jBegin; j < jEnd; ++j)
                         {
                             for (size_t i = iBegin; i < iEnd; ++i)
                             {
                                 CorrectPoint(coarser, finer, n, i, j);
                             }
                         }
                     });
}
}  // namespace CubbyFlow
