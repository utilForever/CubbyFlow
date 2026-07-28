// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/FDM/FDMMGLinearSystem3.hpp>

#include <array>

namespace CubbyFlow
{
void FDMMGLinearSystem3::Clear()
{
    A.levels.clear();
    x.levels.clear();
    b.levels.clear();
}

size_t FDMMGLinearSystem3::GetNumberOfLevels() const
{
    return A.levels.size();
}

void FDMMGLinearSystem3::ResizeWithCoarsest(const Vector3UZ &coarsestResolution,
                                            size_t numberOfLevels)
{
    FDMMGUtils3::ResizeArrayWithCoarsest(coarsestResolution, numberOfLevels,
                                         &A.levels);
    FDMMGUtils3::ResizeArrayWithCoarsest(coarsestResolution, numberOfLevels,
                                         &x.levels);
    FDMMGUtils3::ResizeArrayWithCoarsest(coarsestResolution, numberOfLevels,
                                         &b.levels);
}

void FDMMGLinearSystem3::ResizeWithFinest(const Vector3UZ &finestResolution,
                                          size_t maxNumberOfLevels)
{
    FDMMGUtils3::ResizeArrayWithFinest(finestResolution, maxNumberOfLevels,
                                       &A.levels);
    FDMMGUtils3::ResizeArrayWithFinest(finestResolution, maxNumberOfLevels,
                                       &x.levels);
    FDMMGUtils3::ResizeArrayWithFinest(finestResolution, maxNumberOfLevels,
                                       &b.levels);
}

void FDMMGUtils3::Restrict(const FDMVector3 &finer, FDMVector3 *coarser)
{
    assert(finer.Size().x == 2 * coarser->Size().x);
    assert(finer.Size().y == 2 * coarser->Size().y);
    assert(finer.Size().z == 2 * coarser->Size().z);

    // --*--|--*--|--*--|--*--
    //  1/8   3/8   3/8   1/8
    //           to
    // -----|-----*-----|-----
    static const std::array<double, 4> kernel = { { 0.125, 0.375, 0.375,
                                                    0.125 } };

    const Vector3UZ n = coarser->Size();
    ParallelRangeFor(
        ZERO_SIZE, n.x, ZERO_SIZE, n.y, ZERO_SIZE, n.z,
        [&n, &finer, &coarser](size_t iBegin, size_t iEnd, size_t jBegin,
                               size_t jEnd, size_t kBegin, size_t kEnd) {
            std::array<size_t, 4> kIndices{};

            for (size_t k = kBegin; k < kEnd; ++k)
            {
                kIndices[0] = (k > 0) ? 2 * k - 1 : 2 * k;
                kIndices[1] = 2 * k;
                kIndices[2] = 2 * k + 1;
                kIndices[3] = (k + 1 < n.z) ? 2 * k + 2 : 2 * k + 1;

                std::array<size_t, 4> jIndices{};

                for (size_t j = jBegin; j < jEnd; ++j)
                {
                    jIndices[0] = (j > 0) ? 2 * j - 1 : 2 * j;
                    jIndices[1] = 2 * j;
                    jIndices[2] = 2 * j + 1;
                    jIndices[3] = (j + 1 < n.y) ? 2 * j + 2 : 2 * j + 1;

                    std::array<size_t, 4> iIndices{};
                    for (size_t i = iBegin; i < iEnd; ++i)
                    {
                        iIndices[0] = (i > 0) ? 2 * i - 1 : 2 * i;
                        iIndices[1] = 2 * i;
                        iIndices[2] = 2 * i + 1;
                        iIndices[3] = (i + 1 < n.x) ? 2 * i + 2 : 2 * i + 1;

                        double sum = 0.0;
                        for (size_t z = 0; z < 4; ++z)
                        {
                            for (size_t y = 0; y < 4; ++y)
                            {
                                for (size_t x = 0; x < 4; ++x)
                                {
                                    const double w =
                                        kernel[x] * kernel[y] * kernel[z];
                                    sum += w * finer(iIndices[x], jIndices[y],
                                                     kIndices[z]);
                                }
                            }
                        }

                        (*coarser)(i, j, k) = sum;
                    }
                }
            }
        });
}

void FDMMGUtils3::Correct(const FDMVector3 &coarser, FDMVector3 *finer)
{
    assert(finer->Size().x == 2 * coarser.Size().x);
    assert(finer->Size().y == 2 * coarser.Size().y);
    assert(finer->Size().z == 2 * coarser.Size().z);

    // -----|-----*-----|-----
    //           to
    //  1/4   3/4   3/4   1/4
    // --*--|--*--|--*--|--*--
    const Vector3UZ n = finer->Size();
    ParallelRangeFor(
        ZERO_SIZE, n.x, ZERO_SIZE, n.y, ZERO_SIZE, n.z,
        [&n, &coarser, &finer](size_t iBegin, size_t iEnd, size_t jBegin,
                               size_t jEnd, size_t kBegin, size_t kEnd) {
            for (size_t k = kBegin; k < kEnd; ++k)
            {
                for (size_t j = jBegin; j < jEnd; ++j)
                {
                    for (size_t i = iBegin; i < iEnd; ++i)
                    {
                        std::array<size_t, 2> iIndices{};
                        std::array<size_t, 2> jIndices{};
                        std::array<size_t, 2> kIndices{};
                        std::array<double, 2> iWeights{};
                        std::array<double, 2> jWeights{};
                        std::array<double, 2> kWeights{};

                        const size_t ci = i / 2;
                        const size_t cj = j / 2;
                        const size_t ck = k / 2;

                        if (i % 2 == 0)
                        {
                            iIndices[0] = (i > 1) ? ci - 1 : ci;
                            iIndices[1] = ci;
                            iWeights[0] = 0.25;
                            iWeights[1] = 0.75;
                        }
                        else
                        {
                            iIndices[0] = ci;
                            iIndices[1] = (i + 1 < n.x) ? ci + 1 : ci;
                            iWeights[0] = 0.75;
                            iWeights[1] = 0.25;
                        }

                        if (j % 2 == 0)
                        {
                            jIndices[0] = (j > 1) ? cj - 1 : cj;
                            jIndices[1] = cj;
                            jWeights[0] = 0.25;
                            jWeights[1] = 0.75;
                        }
                        else
                        {
                            jIndices[0] = cj;
                            jIndices[1] = (j + 1 < n.y) ? cj + 1 : cj;
                            jWeights[0] = 0.75;
                            jWeights[1] = 0.25;
                        }

                        if (k % 2 == 0)
                        {
                            kIndices[0] = (k > 1) ? ck - 1 : ck;
                            kIndices[1] = ck;
                            kWeights[0] = 0.25;
                            kWeights[1] = 0.75;
                        }
                        else
                        {
                            kIndices[0] = ck;
                            kIndices[1] = (k + 1 < n.y) ? ck + 1 : ck;
                            kWeights[0] = 0.75;
                            kWeights[1] = 0.25;
                        }

                        for (size_t z = 0; z < 2; ++z)
                        {
                            for (size_t y = 0; y < 2; ++y)
                            {
                                for (size_t x = 0; x < 2; ++x)
                                {
                                    const double w =
                                        iWeights[x] * jWeights[y] *
                                        kWeights[z] *
                                        coarser(iIndices[x], jIndices[y],
                                                kIndices[z]);
                                    (*finer)(i, j, k) += w;
                                }
                            }
                        }
                    }
                }
            }
        });
}
}  // namespace CubbyFlow