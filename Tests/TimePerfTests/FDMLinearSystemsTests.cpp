#include "benchmark/benchmark.h"

#include <Core/Array/ArrayView.hpp>
#include <Core/FDM/FDMLinearSystem2.hpp>
#include <Core/FDM/FDMLinearSystem3.hpp>

#include <random>

using CubbyFlow::Array3;
using CubbyFlow::FDMCompressedLinearSystem3;
using CubbyFlow::FDMMatrix2;
using CubbyFlow::FDMMatrix3;
using CubbyFlow::FDMVector2;
using CubbyFlow::FDMVector3;
using CubbyFlow::Vector3UZ;

class FDMBLAS2 : public ::benchmark::Fixture
{
 public:
    FDMMatrix2 m;
    FDMVector2 a;
    FDMVector2 b;

    void SetUp(const ::benchmark::State& state)
    {
        const auto dim = static_cast<size_t>(state.range(0));

        m.Resize(dim, dim);
        a.Resize(dim, dim);
        b.Resize(dim, dim);

        std::mt19937 rng;
        std::uniform_real_distribution<> d(0.0, 1.0);

        ForEachIndex(m.Size(), [this, &d, &rng](size_t i, size_t j) {
            m(i, j).center = d(rng);
            m(i, j).right = d(rng);
            m(i, j).up = d(rng);
            a(i, j) = d(rng);
        });
    }
};

class FDMBLAS3 : public ::benchmark::Fixture
{
 public:
    FDMMatrix3 m;
    FDMVector3 a;
    FDMVector3 b;

    void SetUp(const ::benchmark::State& state)
    {
        const auto dim = static_cast<size_t>(state.range(0));

        m.Resize(dim, dim, dim);
        a.Resize(dim, dim, dim);
        b.Resize(dim, dim, dim);

        std::mt19937 rng;
        std::uniform_real_distribution<> d(0.0, 1.0);

        ForEachIndex(m.Size(), [this, &d, &rng](size_t i, size_t j, size_t k) {
            m(i, j, k).center = d(rng);
            m(i, j, k).right = d(rng);
            m(i, j, k).up = d(rng);
            m(i, j, k).front = d(rng);
            a(i, j, k) = d(rng);
        });
    }
};

class FDMCompressedBLAS3 : public ::benchmark::Fixture
{
 public:
    FDMCompressedLinearSystem3 system;

    void SetUp(const ::benchmark::State& state)
    {
        const auto dim = static_cast<size_t>(state.range(0));

        BuildSystem(&system, { dim, dim, dim });
    }

    static void BuildSystem(FDMCompressedLinearSystem3* system,
                            const Vector3UZ& size)
    {
        system->Clear();

        Array3<size_t> coordToIndex(size);
        const auto acc = coordToIndex.View();

        ForEachIndex(coordToIndex.Size(), [&acc, &coordToIndex, &size, &system](
                                              size_t i, size_t j, size_t k) {
            const size_t cIdx = acc.Index(i, j, k);
            const size_t lIdx = acc.Index(i - 1, j, k);
            const size_t rIdx = acc.Index(i + 1, j, k);
            const size_t dIdx = acc.Index(i, j - 1, k);
            const size_t uIdx = acc.Index(i, j + 1, k);
            const size_t bIdx = acc.Index(i, j, k - 1);
            const size_t fIdx = acc.Index(i, j, k + 1);

            coordToIndex[cIdx] = system->b.GetRows();
            double bijk = 0.0;

            std::vector<double> row(1, 0.0);
            std::vector<size_t> colIdx(1, cIdx);

            if (i > 0)
            {
                row[0] += 1.0;
                row.push_back(-1.0);
                colIdx.push_back(lIdx);
            }
            if (i < size.x - 1)
            {
                row[0] += 1.0;
                row.push_back(-1.0);
                colIdx.push_back(rIdx);
            }

            if (j > 0)
            {
                row[0] += 1.0;
                row.push_back(-1.0);
                colIdx.push_back(dIdx);
            }
            else
            {
                bijk += 1.0;
            }

            if (j < size.y - 1)
            {
                row[0] += 1.0;
                row.push_back(-1.0);
                colIdx.push_back(uIdx);
            }
            else
            {
                bijk -= 1.0;
            }

            if (k > 0)
            {
                row[0] += 1.0;
                row.push_back(-1.0);
                colIdx.push_back(bIdx);
            }
            else
            {
                bijk += 1.0;
            }

            if (k < size.z - 1)
            {
                row[0] += 1.0;
                row.push_back(-1.0);
                colIdx.push_back(fIdx);
            }
            else
            {
                bijk -= 1.0;
            }

            system->A.AddRow(row, colIdx);
            system->b.AddElement(bijk);
        });

        system->x.Resize(system->b.GetRows(), 0.0);
    }
};

BENCHMARK_DEFINE_F(FDMBLAS2, MVM)(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        CubbyFlow::FDMBLAS2::MVM(m, a, &b);
    }
}

BENCHMARK_REGISTER_F(FDMBLAS2, MVM)->Arg(1 << 6)->Arg(1 << 8)->Arg(1 << 10);

BENCHMARK_DEFINE_F(FDMBLAS3, MVM)(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        CubbyFlow::FDMBLAS3::MVM(m, a, &b);
    }
}

BENCHMARK_REGISTER_F(FDMBLAS3, MVM)->Arg(1 << 4)->Arg(1 << 6)->Arg(1 << 8);

BENCHMARK_DEFINE_F(FDMCompressedBLAS3, MVM)(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        CubbyFlow::FDMCompressedBLAS3::MVM(system.A, system.b, &system.x);
    }
}

BENCHMARK_REGISTER_F(FDMCompressedBLAS3, MVM)
    ->Arg(1 << 4)
    ->Arg(1 << 6)
    ->Arg(1 << 8);
