#ifndef CUBBYFLOW_FDM_LINEAR_SYSTEM_SOLVER_TEST_HELPER2_HPP
#define CUBBYFLOW_FDM_LINEAR_SYSTEM_SOLVER_TEST_HELPER2_HPP

#include <Core/Array/ArrayView.hpp>
#include <Core/Solver/FDM/FDMLinearSystemSolver2.hpp>

namespace CubbyFlow
{
class FDMLinearSystemSolverTestHelper2
{
 public:
    static void BuildTestLinearSystem(FDMLinearSystem2* system,
                                      const Vector2UZ& size)
    {
        system->A.Resize(size);
        system->x.Resize(size);
        system->b.Resize(size);

        ForEachIndex(system->A.Size(), [&system](size_t i, size_t j) {
            if (i > 0)
            {
                system->A(i, j).center += 1.0;
            }
            if (i < system->A.Width() - 1)
            {
                system->A(i, j).center += 1.0;
                system->A(i, j).right -= 1.0;
            }

            if (j > 0)
            {
                system->A(i, j).center += 1.0;
            }
            else
            {
                system->b(i, j) += 1.0;
            }

            if (j < system->A.Height() - 1)
            {
                system->A(i, j).center += 1.0;
                system->A(i, j).up -= 1.0;
            }
            else
            {
                system->b(i, j) -= 1.0;
            }
        });
    }

    static void BuildTestCompressedLinearSystem(
        FDMCompressedLinearSystem2* system, const Vector2UZ& size)
    {
        Array2<size_t> coordToIndex(size);
        const auto acc = coordToIndex.View();

        ForEachIndex(coordToIndex.Size(), [&acc, &coordToIndex, &system, &size](
                                              size_t i, size_t j) {
            BuildCompressedRow(system, size, acc, &coordToIndex, i, j);
        });

        system->x.Resize(system->b.GetRows(), 0.0);
    }

 private:
    static void BuildCompressedRow(FDMCompressedLinearSystem2* system,
                                   const Vector2UZ& size,
                                   const ArrayView2<size_t>& acc,
                                   Array2<size_t>* coordToIndex, size_t i,
                                   size_t j)
    {
        const size_t cIdx = acc.Index(i, j);
        (*coordToIndex)[cIdx] = system->b.GetRows();
        double bij = 0.0;
        std::vector<double> row(1, 0.0);
        std::vector<size_t> colIdx(1, cIdx);

        const auto addNeighbor = [&](size_t idx) {
            row[0] += 1.0;
            row.push_back(-1.0);
            colIdx.push_back(idx);
        };
        if (i > 0)
        {
            addNeighbor(acc.Index(i - 1, j));
        }
        if (i + 1 < size.x)
        {
            addNeighbor(acc.Index(i + 1, j));
        }
        if (j > 0)
        {
            addNeighbor(acc.Index(i, j - 1));
        }
        else
        {
            bij += 1.0;
        }
        if (j + 1 < size.y)
        {
            addNeighbor(acc.Index(i, j + 1));
        }
        else
        {
            bij -= 1.0;
        }

        system->A.AddRow(row, colIdx);
        system->b.AddElement(bij);
    }
};
}  // namespace CubbyFlow

#endif
