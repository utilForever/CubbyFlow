#ifndef CUBBYFLOW_FDM_LINEAR_SYSTEM_SOLVER_TEST_HELPER3_HPP
#define CUBBYFLOW_FDM_LINEAR_SYSTEM_SOLVER_TEST_HELPER3_HPP

#include <Core/Array/ArrayView.hpp>
#include <Core/Solver/FDM/FDMLinearSystemSolver3.hpp>

namespace CubbyFlow
{
class FDMLinearSystemSolverTestHelper3
{
 public:
    static void BuildTestLinearSystem(FDMLinearSystem3* system,
                                      const Vector3UZ& size)
    {
        system->A.Resize(size);
        system->x.Resize(size);
        system->b.Resize(size);

        ForEachIndex(system->A.Size(), [&system](size_t i, size_t j, size_t k) {
            if (i > 0)
            {
                system->A(i, j, k).center += 1.0;
            }
            if (i < system->A.Width() - 1)
            {
                system->A(i, j, k).center += 1.0;
                system->A(i, j, k).right -= 1.0;
            }

            if (j > 0)
            {
                system->A(i, j, k).center += 1.0;
            }
            else
            {
                system->b(i, j, k) += 1.0;
            }

            if (j < system->A.Height() - 1)
            {
                system->A(i, j, k).center += 1.0;
                system->A(i, j, k).up -= 1.0;
            }
            else
            {
                system->b(i, j, k) -= 1.0;
            }

            if (k > 0)
            {
                system->A(i, j, k).center += 1.0;
            }
            if (k < system->A.Depth() - 1)
            {
                system->A(i, j, k).center += 1.0;
                system->A(i, j, k).front -= 1.0;
            }
        });
    }

    static void BuildTestCompressedLinearSystem(
        FDMCompressedLinearSystem3* system, const Vector3UZ& size)
    {
        Array3<size_t> coordToIndex(size);
        const auto acc = coordToIndex.View();

        ForEachIndex(coordToIndex.Size(), [&acc, &coordToIndex, &system, &size](
                                              size_t i, size_t j, size_t k) {
            BuildCompressedRow(system, size, acc, &coordToIndex, i, j, k);
        });

        system->x.Resize(system->b.GetRows(), 0.0);
    }

 private:
    static void BuildCompressedRow(FDMCompressedLinearSystem3* system,
                                   const Vector3UZ& size,
                                   const ArrayView3<size_t>& acc,
                                   Array3<size_t>* coordToIndex, size_t i,
                                   size_t j, size_t k)
    {
        const size_t cIdx = acc.Index(i, j, k);
        (*coordToIndex)[cIdx] = system->b.GetRows();

        double bijk = 0.0;
        std::vector<double> row(1, 0.0);
        std::vector<size_t> colIdx(1, cIdx);

        const auto addNeighbor = [&](size_t idx) {
            row[0] += 1.0;
            row.push_back(-1.0);
            colIdx.push_back(idx);
        };

        if (i > 0)
        {
            addNeighbor(acc.Index(i - 1, j, k));
        }

        if (i + 1 < size.x)
        {
            addNeighbor(acc.Index(i + 1, j, k));
        }

        if (j > 0)
        {
            addNeighbor(acc.Index(i, j - 1, k));
        }
        else
        {
            bijk += 1.0;
        }

        if (j + 1 < size.y)
        {
            addNeighbor(acc.Index(i, j + 1, k));
        }
        else
        {
            bijk -= 1.0;
        }

        if (k > 0)
        {
            addNeighbor(acc.Index(i, j, k - 1));
        }
        else
        {
            bijk += 1.0;
        }

        if (k + 1 < size.z)
        {
            addNeighbor(acc.Index(i, j, k + 1));
        }
        else
        {
            bijk -= 1.0;
        }

        system->A.AddRow(row, colIdx);
        system->b.AddElement(bijk);
    }
};
}  // namespace CubbyFlow

#endif
