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
            const size_t cIdx = acc.Index(i, j, k);

            coordToIndex[cIdx] = system->b.GetRows();
            double bijk = 0.0;

            std::vector<double> row(1, 0.0);
            std::vector<size_t> colIdx(1, cIdx);

            if (i > 0)
            {
                const size_t lIdx = acc.Index(i - 1, j, k);
                row[0] += 1.0;
                row.push_back(-1.0);
                colIdx.push_back(lIdx);
            }
            if (i < size.x - 1)
            {
                const size_t rIdx = acc.Index(i + 1, j, k);
                row[0] += 1.0;
                row.push_back(-1.0);
                colIdx.push_back(rIdx);
            }

            if (j > 0)
            {
                const size_t dIdx = acc.Index(i, j - 1, k);
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
                const size_t uIdx = acc.Index(i, j + 1, k);
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
                const size_t bIdx = acc.Index(i, j, k - 1);
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
                const size_t fIdx = acc.Index(i, j, k + 1);
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
}  // namespace CubbyFlow

#endif