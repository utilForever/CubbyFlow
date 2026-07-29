#ifndef CUBBYFLOW_FDM_LINEAR_SYSTEM_SOLVER_TEST_HELPER3_HPP
#define CUBBYFLOW_FDM_LINEAR_SYSTEM_SOLVER_TEST_HELPER3_HPP

#include <Core/Array/ArrayView.hpp>
#include <Core/Solver/FDM/FDMLinearSystemSolver3.hpp>

#include <vector>

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
    struct CompressedRow
    {
        explicit CompressedRow(size_t centerIndex) : columns{ centerIndex }
        {
            // Do nothing
        }

        void AddNeighbor(size_t index)
        {
            values[0] += 1.0;
            values.push_back(-1.0);
            columns.push_back(index);
        }

        double rhs = 0.0;
        std::vector<double> values{ 0.0 };
        std::vector<size_t> columns;
    };

    static void AppendXNeighbors(const Vector3UZ& size,
                                 const ArrayView3<size_t>& acc,
                                 const Vector3UZ& index, CompressedRow* row)
    {
        if (index.x > 0)
        {
            row->AddNeighbor(acc.Index(index.x - 1, index.y, index.z));
        }

        if (index.x + 1 < size.x)
        {
            row->AddNeighbor(acc.Index(index.x + 1, index.y, index.z));
        }
    }

    static void AppendYNeighbors(const Vector3UZ& size,
                                 const ArrayView3<size_t>& acc,
                                 const Vector3UZ& index, CompressedRow* row)
    {
        if (index.y > 0)
        {
            row->AddNeighbor(acc.Index(index.x, index.y - 1, index.z));
        }
        else
        {
            row->rhs += 1.0;
        }

        if (index.y + 1 < size.y)
        {
            row->AddNeighbor(acc.Index(index.x, index.y + 1, index.z));
        }
        else
        {
            row->rhs -= 1.0;
        }
    }

    static void AppendZNeighbors(const Vector3UZ& size,
                                 const ArrayView3<size_t>& acc,
                                 const Vector3UZ& index, CompressedRow* row)
    {
        if (index.z > 0)
        {
            row->AddNeighbor(acc.Index(index.x, index.y, index.z - 1));
        }
        else
        {
            row->rhs += 1.0;
        }

        if (index.z + 1 < size.z)
        {
            row->AddNeighbor(acc.Index(index.x, index.y, index.z + 1));
        }
        else
        {
            row->rhs -= 1.0;
        }
    }

    static void BuildCompressedRow(FDMCompressedLinearSystem3* system,
                                   const Vector3UZ& size,
                                   const ArrayView3<size_t>& acc,
                                   Array3<size_t>* coordToIndex, size_t i,
                                   size_t j, size_t k)
    {
        const Vector3UZ index{ i, j, k };
        const size_t centerIndex = acc.Index(index);
        (*coordToIndex)[centerIndex] = system->b.GetRows();

        CompressedRow row(centerIndex);
        AppendXNeighbors(size, acc, index, &row);
        AppendYNeighbors(size, acc, index, &row);
        AppendZNeighbors(size, acc, index, &row);

        system->A.AddRow(row.values, row.columns);
        system->b.AddElement(row.rhs);
    }
};
}  // namespace CubbyFlow

#endif
