// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_PARALLEL_HPP
#define CUBBYFLOW_PARALLEL_HPP

#include <iterator>

namespace CubbyFlow
{
//! Execution policy tag.
enum class ExecutionPolicy
{
    Serial,
    Parallel
};

//!
//! \brief      Fills from \p begin to \p end with \p value in parallel.
//!
//! This function fills a container specified by begin and end iterators in
//! parallel. The order of the filling is not guaranteed due to the nature of
//! parallel execution.
//!
//! \param[in]  begin          The begin iterator of a container.
//! \param[in]  end            The end iterator of a container.
//! \param[in]  value          The value to fill a container.
//! \param[in]  policy         The execution policy (parallel or serial).
//!
//! \tparam     RandomIterator Random iterator type.
//! \tparam     T              Value type of a container.
//!
template <typename RandomIterator, typename T>
void ParallelFill(const RandomIterator& begin, const RandomIterator& end,
                  const T& value,
                  ExecutionPolicy policy = ExecutionPolicy::Parallel);

//!
//! \brief      Makes a for-loop from \p beginIndex \p to endIndex in parallel.
//!
//! This function makes a for-loop specified by begin and end indices in
//! parallel. The order of the visit is not guaranteed due to the nature of
//! parallel execution.
//!
//! \param[in]  beginIndex The begin index.
//! \param[in]  endIndex   The end index.
//! \param[in]  function   The function to call for each index.
//! \param[in]  policy     The execution policy (parallel or serial).
//!
//! \tparam     IndexType  Index type.
//! \tparam     Function   Function type.
//!
template <typename IndexType, typename Function>
void ParallelFor(IndexType beginIndex, IndexType endIndex,
                 const Function& function,
                 ExecutionPolicy policy = ExecutionPolicy::Parallel);

//!
//! \brief      Makes a range-loop from \p beginIndex \p to endIndex in
//!             parallel.
//!
//! This function makes a for-loop specified by begin and end indices in
//! parallel. Unlike parallelFor function, the input function object takes range
//! instead of single index. The order of the visit is not guaranteed due to the
//! nature of parallel execution.
//!
//! \param[in]  beginIndex The begin index.
//! \param[in]  endIndex   The end index.
//! \param[in]  function   The function to call for each index range.
//! \param[in]  policy     The execution policy (parallel or serial).
//!
//! \tparam     IndexType  Index type.
//! \tparam     Function   Function type.
//!
template <typename IndexType, typename Function>
void ParallelRangeFor(IndexType beginIndex, IndexType endIndex,
                      const Function& function,
                      ExecutionPolicy policy = ExecutionPolicy::Parallel);

//!
//! \brief      Makes a 2D nested for-loop in parallel.
//!
//! This function makes a 2D nested for-loop specified by begin and end indices
//! for each dimension. X will be the inner-most loop while Y is the outer-most.
//! The order of the visit is not guaranteed due to the nature of parallel
//! execution.
//!
//! \param[in]  beginIndexX The begin index in X dimension.
//! \param[in]  endIndexX   The end index in X dimension.
//! \param[in]  beginIndexY The begin index in Y dimension.
//! \param[in]  endIndexY   The end index in Y dimension.
//! \param[in]  function    The function to call for each index (i, j).
//! \param[in]  policy      The execution policy (parallel or serial).
//!
//! \tparam     IndexType   Index type.
//! \tparam     Function    Function type.
//!
template <typename IndexType, typename Function>
void ParallelFor(IndexType beginIndexX, IndexType endIndexX,
                 IndexType beginIndexY, IndexType endIndexY,
                 const Function& function,
                 ExecutionPolicy policy = ExecutionPolicy::Parallel);

//!
//! \brief      Makes a 2D nested range-loop in parallel.
//!
//! This function makes a 2D nested for-loop specified by begin and end indices
//! for each dimension. X will be the inner-most loop while Y is the outer-most.
//! Unlike parallelFor function, the input function object takes range instead
//! of single index. The order of the visit is not guaranteed due to the nature
//! of parallel execution.
//!
//! \param[in]  beginIndexX The begin index in X dimension.
//! \param[in]  endIndexX   The end index in X dimension.
//! \param[in]  beginIndexY The begin index in Y dimension.
//! \param[in]  endIndexY   The end index in Y dimension.
//! \param[in]  function    The function to call for each index range.
//! \param[in]  policy      The execution policy (parallel or serial).
//!
//! \tparam     IndexType  Index type.
//! \tparam     Function   Function type.
//!
template <typename IndexType, typename Function>
void ParallelRangeFor(IndexType beginIndexX, IndexType endIndexX,
                      IndexType beginIndexY, IndexType endIndexY,
                      const Function& function,
                      ExecutionPolicy policy = ExecutionPolicy::Parallel);

//!
//! \brief      Makes a 3D nested for-loop in parallel.
//!
//! This function makes a 3D nested for-loop specified by begin and end indices
//! for each dimension. X will be the inner-most loop while Z is the outer-most.
//! The order of the visit is not guaranteed due to the nature of parallel
//! execution.
//!
//! \param[in]  beginIndexX The begin index in X dimension.
//! \param[in]  endIndexX   The end index in X dimension.
//! \param[in]  beginIndexY The begin index in Y dimension.
//! \param[in]  endIndexY   The end index in Y dimension.
//! \param[in]  beginIndexZ The begin index in Z dimension.
//! \param[in]  endIndexZ   The end index in Z dimension.
//! \param[in]  function    The function to call for each index (i, j, k).
//! \param[in]  policy      The execution policy (parallel or serial).
//!
//! \tparam     IndexType   Index type.
//! \tparam     Function    Function type.
//!
template <typename IndexType, typename Function>
void ParallelFor(IndexType beginIndexX, IndexType endIndexX,
                 IndexType beginIndexY, IndexType endIndexY,
                 IndexType beginIndexZ, IndexType endIndexZ,
                 const Function& function,
                 ExecutionPolicy policy = ExecutionPolicy::Parallel);

//!
//! \brief      Makes a 3D nested range-loop in parallel.
//!
//! This function makes a 3D nested for-loop specified by begin and end indices
//! for each dimension. X will be the inner-most loop while Z is the outer-most.
//! Unlike parallelFor function, the input function object takes range instead
//! of single index. The order of the visit is not guaranteed due to the nature
//! of parallel execution.
//!
//! \param[in]  beginIndexX The begin index in X dimension.
//! \param[in]  endIndexX   The end index in X dimension.
//! \param[in]  beginIndexY The begin index in Y dimension.
//! \param[in]  endIndexY   The end index in Y dimension.
//! \param[in]  beginIndexZ The begin index in Z dimension.
//! \param[in]  endIndexZ   The end index in Z dimension.
//! \param[in]  function    The function to call for each index (i, j, k).
//! \param[in]  policy      The execution policy (parallel or serial).
//!
//! \tparam     IndexType   Index type.
//! \tparam     Function    Function type.
//!
template <typename IndexType, typename Function>
void ParallelRangeFor(IndexType beginIndexX, IndexType endIndexX,
                      IndexType beginIndexY, IndexType endIndexY,
                      IndexType beginIndexZ, IndexType endIndexZ,
                      const Function& function,
                      ExecutionPolicy policy = ExecutionPolicy::Parallel);

//!
//! \brief      Performs reduce operation in parallel.
//!
//! This function reduces the series of values into a single value using the
//! provided reduce function.
//!
//! \param[in]  beginIndex The begin index.
//! \param[in]  endIndex   The end index.
//! \param[in]  identity   Identity value for the reduce operation.
//! \param[in]  function   The function for reducing subrange.
//! \param[in]  reduce     The reduce operator.
//! \param[in]  policy     The execution policy (parallel or serial).
//!
//! \tparam     IndexType  Index type.
//! \tparam     Value      Value type.
//! \tparam     Function   Reduce function type.
//!
template <typename IndexType, typename Value, typename Function,
          typename Reduce>
Value ParallelReduce(IndexType beginIndex, IndexType endIndex,
                     const Value& identity, const Function& function,
                     const Reduce& reduce,
                     ExecutionPolicy policy = ExecutionPolicy::Parallel);

//!
//! \brief      Sorts a container in parallel.
//!
//! This function sorts a container specified by begin and end iterators.
//!
//! \param[in]  begin          The begin random access iterator.
//! \param[in]  end            The end random access iterator.
//! \param[in]  policy         The execution policy (parallel or serial).
//!
//! \tparam     RandomIterator Iterator type.
//!
template <std::random_access_iterator RandomIterator>
void ParallelSort(RandomIterator begin, RandomIterator end,
                  ExecutionPolicy policy = ExecutionPolicy::Parallel);

//!
//! \brief      Sorts a container in parallel with a custom compare function.
//!
//! This function sorts a container specified by begin and end iterators. It
//! takes extra compare function which returns true if the first argument is
//! less than the second argument.
//!
//! \param[in]  begin           The begin random access iterator.
//! \param[in]  end             The end random access iterator.
//! \param[in]  compare         The compare function.
//! \param[in]  policy          The execution policy (parallel or serial).
//!
//! \tparam     RandomIterator  Iterator type.
//! \tparam     CompareFunction Compare function type.
//!
template <std::random_access_iterator RandomIterator, typename CompareFunction>
void ParallelSort(RandomIterator begin, RandomIterator end,
                  CompareFunction compare,
                  ExecutionPolicy policy = ExecutionPolicy::Parallel);

//! Sets maximum number of threads to use.
void SetMaxNumberOfThreads(unsigned int numThreads);

//! Returns maximum number of threads to use.
unsigned int GetMaxNumberOfThreads();
}  // namespace CubbyFlow

#include <Core/Utils/Parallel-Impl.hpp>

#endif
