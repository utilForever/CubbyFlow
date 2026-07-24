// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_PARALLEL_IMPL_HPP
#define CUBBYFLOW_PARALLEL_IMPL_HPP

#include <Core/Utils/Constants.hpp>
#include <Core/Utils/Parallel.hpp>

#if defined(CUBBYFLOW_TASKING_HPX)
#include <hpx/include/future.hpp>
#include <hpx/include/parallel_fill.hpp>
#include <hpx/include/parallel_for_each.hpp>
#include <hpx/include/parallel_for_loop.hpp>
#include <hpx/include/parallel_reduce.hpp>
#include <hpx/include/parallel_sort.hpp>
#endif

#if defined(CUBBYFLOW_TASKING_TBB)
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/parallel_sort.h>
#include <tbb/task_arena.h>
#elif defined(CUBBYFLOW_TASKING_CPP11THREAD)
#include <thread>
#endif

#include <algorithm>
#include <cmath>
#include <future>
#include <vector>

#undef max
#undef min

namespace CubbyFlow
{
namespace Internal
{
#if defined(CUBBYFLOW_TASKING_HPX)
template <typename Task>
using future = hpx::future<Task>;
#else
template <typename Task>
using future = std::future<Task>;
#endif

template <typename TASK>
using operator_return_t = typename std::invoke_result_t<TASK>;

template <typename TASK>
inline auto Async(TASK&& fn) -> future<operator_return_t<TASK>>
{
#if defined(CUBBYFLOW_TASKING_HPX)
    return hpx::async(std::forward<TASK>(fn));
#elif defined(CUBBYFLOW_TASKING_TBB)
    using package_t = std::packaged_task<operator_return_t<TASK>()>;
    package_t task(std::forward<TASK>(fn));
    auto result = task.get_future();

    struct LocalTBBTask
    {
        // TBB invokes queued functors through a const call operator.
        mutable package_t task;

        void operator()() const
        {
            task();
        }
    };

    tbb::task_arena arena{ tbb::task_arena::attach{} };
    arena.enqueue(LocalTBBTask{ std::move(task) });

    return result;
#elif defined(CUBBYFLOW_TASKING_CPP11THREAD)
    return std::async(std::launch::async, std::forward<TASK>(fn));
#else
    return std::async(std::launch::deferred, std::forward<TASK>(fn));
#endif
}

// Adopted from:
// Radenski, A.
// Shared Memory, Message Passing, and Hybrid Merge Sorts for Standalone and
// Clustered SMPs. Proc PDPTA'11, the  2011 International Conference on Parallel
// and Distributed Processing Techniques and Applications, CSREA Press
// (H. Arabnia, Ed.), 2011, pp. 367 - 373.
template <typename RandomIterator, typename RandomIterator2,
          typename CompareFunction>
void Merge(RandomIterator a, size_t size, RandomIterator2 temp,
           CompareFunction compareFunction)
{
    size_t i1 = 0;
    size_t i2 = size / 2;
    size_t tempi = 0;

    while (i1 < size / 2 && i2 < size)
    {
        if (compareFunction(a[i1], a[i2]))
        {
            temp[tempi] = a[i1];
            i1++;
        }
        else
        {
            temp[tempi] = a[i2];
            i2++;
        }

        tempi++;
    }

    while (i1 < size / 2)
    {
        temp[tempi] = a[i1];
        i1++;
        tempi++;
    }

    while (i2 < size)
    {
        temp[tempi] = a[i2];
        i2++;
        tempi++;
    }

    // Copy sorted temp array into main array, a
    ParallelFor(ZERO_SIZE, size, [&](size_t i) { a[i] = temp[i]; });
}

template <typename RandomIterator, typename RandomIterator2,
          typename CompareFunction>
void ParallelMergeSort(RandomIterator a, size_t size, RandomIterator2 temp,
                       unsigned int numThreads, CompareFunction compareFunction)
{
    if (numThreads == 1)
    {
        std::sort(a, a + size, compareFunction);
    }
    else if (numThreads > 1)
    {
        std::vector<future<void>> pool;
        pool.reserve(2);

        auto launchRange = [compareFunction](RandomIterator begin, size_t k2,
                                             RandomIterator2 temp,
                                             unsigned int numThreads) {
            ParallelMergeSort(begin, k2, temp, numThreads, compareFunction);
        };

        pool.emplace_back(Internal::Async(
            [=]() { launchRange(a, size / 2, temp, numThreads / 2); }));

        pool.emplace_back(Internal::Async([=]() {
            launchRange(a + size / 2, size - size / 2, temp + size / 2,
                        numThreads - numThreads / 2);
        }));

        // Wait for jobs to finish
        for (auto& f : pool)
        {
            if (f.valid())
            {
                f.wait();
            }
        }

        Merge(a, size, temp, compareFunction);
    }
}
}  // namespace Internal

template <typename RandomIterator, typename T>
void ParallelFill(const RandomIterator& begin, const RandomIterator& end,
                  const T& value, ExecutionPolicy policy)
{
    auto diff = end - begin;
    if (diff <= 0)
    {
        return;
    }

#if defined(CUBBYFLOW_TASKING_HPX)
    hpx::parallel::fill(hpx::parallel::execution::par, begin, end, value);
#else
    size_t size = static_cast<size_t>(diff);
    ParallelFor(
        ZERO_SIZE, size, [begin, value](size_t i) { begin[i] = value; },
        policy);
#endif
}

// Adopted from http://ideone.com/Z7zldb
template <typename IndexType, typename Function>
void ParallelFor(IndexType beginIndex, IndexType endIndex,
                 const Function& function, ExecutionPolicy policy)
{
    if (beginIndex > endIndex)
    {
        return;
    }

    if (policy == ExecutionPolicy::Parallel)
    {
#if defined(CUBBYFLOW_TASKING_TBB)
        (void)policy;
        tbb::parallel_for(beginIndex, endIndex, function);
#elif defined(CUBBYFLOW_TASKING_HPX)
        (void)policy;
        hpx::parallel::for_loop(hpx::parallel::execution::par, beginIndex,
                                endIndex, function);
#elif defined(CUBBYFLOW_TASKING_CPP11THREAD)
        // Estimate number of threads in the pool
        const unsigned int numThreadsHint = GetMaxNumberOfThreads();
        const unsigned int numThreads =
            (numThreadsHint == 0u) ? 8u : numThreadsHint;

        // Size of a slice for the range functions
        IndexType n = endIndex - beginIndex + 1;
        IndexType slice = static_cast<IndexType>(
            std::round(n / static_cast<double>(numThreads)));
        slice = std::max(slice, IndexType(1));

        // [Helper] Inner loop
        auto launchRange = [&function](IndexType k1, IndexType k2) {
            for (IndexType k = k1; k < k2; ++k)
            {
                function(k);
            }
        };

        // Create pool and launch jobs
        std::vector<std::thread> pool;
        pool.reserve(numThreads);
        IndexType i1 = beginIndex;
        IndexType i2 = std::min(beginIndex + slice, endIndex);

        for (unsigned int i = 0; i + 1 < numThreads && i1 < endIndex; ++i)
        {
            pool.emplace_back(launchRange, i1, i2);
            i1 = i2;
            i2 = std::min(i2 + slice, endIndex);
        }

        if (i1 < endIndex)
        {
            pool.emplace_back(launchRange, i1, endIndex);
        }

        // Wait for jobs to finish
        for (std::thread& t : pool)
        {
            if (t.joinable())
            {
                t.join();
            }
        }
#else
        (void)policy;

#if defined(CUBBYFLOW_TASKING_OPENMP)
#pragma omp parallel for
#if defined(_MSC_VER) && !defined(__INTEL_COMPILER)
        for (ssize_t i = beginIndex; i < static_cast<ssize_t>(endIndex); ++i)
        {
#else   // !MSVC || Intel
        for (auto i = beginIndex; i < endIndex; ++i)
        {
#endif  // MSVC && !Intel
            function(i);
        }
#else   // CUBBYFLOW_TASKING_SERIAL
        for (auto i = beginIndex; i < endIndex; ++i)
        {
            function(i);
        }
#endif  // CUBBYFLOW_TASKING_OPENMP
#endif
    }
    else
    {
        for (auto i = beginIndex; i < endIndex; ++i)
        {
            function(i);
        }
    }
}

template <typename IndexType, typename Function>
void ParallelRangeFor(IndexType beginIndex, IndexType endIndex,
                      const Function& function, ExecutionPolicy policy)
{
    if (beginIndex > endIndex)
    {
        return;
    }

    if (policy == ExecutionPolicy::Parallel)
    {
#if defined(CUBBYFLOW_TASKING_TBB)
        tbb::parallel_for(
            tbb::blocked_range<IndexType>(beginIndex, endIndex),
            [&function](const tbb::blocked_range<IndexType>& range) {
                function(range.begin(), range.end());
            });
#else
        // Estimate number of threads in the pool
        const unsigned int numThreadsHint = GetMaxNumberOfThreads();
        const unsigned int numThreads =
            numThreadsHint == 0u ? 8u : numThreadsHint;

        // Size of a slice for the range functions
        IndexType n = endIndex - beginIndex + 1;
        IndexType slice = static_cast<IndexType>(
            std::round(n / static_cast<double>(numThreads)));
        slice = std::max(slice, IndexType(1));

        // Create pool and launch jobs
        std::vector<CubbyFlow::Internal::future<void>> pool;
        pool.reserve(numThreads);
        IndexType i1 = beginIndex;
        IndexType i2 = std::min(beginIndex + slice, endIndex);

        for (unsigned int i = 0; i + 1 < numThreads && i1 < endIndex; ++i)
        {
            pool.emplace_back(Internal::Async([=]() { function(i1, i2); }));
            i1 = i2;
            i2 = std::min(i2 + slice, endIndex);
        }

        if (i1 < endIndex)
        {
            pool.emplace_back(
                Internal::Async([=]() { function(i1, endIndex); }));
        }

        // Wait for jobs to finish
        for (auto& f : pool)
        {
            if (f.valid())
            {
                f.wait();
            }
        }
#endif
    }
    else
    {
        function(beginIndex, endIndex);
    }
}

template <typename IndexType, typename Function>
void ParallelFor(IndexType beginIndexX, IndexType endIndexX,
                 IndexType beginIndexY, IndexType endIndexY,
                 const Function& function, ExecutionPolicy policy)
{
    ParallelFor(
        beginIndexY, endIndexY,
        [&](IndexType j) {
            for (IndexType i = beginIndexX; i < endIndexX; ++i)
            {
                function(i, j);
            }
        },
        policy);
}

template <typename IndexType, typename Function>
void ParallelRangeFor(IndexType beginIndexX, IndexType endIndexX,
                      IndexType beginIndexY, IndexType endIndexY,
                      const Function& function, ExecutionPolicy policy)
{
    ParallelRangeFor(
        beginIndexY, endIndexY,
        [&](IndexType jBegin, IndexType jEnd) {
            function(beginIndexX, endIndexX, jBegin, jEnd);
        },
        policy);
}

template <typename IndexType, typename Function>
void ParallelFor(IndexType beginIndexX, IndexType endIndexX,
                 IndexType beginIndexY, IndexType endIndexY,
                 IndexType beginIndexZ, IndexType endIndexZ,
                 const Function& function, ExecutionPolicy policy)
{
    ParallelFor(
        beginIndexZ, endIndexZ,
        [&](IndexType k) {
            for (IndexType j = beginIndexY; j < endIndexY; ++j)
            {
                for (IndexType i = beginIndexX; i < endIndexX; ++i)
                {
                    function(i, j, k);
                }
            }
        },
        policy);
}

template <typename IndexType, typename Function>
void ParallelRangeFor(IndexType beginIndexX, IndexType endIndexX,
                      IndexType beginIndexY, IndexType endIndexY,
                      IndexType beginIndexZ, IndexType endIndexZ,
                      const Function& function, ExecutionPolicy policy)
{
    ParallelRangeFor(
        beginIndexZ, endIndexZ,
        [&](IndexType kBegin, IndexType kEnd) {
            function(beginIndexX, endIndexX, beginIndexY, endIndexY, kBegin,
                     kEnd);
        },
        policy);
}

template <typename IndexType, typename Value, typename Function,
          typename Reduce>
Value ParallelReduce(IndexType beginIndex, IndexType endIndex,
                     const Value& identity, const Function& function,
                     const Reduce& reduce, ExecutionPolicy policy)
{
    if (beginIndex > endIndex)
    {
        return identity;
    }

    if (policy == ExecutionPolicy::Parallel)
    {
#if defined(CUBBYFLOW_TASKING_TBB)
        return tbb::parallel_reduce(
            tbb::blocked_range<IndexType>(beginIndex, endIndex), identity,
            [&function](const tbb::blocked_range<IndexType>& range,
                        const Value& init) {
                return function(range.begin(), range.end(), init);
            },
            reduce);
#else
        // Estimate number of threads in the pool
        const unsigned int numThreadsHint = GetMaxNumberOfThreads();
        const unsigned int numThreads =
            (numThreadsHint == 0u) ? 8u : numThreadsHint;

        // Size of a slice for the range functions
        IndexType n = endIndex - beginIndex + 1;
        IndexType slice = static_cast<IndexType>(
            std::round(n / static_cast<double>(numThreads)));
        slice = std::max(slice, IndexType(1));

        // Results
        std::vector<Value> results(numThreads, identity);

        // [Helper] Inner loop
        auto launchRange = [&](IndexType k1, IndexType k2, unsigned int tid) {
            results[tid] = function(k1, k2, identity);
        };

        // Create pool and launch jobs
        std::vector<CubbyFlow::Internal::future<void>> pool;
        pool.reserve(numThreads);

        IndexType i1 = beginIndex;
        IndexType i2 = std::min(beginIndex + slice, endIndex);
        unsigned int threadID = 0;

        for (; threadID + 1 < numThreads && i1 < endIndex; ++threadID)
        {
            pool.emplace_back(
                Internal::Async([=]() { launchRange(i1, i2, threadID); }));

            i1 = i2;
            i2 = std::min(i2 + slice, endIndex);
        }

        if (i1 < endIndex)
        {
            pool.emplace_back(Internal::Async(
                [=]() { launchRange(i1, endIndex, threadID); }));
        }

        // Wait for jobs to finish
        for (auto& f : pool)
        {
            if (f.valid())
            {
                f.wait();
            }
        }

        // Gather
        Value finalResult = identity;
        for (const Value& val : results)
        {
            finalResult = reduce(val, finalResult);
        }

        return finalResult;
#endif
    }

    (void)reduce;
    return function(beginIndex, endIndex, identity);
}

#ifdef __CUDACC__
template <typename RandomIterator>
#else
template <std::random_access_iterator RandomIterator>
#endif
void ParallelSort(RandomIterator begin, RandomIterator end,
                  ExecutionPolicy policy)
{
    ParallelSort(
        begin, end,
        std::less<typename std::iterator_traits<RandomIterator>::value_type>(),
        policy);
}

#ifdef __CUDACC__
template <typename RandomIterator, typename CompareFunction>
#else
template <std::random_access_iterator RandomIterator, typename CompareFunction>
#endif
void ParallelSort(RandomIterator begin, RandomIterator end,
                  CompareFunction compareFunction, ExecutionPolicy policy)
{
    if (begin > end)
    {
        return;
    }

    if (policy == ExecutionPolicy::Parallel)
    {
#if defined(CUBBYFLOW_TASKING_HPX)
        hpx::parallel::sort(hpx::parallel::execution::par, begin, end,
                            compareFunction);
#elif defined(CUBBYFLOW_TASKING_TBB)
        tbb::parallel_sort(begin, end, compareFunction);
#else

        size_t size = static_cast<size_t>(end - begin);

        using value_type =
            typename std::iterator_traits<RandomIterator>::value_type;
        std::vector<value_type> temp(size);

        // Estimate number of threads in the pool
        const unsigned int numThreadsHint = GetMaxNumberOfThreads();
        const unsigned int numThreads =
            (numThreadsHint == 0u) ? 8u : numThreadsHint;

        Internal::ParallelMergeSort(begin, size, temp.begin(), numThreads,
                                    compareFunction);
#endif
    }
    else
    {
        std::sort(begin, end, compareFunction);
    }
}
}  // namespace CubbyFlow

#endif
