// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_ITERATION_UTILS_IMPL_HPP
#define CUBBYFLOW_ITERATION_UTILS_IMPL_HPP

namespace CubbyFlow
{
namespace Internal
{
template <typename IndexType, size_t N, size_t I>
struct ForEachIndex
{
    template <typename Func, typename... RemainingIndices>
    static void Call(const Vector<IndexType, N>& begin,
                     const Vector<IndexType, N>& end, const Func& func,
                     RemainingIndices... indices)
    {
        for (IndexType i = begin[I - 1]; i < end[I - 1]; ++i)
        {
            ForEachIndex<IndexType, N, I - 1>::Call(begin, end, func, i,
                                                    indices...);
        }
    }
};

template <typename IndexType, size_t N>
struct ForEachIndex<IndexType, N, 1>
{
    template <typename Func, typename... RemainingIndices>
    static void Call(const Vector<IndexType, N>& begin,
                     const Vector<IndexType, N>& end, const Func& func,
                     RemainingIndices... indices)
    {
        for (IndexType i = begin[0]; i < end[0]; ++i)
        {
            func(i, indices...);
        }
    }
};
}  // namespace Internal

template <typename IndexType, size_t N, typename Func>
void ForEachIndex(const Vector<IndexType, N>& begin,
                  const Vector<IndexType, N>& end, const Func& func)
{
    for (IndexType i = begin[N - 1]; i < end[N - 1]; ++i)
    {
        Internal::ForEachIndex<IndexType, N, N - 1>::Call(begin, end, func, i);
    }
}

template <typename IndexType, typename Func>
void ForEachIndex(const Vector<IndexType, 1>& begin,
                  const Vector<IndexType, 1>& end, const Func& func)
{
    for (IndexType i = begin[0]; i < end[0]; ++i)
    {
        func(i);
    }
}

template <typename IndexType, typename Func>
void ForEachIndex(IndexType begin, IndexType end, const Func& func)
{
    for (IndexType i = begin; i < end; ++i)
    {
        func(i);
    }
}

template <typename IndexType, size_t N, typename Func>
void ForEachIndex(const Vector<IndexType, N>& size, const Func& func)
{
    ForEachIndex(Vector<IndexType, N>{}, size, func);
}

template <typename IndexType, typename Func>
void ForEachIndex(const Vector<IndexType, 1>& size, const Func& func)
{
    ForEachIndex(Vector<IndexType, 1>{}, size, func);
}

template <typename IndexType, typename Func>
void ForEachIndex(IndexType size, const Func& func)
{
    ForEachIndex(IndexType{}, size, func);
}

template <typename IndexType, size_t N, typename Func>
void ParallelForEachIndex(const Vector<IndexType, N>& begin,
                          const Vector<IndexType, N>& end, const Func& func,
                          ExecutionPolicy policy)
{
    ParallelFor(
        begin[N - 1], end[N - 1],
        [&begin, &end, &func](IndexType i) {
            Internal::ForEachIndex<IndexType, N, N - 1>::Call(begin, end, func,
                                                              i);
        },
        policy);
}

template <typename IndexType, typename Func>
void ParallelForEachIndex(const Vector<IndexType, 1>& begin,
                          const Vector<IndexType, 1>& end, const Func& func,
                          ExecutionPolicy policy)
{
    ParallelFor(begin[0], end[0], func, policy);
}

template <typename IndexType, typename Func>
void ParallelForEachIndex(IndexType begin, IndexType end, const Func& func,
                          ExecutionPolicy policy)
{
    ParallelFor(begin, end, func, policy);
}

template <typename IndexType, size_t N, typename Func>
void ParallelForEachIndex(const Vector<IndexType, N>& size, const Func& func,
                          ExecutionPolicy policy)
{
    ParallelForEachIndex(Vector<IndexType, N>{}, size, func, policy);
}

template <typename IndexType, typename Func>
void ParallelForEachIndex(const Vector<IndexType, 1>& size, const Func& func,
                          ExecutionPolicy policy)
{
    ParallelForEachIndex(Vector<IndexType, 1>{}, size, func, policy);
}

template <typename IndexType, typename Func>
void ParallelForEachIndex(IndexType size, const Func& func,
                          ExecutionPolicy policy)
{
    ParallelForEachIndex(IndexType{}, size, func, policy);
}
}  // namespace CubbyFlow

#endif