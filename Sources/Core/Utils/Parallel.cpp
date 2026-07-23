// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Utils/Parallel.hpp>

#if defined(CUBBYFLOW_TASKING_TBB)
#include <tbb/task_arena.h>
#include <tbb/task_scheduler_init.h>
#elif defined(CUBBYFLOW_TASKING_OPENMP)
#include <omp.h>
#endif

#include <memory>
#include <thread>

static unsigned int MAX_NUMBER_OF_THREADS = std::thread::hardware_concurrency();

namespace CubbyFlow
{
void SetMaxNumberOfThreads(unsigned int numThreads)
{
#if defined(CUBBYFLOW_TASKING_TBB)
    static std::unique_ptr<tbb::task_scheduler_init> tbbInit;

    if (!tbbInit.get())
    {
        tbbInit = std::make_unique<tbb::task_scheduler_init>(numThreads);
    }
    else
    {
        tbbInit->terminate();
        tbbInit->initialize(numThreads);
    }
#elif defined(CUBBYFLOW_TASKING_OPENMP)
    omp_set_num_threads(numThreads);
#endif
    MAX_NUMBER_OF_THREADS = std::max(numThreads, 1u);
}

unsigned int GetMaxNumberOfThreads()
{
    return MAX_NUMBER_OF_THREADS;
}
}  // namespace CubbyFlow
