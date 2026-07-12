// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_MACROS_HPP
#define CUBBYFLOW_MACROS_HPP

#ifdef CUBBYFLOW_USE_CUDA

// Host vs. device
#if defined(__CUDACC__) || defined(__HIPCC__)
#define CUBBYFLOW_CUDA_DEVICE __device__
#define CUBBYFLOW_CUDA_HOST __host__
#else
#define CUBBYFLOW_CUDA_DEVICE
#define CUBBYFLOW_CUDA_HOST
#endif  // __CUDACC__ || __HIPCC__
#define CUBBYFLOW_CUDA_HOST_DEVICE CUBBYFLOW_CUDA_HOST CUBBYFLOW_CUDA_DEVICE

// Alignment
#if defined(__CUDACC__) || defined(__HIPCC__)  // NVCC or hipcc
#define CUBBYFLOW_CUDA_ALIGN(n) __align__(n)
#elif defined(__GNUC__)  // GCC
#define CUBBYFLOW_CUDA_ALIGN(n) __attribute__((aligned(n)))
#elif defined(_MSC_VER)  // MSVC
#define CUBBYFLOW_CUDA_ALIGN(n) __declspec(align(n))
#else
#error "Don't know how to handle CUBBYFLOW_CUDA_ALIGN"
#endif  // __CUDACC__ || __HIPCC__

// Exception
// The result is bound to a typed local first: HIP marks the runtime error type
// nodiscard, so feeding an API call straight into the comparison trips
// -Werror=unused-value under clang. Binding consumes the value explicitly.
#define _CUBBYFLOW_CUDA_CHECK(result, msg, file, line)                      \
    {                                                                       \
        cudaError_t _cubbyflow_cuda_err = (result);                         \
        if (_cubbyflow_cuda_err != cudaSuccess)                             \
        {                                                                   \
            fprintf(stderr, "CUDA error at %s:%d code=%d (%s) \"%s\" \n",    \
                    file, line,                                             \
                    static_cast<unsigned int>(_cubbyflow_cuda_err),         \
                    cudaGetErrorString(_cubbyflow_cuda_err), msg);          \
            static_cast<void>(cudaDeviceReset());                           \
            exit(EXIT_FAILURE);                                            \
        }                                                                   \
    }

#define CUBBYFLOW_CUDA_CHECK(expression) \
    _CUBBYFLOW_CUDA_CHECK((expression), #expression, __FILE__, __LINE__)

#define CUBBYFLOW_CUDA_CHECK_LAST_ERROR(msg) \
    _CUBBYFLOW_CUDA_CHECK(cudaGetLastError(), msg, __FILE__, __LINE__)

#endif  // CUBBYFLOW_USE_CUDA

#if defined(_WIN32) || defined(_WIN64)
#define CUBBYFLOW_WINDOWS
#elif defined(__APPLE__)
#define CUBBYFLOW_APPLE
#ifndef CUBBYFLOW_IOS
#define CUBBYFLOW_MACOSX
#endif
#elif defined(linux) || defined(__linux__)
#define CUBBYFLOW_LINUX
#endif

#if defined(CUBBYFLOW_WINDOWS) && defined(_MSC_VER)
#include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#else
#include <sys/types.h>
#endif

#ifndef UNUSED_VARIABLE
#define UNUSED_VARIABLE(x) ((void)x)
#endif

#endif