// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_CUDA_ARRAY_IMPL_HPP
#define CUBBYFLOW_CUDA_ARRAY_IMPL_HPP

#ifdef CUBBYFLOW_USE_CUDA

namespace CubbyFlow
{
#if defined(__CUDACC__) || defined(__HIPCC__)
namespace Internal
{
template <typename T, size_t N, size_t I>
struct CUDABlockCopyHelper
{
    template <typename... RemainingIndices>
    CUBBYFLOW_CUDA_HOST_DEVICE static void Call(CUDAArrayView<const T, N> src,
                                                CUDAStdArray<size_t, N> size,
                                                CUDAArrayView<T, N> dst,
                                                RemainingIndices... indices)
    {
        for (size_t i = 0; i < size[I - 1]; ++i)
        {
            CUDABlockCopyHelper<T, N, I - 1>::Call(src, size, dst, i,
                                                   indices...);
        }
    }
};

template <typename T, size_t N>
struct CUDABlockCopyHelper<T, N, 1>
{
    template <typename... RemainingIndices>
    CUBBYFLOW_CUDA_HOST_DEVICE static void Call(CUDAArrayView<const T, N> src,
                                                CUDAStdArray<size_t, N> size,
                                                CUDAArrayView<T, N> dst,
                                                RemainingIndices... indices)
    {
        for (size_t i = 0; i < size[0]; ++i)
        {
            dst(i, indices...) = src(i, indices...);
        }
    }
};

template <typename T, size_t N>
__global__ void CUDABlockCopyKernelN(CUDAArrayView<const T, N> src,
                                     CUDAStdArray<size_t, N> size,
                                     CUDAArrayView<T, N> dst)
{
    size_t i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < size[N - 1])
    {
        CUDABlockCopyHelper<T, N, N - 1>::Call(src, size, dst, i);
    }
}

template <typename T>
__global__ void CUDABlockCopyKernel1(CUDAArrayView<const T, 1> src,
                                     CUDAStdArray<size_t, 1> size,
                                     CUDAArrayView<T, 1> dst)
{
    size_t i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < size[0])
    {
        dst[i] = src[i];
    }
}

template <typename T, size_t N>
struct CUDABlockCopy
{
    static void Call(CUDAArrayView<const T, N> src,
                     CUDAStdArray<size_t, N> size, CUDAArrayView<T, N> dst)
    {
        if (size[N - 1] == 0)
        {
            return;
        }

        // Assuming i-major
        unsigned int numBlocks, numThreads;
        CUDAComputeGridSize((unsigned int)size[N - 1], 256, numBlocks,
                            numThreads);
        CUDABlockCopyKernelN<<<numBlocks, numThreads>>>(src, size, dst);
        CUBBYFLOW_CUDA_CHECK_LAST_ERROR(
            "Failed executing CUDABlockCopyKernelN");
    }
};

template <typename T>
struct CUDABlockCopy<T, 1>
{
    static void Call(CUDAArrayView<const T, 1> src,
                     CUDAStdArray<size_t, 1> size, CUDAArrayView<T, 1> dst)
    {
        if (size[0] == 0)
        {
            return;
        }

        // Assuming i-major
        unsigned int numBlocks, numThreads;
        CUDAComputeGridSize((unsigned int)size[0], 256, numBlocks, numThreads);
        CUDABlockCopyKernel1<<<numBlocks, numThreads>>>(src, size, dst);
        CUBBYFLOW_CUDA_CHECK_LAST_ERROR(
            "Failed executing CUDABlockCopyKernel1");
    }
};
}  // namespace Internal
#endif

template <typename T, size_t N>
CUDAArray<T, N>::CUDAArray() : Base()
{
    // Do nothing
}

template <typename T, size_t N>
CUDAArray<T, N>::CUDAArray(const CUDAStdArray<size_t, N>& size,
                           const T& initVal)
    : CUDAArray{}
{
    size_t l = size[0];

    for (size_t i = 1; i < N; ++i)
    {
        l *= size[i];
    }

    m_data.Resize(l, initVal);
    Base::SetPtrAndSize(m_data.data(), size);
}

#if defined(__CUDACC__) || defined(__HIPCC__)
template <typename T, size_t N>
template <typename... Args>
CUDAArray<T, N>::CUDAArray(size_t nx, Args... args) : CUDAArray{}
{
    Vector<size_t, N> newSizeVec;
    T initVal;
    Internal::GetSizeAndInitVal<T, N, N - 1>::Call(newSizeVec, initVal, nx,
                                                   args...);
    CUDAStdArray<size_t, N> newSize(newSizeVec);
    CUDAArray newArray(newSize, initVal);
    *this = std::move(newArray);
}

template <typename T, size_t N>
CUDAArray<T, N>::CUDAArray(NestedInitializerListsT<T, N> lst) : CUDAArray{}
{
    Vector<size_t, N> newSize;
    Internal::GetSizeFromInitList<T, N, N>::Call(newSize, lst);

    Array<T, N> newCpuArray(newSize);
    Internal::SetArrayFromInitList<T, N, N>::Call(newCpuArray, lst);
    CopyFrom(newCpuArray);
}
#endif

template <typename T, size_t N>
template <size_t M>
CUDAArray<T, N>::CUDAArray(
    const std::enable_if_t<(M == 1), std::vector<T>>& vec)
    : CUDAArray()
{
    CopyFrom(vec);
}

template <typename T, size_t N>
template <typename OtherDerived>
CUDAArray<T, N>::CUDAArray(const ArrayBase<T, N, OtherDerived>& other)
    : CUDAArray()
{
    CopyFrom(other);
}

template <typename T, size_t N>
template <typename OtherDerived>
CUDAArray<T, N>::CUDAArray(const CUDAArrayBase<T, N, OtherDerived>& other)
    : CUDAArray()
{
    CopyFrom(other);
}

template <typename T, size_t N>
CUDAArray<T, N>::CUDAArray(const CUDAArray& other) : CUDAArray{}
{
    CopyFrom(other);
}

template <typename T, size_t N>
CUDAArray<T, N>::CUDAArray(CUDAArray&& other) noexcept : CUDAArray{}
{
    *this = std::move(other);
}

template <typename T, size_t N>
template <size_t M>
CUDAArray<T, N>& CUDAArray<T, N>::operator=(
    const std::enable_if_t<(M == 1), std::vector<T>>& vec)
{
    CopyFrom(vec);
    return *this;
}

template <typename T, size_t N>
template <typename OtherDerived>
CUDAArray<T, N>& CUDAArray<T, N>::operator=(
    const ArrayBase<T, N, OtherDerived>& other)
{
    CopyFrom(other);
    return *this;
}

template <typename T, size_t N>
template <typename OtherDerived>
CUDAArray<T, N>& CUDAArray<T, N>::operator=(
    const ArrayBase<const T, N, OtherDerived>& other)
{
    CopyFrom(other);
    return *this;
}

template <typename T, size_t N>
template <typename OtherDerived>
CUDAArray<T, N>& CUDAArray<T, N>::operator=(
    const CUDAArrayBase<T, N, OtherDerived>& other)
{
    CopyFrom(other);
    return *this;
}

template <typename T, size_t N>
template <typename OtherDerived>
CUDAArray<T, N>& CUDAArray<T, N>::operator=(
    const CUDAArrayBase<const T, N, OtherDerived>& other)
{
    CopyFrom(other);
    return *this;
}

template <typename T, size_t N>
CUDAArray<T, N>& CUDAArray<T, N>::operator=(const CUDAArray& other)
{
    m_data = other.m_data;
    Base::SetPtrAndSize(m_data.data(), other.Size());
    return *this;
}

template <typename T, size_t N>
CUDAArray<T, N>& CUDAArray<T, N>::operator=(CUDAArray&& other) noexcept
{
    Swap(other);
    other.Clear();
    return *this;
}

template <typename T, size_t N>
template <typename A, size_t M>
std::enable_if_t<(M == 1), void> CUDAArray<T, N>::CopyFrom(
    const std::vector<T, A>& vec)
{
    CUDAArray newArray(vec.size());
    newArray.m_data.CopyFrom(vec);
    newArray.SetPtrAndSize(newArray.m_data.data(), newArray.Size());
    *this = std::move(newArray);
}

template <typename T, size_t N>
template <typename OtherDerived>
void CUDAArray<T, N>::CopyFrom(const ArrayBase<T, N, OtherDerived>& other)
{
    CUDAArray newArray(other.Size());
    CUDACopyHostToDevice(other.data(), other.Length(), newArray.data());
    *this = std::move(newArray);
}

template <typename T, size_t N>
template <typename OtherDerived>
void CUDAArray<T, N>::CopyFrom(const ArrayBase<const T, N, OtherDerived>& other)
{
    CUDAArray newArray(other.Size());
    CUDACopyHostToDevice(other.data(), other.Length(), newArray.data());
    *this = std::move(newArray);
}

template <typename T, size_t N>
template <typename OtherDerived>
void CUDAArray<T, N>::CopyFrom(const CUDAArrayBase<T, N, OtherDerived>& other)
{
    CUDAArray newArray(other.Size());
    CUDACopyDeviceToDevice(other.data(), other.Length(), newArray.data());
    *this = std::move(newArray);
}

template <typename T, size_t N>
template <typename OtherDerived>
void CUDAArray<T, N>::CopyFrom(
    const CUDAArrayBase<const T, N, OtherDerived>& other)
{
    CUDAArray newArray(other.Size());
    CUDACopyDeviceToDevice(other.data(), other.Length(), newArray.data());
    *this = std::move(newArray);
}

template <typename T, size_t N>
template <typename A, size_t M>
std::enable_if_t<(M == 1), void> CUDAArray<T, N>::CopyTo(std::vector<T, A>& vec)
{
    vec.resize(Length());
    CUDACopyDeviceToHost(data(), Length(), vec.data());
}

template <typename T, size_t N>
void CUDAArray<T, N>::CopyTo(Array<T, N>& other)
{
    other.Resize(m_size.ToVector());
    CUDACopyDeviceToHost(data(), Length(), other.data());
}

template <typename T, size_t N>
void CUDAArray<T, N>::CopyTo(ArrayView<T, N>& other)
{
    assert(m_size.ToVector() == other.Size());
    CUDACopyDeviceToHost(data(), Length(), other.data());
}

template <typename T, size_t N>
void CUDAArray<T, N>::CopyTo(CUDAArray<T, N>& other)
{
    other.Resize(m_size.ToVector());
    CUDACopyDeviceToDevice(data(), Length(), other.data());
}

template <typename T, size_t N>
void CUDAArray<T, N>::CopyTo(CUDAArrayView<T, N>& other)
{
    assert(Length() == other.Length());
    CUDACopyDeviceToDevice(data(), Length(), other.data());
}

template <typename T, size_t N>
void CUDAArray<T, N>::Fill(const T& val)
{
    m_data.Fill(val);
}

#if defined(__CUDACC__) || defined(__HIPCC__)
template <typename T, size_t N>
void CUDAArray<T, N>::Resize(CUDAStdArray<size_t, N> newSize, const T& initVal)
{
    CUDAArray newArray(newSize, initVal);
    CUDAStdArray<size_t, N> minSize;

    for (size_t i = 0; i < N; ++i)
    {
        minSize[i] = std::min(m_size[i], newArray.m_size[i]);
    }

    Internal::CUDABlockCopy<T, N>::Call(View(), minSize, newArray.View());

    *this = std::move(newArray);
}

template <typename T, size_t N>
template <typename... Args>
void CUDAArray<T, N>::Resize(size_t nx, Args... args)
{
    Vector<size_t, N> newSizeVec;
    T initVal;

    Internal::GetSizeAndInitVal<T, N, N - 1>::Call(newSizeVec, initVal, nx,
                                                   args...);

    CUDAStdArray<size_t, N> newSize(newSizeVec);
    Resize(newSize, initVal);
}
#endif

template <typename T, size_t N>
template <size_t M>
std::enable_if_t<(M == 1), void> CUDAArray<T, N>::Append(const T& val)
{
    m_data.PushBack(val);
    Base::SetPtrAndSize(m_data.data(), m_data.Size());
}

template <typename T, size_t N>
template <typename A, size_t M>
std::enable_if_t<(M == 1), void> CUDAArray<T, N>::Append(
    const std::vector<T, A>& extra)
{
    m_data.Append(extra);
    m_size[0] = m_data.size();
}

template <typename T, size_t N>
template <typename OtherDerived, size_t M>
std::enable_if_t<(M == 1), void> CUDAArray<T, N>::Append(
    const ArrayBase<T, N, OtherDerived>& extra)
{
    CUDAArray newArray(Length() + extra.Length());
    CUDACopy(data(), Length(), newArray.data());
    CUDACopyHostToDevice(extra.data(), extra.Length(),
                         newArray.data() + m_size[0]);
    Swap(newArray);
}

template <typename T, size_t N>
template <typename OtherDerived, size_t M>
std::enable_if_t<(M == 1), void> CUDAArray<T, N>::Append(
    const CUDAArrayBase<T, N, OtherDerived>& extra)
{
    CUDAArray newArray(Length() + extra.Length());
    CUDACopy(data(), Length(), newArray.data());
    CUDACopy(extra.data(), extra.Length(), newArray.data() + m_size[0]);
    Swap(newArray);
}

template <typename T, size_t N>
void CUDAArray<T, N>::Clear()
{
    Base::ClearPtrAndSize();
    m_data.Clear();
}

template <typename T, size_t N>
void CUDAArray<T, N>::Swap(CUDAArray& other)
{
    Base::SwapPtrAndSize(other);
    m_data.Swap(other.m_data);
}

template <typename T, size_t N>
CUDAArrayView<T, N> CUDAArray<T, N>::View()
{
    return CUDAArrayView<T, N>(*this);
};

template <typename T, size_t N>
CUDAArrayView<const T, N> CUDAArray<T, N>::View() const
{
    return CUDAArrayView<const T, N>(*this);
};
}  // namespace CubbyFlow

#endif

#endif