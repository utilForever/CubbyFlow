// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_ARRAY_SAMPLERS_IMPL_HPP
#define CUBBYFLOW_ARRAY_SAMPLERS_IMPL_HPP

#include <Core/Math/MathUtils.hpp>

namespace CubbyFlow
{
namespace Internal
{
template <typename T, size_t N, size_t I>
struct Lerp
{
    using ScalarType = typename GetScalarType<T>::value;

    template <typename View, typename... RemainingIndices>
    static auto Call(const View& view, Vector<ssize_t, N> i,
                     Vector<ScalarType, N> t, RemainingIndices... indices)
    {
        using Next = Lerp<T, N, I - 1>;

        return CubbyFlow::Lerp(Next::Call(view, i, t, i[I - 1], indices...),
                               Next::Call(view, i, t, i[I - 1] + 1, indices...),
                               t[I - 1]);
    }
};

template <typename T, size_t N>
struct Lerp<T, N, 1>
{
    using ScalarType = typename GetScalarType<T>::value;

    template <typename View, typename... RemainingIndices>
    static auto Call(const View& view, Vector<ssize_t, N> i,
                     Vector<ScalarType, N> t, RemainingIndices... indices)
    {
        return CubbyFlow::Lerp(view(i[0], indices...),
                               view(i[0] + 1, indices...), t[0]);
    }
};

template <typename T, size_t N, size_t I>
struct Cubic
{
    using ScalarType = typename GetScalarType<T>::value;

    template <typename View, typename CubicInterpolationOp,
              typename... RemainingIndices>
    static auto Call(const View& view, Vector<ssize_t, N> i,
                     Vector<ScalarType, N> t, CubicInterpolationOp op,
                     RemainingIndices... indices)
    {
        using Next = Cubic<T, N, I - 1>;

        return op(
            Next::Call(view, i, t, op, std::max(i[I - 1] - 1, ZERO_SSIZE),
                       indices...),
            Next::Call(view, i, t, op, i[I - 1], indices...),
            Next::Call(view, i, t, op,
                       std::min(i[I - 1] + 1,
                                static_cast<ssize_t>(view.Size()[I - 1]) - 1),
                       indices...),
            Next::Call(view, i, t, op,
                       std::min(i[I - 1] + 2,
                                static_cast<ssize_t>(view.Size()[I - 1]) - 1),
                       indices...),
            t[I - 1]);
    }
};

template <typename T, size_t N>
struct Cubic<T, N, 1>
{
    using ScalarType = typename GetScalarType<T>::value;

    template <typename View, typename CubicInterpolationOp,
              typename... RemainingIndices>
    static auto Call(const View& view, Vector<ssize_t, N> i,
                     Vector<ScalarType, N> t, CubicInterpolationOp op,
                     RemainingIndices... indices)
    {
        return op(
            view(std::max(i[0] - 1, ZERO_SSIZE), indices...),
            view(i[0], indices...),
            view(std::min(i[0] + 1, static_cast<ssize_t>(view.Size()[0]) - 1),
                 indices...),
            view(std::min(i[0] + 2, static_cast<ssize_t>(view.Size()[0]) - 1),
                 indices...),
            t[0]);
    }
};

template <typename T, size_t N, size_t I>
struct GetCoordinatesAndWeights
{
    using ScalarType = typename GetScalarType<T>::value;

    template <typename Coords, typename Weights, typename... RemainingIndices>
    static void Call(Coords& c, Weights& w, Vector<size_t, N> i,
                     Vector<ScalarType, N> t, T acc, RemainingIndices... idx)
    {
        using Next = GetCoordinatesAndWeights<T, N, I - 1>;

        Next::Call(c, w, i, t, acc * (1 - t[I - 1]), 0, idx...);
        Next::Call(c, w, i, t, acc * t[I - 1], 1, idx...);
    }
};

template <typename T, size_t N>
struct GetCoordinatesAndWeights<T, N, 1>
{
    using ScalarType = typename GetScalarType<T>::value;

    template <typename Coords, typename Weights, typename... RemainingIndices>
    static void Call(Coords& c, Weights& w, Vector<size_t, N> i,
                     Vector<ScalarType, N> t, T acc, RemainingIndices... idx)
    {
        c(0, idx...) = Vector<size_t, N>(0, idx...) + i;
        c(1, idx...) = Vector<size_t, N>(1, idx...) + i;

        w(0, idx...) = acc * (1 - t[0]);
        w(1, idx...) = acc * (t[0]);
    }
};

template <typename T, size_t N, size_t I>
struct GetCoordinatesAndGradientWeights
{
    template <typename Coords, typename Weights, typename... RemainingIndices>
    static void Call(Coords& c, Weights& w, Vector<size_t, N> i, Vector<T, N> t,
                     Vector<T, N> acc, RemainingIndices... idx)
    {
        Vector<T, N> w0 = Vector<T, N>::MakeConstant(1 - t[I - 1]);
        w0[I - 1] = -1;
        Vector<T, N> w1 = Vector<T, N>::MakeConstant(t[I - 1]);
        w1[I - 1] = 1;

        using Next = GetCoordinatesAndGradientWeights<T, N, I - 1>;
        Next::Call(c, w, i, t, ElemMul(acc, w0), 0, idx...);
        Next::Call(c, w, i, t, ElemMul(acc, w1), 1, idx...);
    }
};

template <typename T, size_t N>
struct GetCoordinatesAndGradientWeights<T, N, 1>
{
    template <typename Coords, typename Weights, typename... RemainingIndices>
    static void Call(Coords& c, Weights& w, Vector<size_t, N> i, Vector<T, N> t,
                     Vector<T, N> acc, RemainingIndices... idx)
    {
        c(0, idx...) = Vector<size_t, N>(0, idx...) + i;
        c(1, idx...) = Vector<size_t, N>(1, idx...) + i;

        Vector<T, N> w0 = Vector<T, N>::MakeConstant(1 - t[0]);
        w0[0] = -1;
        Vector<T, N> w1 = Vector<T, N>::MakeConstant(t[0]);
        w1[0] = 1;

        w(0, idx...) = ElemMul(acc, w0);
        w(1, idx...) = ElemMul(acc, w1);
    }
};
}  // namespace Internal

template <typename T, size_t N>
NearestArraySampler<T, N>::NearestArraySampler(
    const ArrayView<const T, N>& view, const VectorType& gridSpacing,
    const VectorType& gridOrigin)
    : m_view(view),
      m_gridSpacing(gridSpacing),
      m_invGridSpacing(ScalarType{ 1 } / gridSpacing),
      m_gridOrigin(gridOrigin)
{
    // Do nothing
}

template <typename T, size_t N>
NearestArraySampler<T, N>::NearestArraySampler(
    const NearestArraySampler& other) = default;

template <typename T, size_t N>
NearestArraySampler<T, N>::NearestArraySampler(
    NearestArraySampler&& other) noexcept = default;

template <typename T, size_t N>
NearestArraySampler<T, N>& NearestArraySampler<T, N>::operator=(
    const NearestArraySampler& other) = default;

template <typename T, size_t N>
NearestArraySampler<T, N>& NearestArraySampler<T, N>::operator=(
    NearestArraySampler&& other) noexcept = default;

template <typename T, size_t N>
T NearestArraySampler<T, N>::operator()(const VectorType& pt) const
{
    return m_view(GetCoordinate(pt));
}

template <typename T, size_t N>
typename NearestArraySampler<T, N>::CoordIndexType
NearestArraySampler<T, N>::GetCoordinate(const VectorType& pt) const
{
    Vector<ssize_t, N> is;
    VectorType npt = ElemMul(pt - m_gridOrigin, m_invGridSpacing);
    Vector<ssize_t, N> size = m_view.Size().template CastTo<ssize_t>();

    for (size_t i = 0; i < N; ++i)
    {
        Vector<ScalarType, N> ts;
        GetBarycentric(npt[i], 0, size[i], is[i], ts[i]);
        is[i] =
            std::min(static_cast<ssize_t>(is[i] + ts[i] + 0.5), size[i] - 1);
    }

    return is.template CastTo<size_t>();
}

template <typename T, size_t N>
std::function<T(const typename NearestArraySampler<T, N>::VectorType&)>
NearestArraySampler<T, N>::Functor() const
{
    NearestArraySampler sampler(*this);

    return [sampler](const VectorType& x) -> T { return sampler(x); };
}

template <typename T, size_t N>
LinearArraySampler<T, N>::LinearArraySampler(const ArrayView<const T, N>& view,
                                             const VectorType& gridSpacing,
                                             const VectorType& gridOrigin)
    : m_view(view),
      m_gridSpacing(gridSpacing),
      m_invGridSpacing(ScalarType{ 1 } / gridSpacing),
      m_gridOrigin(gridOrigin)
{
    // Do nothing
}

template <typename T, size_t N>
LinearArraySampler<T, N>::LinearArraySampler(const LinearArraySampler& other) =
    default;

template <typename T, size_t N>
LinearArraySampler<T, N>::LinearArraySampler(
    LinearArraySampler&& other) noexcept = default;

template <typename T, size_t N>
LinearArraySampler<T, N>& LinearArraySampler<T, N>::operator=(
    const LinearArraySampler& other) = default;

template <typename T, size_t N>
LinearArraySampler<T, N>& LinearArraySampler<T, N>::operator=(
    LinearArraySampler&& other) noexcept = default;

template <typename T, size_t N>
T LinearArraySampler<T, N>::operator()(const VectorType& pt) const
{
    Vector<ssize_t, N> is;
    Vector<ScalarType, N> ts;
    VectorType npt = ElemMul(pt - m_gridOrigin, m_invGridSpacing);
    Vector<ssize_t, N> size = m_view.Size().template CastTo<ssize_t>();

    for (size_t i = 0; i < N; ++i)
    {
        GetBarycentric(npt[i], 0, size[i], is[i], ts[i]);
    }

    return Internal::Lerp<T, N, N>::Call(m_view, is, ts);
}

template <typename T, size_t N>
void LinearArraySampler<T, N>::GetCoordinatesAndWeights(
    const VectorType& pt, std::array<CoordIndexType, FLAT_KERNEL_SIZE>& indices,
    std::array<ScalarType, FLAT_KERNEL_SIZE>& weights) const
{
    Vector<ssize_t, N> is;
    Vector<ScalarType, N> ts;
    VectorType npt = ElemMul(pt - m_gridOrigin, m_invGridSpacing);
    Vector<ssize_t, N> size = m_view.Size().template CastTo<ssize_t>();

    for (size_t i = 0; i < N; ++i)
    {
        GetBarycentric(npt[i], 0, size[i], is[i], ts[i]);
    }

    Vector<size_t, N> viewSize = Vector<size_t, N>::MakeConstant(2);
    ArrayView<CoordIndexType, N> indexView(indices.data(), viewSize);
    ArrayView<ScalarType, N> weightView(weights.data(), viewSize);

    Internal::GetCoordinatesAndWeights<ScalarType, N, N>::Call(
        indexView, weightView, is.template CastTo<size_t>(), ts, 1);
}

template <typename T, size_t N>
void LinearArraySampler<T, N>::GetCoordinatesAndGradientWeights(
    const VectorType& pt, std::array<CoordIndexType, FLAT_KERNEL_SIZE>& indices,
    std::array<VectorType, FLAT_KERNEL_SIZE>& weights) const
{
    Vector<ssize_t, N> is;
    Vector<ScalarType, N> ts;
    VectorType npt = ElemMul(pt - m_gridOrigin, m_invGridSpacing);
    Vector<ssize_t, N> size = m_view.Size().template CastTo<ssize_t>();

    for (size_t i = 0; i < N; ++i)
    {
        GetBarycentric(npt[i], 0, size[i], is[i], ts[i]);
    }

    Vector<size_t, N> viewSize = Vector<size_t, N>::MakeConstant(2);
    ArrayView<CoordIndexType, N> indexView(indices.data(), viewSize);
    ArrayView<VectorType, N> weightView(weights.data(), viewSize);

    Internal::GetCoordinatesAndGradientWeights<ScalarType, N, N>::Call(
        indexView, weightView, is.template CastTo<size_t>(), ts,
        m_invGridSpacing);
}

template <typename T, size_t N>
std::function<T(const typename LinearArraySampler<T, N>::VectorType&)>
LinearArraySampler<T, N>::Functor() const
{
    LinearArraySampler sampler(*this);

    return [sampler](const VectorType& x) -> T { return sampler(x); };
}

template <typename T, size_t N, typename CIOp>
CubicArraySampler<T, N, CIOp>::CubicArraySampler(
    const ArrayView<const T, N>& view, const VectorType& gridSpacing,
    const VectorType& gridOrigin)
    : m_view(view),
      m_gridSpacing(gridSpacing),
      m_invGridSpacing(ScalarType{ 1 } / gridSpacing),
      m_gridOrigin(gridOrigin)
{
    // Do nothing
}

template <typename T, size_t N, typename CIOp>
CubicArraySampler<T, N, CIOp>::CubicArraySampler(
    const CubicArraySampler& other) = default;

template <typename T, size_t N, typename CIOp>
CubicArraySampler<T, N, CIOp>::CubicArraySampler(
    CubicArraySampler&& other) noexcept = default;

template <typename T, size_t N, typename CIOp>
CubicArraySampler<T, N, CIOp>& CubicArraySampler<T, N, CIOp>::operator=(
    const CubicArraySampler& other) = default;

template <typename T, size_t N, typename CIOp>
CubicArraySampler<T, N, CIOp>& CubicArraySampler<T, N, CIOp>::operator=(
    CubicArraySampler&& other) noexcept = default;

template <typename T, size_t N, typename CIOp>
T CubicArraySampler<T, N, CIOp>::operator()(const VectorType& pt) const
{
    Vector<ssize_t, N> is;
    Vector<ScalarType, N> ts;
    VectorType npt = ElemMul(pt - m_gridOrigin, m_invGridSpacing);
    Vector<ssize_t, N> size = m_view.Size().template CastTo<ssize_t>();

    for (size_t i = 0; i < N; ++i)
    {
        GetBarycentric(npt[i], 0, size[i], is[i], ts[i]);
    }

    return Internal::Cubic<T, N, N>::Call(m_view, is, ts, CIOp());
}

template <typename T, size_t N, typename CIOp>
std::function<T(const typename CubicArraySampler<T, N, CIOp>::VectorType&)>
CubicArraySampler<T, N, CIOp>::Functor() const
{
    CubicArraySampler sampler(*this);

    return [sampler](const VectorType& x) -> T { return sampler(x); };
}
}  // namespace CubbyFlow

#endif
