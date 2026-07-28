// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_FUNCTORS_HPP
#define CUBBYFLOW_FUNCTORS_HPP

#include <limits>

namespace CubbyFlow
{
//! No-op operator.
template <typename T>
struct NoOp
{
    constexpr T operator()(const T& a) const;
};

//! Type casting operator.
template <typename T, typename U>
struct TypeCast
{
    constexpr U operator()(const T& a) const;
};

//! Performs std::ceil.
template <typename T>
struct DoCeil
{
    constexpr T operator()(const T& a) const;
};

//! Performs std::floor.
template <typename T>
struct DoFloor
{
    constexpr T operator()(const T& a) const;
};

//! Reverse minus operator.
template <typename T>
struct RMinus
{
    constexpr T operator()(const T& a, const T& b) const;
};

//! Reverse divides operator.
template <typename T>
struct RDivides
{
    constexpr T operator()(const T& a, const T& b) const;
};

//! Takes minimum value.
template <typename T>
struct DoMin
{
    constexpr T operator()(const T& a, const T& b) const;
};

//! Takes maximum value.
template <typename T>
struct DoMax
{
    constexpr T operator()(const T& a, const T& b) const;
};

//! Takes absolute minimum value.
template <typename T>
struct DoAbsMin
{
    constexpr T operator()(const T& a, const T& b) const;
};

//! Takes absolute maximum value.
template <typename T>
struct DoAbsMax
{
    constexpr T operator()(const T& a, const T& b) const;
};

//! True if similar.
template <typename T>
struct SimilarTo
{
    explicit constexpr SimilarTo(
        double _tolerance = std::numeric_limits<double>::epsilon())
        : tolerance(_tolerance)
    {
        // Do nothing
    }

    constexpr bool operator()(const T& a, const T& b) const;

    double tolerance;
};

//! Clamps the input value with low/high.
template <typename T>
struct DoClamp
{
    constexpr T operator()(const T& a, const T& low, const T& high) const;
};
}  // namespace CubbyFlow

#include <Core/Utils/Functors-Impl.hpp>

#endif