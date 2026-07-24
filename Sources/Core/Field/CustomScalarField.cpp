// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Field/CustomScalarField.hpp>
#include <utility>

namespace CubbyFlow
{
template <size_t N>
CustomScalarField<N>::CustomScalarField(
    std::function<double(const Vector<double, N>&)> customFunction,
    double derivativeResolution)
    : m_customFunction(std::move(customFunction)),
      m_resolution(derivativeResolution)
{
    // Do nothing
}

template <size_t N>
CustomScalarField<N>::CustomScalarField(
    std::function<double(const Vector<double, N>&)> customFunction,
    std::function<Vector<double, N>(const Vector<double, N>&)>
        customGradientFunction,
    double derivativeResolution)
    : m_customFunction(std::move(customFunction)),
      m_customGradientFunction(std::move(customGradientFunction)),
      m_resolution(derivativeResolution)
{
    // Do nothing
}

template <size_t N>
CustomScalarField<N>::CustomScalarField(
    std::function<double(const Vector<double, N>&)> customFunction,
    std::function<Vector<double, N>(const Vector<double, N>&)>
        customGradientFunction,
    std::function<double(const Vector<double, N>&)> customLaplacianFunction)
    : m_customFunction(std::move(customFunction)),
      m_customGradientFunction(std::move(customGradientFunction)),
      m_customLaplacianFunction(std::move(customLaplacianFunction))
{
    // Do nothing
}

template <size_t N>
double CustomScalarField<N>::Sample(const Vector<double, N>& x) const
{
    return m_customFunction(x);
}

template <size_t N>
std::function<double(const Vector<double, N>&)> CustomScalarField<N>::Sampler()
    const
{
    return m_customFunction;
}

template <size_t N>
Vector<double, N> CustomScalarField<N>::Gradient(
    const Vector<double, N>& x) const
{
    if (m_customGradientFunction)
    {
        return m_customGradientFunction(x);
    }

    Vector<double, N> result;

    for (size_t i = 0; i < N; ++i)
    {
        Vector<double, N> pt;
        pt[i] = 0.5 * m_resolution;

        const double left = m_customFunction(x - pt);
        const double right = m_customFunction(x + pt);
        result[i] = (right - left) / m_resolution;
    }

    return result;
}

template <size_t N>
double CustomScalarField<N>::Laplacian(const Vector<double, N>& x) const
{
    if (m_customLaplacianFunction)
    {
        return m_customLaplacianFunction(x);
    }

    const double center = m_customFunction(x);
    double sum = -4.0 * center;

    for (size_t i = 0; i < N; ++i)
    {
        Vector<double, N> pt;
        pt[i] = 0.5 * m_resolution;

        const double left = m_customFunction(x - pt);
        const double right = m_customFunction(x + pt);
        sum += right + left;
    }

    return sum / (m_resolution * m_resolution);
}

template <size_t N>
typename CustomScalarField<N>::Builder CustomScalarField<N>::GetBuilder()
{
    return Builder{};
}

template <size_t N>
typename CustomScalarField<N>::Builder&
CustomScalarField<N>::Builder::WithFunction(
    const std::function<double(const Vector<double, N>&)>& func)
{
    m_customFunction = func;
    return *this;
}

template <size_t N>
typename CustomScalarField<N>::Builder&
CustomScalarField<N>::Builder::WithGradientFunction(
    const std::function<Vector<double, N>(const Vector<double, N>&)>& func)
{
    m_customGradientFunction = func;
    return *this;
}

template <size_t N>
typename CustomScalarField<N>::Builder&
CustomScalarField<N>::Builder::WithLaplacianFunction(
    const std::function<double(const Vector<double, N>&)>& func)
{
    m_customLaplacianFunction = func;
    return *this;
}

template <size_t N>
typename CustomScalarField<N>::Builder&
CustomScalarField<N>::Builder::WithDerivativeResolution(double resolution)
{
    m_resolution = resolution;
    return *this;
}

template <size_t N>
CustomScalarField<N> CustomScalarField<N>::Builder::Build() const
{
    if (m_customLaplacianFunction)
    {
        return CustomScalarField{ m_customFunction, m_customGradientFunction,
                                  m_customLaplacianFunction };
    }

    return CustomScalarField{ m_customFunction, m_customGradientFunction,
                              m_resolution };
}

template <size_t N>
std::shared_ptr<CustomScalarField<N>>
CustomScalarField<N>::Builder::MakeShared() const
{
    if (m_customLaplacianFunction)
    {
        return std::make_shared<CustomScalarField>(m_customFunction,
                                                   m_customGradientFunction,
                                                   m_customLaplacianFunction);
    }

    return std::make_shared<CustomScalarField>(
        m_customFunction, m_customGradientFunction, m_resolution);
}

template class CustomScalarField<2>;

template class CustomScalarField<3>;
}  // namespace CubbyFlow