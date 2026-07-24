// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Field/CustomVectorField.hpp>

namespace CubbyFlow
{
namespace Internal
{
double Curl(const std::function<Vector2D(const Vector2D&)>& func,
            const Vector2D& x, double resolution)
{
    const Vector2D left = func(x - Vector2D{ 0.5 * resolution, 0.0 });
    const Vector2D right = func(x + Vector2D{ 0.5 * resolution, 0.0 });
    const Vector2D bottom = func(x - Vector2D{ 0.0, 0.5 * resolution });
    const Vector2D top = func(x + Vector2D{ 0.0, 0.5 * resolution });

    const double Fx_ym = bottom.x;
    const double Fx_yp = top.x;

    const double Fy_xm = left.y;
    const double Fy_xp = right.y;

    return 0.5 * (Fy_xp - Fy_xm) / resolution -
           0.5 * (Fx_yp - Fx_ym) / resolution;
}

Vector3D Curl(const std::function<Vector3D(const Vector3D&)>& func,
              const Vector3D& x, double resolution)
{
    const Vector3D left = func(x - Vector3D{ 0.5 * resolution, 0.0, 0.0 });
    const Vector3D right = func(x + Vector3D{ 0.5 * resolution, 0.0, 0.0 });
    const Vector3D bottom = func(x - Vector3D{ 0.0, 0.5 * resolution, 0.0 });
    const Vector3D top = func(x + Vector3D{ 0.0, 0.5 * resolution, 0.0 });
    const Vector3D back = func(x - Vector3D{ 0.0, 0.0, 0.5 * resolution });
    const Vector3D front = func(x + Vector3D{ 0.0, 0.0, 0.5 * resolution });

    const double Fx_ym = bottom.x;
    const double Fx_yp = top.x;
    const double Fx_zm = back.x;
    const double Fx_zp = front.x;

    const double Fy_xm = left.y;
    const double Fy_xp = right.y;
    const double Fy_zm = back.y;
    const double Fy_zp = front.y;

    const double Fz_xm = left.z;
    const double Fz_xp = right.z;
    const double Fz_ym = bottom.z;
    const double Fz_yp = top.z;

    return Vector3D{
        0.5 * (Fz_yp - Fz_ym) / resolution - 0.5 * (Fy_zp - Fy_zm) / resolution,
        0.5 * (Fx_zp - Fx_zm) / resolution - 0.5 * (Fz_xp - Fz_xm) / resolution,
        0.5 * (Fy_xp - Fy_xm) / resolution - 0.5 * (Fx_yp - Fx_ym) / resolution
    };
}
}  // namespace Internal

template <size_t N>
CustomVectorField<N>::CustomVectorField(
    std::function<Vector<double, N>(const Vector<double, N>&)> customFunction,
    double derivativeResolution)
    : m_customFunction(std::move(customFunction)),
      m_resolution(derivativeResolution)
{
    // Do nothing
}

template <size_t N>
CustomVectorField<N>::CustomVectorField(
    std::function<Vector<double, N>(const Vector<double, N>&)> customFunction,
    std::function<double(const Vector<double, N>&)> customDivergenceFunction,
    double derivativeResolution)
    : m_customFunction(std::move(customFunction)),
      m_customDivergenceFunction(std::move(customDivergenceFunction)),
      m_resolution(derivativeResolution)
{
    // Do nothing
}

template <size_t N>
CustomVectorField<N>::CustomVectorField(
    std::function<Vector<double, N>(const Vector<double, N>&)> customFunction,
    std::function<double(const Vector<double, N>&)> customDivergenceFunction,
    std::function<typename GetCurl<N>::Type(const Vector<double, N>&)>
        customCurlFunction)
    : m_customFunction(std::move(customFunction)),
      m_customDivergenceFunction(std::move(customDivergenceFunction)),
      m_customCurlFunction(std::move(customCurlFunction))
{
    // Do nothing
}

template <size_t N>
Vector<double, N> CustomVectorField<N>::Sample(const Vector<double, N>& x) const
{
    return m_customFunction(x);
}

template <size_t N>
std::function<Vector<double, N>(const Vector<double, N>&)>
CustomVectorField<N>::Sampler() const
{
    return m_customFunction;
}

template <size_t N>
double CustomVectorField<N>::Divergence(const Vector<double, N>& x) const
{
    if (m_customDivergenceFunction)
    {
        return m_customDivergenceFunction(x);
    }

    double sum = 0.0;

    for (size_t i = 0; i < N; ++i)
    {
        Vector<double, N> pt;
        pt[i] = 0.5 * m_resolution;

        const double left = m_customFunction(x - pt)[i];
        const double right = m_customFunction(x + pt)[i];
        sum += right - left;
    }

    return sum / m_resolution;
}

template <size_t N>
typename GetCurl<N>::Type CustomVectorField<N>::Curl(
    const Vector<double, N>& x) const
{
    if (m_customCurlFunction)
    {
        return m_customCurlFunction(x);
    }

    return Internal::Curl(m_customFunction, x, m_resolution);
}

template <size_t N>
typename CustomVectorField<N>::Builder CustomVectorField<N>::GetBuilder()
{
    return Builder{};
}

template <size_t N>
typename CustomVectorField<N>::Builder&
CustomVectorField<N>::Builder::WithFunction(
    const std::function<Vector<double, N>(const Vector<double, N>&)>& func)
{
    m_customFunction = func;
    return *this;
}

template <size_t N>
typename CustomVectorField<N>::Builder&
CustomVectorField<N>::Builder::WithDivergenceFunction(
    const std::function<double(const Vector<double, N>&)>& func)
{
    m_customDivergenceFunction = func;
    return *this;
}

template <size_t N>
typename CustomVectorField<N>::Builder&
CustomVectorField<N>::Builder::WithCurlFunction(
    const std::function<typename GetCurl<N>::Type(const Vector<double, N>&)>&
        func)
{
    m_customCurlFunction = func;
    return *this;
}

template <size_t N>
typename CustomVectorField<N>::Builder&
CustomVectorField<N>::Builder::WithDerivativeResolution(double resolution)
{
    m_resolution = resolution;
    return *this;
}

template <size_t N>
CustomVectorField<N> CustomVectorField<N>::Builder::Build() const
{
    if (m_customCurlFunction)
    {
        return CustomVectorField{ m_customFunction, m_customDivergenceFunction,
                                  m_customCurlFunction };
    }

    return CustomVectorField{ m_customFunction, m_customDivergenceFunction,
                              m_resolution };
}

template <size_t N>
std::shared_ptr<CustomVectorField<N>>
CustomVectorField<N>::Builder::MakeShared() const
{
    if (m_customCurlFunction)
    {
        return std::make_shared<CustomVectorField>(
            m_customFunction, m_customDivergenceFunction, m_customCurlFunction);
    }

    return std::make_shared<CustomVectorField>(
        m_customFunction, m_customDivergenceFunction, m_resolution);
}

template class CustomVectorField<2>;

template class CustomVectorField<3>;
}  // namespace CubbyFlow