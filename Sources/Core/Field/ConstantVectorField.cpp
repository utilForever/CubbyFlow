// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Field/ConstantVectorField.hpp>
#include <Core/Utils/Macros.hpp>

namespace CubbyFlow
{
template <size_t N>
ConstantVectorField<N>::ConstantVectorField(const Vector<double, N>& value)
    : m_value(value)
{
    // Do nothing
}

template <size_t N>
Vector<double, N> ConstantVectorField<N>::Sample(
    const Vector<double, N>& x) const
{
    UNUSED_VARIABLE(x);
    return m_value;
}

template <size_t N>
std::function<Vector<double, N>(const Vector<double, N>&)>
ConstantVectorField<N>::Sampler() const
{
    return [this](const Vector<double, N>&) -> Vector<double, N> {
        return m_value;
    };
}

template <size_t N>
typename ConstantVectorField<N>::Builder ConstantVectorField<N>::GetBuilder()
{
    return Builder{};
}

template <size_t N>
typename ConstantVectorField<N>::Builder&
ConstantVectorField<N>::Builder::WithValue(const Vector<double, N>& value)
{
    m_value = value;
    return *this;
}

template <size_t N>
ConstantVectorField<N> ConstantVectorField<N>::Builder::Build() const
{
    return ConstantVectorField{ m_value };
}

template <size_t N>
std::shared_ptr<ConstantVectorField<N>>
ConstantVectorField<N>::Builder::MakeShared() const
{
    return std::make_shared<ConstantVectorField>(m_value);
}

template class ConstantVectorField<2>;

template class ConstantVectorField<3>;
}  // namespace CubbyFlow
