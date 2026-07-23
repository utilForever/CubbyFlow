// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Field/ConstantScalarField.hpp>
#include <Core/Utils/Macros.hpp>

namespace CubbyFlow
{
template <size_t N>
ConstantScalarField<N>::ConstantScalarField(double value) : m_value(value)
{
    // Do nothing
}

template <size_t N>
double ConstantScalarField<N>::Sample(const Vector<double, N>& x) const
{
    UNUSED_VARIABLE(x);
    return m_value;
}

template <size_t N>
std::function<double(const Vector<double, N>&)>
ConstantScalarField<N>::Sampler() const
{
    double value = m_value;
    return [value](const Vector<double, N>&) -> double { return value; };
}

template <size_t N>
typename ConstantScalarField<N>::Builder ConstantScalarField<N>::GetBuilder()
{
    return Builder{};
}

template <size_t N>
typename ConstantScalarField<N>::Builder&
ConstantScalarField<N>::Builder::WithValue(double value)
{
    m_value = value;
    return *this;
}

template <size_t N>
ConstantScalarField<N> ConstantScalarField<N>::Builder::Build() const
{
    return ConstantScalarField{ m_value };
}

template <size_t N>
std::shared_ptr<ConstantScalarField<N>>
ConstantScalarField<N>::Builder::MakeShared() const
{
    return std::make_shared<ConstantScalarField>(m_value);
}

template class ConstantScalarField<2>;

template class ConstantScalarField<3>;
}  // namespace CubbyFlow
