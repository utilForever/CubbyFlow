// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_CUSTOM_SCALAR_FIELD_HPP
#define CUBBYFLOW_CUSTOM_SCALAR_FIELD_HPP

#include <Core/Field/ScalarField.hpp>

namespace CubbyFlow
{
//! N-D scalar field with custom field function.
template <size_t N>
class CustomScalarField final : public ScalarField<N>
{
 public:
    class Builder;

    //!
    //! \brief Constructs a field with given function.
    //!
    //! This constructor creates a field with user-provided function object.
    //! To compute derivatives, such as gradient and Laplacian, finite
    //! differencing is used. Thus, the differencing resolution also can be
    //! provided as the last parameter.
    //!
    explicit CustomScalarField(
        std::function<double(const Vector<double, N>&)> customFunction,
        double derivativeResolution = 1e-3);

    //!
    //! \brief Constructs a field with given field and gradient function.
    //!
    //! This constructor creates a field with user-provided field and gradient
    //! function objects. To compute Laplacian, finite differencing is used.
    //! Thus, the differencing resolution also can be provided as the last
    //! parameter.
    //!
    CustomScalarField(
        std::function<double(const Vector<double, N>&)> customFunction,
        std::function<Vector<double, N>(const Vector<double, N>&)>
            customGradientFunction,
        double derivativeResolution = 1e-3);

    //! Constructs a field with given field, gradient, and Laplacian function.
    CustomScalarField(
        std::function<double(const Vector<double, N>&)> customFunction,
        std::function<Vector<double, N>(const Vector<double, N>&)>
            customGradientFunction,
        std::function<double(const Vector<double, N>&)>
            customLaplacianFunction);

    //! Returns the sampled value at given position \p x.
    [[nodiscard]] double Sample(const Vector<double, N>& x) const override;

    //! Returns the sampler function.
    [[nodiscard]] std::function<double(const Vector<double, N>&)> Sampler()
        const override;

    //! Returns the gradient vector at given position \p x.
    [[nodiscard]] Vector<double, N> Gradient(
        const Vector<double, N>& x) const override;

    //! Returns the Laplacian at given position \p x.
    [[nodiscard]] double Laplacian(const Vector<double, N>& x) const override;

    //! Returns builder for CustomScalarField.
    static Builder GetBuilder();

 private:
    std::function<double(const Vector<double, N>&)> m_customFunction;
    std::function<Vector<double, N>(const Vector<double, N>&)>
        m_customGradientFunction;
    std::function<double(const Vector<double, N>&)> m_customLaplacianFunction;
    double m_resolution = 1e-3;
};

//! 2-D CustomScalarField type.
using CustomScalarField2 = CustomScalarField<2>;

//! 3-D CustomScalarField type.
using CustomScalarField3 = CustomScalarField<3>;

//! Shared pointer type for the CustomScalarField2.
using CustomScalarField2Ptr = std::shared_ptr<CustomScalarField2>;

//! Shared pointer type for the CustomScalarField3.
using CustomScalarField3Ptr = std::shared_ptr<CustomScalarField3>;

//!
//! \brief Front-end to create CustomScalarField objects step by step.
//!
template <size_t N>
class CustomScalarField<N>::Builder final
{
 public:
    //! Returns builder with field function.
    Builder& WithFunction(
        const std::function<double(const Vector<double, N>&)>& func);

    //! Returns builder with divergence function.
    Builder& WithGradientFunction(
        const std::function<Vector<double, N>(const Vector<double, N>&)>& func);

    //! Returns builder with curl function.
    Builder& WithLaplacianFunction(
        const std::function<double(const Vector<double, N>&)>& func);

    //! Returns builder with derivative resolution.
    Builder& WithDerivativeResolution(double resolution);

    //! Builds CustomScalarField.
    CustomScalarField<N> Build() const;

    //! Builds shared pointer of CustomScalarField instance.
    std::shared_ptr<CustomScalarField<N>> MakeShared() const;

 private:
    double m_resolution = 1e-3;
    std::function<double(const Vector<double, N>&)> m_customFunction;
    std::function<Vector<double, N>(const Vector<double, N>&)>
        m_customGradientFunction;
    std::function<double(const Vector<double, N>&)> m_customLaplacianFunction;
};
}  // namespace CubbyFlow

#endif