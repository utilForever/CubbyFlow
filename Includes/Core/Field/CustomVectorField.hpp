// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_CUSTOM_VECTOR_FIELD_HPP
#define CUBBYFLOW_CUSTOM_VECTOR_FIELD_HPP

#include <Core/Field/VectorField.hpp>

namespace CubbyFlow
{
//! N-D vector field with custom field function.
template <size_t N>
class CustomVectorField final : public VectorField<N>
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
    explicit CustomVectorField(
        std::function<Vector<double, N>(const Vector<double, N>&)>
            customFunction,
        double derivativeResolution = 1e-3);

    //!
    //! \brief Constructs a field with given field and gradient function.
    //!
    //! This constructor creates a field with user-provided field and gradient
    //! function objects. To compute Laplacian, finite differencing is used.
    //! Thus, the differencing resolution also can be provided as the last
    //! parameter.
    //!
    CustomVectorField(std::function<Vector<double, N>(const Vector<double, N>&)>
                          customFunction,
                      std::function<double(const Vector<double, N>&)>
                          customDivergenceFunction,
                      double derivativeResolution = 1e-3);

    //! Constructs a field with given field, gradient, and Laplacian function.
    CustomVectorField(
        std::function<Vector<double, N>(const Vector<double, N>&)>
            customFunction,
        std::function<double(const Vector<double, N>&)>
            customDivergenceFunction,
        std::function<typename GetCurl<N>::Type(const Vector<double, N>&)>
            customCurlFunction);

    //! Returns the sampled value at given position \p x.
    [[nodiscard]] Vector<double, N> Sample(
        const Vector<double, N>& x) const override;

    //! Returns the divergence at given position \p x.
    [[nodiscard]] double Divergence(const Vector<double, N>& x) const override;

    //! Returns the curl at given position \p x.
    [[nodiscard]] typename GetCurl<N>::Type Curl(
        const Vector<double, N>& x) const override;

    //! Returns the sampler function.
    [[nodiscard]] std::function<Vector<double, N>(const Vector<double, N>&)>
    Sampler() const override;

    //! Returns builder for CustomVectorField.
    static Builder GetBuilder();

 private:
    std::function<Vector<double, N>(const Vector<double, N>&)> m_customFunction;
    std::function<double(const Vector<double, N>&)> m_customDivergenceFunction;
    std::function<typename GetCurl<N>::Type(const Vector<double, N>&)>
        m_customCurlFunction;
    double m_resolution = 1e-3;
};

//! 2-D CustomVectorField type.
using CustomVectorField2 = CustomVectorField<2>;

//! 3-D CustomVectorField type.
using CustomVectorField3 = CustomVectorField<3>;

//! Shared pointer type for the CustomVectorField2.
using CustomVectorField2Ptr = std::shared_ptr<CustomVectorField2>;

//! Shared pointer type for the CustomVectorField3.
using CustomVectorField3Ptr = std::shared_ptr<CustomVectorField3>;

//!
//! \brief Front-end to create CustomVectorField objects step by step.
//!
template <size_t N>
class CustomVectorField<N>::Builder final
{
 public:
    //! Returns builder with field function.
    Builder& WithFunction(
        const std::function<Vector<double, N>(const Vector<double, N>&)>& func);

    //! Returns builder with divergence function.
    Builder& WithDivergenceFunction(
        const std::function<double(const Vector<double, N>&)>& func);

    //! Returns builder with curl function.
    Builder& WithCurlFunction(const std::function<typename GetCurl<N>::Type(
                                  const Vector<double, N>&)>& func);

    //! Returns builder with derivative resolution.
    Builder& WithDerivativeResolution(double resolution);

    //! Builds CustomVectorField.
    CustomVectorField Build() const;

    //! Builds shared pointer of CustomVectorField instance.
    std::shared_ptr<CustomVectorField> MakeShared() const;

 private:
    std::function<Vector<double, N>(const Vector<double, N>&)> m_customFunction;
    std::function<double(const Vector<double, N>&)> m_customDivergenceFunction;
    std::function<typename GetCurl<N>::Type(const Vector<double, N>&)>
        m_customCurlFunction;
    double m_resolution = 1e-3;
};
}  // namespace CubbyFlow

#endif