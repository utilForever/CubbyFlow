// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_SNOW_CONSTITUTIVE_MODEL_IMPL_HPP
#define CUBBYFLOW_SNOW_CONSTITUTIVE_MODEL_IMPL_HPP

#include <Core/Math/SVD.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace CubbyFlow
{
template <size_t N>
SnowConstitutiveModel<N>::SnowConstitutiveModel(double youngsModulus,
                                                double poissonRatio,
                                                double criticalCompression,
                                                double criticalStretch,
                                                double hardeningCoefficient)
    : m_criticalCompression(criticalCompression),
      m_criticalStretch(criticalStretch),
      m_hardeningCoefficient(hardeningCoefficient)
{
    if (const std::array parameterChecks = { std::isfinite(youngsModulus),
                                             youngsModulus > 0.0,
                                             std::isfinite(poissonRatio),
                                             poissonRatio > -1.0,
                                             poissonRatio < 0.5,
                                             std::isfinite(criticalCompression),
                                             criticalCompression >= 0.0,
                                             criticalCompression < 1.0,
                                             std::isfinite(criticalStretch),
                                             criticalStretch >= 0.0,
                                             std::isfinite(
                                                 hardeningCoefficient),
                                             hardeningCoefficient >= 0.0 };
        !std::ranges::all_of(parameterChecks,
                             [](bool isValid) { return isValid; }))
    {
        throw std::invalid_argument{ "Invalid snow material parameters." };
    }

    m_mu0 = youngsModulus / (2.0 * (1.0 + poissonRatio));
    m_lambda0 = youngsModulus * poissonRatio /
                ((1.0 + poissonRatio) * (1.0 - 2.0 * poissonRatio));

    if (!std::isfinite(m_mu0) || !std::isfinite(m_lambda0))
    {
        throw std::invalid_argument{ "Invalid snow material parameters." };
    }
}

template <size_t N>
typename SnowConstitutiveModel<N>::State SnowConstitutiveModel<N>::Update(
    const MatrixType& deformationGradientIncrement, const State& state) const
{
    ValidateDeformation(deformationGradientIncrement);
    ValidateDeformation(state.elastic);
    ValidateDeformation(state.plastic);

    const MatrixType trialElastic =
        deformationGradientIncrement * state.elastic;
    const MatrixType trialTotal = trialElastic * state.plastic;

    ValidateDeformation(trialElastic);
    ValidateDeformation(trialTotal);

    MatrixType u;
    Vector<double, N> singularValues;
    MatrixType v;

    SVD(trialElastic, u, singularValues, v);

    for (size_t i = 0; i < N; ++i)
    {
        singularValues[i] =
            std::clamp(singularValues[i], 1.0 - m_criticalCompression,
                       1.0 + m_criticalStretch);
    }

    State result;
    result.elastic =
        u * MatrixType::MakeScaleMatrix(singularValues) * v.Transposed();
    result.plastic = result.elastic.Inverse() * trialTotal;

    ValidateDeformation(result.elastic);
    ValidateDeformation(result.plastic);

    return result;
}

template <size_t N>
typename SnowConstitutiveModel<N>::MatrixType
SnowConstitutiveModel<N>::ComputeKirchhoffStress(const State& state) const
{
    ValidateDeformation(state.elastic);
    ValidateDeformation(state.plastic);

    MatrixType u;
    Vector<double, N> singularValues;
    MatrixType v;

    SVD(state.elastic, u, singularValues, v);

    const MatrixType rotation = u * v.Transposed();
    const double elasticDeterminant = state.elastic.Determinant();
    const double hardening = ComputeHardening(state);

    const double mu = m_mu0 * hardening;
    const double lambda = m_lambda0 * hardening;
    const MatrixType stress =
        2.0 * mu * (state.elastic - rotation) * state.elastic.Transposed() +
        lambda * (elasticDeterminant - 1.0) * elasticDeterminant *
            MatrixType::MakeIdentity();

    if (!IsFinite(stress))
    {
        throw std::invalid_argument{ "Non-finite snow stress." };
    }

    return stress;
}

template <size_t N>
double SnowConstitutiveModel<N>::ComputeWaveSpeed(const State& state,
                                                  double referenceDensity) const
{
    ValidateDeformation(state.elastic);
    ValidateDeformation(state.plastic);

    if (!std::isfinite(referenceDensity) || referenceDensity <= 0.0)
    {
        throw std::invalid_argument{ "Invalid snow reference density." };
    }

    MatrixType u;
    Vector<double, N> singularValues;
    MatrixType v;
    SVD(state.elastic, u, singularValues, v);

    const double elasticDeterminant = state.elastic.Determinant();
    const double totalDeterminant =
        elasticDeterminant * state.plastic.Determinant();
    const double currentDensity = referenceDensity / totalDeterminant;
    const double hardening = ComputeHardening(state);
    const double mu = m_mu0 * hardening;
    const double lambda = m_lambda0 * hardening;
    double maxCandidate = 0.0;

    for (size_t a = 0; a < N; ++a)
    {
        const double sigmaA = singularValues[a];
        maxCandidate = std::max(
            maxCandidate, 2.0 * mu * sigmaA * sigmaA +
                              lambda * elasticDeterminant * elasticDeterminant);

        for (size_t b = 0; b < N; ++b)
        {
            if (a == b)
            {
                continue;
            }

            const double sigmaB = singularValues[b];
            maxCandidate = std::max(maxCandidate, 2.0 * mu * sigmaB * sigmaB *
                                                      (sigmaA + sigmaB - 1.0) /
                                                      (sigmaA + sigmaB));
        }
    }

    const double kappa = maxCandidate / totalDeterminant;
    const double waveSpeed = std::sqrt(kappa / currentDensity);

    if (!std::isfinite(totalDeterminant) || totalDeterminant <= 0.0 ||
        !std::isfinite(currentDensity) || currentDensity <= 0.0 ||
        !std::isfinite(kappa) || kappa <= 0.0 || !std::isfinite(waveSpeed))
    {
        throw std::invalid_argument{ "Invalid snow wave speed." };
    }

    return waveSpeed;
}

template <size_t N>
double SnowConstitutiveModel<N>::ComputeHardening(const State& state) const
{
    const double hardening =
        std::exp(m_hardeningCoefficient * (1.0 - state.plastic.Determinant()));

    if (!std::isfinite(hardening))
    {
        throw std::invalid_argument{ "Non-finite snow hardening." };
    }

    return hardening;
}

template <size_t N>
bool SnowConstitutiveModel<N>::IsFinite(const MatrixType& matrix)
{
    return std::ranges::all_of(
        matrix, [](double value) { return std::isfinite(value); });
}

template <size_t N>
void SnowConstitutiveModel<N>::ValidateDeformation(const MatrixType& matrix)
{
    const double determinant = matrix.Determinant();

    if (!IsFinite(matrix) || !std::isfinite(determinant) || determinant <= 0.0)
    {
        throw std::invalid_argument{ "Invalid snow deformation gradient." };
    }
}
}  // namespace CubbyFlow

#endif
