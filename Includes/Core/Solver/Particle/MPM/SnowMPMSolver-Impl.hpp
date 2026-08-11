// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_SNOW_MPM_SOLVER_IMPL_HPP
#define CUBBYFLOW_SNOW_MPM_SOLVER_IMPL_HPP

#include <Core/Utils/Parallel.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace CubbyFlow
{
template <size_t N>
struct SnowMPMSolver<N>::LinearSystem
{
    const SnowMPMSolver* solver;
    const Array1<SizeType>* activeNodes;
    const Array1<ssize_t>* nodeToActive;
    const Array1<uint8_t>* constrained;
    double dtSquared;

    void Multiply(const VectorND& input, VectorND* output) const;
};

template <size_t N>
struct SnowMPMSolver<N>::LinearSystemBLAS : BLAS<double, VectorND, LinearSystem>
{
    using System = LinearSystem;
    using Base = BLAS<double, VectorND, System>;

    static void MVM(const System& system, const VectorND& vector,
                    VectorND* result)
    {
        system.Multiply(vector, result);
    }

    static void Residual(const System& system, const VectorND& x,
                         const VectorND& b, VectorND* result)
    {
        system.Multiply(x, result);
        Base::AXPlusY(-1.0, *result, b, result);
    }
};

template <size_t N>
void SnowMPMSolver<N>::LinearSystem::Multiply(const VectorND& input,
                                              VectorND* output) const
{
    VectorND projectedInput = input;

    for (size_t i = 0; i < projectedInput.GetRows(); ++i)
    {
        if ((*constrained)[i] != 0)
        {
            projectedInput[i] = 0.0;
        }
    }

    VectorND hessian(input.GetRows(), 0.0);

    solver->ApplyElasticHessian(*activeNodes, *nodeToActive, projectedInput,
                                &hessian);
    output->Resize(input.GetRows(), 0.0);
    output->Fill(0.0);

    const auto& gridMass = solver->m_mpmSystemData->GridMass();

    for (size_t slot = 0; slot < activeNodes->Length(); ++slot)
    {
        const double mass = gridMass((*activeNodes)[slot]);

        for (size_t axis = 0; axis < N; ++axis)
        {
            const size_t row = slot * N + axis;

            if ((*constrained)[row] == 0)
            {
                (*output)[row] =
                    mass * projectedInput[row] + dtSquared * hessian[row];
            }
        }
    }
}

template <size_t N>
SnowMPMSolver<N>::SnowMPMSolver(const SizeType& resolution,
                                const VectorType& gridSpacing,
                                const VectorType& gridOrigin, double radius,
                                double mass)
    : Base{ radius, mass },
      m_mpmSystemData{ std::make_shared<MPMSystemData<N>>(
          resolution, gridSpacing, gridOrigin) }
{
    if (!std::isfinite(radius) || radius < 0.0 || !std::isfinite(mass) ||
        mass <= 0.0)
    {
        throw std::invalid_argument{ "Invalid snow MPM particle parameters." };
    }

    m_mpmSystemData->SetRadius(radius);
    m_mpmSystemData->SetMass(mass);
    this->SetParticleSystemData(m_mpmSystemData);
    this->SetIsUsingFixedSubTimeSteps(false);
}

template <size_t N>
std::shared_ptr<MPMSystemData<N>> SnowMPMSolver<N>::GetMPMSystemData() const
{
    return m_mpmSystemData;
}

template <size_t N>
double SnowMPMSolver<N>::GetTimeStepLimitScale() const
{
    return m_timeStepLimitScale;
}

template <size_t N>
void SnowMPMSolver<N>::SetTimeStepLimitScale(double newScale)
{
    if (!std::isfinite(newScale) || newScale <= 0.0 || newScale > 1.0)
    {
        throw std::invalid_argument{
            "Time-step limit scale must be in (0, 1]."
        };
    }

    m_timeStepLimitScale = newScale;
}

template <size_t N>
bool SnowMPMSolver<N>::GetIsUsingSemiImplicit() const
{
    return m_isUsingSemiImplicit;
}

template <size_t N>
void SnowMPMSolver<N>::SetIsUsingSemiImplicit(bool isUsing)
{
    m_isUsingSemiImplicit = isUsing;
}

template <size_t N>
unsigned int SnowMPMSolver<N>::GetMaxNumberOfIterations() const
{
    return m_maxNumberOfIterations;
}

template <size_t N>
void SnowMPMSolver<N>::SetMaxNumberOfIterations(
    unsigned int maxNumberOfIterations)
{
    m_maxNumberOfIterations = maxNumberOfIterations;
}

template <size_t N>
double SnowMPMSolver<N>::GetTolerance() const
{
    return m_tolerance;
}

template <size_t N>
void SnowMPMSolver<N>::SetTolerance(double tolerance)
{
    if (!std::isfinite(tolerance) || tolerance <= 0.0)
    {
        throw std::invalid_argument{
            "Semi-implicit tolerance must be positive and finite."
        };
    }

    m_tolerance = tolerance;
}

template <size_t N>
unsigned int SnowMPMSolver<N>::GetLastNumberOfIterations() const
{
    return m_lastNumberOfIterations;
}

template <size_t N>
double SnowMPMSolver<N>::GetLastResidual() const
{
    return m_lastResidual;
}

template <size_t N>
int SnowMPMSolver<N>::GetClosedDomainBoundaryFlag() const
{
    return m_closedDomainBoundaryFlag;
}

template <size_t N>
void SnowMPMSolver<N>::SetClosedDomainBoundaryFlag(int flag)
{
    m_closedDomainBoundaryFlag = flag;
}

template <size_t N>
SnowMPMSolver<N>::Builder SnowMPMSolver<N>::GetBuilder()
{
    return Builder{};
}

template <size_t N>
void SnowMPMSolver<N>::OnInitialize()
{
    Base::OnInitialize();
    m_mpmSystemData->TransferFromParticlesToGrid();
    InitializeReferenceVolumes();
    m_maxVelocityGradient = ComputeMaxVelocityGradient();
}

template <size_t N>
unsigned int SnowMPMSolver<N>::GetNumberOfSubTimeSteps(
    double timeIntervalInSeconds) const
{
    if (!std::isfinite(timeIntervalInSeconds) || timeIntervalInSeconds <= 0.0)
    {
        return 1;
    }

    const auto velocities = m_mpmSystemData->Velocities();
    const auto masses = m_mpmSystemData->ParticleMasses();
    const auto volumes = m_mpmSystemData->InitialVolumes();
    const auto states = m_mpmSystemData->DeformationStates();
    const auto spacing = m_mpmSystemData->GridMass().GridSpacing();
    double minSpacing = spacing[0];
    double maxVelocity = 0.0;
    double maxWaveSpeed = 0.0;

    for (size_t axis = 1; axis < N; ++axis)
    {
        minSpacing = std::min(minSpacing, spacing[axis]);
    }

    for (size_t i = 0; i < velocities.Length(); ++i)
    {
        maxVelocity = std::max(maxVelocity, velocities[i].Length());

        if (volumes[i] == 0.0)
        {
            continue;
        }

        if (!std::isfinite(volumes[i]) || volumes[i] < 0.0)
        {
            throw std::invalid_argument{ "Invalid snow reference volume." };
        }

        maxWaveSpeed =
            std::max(maxWaveSpeed, m_constitutiveModel.ComputeWaveSpeed(
                                       states[i], masses[i] / volumes[i]));
    }

    double desiredTimeStep = std::numeric_limits<double>::infinity();
    if (maxVelocity > 0.0)
    {
        desiredTimeStep = std::min(desiredTimeStep, minSpacing / maxVelocity);
    }
    if (!m_isUsingSemiImplicit && maxWaveSpeed > 0.0)
    {
        desiredTimeStep = std::min(desiredTimeStep, minSpacing / maxWaveSpeed);
    }
    if (m_maxVelocityGradient > 0.0)
    {
        desiredTimeStep =
            std::min(desiredTimeStep, 0.2 / m_maxVelocityGradient);
    }

    if (!std::isfinite(desiredTimeStep))
    {
        return 1;
    }

    const double count = std::ceil(timeIntervalInSeconds /
                                   (m_timeStepLimitScale * desiredTimeStep));
    const double maxCount =
        static_cast<double>(std::numeric_limits<unsigned int>::max());

    return static_cast<unsigned int>(std::clamp(count, 1.0, maxCount));
}

template <size_t N>
void SnowMPMSolver<N>::AccumulateForces(double timeStepInSeconds)
{
    (void)timeStepInSeconds;
}

template <size_t N>
void SnowMPMSolver<N>::OnBeginAdvanceTimeStep(double timeStepInSeconds)
{
    m_mpmSystemData->TransferFromParticlesToGrid();
    InitializeReferenceVolumes();
    UpdateGridVelocities(timeStepInSeconds);

    Array1<SizeType> activeNodes;
    Array1<ssize_t> nodeToActive;

    BuildActiveNodes(&activeNodes, &nodeToActive);

    Array1<uint8_t> constrained(activeNodes.Length() * N, uint8_t{ 0 });

    ConstrainGridVelocities(activeNodes, nodeToActive, &constrained);

    if (m_isUsingSemiImplicit)
    {
        SolveGridVelocities(timeStepInSeconds, activeNodes, nodeToActive,
                            constrained);
        ConstrainGridVelocities(activeNodes, nodeToActive, nullptr);
    }
    else
    {
        m_lastNumberOfIterations = 0;
        m_lastResidual = 0.0;
    }

    UpdateDeformation(timeStepInSeconds);
    m_mpmSystemData->TransferFromGridToParticles();
}

template <size_t N>
void SnowMPMSolver<N>::OnEndAdvanceTimeStep(double timeStepInSeconds)
{
    Base::OnEndAdvanceTimeStep(timeStepInSeconds);
    ConstrainParticlesToDomain();
}

template <size_t N>
SnowMPMSolver<N>::SizeType SnowMPMSolver<N>::ClampIndex(
    const Vector<ssize_t, N>& index, const SizeType& dataSize)
{
    SizeType result;

    for (size_t axis = 0; axis < N; ++axis)
    {
        result[axis] = static_cast<size_t>(std::clamp<ssize_t>(
            index[axis], 0, static_cast<ssize_t>(dataSize[axis] - 1)));
    }

    return result;
}

template <size_t N>
void SnowMPMSolver<N>::InitializeReferenceVolumes()
{
    const auto positions = m_mpmSystemData->Positions();
    const auto masses = m_mpmSystemData->ParticleMasses();
    auto volumes = m_mpmSystemData->InitialVolumes();
    const auto& gridMass = m_mpmSystemData->GridMass();
    const auto dataSize = gridMass.DataSize();
    const auto spacing = gridMass.GridSpacing();
    const auto dataOrigin = gridMass.DataOrigin();
    double cellVolume = 1.0;

    for (size_t axis = 0; axis < N; ++axis)
    {
        cellVolume *= spacing[axis];
    }

    if (!std::isfinite(cellVolume) || cellVolume <= 0.0)
    {
        throw std::invalid_argument{ "Invalid snow MPM cell volume." };
    }

    for (size_t i = 0; i < volumes.Length(); ++i)
    {
        if (volumes[i] != 0.0)
        {
            if (!std::isfinite(volumes[i]) || volumes[i] < 0.0)
            {
                throw std::invalid_argument{ "Invalid snow reference volume." };
            }
            continue;
        }

        double referenceDensity = 0.0;
        const auto stencil = CubicBSplineKernel<N>::GetStencil(
            positions[i], spacing, dataOrigin);

        for (const auto& entry : stencil)
        {
            referenceDensity += gridMass(ClampIndex(entry.index, dataSize)) *
                                entry.weight / cellVolume;
        }

        if (!std::isfinite(referenceDensity) || referenceDensity <= 0.0)
        {
            throw std::invalid_argument{ "Invalid snow reference density." };
        }

        volumes[i] = masses[i] / referenceDensity;
    }
}

template <size_t N>
void SnowMPMSolver<N>::UpdateGridVelocities(double timeStepInSeconds)
{
    const auto positions = m_mpmSystemData->Positions();
    const auto velocities = m_mpmSystemData->Velocities();
    const auto masses = m_mpmSystemData->ParticleMasses();
    const auto volumes = m_mpmSystemData->InitialVolumes();
    const auto states = m_mpmSystemData->DeformationStates();
    const auto& gridMass = m_mpmSystemData->GridMass();
    auto& gridVelocities = m_mpmSystemData->GridVelocities();
    const auto dataSize = gridMass.DataSize();
    const auto spacing = gridMass.GridSpacing();
    const auto dataOrigin = gridMass.DataOrigin();

    for (size_t i = 0; i < positions.Length(); ++i)
    {
        const VectorType relativeVelocity =
            velocities[i] - this->GetWind()->Sample(positions[i]);
        const VectorType externalForce =
            masses[i] * this->GetGravity() -
            this->GetDragCoefficient() * relativeVelocity;
        const MatrixType stress =
            m_constitutiveModel.ComputeKirchhoffStress(states[i]);
        const auto stencil = CubicBSplineKernel<N>::GetStencil(
            positions[i], spacing, dataOrigin);

        for (const auto& entry : stencil)
        {
            if (entry.weight == 0.0)
            {
                continue;
            }

            const auto index = ClampIndex(entry.index, dataSize);
            const double nodeMass = gridMass(index);
            if (nodeMass > 0.0)
            {
                gridVelocities(index) +=
                    timeStepInSeconds *
                    (entry.weight * externalForce -
                     volumes[i] * stress * entry.gradient) /
                    nodeMass;
            }
        }
    }
}

template <size_t N>
void SnowMPMSolver<N>::BuildActiveNodes(Array1<SizeType>* activeNodes,
                                        Array1<ssize_t>* nodeToActive) const
{
    const auto& gridMass = m_mpmSystemData->GridMass();
    const auto dataView = gridMass.DataView();

    activeNodes->Clear();
    nodeToActive->Resize(dataView.Length(), ssize_t{ -1 });
    nodeToActive->Fill(ssize_t{ -1 });

    gridMass.ForEachDataPointIndex([&gridMass, activeNodes, dataView,
                                    nodeToActive](const SizeType& index) {
        if (gridMass(index) > 0.0)
        {
            (*nodeToActive)[dataView.Index(index)] =
                static_cast<ssize_t>(activeNodes->Length());
            activeNodes->Append(index);
        }
    });
}

template <size_t N>
void SnowMPMSolver<N>::ConstrainGridVelocities(
    const Array1<SizeType>& activeNodes, const Array1<ssize_t>& nodeToActive,
    Array1<uint8_t>* constrained)
{
    const auto& gridMass = m_mpmSystemData->GridMass();
    auto& gridVelocities = m_mpmSystemData->GridVelocities();
    const auto dataView = gridMass.DataView();

    if (constrained != nullptr)
    {
        constrained->Resize(activeNodes.Length() * N, uint8_t{ 0 });
        constrained->Fill(uint8_t{ 0 });
    }

    gridVelocities.ParallelForEachDataPointIndex(
        [this, constrained, &gridMass, &nodeToActive,
         dataView](const SizeType& index) {
            if (gridMass(index) <= 0.0)
            {
                return;
            }

            const ssize_t active = nodeToActive[dataView.Index(index)];

            if (active < 0)
            {
                return;
            }

            ConstrainGridVelocityAtNode(index, static_cast<size_t>(active),
                                        constrained);
        });
}

template <size_t N>
void SnowMPMSolver<N>::ConstrainGridVelocityAtNode(const SizeType& index,
                                                   size_t active,
                                                   Array1<uint8_t>* constrained)
{
    auto& gridVelocities = m_mpmSystemData->GridVelocities();
    VectorType velocity = gridVelocities(index);

    ApplyGridColliderConstraint(index, active, constrained, &velocity);
    ApplyGridDomainConstraint(index, active, constrained, &velocity);

    gridVelocities(index) = velocity;
}

template <size_t N>
void SnowMPMSolver<N>::ApplyGridColliderConstraint(const SizeType& index,
                                                   size_t active,
                                                   Array1<uint8_t>* constrained,
                                                   VectorType* velocity) const
{
    const auto collider = this->GetCollider();
    if (collider == nullptr)
    {
        return;
    }

    const VectorType incoming = *velocity;
    VectorType position =
        m_mpmSystemData->GridVelocities().DataPosition()(index);

    collider->ResolveCollision(0.0, 0.0, &position, velocity);

    if (constrained != nullptr && *velocity != incoming)
    {
        for (size_t axis = 0; axis < N; ++axis)
        {
            (*constrained)[active * N + axis] = uint8_t{ 1 };
        }
    }
}

template <size_t N>
void SnowMPMSolver<N>::ApplyGridDomainConstraint(const SizeType& index,
                                                 size_t active,
                                                 Array1<uint8_t>* constrained,
                                                 VectorType* velocity) const
{
    static constexpr std::array lowerFlags{ DIRECTION_LEFT, DIRECTION_DOWN,
                                            DIRECTION_BACK };
    static constexpr std::array upperFlags{ DIRECTION_RIGHT, DIRECTION_UP,
                                            DIRECTION_FRONT };
    const auto dataSize = m_mpmSystemData->GridVelocities().DataSize();

    for (size_t axis = 0; axis < N; ++axis)
    {
        const bool exceedsLower =
            (m_closedDomainBoundaryFlag & lowerFlags[axis]) != 0 &&
            index[axis] == 0 && (*velocity)[axis] < 0.0;
        const bool exceedsUpper =
            (m_closedDomainBoundaryFlag & upperFlags[axis]) != 0 &&
            index[axis] == dataSize[axis] - 1 && (*velocity)[axis] > 0.0;

        if (exceedsLower || exceedsUpper)
        {
            (*velocity)[axis] = 0.0;

            if (constrained != nullptr)
            {
                (*constrained)[active * N + axis] = uint8_t{ 1 };
            }
        }
    }
}

template <size_t N>
SnowMPMSolver<N>::MatrixType
SnowMPMSolver<N>::ComputeParticleDeformationDifferential(
    const Stencil& stencil, const Array1<ssize_t>& nodeToActive,
    const VectorND& input, const MatrixType& elastic) const
{
    const auto& gridMass = m_mpmSystemData->GridMass();
    const auto dataSize = gridMass.DataSize();
    const auto dataView = gridMass.DataView();
    MatrixType result;

    for (const auto& entry : stencil)
    {
        if (entry.weight == 0.0)
        {
            continue;
        }

        const SizeType index = ClampIndex(entry.index, dataSize);
        const ssize_t active = nodeToActive[dataView.Index(index)];

        if (active < 0)
        {
            continue;
        }

        VectorType velocityDifferential;

        for (size_t axis = 0; axis < N; ++axis)
        {
            velocityDifferential[axis] =
                input[static_cast<size_t>(active) * N + axis];
        }

        for (size_t row = 0; row < N; ++row)
        {
            for (size_t column = 0; column < N; ++column)
            {
                result(row, column) +=
                    velocityDifferential[row] * entry.gradient[column];
            }
        }
    }

    result *= elastic;
    return result;
}

template <size_t N>
void SnowMPMSolver<N>::AccumulateParticleHessian(
    const Stencil& stencil, const Array1<ssize_t>& nodeToActive, double volume,
    const MatrixType& elastic, const MatrixType& stressDifferential,
    VectorND* output) const
{
    const auto& gridMass = m_mpmSystemData->GridMass();
    const auto dataSize = gridMass.DataSize();
    const auto dataView = gridMass.DataView();

    for (const auto& entry : stencil)
    {
        if (entry.weight == 0.0)
        {
            continue;
        }

        const SizeType index = ClampIndex(entry.index, dataSize);
        const ssize_t active = nodeToActive[dataView.Index(index)];

        if (active < 0)
        {
            continue;
        }

        const VectorType contribution =
            volume * stressDifferential * elastic.Transposed() * entry.gradient;

        for (size_t axis = 0; axis < N; ++axis)
        {
            (*output)[static_cast<size_t>(active) * N + axis] +=
                contribution[axis];
        }
    }
}

template <size_t N>
void SnowMPMSolver<N>::ApplyElasticHessian(const Array1<SizeType>& activeNodes,
                                           const Array1<ssize_t>& nodeToActive,
                                           const VectorND& input,
                                           VectorND* output) const
{
    const auto positions = m_mpmSystemData->Positions();
    const auto volumes = m_mpmSystemData->InitialVolumes();
    const auto states = m_mpmSystemData->DeformationStates();
    const auto& gridMass = m_mpmSystemData->GridMass();
    const auto spacing = gridMass.GridSpacing();
    const auto dataOrigin = gridMass.DataOrigin();

    output->Resize(activeNodes.Length() * N, 0.0);
    output->Fill(0.0);

    for (size_t p = 0; p < positions.Length(); ++p)
    {
        const auto stencil = CubicBSplineKernel<N>::GetStencil(
            positions[p], spacing, dataOrigin);
        const MatrixType differential = ComputeParticleDeformationDifferential(
            stencil, nodeToActive, input, states[p].elastic);
        const MatrixType stressDifferential =
            m_constitutiveModel.ComputeFirstPiolaStressDifferential(
                states[p], differential);

        AccumulateParticleHessian(stencil, nodeToActive, volumes[p],
                                  states[p].elastic, stressDifferential,
                                  output);
    }
}

template <size_t N>
VectorND SnowMPMSolver<N>::GatherActiveGridVelocities(
    const Array1<SizeType>& activeNodes) const
{
    const auto& gridVelocities = m_mpmSystemData->GridVelocities();
    VectorND result(activeNodes.Length() * N, 0.0);

    for (size_t active = 0; active < activeNodes.Length(); ++active)
    {
        const VectorType velocity = gridVelocities(activeNodes[active]);

        for (size_t axis = 0; axis < N; ++axis)
        {
            result[active * N + axis] = velocity[axis];
        }
    }

    return result;
}

template <size_t N>
VectorND SnowMPMSolver<N>::BuildSemiImplicitRightHandSide(
    double dtSquared, const Array1<SizeType>& activeNodes,
    const Array1<ssize_t>& nodeToActive, const Array1<uint8_t>& constrained,
    const VectorND& velocities) const
{
    VectorND hessian(velocities.GetRows(), 0.0);
    ApplyElasticHessian(activeNodes, nodeToActive, velocities, &hessian);

    VectorND result(velocities.GetRows(), 0.0);
    for (size_t i = 0; i < result.GetRows(); ++i)
    {
        if (constrained[i] == 0)
        {
            result[i] = -dtSquared * hessian[i];
        }
    }

    return result;
}

template <size_t N>
VectorND SnowMPMSolver<N>::SolveGridVelocityCorrection(
    double dtSquared, const Array1<SizeType>& activeNodes,
    const Array1<ssize_t>& nodeToActive, const Array1<uint8_t>& constrained,
    const VectorND& rhs, double initialResidual)
{
    const LinearSystem system{ this, &activeNodes, &nodeToActive, &constrained,
                               dtSquared };
    VectorND correction(rhs.GetRows(), 0.0);
    VectorND residual(rhs.GetRows(), 0.0);
    VectorND direction(rhs.GetRows(), 0.0);
    VectorND product(rhs.GetRows(), 0.0);
    VectorND image(rhs.GetRows(), 0.0);
    double residualNorm = initialResidual;

    CR<LinearSystemBLAS>(system, rhs, m_maxNumberOfIterations,
                         m_tolerance * initialResidual, &correction, &residual,
                         &direction, &product, &image,
                         &m_lastNumberOfIterations, &residualNorm);

    m_lastResidual = residualNorm / initialResidual;
    return correction;
}

template <size_t N>
VectorND SnowMPMSolver<N>::ComputeGridVelocityUpdate(
    double timeStepInSeconds, const Array1<SizeType>& activeNodes,
    const Array1<ssize_t>& nodeToActive, const Array1<uint8_t>& constrained)
{
    const double dtSquared = timeStepInSeconds * timeStepInSeconds;
    const VectorND vStar = GatherActiveGridVelocities(activeNodes);
    const VectorND rhs = BuildSemiImplicitRightHandSide(
        dtSquared, activeNodes, nodeToActive, constrained, vStar);
    const double initialResidual = LinearSystemBLAS::L2Norm(rhs);

    if (!std::isfinite(initialResidual))
    {
        m_lastResidual = std::numeric_limits<double>::infinity();
        throw std::runtime_error{
            "Semi-implicit snow solve failed to converge."
        };
    }
    if (initialResidual == 0.0)
    {
        return vStar;
    }

    const VectorND correction =
        SolveGridVelocityCorrection(dtSquared, activeNodes, nodeToActive,
                                    constrained, rhs, initialResidual);
    VectorND result(vStar + correction);

    if (const bool isFinite = std::ranges::all_of(
            result, [](double value) { return std::isfinite(value); });
        !isFinite || !std::isfinite(m_lastResidual) ||
        m_lastResidual > m_tolerance)
    {
        throw std::runtime_error{
            "Semi-implicit snow solve failed to converge."
        };
    }

    return result;
}

template <size_t N>
void SnowMPMSolver<N>::StoreActiveGridVelocities(
    const Array1<SizeType>& activeNodes, const VectorND& velocities)
{
    auto& gridVelocities = m_mpmSystemData->GridVelocities();

    for (size_t active = 0; active < activeNodes.Length(); ++active)
    {
        VectorType velocity;

        for (size_t axis = 0; axis < N; ++axis)
        {
            velocity[axis] = velocities[active * N + axis];
        }

        gridVelocities(activeNodes[active]) = velocity;
    }
}

template <size_t N>
void SnowMPMSolver<N>::SolveGridVelocities(double timeStepInSeconds,
                                           const Array1<SizeType>& activeNodes,
                                           const Array1<ssize_t>& nodeToActive,
                                           const Array1<uint8_t>& constrained)
{
    auto& gridVelocities = m_mpmSystemData->GridVelocities();
    const auto& gridVelocitiesBeforeUpdate =
        m_mpmSystemData->GridVelocitiesBeforeUpdate();

    m_lastNumberOfIterations = 0;
    m_lastResidual = 0.0;

    try
    {
        if (!activeNodes.IsEmpty())
        {
            const VectorND nextVelocities = ComputeGridVelocityUpdate(
                timeStepInSeconds, activeNodes, nodeToActive, constrained);
            StoreActiveGridVelocities(activeNodes, nextVelocities);
        }
    }
    catch (...)
    {
        gridVelocities.Set(gridVelocitiesBeforeUpdate);
        throw;
    }
}

template <size_t N>
SnowMPMSolver<N>::MatrixType SnowMPMSolver<N>::ComputeVelocityGradient(
    size_t particleIndex) const
{
    const auto positions = m_mpmSystemData->Positions();
    const auto& gridVelocities = m_mpmSystemData->GridVelocities();
    const auto dataSize = gridVelocities.DataSize();
    const auto stencil = CubicBSplineKernel<N>::GetStencil(
        positions[particleIndex], gridVelocities.GridSpacing(),
        gridVelocities.DataOrigin());
    MatrixType result;

    for (const auto& entry : stencil)
    {
        if (entry.weight != 0.0)
        {
            const VectorType velocity =
                gridVelocities(ClampIndex(entry.index, dataSize));
            for (size_t row = 0; row < N; ++row)
            {
                for (size_t column = 0; column < N; ++column)
                {
                    result(row, column) +=
                        velocity[row] * entry.gradient[column];
                }
            }
        }
    }

    return result;
}

template <size_t N>
double SnowMPMSolver<N>::ComputeMaxVelocityGradient() const
{
    double result = 0.0;

    for (size_t i = 0; i < m_mpmSystemData->NumberOfParticles(); ++i)
    {
        result = std::max(result, ComputeVelocityGradient(i).AbsMax());
    }

    return result;
}

template <size_t N>
void SnowMPMSolver<N>::UpdateDeformation(double timeStepInSeconds)
{
    auto states = m_mpmSystemData->DeformationStates();
    m_maxVelocityGradient = 0.0;

    for (size_t i = 0; i < states.Length(); ++i)
    {
        const MatrixType velocityGradient = ComputeVelocityGradient(i);
        m_maxVelocityGradient =
            std::max(m_maxVelocityGradient, velocityGradient.AbsMax());
        states[i] = m_constitutiveModel.Update(
            MatrixType::MakeIdentity() + timeStepInSeconds * velocityGradient,
            states[i]);
    }
}

template <size_t N>
void SnowMPMSolver<N>::ConstrainParticlesToDomain()
{
    static constexpr std::array lowerFlags{ DIRECTION_LEFT, DIRECTION_DOWN,
                                            DIRECTION_BACK };
    static constexpr std::array upperFlags{ DIRECTION_RIGHT, DIRECTION_UP,
                                            DIRECTION_FRONT };
    const auto domain = m_mpmSystemData->GridMass().GetBoundingBox();
    auto positions = m_mpmSystemData->Positions();
    auto velocities = m_mpmSystemData->Velocities();

    ParallelFor(
        ZERO_SIZE, positions.Length(),
        [&domain, &positions, &velocities, this](size_t i) {
            for (size_t axis = 0; axis < N; ++axis)
            {
                if ((m_closedDomainBoundaryFlag & lowerFlags[axis]) != 0 &&
                    positions[i][axis] <= domain.lowerCorner[axis])
                {
                    positions[i][axis] = domain.lowerCorner[axis];
                    velocities[i][axis] = std::max(velocities[i][axis], 0.0);
                }
                if ((m_closedDomainBoundaryFlag & upperFlags[axis]) != 0 &&
                    positions[i][axis] >= domain.upperCorner[axis])
                {
                    positions[i][axis] = domain.upperCorner[axis];
                    velocities[i][axis] = std::min(velocities[i][axis], 0.0);
                }
            }
        });
}

template <size_t N>
SnowMPMSolver<N>::Builder& SnowMPMSolver<N>::Builder::WithResolution(
    const SizeType& resolution)
{
    m_resolution = resolution;
    return *this;
}

template <size_t N>
SnowMPMSolver<N>::Builder& SnowMPMSolver<N>::Builder::WithGridSpacing(
    const VectorType& gridSpacing)
{
    m_gridSpacing = gridSpacing;
    return *this;
}

template <size_t N>
SnowMPMSolver<N>::Builder& SnowMPMSolver<N>::Builder::WithOrigin(
    const VectorType& gridOrigin)
{
    m_gridOrigin = gridOrigin;
    return *this;
}

template <size_t N>
SnowMPMSolver<N>::Builder& SnowMPMSolver<N>::Builder::WithRadius(double radius)
{
    m_radius = radius;
    return *this;
}

template <size_t N>
SnowMPMSolver<N>::Builder& SnowMPMSolver<N>::Builder::WithMass(double mass)
{
    m_mass = mass;
    return *this;
}

template <size_t N>
SnowMPMSolver<N> SnowMPMSolver<N>::Builder::Build() const
{
    return SnowMPMSolver{ m_resolution, m_gridSpacing, m_gridOrigin, m_radius,
                          m_mass };
}

template <size_t N>
std::shared_ptr<SnowMPMSolver<N>> SnowMPMSolver<N>::Builder::MakeShared() const
{
    return std::make_shared<SnowMPMSolver>(m_resolution, m_gridSpacing,
                                           m_gridOrigin, m_radius, m_mass);
}
}  // namespace CubbyFlow

#endif
