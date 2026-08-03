#include "gtest/gtest.h"

#include <Core/Particle/MPM/SnowConstitutiveModel.hpp>

#include <cmath>
#include <limits>
#include <stdexcept>

using namespace CubbyFlow;

namespace
{
template <size_t N>
using MatrixD = Matrix<double, N, N>;

template <size_t N>
MatrixD<N> MakeStretch(double value)
{
    MatrixD<N> result = MatrixD<N>::MakeIdentity();
    result(0, 0) = value;

    return result;
}

template <size_t N>
MatrixD<N> MakeQuarterTurn()
{
    MatrixD<N> result = MatrixD<N>::MakeIdentity();
    result(0, 0) = 0.0;
    result(1, 1) = 0.0;
    result(0, 1) = -1.0;
    result(1, 0) = 1.0;

    return result;
}

template <size_t N>
void ExpectIdentityStateAndZeroStress()
{
    const SnowConstitutiveModel<N> model;
    const SnowDeformationState<N> initial;
    const auto identity = MatrixD<N>::MakeIdentity();
    const auto updated = model.Update(identity, initial);

    EXPECT_TRUE(updated.elastic.IsSimilar(identity, 1e-12));
    EXPECT_TRUE(updated.plastic.IsSimilar(identity, 1e-12));
    EXPECT_NEAR(model.ComputeKirchhoffStress(updated).FrobeniusNorm(), 0.0,
                1e-12);
}

template <size_t N>
void ExpectElasticDeformationUnchanged()
{
    const SnowConstitutiveModel<N> model;
    const auto deformation = MakeStretch<N>(1.005);
    const auto updated = model.Update(deformation, {});

    EXPECT_TRUE(updated.elastic.IsSimilar(deformation, 1e-12));
    EXPECT_TRUE(updated.plastic.IsSimilar(MatrixD<N>::MakeIdentity(), 1e-12));
}

template <size_t N>
void ExpectIncrementAppliedOnLeft()
{
    SnowDeformationState<N> state;
    state.elastic = MakeStretch<N>(1.005);

    const auto updated =
        SnowConstitutiveModel<N>{}.Update(MakeQuarterTurn<N>(), state);

    MatrixD<N> expected = MatrixD<N>::MakeIdentity();
    expected(0, 0) = 0.0;
    expected(1, 1) = 0.0;
    expected(0, 1) = -1.0;
    expected(1, 0) = 1.005;

    EXPECT_TRUE(updated.elastic.IsSimilar(expected, 1e-10));
    EXPECT_TRUE(updated.plastic.IsSimilar(MatrixD<N>::MakeIdentity(), 1e-10));
}

template <size_t N>
void ExpectRigidRotationHasZeroStress()
{
    SnowDeformationState<N> state;
    state.elastic = MakeQuarterTurn<N>();

    const auto stress =
        SnowConstitutiveModel<N>{}.ComputeKirchhoffStress(state);

    EXPECT_NEAR(stress.FrobeniusNorm(), 0.0, 1e-10);
}

template <size_t N>
void ExpectKnownElasticStress()
{
    const SnowConstitutiveModel<N> model{ 1000.0, 0.2, 0.025, 0.0075, 0.0 };

    SnowDeformationState<N> state;
    state.elastic = MakeStretch<N>(1.005);

    const auto stress = model.ComputeKirchhoffStress(state);

    MatrixD<N> expected = 1.39583333333333 * MatrixD<N>::MakeIdentity();
    expected(0, 0) = 5.58333333333333;

    EXPECT_TRUE(stress.IsSimilar(expected, 1e-10));
}

template <size_t N>
void ExpectCompressionClamped()
{
    const SnowConstitutiveModel<N> model;
    const auto deformation = MakeStretch<N>(0.9);
    const auto updated = model.Update(deformation, {});

    EXPECT_NEAR(updated.elastic(0, 0), 0.975, 1e-12);
    EXPECT_LT(updated.plastic(0, 0), 1.0);
}

template <size_t N>
void ExpectStretchClamped()
{
    const SnowConstitutiveModel<N> model;
    const auto deformation = MakeStretch<N>(1.1);
    const auto updated = model.Update(deformation, {});

    EXPECT_NEAR(updated.elastic(0, 0), 1.0075, 1e-12);
    EXPECT_GT(updated.plastic(0, 0), 1.0);
}

template <size_t N>
void ExpectProjectionPreservesTotalDeformation()
{
    SnowDeformationState<N> initial;
    initial.plastic = MakeStretch<N>(0.8);

    const auto increment = MakeStretch<N>(0.9);
    const MatrixD<N> expectedTotal =
        increment * initial.elastic * initial.plastic;
    const auto updated = SnowConstitutiveModel<N>{}.Update(increment, initial);
    const MatrixD<N> actualTotal = updated.elastic * updated.plastic;

    EXPECT_TRUE(actualTotal.IsSimilar(expectedTotal, 1e-12));
}

template <size_t N>
void ExpectPlasticCompressionHardens()
{
    const SnowConstitutiveModel<N> model{ 1000.0, 0.2, 0.025, 0.0075, 10.0 };

    SnowDeformationState<N> unpacked;
    unpacked.elastic = MakeStretch<N>(1.005);

    auto packed = unpacked;
    packed.plastic = MakeStretch<N>(0.8);

    const auto unpackedStress = model.ComputeKirchhoffStress(unpacked);
    const auto packedStress = model.ComputeKirchhoffStress(packed);

    EXPECT_NEAR(packedStress(0, 0) / unpackedStress(0, 0), std::exp(2.0),
                1e-10);
}

template <size_t N>
void ExpectRotatedStretchProjectsAndSoftens()
{
    const SnowConstitutiveModel<N> model{ 1000.0, 0.2, 0.025, 0.0075, 10.0 };
    const auto rotation = MakeQuarterTurn<N>();
    const MatrixD<N> deformation = rotation * MakeStretch<N>(1.1);
    const MatrixD<N> expectedElastic = rotation * MakeStretch<N>(1.0075);
    const auto updated = model.Update(deformation, {});

    EXPECT_TRUE(updated.elastic.IsSimilar(expectedElastic, 1e-10));
    EXPECT_TRUE(
        (updated.elastic * updated.plastic).IsSimilar(deformation, 1e-10));

    auto unpacked = updated;
    unpacked.plastic = MatrixD<N>::MakeIdentity();

    const double plasticDeterminant = updated.plastic.Determinant();
    const auto unpackedStress = model.ComputeKirchhoffStress(unpacked);
    const auto softenedStress = model.ComputeKirchhoffStress(updated);

    EXPECT_GT(plasticDeterminant, 1.0);
    EXPECT_NEAR(softenedStress.FrobeniusNorm() / unpackedStress.FrobeniusNorm(),
                std::exp(10.0 * (1.0 - plasticDeterminant)), 1e-10);
}

template <size_t N>
void ExpectInvalidStatesRejected()
{
    auto nonFinite = MatrixD<N>::MakeIdentity();
    nonFinite(0, 0) = std::numeric_limits<double>::quiet_NaN();

    EXPECT_THROW((void)SnowConstitutiveModel<N>{}.Update(nonFinite, {}),
                 std::invalid_argument);

    SnowDeformationState<N> inverted;
    inverted.plastic = MakeStretch<N>(-1.0);

    EXPECT_THROW(
        (void)SnowConstitutiveModel<N>{}.ComputeKirchhoffStress(inverted),
        std::invalid_argument);
}
}  // namespace

#define RUN_FOR_2D_AND_3D(function) \
    do                              \
    {                               \
        {                           \
            SCOPED_TRACE("N = 2");  \
            function<2>();          \
        }                           \
        {                           \
            SCOPED_TRACE("N = 3");  \
            function<3>();          \
        }                           \
    } while (false)

TEST(SnowConstitutiveModel, Identity)
{
    RUN_FOR_2D_AND_3D(ExpectIdentityStateAndZeroStress);
}

TEST(SnowConstitutiveModel, ElasticDeformation)
{
    RUN_FOR_2D_AND_3D(ExpectElasticDeformationUnchanged);
}

TEST(SnowConstitutiveModel, IncrementAppliedOnLeft)
{
    RUN_FOR_2D_AND_3D(ExpectIncrementAppliedOnLeft);
}

TEST(SnowConstitutiveModel, RigidRotationStress)
{
    RUN_FOR_2D_AND_3D(ExpectRigidRotationHasZeroStress);
}

TEST(SnowConstitutiveModel, ElasticStress)
{
    RUN_FOR_2D_AND_3D(ExpectKnownElasticStress);
}

TEST(SnowConstitutiveModel, Compression)
{
    RUN_FOR_2D_AND_3D(ExpectCompressionClamped);
}

TEST(SnowConstitutiveModel, Stretch)
{
    RUN_FOR_2D_AND_3D(ExpectStretchClamped);
}

TEST(SnowConstitutiveModel, PlasticProjection)
{
    RUN_FOR_2D_AND_3D(ExpectProjectionPreservesTotalDeformation);
}

TEST(SnowConstitutiveModel, MultiAxisProjection3D)
{
    MatrixD<3> deformation = MatrixD<3>::MakeIdentity();
    deformation(0, 0) = 0.9;
    deformation(1, 1) = 1.1;
    deformation(2, 2) = 1.2;

    const auto updated = SnowConstitutiveModel3{}.Update(deformation, {});

    EXPECT_NEAR(updated.elastic(0, 0), 0.975, 1e-12);
    EXPECT_NEAR(updated.elastic(1, 1), 1.0075, 1e-12);
    EXPECT_NEAR(updated.elastic(2, 2), 1.0075, 1e-12);
    EXPECT_TRUE(
        (updated.elastic * updated.plastic).IsSimilar(deformation, 1e-12));
}

TEST(SnowConstitutiveModel, Hardening)
{
    RUN_FOR_2D_AND_3D(ExpectPlasticCompressionHardens);
}

TEST(SnowConstitutiveModel, RotatedStretchProjectionAndSoftening)
{
    RUN_FOR_2D_AND_3D(ExpectRotatedStretchProjectsAndSoftens);
}

TEST(SnowConstitutiveModel, InvalidState)
{
    RUN_FOR_2D_AND_3D(ExpectInvalidStatesRejected);
}

TEST(SnowConstitutiveModel, InvalidParameters)
{
    EXPECT_THROW(SnowConstitutiveModel2{ 0.0 }, std::invalid_argument);
    EXPECT_THROW((SnowConstitutiveModel2{ 1.0, 0.5 }), std::invalid_argument);
    EXPECT_THROW((SnowConstitutiveModel2{ 1.0, 0.2, 1.0 }),
                 std::invalid_argument);
    EXPECT_THROW((SnowConstitutiveModel2{ 1.0, 0.2, 0.1, -0.1 }),
                 std::invalid_argument);
    EXPECT_THROW((SnowConstitutiveModel2{ 1.0, 0.2, 0.1, 0.1, -1.0 }),
                 std::invalid_argument);
}
