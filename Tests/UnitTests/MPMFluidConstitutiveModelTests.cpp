#include "gtest/gtest.h"

#include <Core/Particle/MPM/MPMFluidConstitutiveModel.hpp>

#include <limits>
#include <stdexcept>

using namespace CubbyFlow;

namespace
{
template <size_t N>
void ExpectDensityFromMassAndVolume()
{
    const MPMFluidConstitutiveModel<N> model;

    EXPECT_DOUBLE_EQ(model.ComputeDensity(2.0, 0.002), 1000.0);
}

template <size_t N>
void ExpectPressureFollowsEquationOfState()
{
    const MPMFluidConstitutiveModel<N> model{ 1000.0, 10.0, 2.0, 0.25 };

    EXPECT_DOUBLE_EQ(model.ComputePressure(1000.0), 0.0);
    EXPECT_DOUBLE_EQ(model.ComputePressure(2000.0), 150000.0);
    EXPECT_DOUBLE_EQ(model.ComputePressure(500.0), -9375.0);
}

template <size_t N>
void ExpectCauchyStressIsNegativeIsotropicPressure()
{
    const MPMFluidConstitutiveModel<N> model;
    const auto stress = model.ComputeCauchyStress(123.0);

    for (size_t row = 0; row < N; ++row)
    {
        for (size_t column = 0; column < N; ++column)
        {
            EXPECT_DOUBLE_EQ(stress(row, column), row == column ? -123.0 : 0.0);
        }
    }
}

template <size_t N>
void ExpectDefaultParametersAreExposed()
{
    const MPMFluidConstitutiveModel<N> model;

    EXPECT_DOUBLE_EQ(model.GetTargetDensity(), 1000.0);
    EXPECT_DOUBLE_EQ(model.GetSpeedOfSound(), 100.0);
    EXPECT_DOUBLE_EQ(model.GetEosExponent(), 7.0);
    EXPECT_DOUBLE_EQ(model.GetNegativePressureScale(), 0.0);
}

template <size_t N>
void ExpectInvalidParametersAreRejected()
{
    using Model = MPMFluidConstitutiveModel<N>;
    constexpr double inf = std::numeric_limits<double>::infinity();
    constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    constexpr double max = std::numeric_limits<double>::max();
    constexpr double min = std::numeric_limits<double>::denorm_min();

    EXPECT_THROW(Model{ 0.0 }, std::invalid_argument);
    EXPECT_THROW(Model{ inf }, std::invalid_argument);
    EXPECT_THROW(Model{ nan }, std::invalid_argument);
    EXPECT_THROW((Model{ 1000.0, 0.0 }), std::invalid_argument);
    EXPECT_THROW((Model{ 1000.0, inf }), std::invalid_argument);
    EXPECT_THROW((Model{ 1000.0, nan }), std::invalid_argument);
    EXPECT_THROW((Model{ 1000.0, min }), std::invalid_argument);
    EXPECT_THROW((Model{ 1000.0, 100.0, 0.5 }), std::invalid_argument);
    EXPECT_THROW((Model{ 1000.0, 100.0, inf }), std::invalid_argument);
    EXPECT_THROW((Model{ 1000.0, 100.0, nan }), std::invalid_argument);
    EXPECT_THROW((Model{ 1000.0, 100.0, 7.0, -0.1 }), std::invalid_argument);
    EXPECT_THROW((Model{ 1000.0, 100.0, 7.0, 1.1 }), std::invalid_argument);
    EXPECT_THROW((Model{ 1000.0, 100.0, 7.0, nan }), std::invalid_argument);
    EXPECT_THROW((Model{ max, 2.0 }), std::invalid_argument);
}

template <size_t N>
void ExpectInvalidDensityInputsAreRejected()
{
    const MPMFluidConstitutiveModel<N> model;
    constexpr double inf = std::numeric_limits<double>::infinity();
    constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    constexpr double max = std::numeric_limits<double>::max();
    constexpr double min = std::numeric_limits<double>::denorm_min();

    EXPECT_THROW((void)model.ComputeDensity(0.0, 1.0), std::invalid_argument);
    EXPECT_THROW((void)model.ComputeDensity(-1.0, 1.0), std::invalid_argument);
    EXPECT_THROW((void)model.ComputeDensity(inf, 1.0), std::invalid_argument);
    EXPECT_THROW((void)model.ComputeDensity(nan, 1.0), std::invalid_argument);
    EXPECT_THROW((void)model.ComputeDensity(1.0, 0.0), std::invalid_argument);
    EXPECT_THROW((void)model.ComputeDensity(1.0, -1.0), std::invalid_argument);
    EXPECT_THROW((void)model.ComputeDensity(1.0, inf), std::invalid_argument);
    EXPECT_THROW((void)model.ComputeDensity(1.0, nan), std::invalid_argument);
    EXPECT_THROW((void)model.ComputeDensity(max, min), std::invalid_argument);
    EXPECT_THROW((void)model.ComputeDensity(min, max), std::invalid_argument);
}

template <size_t N>
void ExpectInvalidPressureInputsAreRejected()
{
    const MPMFluidConstitutiveModel<N> model;
    constexpr double inf = std::numeric_limits<double>::infinity();
    constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    constexpr double max = std::numeric_limits<double>::max();

    EXPECT_THROW((void)model.ComputePressure(0.0), std::invalid_argument);
    EXPECT_THROW((void)model.ComputePressure(-1.0), std::invalid_argument);
    EXPECT_THROW((void)model.ComputePressure(inf), std::invalid_argument);
    EXPECT_THROW((void)model.ComputePressure(nan), std::invalid_argument);
    EXPECT_THROW((void)model.ComputePressure(max), std::invalid_argument);
}

template <size_t N>
void ExpectInvalidStressInputsAreRejected()
{
    const MPMFluidConstitutiveModel<N> model;
    constexpr double inf = std::numeric_limits<double>::infinity();
    constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    EXPECT_THROW((void)model.ComputeCauchyStress(inf), std::invalid_argument);
    EXPECT_THROW((void)model.ComputeCauchyStress(nan), std::invalid_argument);
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

TEST(MPMFluidConstitutiveModel, DensityFromMassAndVolume)
{
    RUN_FOR_2D_AND_3D(ExpectDensityFromMassAndVolume);
}

TEST(MPMFluidConstitutiveModel, PressureFollowsEquationOfState)
{
    RUN_FOR_2D_AND_3D(ExpectPressureFollowsEquationOfState);
}

TEST(MPMFluidConstitutiveModel, CauchyStress)
{
    RUN_FOR_2D_AND_3D(ExpectCauchyStressIsNegativeIsotropicPressure);
}

TEST(MPMFluidConstitutiveModel, DefaultParameters)
{
    RUN_FOR_2D_AND_3D(ExpectDefaultParametersAreExposed);
}

TEST(MPMFluidConstitutiveModel, InvalidParameters)
{
    RUN_FOR_2D_AND_3D(ExpectInvalidParametersAreRejected);
}

TEST(MPMFluidConstitutiveModel, InvalidDensityInputs)
{
    RUN_FOR_2D_AND_3D(ExpectInvalidDensityInputsAreRejected);
}

TEST(MPMFluidConstitutiveModel, InvalidPressureInputs)
{
    RUN_FOR_2D_AND_3D(ExpectInvalidPressureInputsAreRejected);
}

TEST(MPMFluidConstitutiveModel, InvalidStressInputs)
{
    RUN_FOR_2D_AND_3D(ExpectInvalidStressInputsAreRejected);
}
