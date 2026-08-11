#include "gtest/gtest.h"

#include <Core/Math/CG.hpp>
#include <Core/Matrix/Matrix.hpp>

using namespace CubbyFlow;

TEST(CG, Solve)
{
    // Solve:
    // | 4 1 | |x|   |1|
    // | 1 3 | |y| = |2|

    const Matrix2x2D matrix(4.0, 1.0, 1.0, 3.0);
    const Vector2D rhs(1.0, 2.0);

    typedef BLAS<double, Vector2D, Matrix2x2D> BLASType;

    {
        // Zero iteration should give proper residual from iteration data.
        Vector2D x, r, d, q, s;
        unsigned int lastNumIter;
        double lastResidualNorm;

        CG<BLASType>(matrix, rhs, 0, 0.0, &x, &r, &d, &q, &s, &lastNumIter,
                     &lastResidualNorm);

        EXPECT_DOUBLE_EQ(0.0, x.x);
        EXPECT_DOUBLE_EQ(0.0, x.y);

        EXPECT_DOUBLE_EQ(std::sqrt(5.0), lastResidualNorm);
        EXPECT_EQ(0u, lastNumIter);
    }

    {
        Vector2D x, r, d, q, s;
        unsigned int lastNumIter;
        double lastResidualNorm;

        CG<BLASType>(matrix, rhs, 10, 0.0, &x, &r, &d, &q, &s, &lastNumIter,
                     &lastResidualNorm);

        EXPECT_DOUBLE_EQ(1.0 / 11.0, x.x);
        EXPECT_DOUBLE_EQ(7.0 / 11.0, x.y);

        EXPECT_LE(lastResidualNorm, std::numeric_limits<double>::epsilon());
        EXPECT_LE(lastNumIter, 2u);
    }
}

TEST(PCG, Solve)
{
    // Solve:
    // | 4 1 | |x|   |1|
    // | 1 3 | |y| = |2|

    const Matrix2x2D matrix(4.0, 1.0, 1.0, 3.0);
    const Vector2D rhs(1.0, 2.0);

    typedef BLAS<double, Vector2D, Matrix2x2D> BLASType;

    struct DiagonalPreconditioner
    {
        Vector2D precond;

        void Build(const Matrix2x2D& matrix)
        {
            precond.x = matrix(0, 0);
            precond.y = matrix(1, 1);
        }

        void Solve(const Vector2D& b, Vector2D* x) const
        {
            *x = ElemDiv(b, precond);
        }
    };

    {
        Vector2D x, r, d, q, s;
        unsigned int lastNumIter;
        double lastResidualNorm;
        DiagonalPreconditioner precond;

        precond.Build(matrix);

        PCG<BLASType>(matrix, rhs, 10, 0.0, &precond, &x, &r, &d, &q, &s,
                      &lastNumIter, &lastResidualNorm);

        EXPECT_DOUBLE_EQ(1.0 / 11.0, x.x);
        EXPECT_DOUBLE_EQ(7.0 / 11.0, x.y);

        EXPECT_LE(lastResidualNorm, std::numeric_limits<double>::epsilon());
        EXPECT_LE(lastNumIter, 2u);
    }
}

TEST(CR, SolveSymmetricSystems)
{
    using BLASType = BLAS<double, Vector2D, Matrix2x2D>;

    for (const auto& matrix :
         { Matrix2x2D(4.0, 1.0, 1.0, 3.0), Matrix2x2D(2.0, 0.0, 0.0, -1.0) })
    {
        const Vector2D expected(1.0, 1.0);
        const Vector2D rhs = matrix * expected;
        Vector2D x, r, d, q, s;
        unsigned int iterations = 0;
        double residual = 0.0;

        CR<BLASType>(matrix, rhs, 10, 1e-14, &x, &r, &d, &q, &s, &iterations,
                     &residual);

        EXPECT_TRUE(x.IsSimilar(expected, 1e-12));
        EXPECT_LE(residual, 1e-12);
        EXPECT_LE(iterations, 2u);
    }
}

TEST(CR, ZeroIterationsReportsInitialResidual)
{
    using BLASType = BLAS<double, Vector2D, Matrix2x2D>;
    const Matrix2x2D matrix(4.0, 1.0, 1.0, 3.0);
    const Vector2D rhs(1.0, 2.0);
    Vector2D x, r, d, q, s;
    unsigned int iterations = 1;
    double residual = 0.0;

    CR<BLASType>(matrix, rhs, 0, 0.0, &x, &r, &d, &q, &s, &iterations,
                 &residual);

    EXPECT_EQ(iterations, 0u);
    EXPECT_DOUBLE_EQ(residual, std::sqrt(5.0));
    EXPECT_EQ(x, Vector2D{});
}

TEST(CR, AlreadyConverged)
{
    using BLASType = BLAS<double, Vector2D, Matrix2x2D>;
    const Matrix2x2D matrix(4.0, 1.0, 1.0, 3.0);
    Vector2D x(1.0, 1.0);
    const Vector2D rhs = matrix * x;
    Vector2D r, d, q, s;
    unsigned int iterations = 1;
    double residual = 1.0;

    CR<BLASType>(matrix, rhs, 10, 1e-14, &x, &r, &d, &q, &s, &iterations,
                 &residual);

    EXPECT_EQ(iterations, 0u);
    EXPECT_LE(residual, 1e-14);
}

TEST(CR, SingularSystemReportsBreakdown)
{
    using BLASType = BLAS<double, Vector2D, Matrix2x2D>;
    const Matrix2x2D matrix;
    const Vector2D rhs(1.0, 2.0);
    Vector2D x;
    Vector2D r;
    Vector2D d;
    Vector2D q;
    Vector2D s;
    unsigned int iterations = 1;
    double residual = 0.0;

    CR<BLASType>(matrix, rhs, 10, 1e-14, &x, &r, &d, &q, &s, &iterations,
                 &residual);

    EXPECT_EQ(iterations, 0u);
    EXPECT_EQ(x, Vector2D{});
    EXPECT_TRUE(std::isinf(residual));
}
