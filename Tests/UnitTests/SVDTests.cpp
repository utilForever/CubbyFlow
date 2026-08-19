#include "gtest/gtest.h"

#include <Core/Math/SVD.hpp>

using namespace CubbyFlow;

TEST(SVD, Float)
{
    MatrixMxNF a{ { 0, 1 }, { 1, 1 }, { 1, 0 } };

    MatrixMxNF u, v;
    VectorNF w;

    SVD(a, u, w, v);

    MatrixMxNF w2(2, 2);
    w2(0, 0) = w[0];
    w2(1, 1) = w[1];

    MatrixMxNF aApprox = u * w2 * v.Transposed();
    EXPECT_TRUE(a.IsSimilar(aApprox, 1e-6));
}

TEST(SVD, Double)
{
    MatrixMxND a{ { 0, 1 }, { 1, 1 }, { 1, 0 } };

    MatrixMxND u, v;
    VectorND w;

    SVD(a, u, w, v);

    MatrixMxND w2(2, 2);
    w2(0, 0) = w[0];
    w2(1, 1) = w[1];

    MatrixMxND aApprox = u * w2 * v.Transposed();
    EXPECT_TRUE(a.IsSimilar(aApprox, 1e-12));
}

TEST(SVD, FixedSizeNonnegativeSingularValues)
{
    const Matrix3x3D a{
        { 0.99999999999999967, 8.4402182212980653e-17, 5.888597690270359e-17 },
        { -2.8206308027881296e-16, 0.99999999999999956,
          4.5886721400715257e-17 },
        { 1.1190377808845786e-16, 4.5625389093404679e-17, 0.99999999999999956 }
    };
    Matrix3x3D u;
    Vector3D w;
    Matrix3x3D v;

    SVD(a, u, w, v);

    EXPECT_GE(w.x, 0.0);
    EXPECT_GE(w.y, 0.0);
    EXPECT_GE(w.z, 0.0);
    EXPECT_TRUE(a.IsSimilar(u * Matrix3x3D::MakeScaleMatrix(w) * v.Transposed(),
                            1e-12));
}
