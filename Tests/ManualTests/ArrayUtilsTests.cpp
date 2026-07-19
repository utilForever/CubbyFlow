#include "gtest/gtest.h"

#include <ManualTests.hpp>

#include <Core/Array/Array.hpp>
#include <Core/Array/ArrayUtils.hpp>
#include <Core/Matrix/Matrix.hpp>

using namespace CubbyFlow;

CUBBYFLOW_TESTS(ArrayUtils);

CUBBYFLOW_BEGIN_TEST_F(ArrayUtils, ExtrapolateToRegion2)
{
    Array2<double> data(128, 192, 0.0);
    Array2<char> valid(128, 192, char{});

    for (int j = 0; j < 192; ++j)
    {
        for (int i = 0; i < 128; ++i)
        {
            Vector2D pt(i / 128.0, j / 128.0);

            data(i, j) =
                std::sin(4 * PI_DOUBLE * pt.x) * std::sin(4 * PI_DOUBLE * pt.y);

            if (pt.DistanceTo(Vector2D(0.5, 0.5)) < 0.15 ||
                pt.DistanceTo(Vector2D(0.5, 0.9)) < 0.15)
            {
                valid(i, j) = 1;
            }
            else
            {
                valid(i, j) = 0;
            }
        }
    }

    SaveData(data.View(), "data0.npy");
    SaveData(valid.View(), "valid0.npy");

    ExtrapolateToRegion(data.View(), valid.View(), 10, data.View());

    SaveData(data.View(), "data10.npy");
    SaveData(valid.View(), "valid10.npy");
}
CUBBYFLOW_END_TEST_F
