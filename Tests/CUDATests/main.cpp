// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#define DOCTEST_CONFIG_IMPLEMENT
#include <doctest/doctest.h>

#include <Core/Utils/Logging.hpp>

#include <fstream>

int main()
{
    doctest::Context context;

    std::ofstream logFile("cuda_tests.log");
    if (logFile)
    {
        CubbyFlow::Logging::SetAllStream(&logFile);
    }

    // Run queries, or run tests unless --no-run is specified
    const int res = context.run();

    return res;
}