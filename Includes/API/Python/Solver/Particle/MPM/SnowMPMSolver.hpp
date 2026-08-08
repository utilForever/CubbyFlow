// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_PYTHON_SNOW_MPM_SOLVER_HPP
#define CUBBYFLOW_PYTHON_SNOW_MPM_SOLVER_HPP

#include <pybind11/pybind11.h>

void AddSnowMPMSolver2(pybind11::module& m);
void AddSnowMPMSolver3(pybind11::module& m);

#endif
