// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <API/Python/Solver/Particle/MPM/MPMFluidSolver.hpp>
#include <Core/Solver/Particle/MPM/MPMFluidSolver.hpp>

using namespace CubbyFlow;

namespace
{
template <size_t N, typename Base>
void AddMPMFluidSolver(pybind11::module& m, const char* name)
{
    using Solver = MPMFluidSolver<N>;
    using SolverPtr = std::shared_ptr<Solver>;
    using VectorType = Vector<double, N>;
    using SizeType = Vector<size_t, N>;

    pybind11::class_<Solver, SolverPtr, Base>(m, name)
        .def(pybind11::init<const SizeType&, const VectorType&,
                            const VectorType&, double, double, double, double,
                            double, double>(),
             pybind11::arg("resolution") = SizeType::MakeConstant(32),
             pybind11::arg("gridSpacing") = VectorType::MakeConstant(1.0),
             pybind11::arg("gridOrigin") = VectorType{},
             pybind11::arg("radius") = 1e-3, pybind11::arg("mass") = 1e-3,
             pybind11::arg("targetDensity") = WATER_DENSITY,
             pybind11::arg("speedOfSound") = 100.0,
             pybind11::arg("eosExponent") = 7.0,
             pybind11::arg("negativePressureScale") = 0.0)
        .def_property_readonly("mpmSystemData", &Solver::GetMPMSystemData)
        .def_property("timeStepLimitScale", &Solver::GetTimeStepLimitScale,
                      &Solver::SetTimeStepLimitScale)
        .def_property_readonly(
            "targetDensity",
            [](const Solver& solver) {
                return solver.GetConstitutiveModel().GetTargetDensity();
            })
        .def_property_readonly(
            "speedOfSound",
            [](const Solver& solver) {
                return solver.GetConstitutiveModel().GetSpeedOfSound();
            })
        .def_property_readonly(
            "eosExponent",
            [](const Solver& solver) {
                return solver.GetConstitutiveModel().GetEosExponent();
            })
        .def_property_readonly(
            "negativePressureScale", [](const Solver& solver) {
                return solver.GetConstitutiveModel().GetNegativePressureScale();
            });
}
}  // namespace

void AddMPMFluidSolver2(pybind11::module& m)
{
    AddMPMFluidSolver<2, ParticleSystemSolver2>(m, "MPMFluidSolver2");
}

void AddMPMFluidSolver3(pybind11::module& m)
{
    AddMPMFluidSolver<3, ParticleSystemSolver3>(m, "MPMFluidSolver3");
}
