// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <API/Python/Solver/Particle/MPM/SnowMPMSolver.hpp>
#include <Core/Solver/Particle/MPM/SnowMPMSolver.hpp>

using namespace CubbyFlow;

namespace
{
template <size_t N, typename Base>
void AddSnowMPMSolver(pybind11::module& m, const char* name)
{
    using Solver = SnowMPMSolver<N>;
    using SolverPtr = std::shared_ptr<Solver>;
    using VectorType = Vector<double, N>;
    using SizeType = Vector<size_t, N>;

    pybind11::class_<Solver, SolverPtr, Base>(m, name)
        .def(pybind11::init<const SizeType&, const VectorType&,
                            const VectorType&, double, double>(),
             pybind11::arg("resolution") = SizeType::MakeConstant(32),
             pybind11::arg("gridSpacing") = VectorType::MakeConstant(1.0),
             pybind11::arg("gridOrigin") = VectorType{},
             pybind11::arg("radius") = 1e-3, pybind11::arg("mass") = 1e-3)
        .def_property_readonly("mpmSystemData", &Solver::GetMPMSystemData)
        .def_property("timeStepLimitScale", &Solver::GetTimeStepLimitScale,
                      &Solver::SetTimeStepLimitScale)
        .def_property("closedDomainBoundaryFlag",
                      &Solver::GetClosedDomainBoundaryFlag,
                      &Solver::SetClosedDomainBoundaryFlag)
        .def_property("isUsingSemiImplicit", &Solver::GetIsUsingSemiImplicit,
                      &Solver::SetIsUsingSemiImplicit)
        .def_property("maxNumberOfIterations",
                      &Solver::GetMaxNumberOfIterations,
                      &Solver::SetMaxNumberOfIterations)
        .def_property("tolerance", &Solver::GetTolerance, &Solver::SetTolerance)
        .def_property_readonly("lastNumberOfIterations",
                               &Solver::GetLastNumberOfIterations)
        .def_property_readonly("lastResidual", &Solver::GetLastResidual);
}
}  // namespace

void AddSnowMPMSolver2(pybind11::module& m)
{
    AddSnowMPMSolver<2, ParticleSystemSolver2>(m, "SnowMPMSolver2");
}

void AddSnowMPMSolver3(pybind11::module& m)
{
    AddSnowMPMSolver<3, ParticleSystemSolver3>(m, "SnowMPMSolver3");
}
