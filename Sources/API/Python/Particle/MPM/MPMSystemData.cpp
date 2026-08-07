// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <API/Python/Particle/MPM/MPMSystemData.hpp>
#include <Core/Particle/MPM/MPMSystemData.hpp>

#include <pybind11/numpy.h>

using namespace CubbyFlow;

namespace
{
namespace py = pybind11;

template <size_t N>
using MPMClass = py::class_<MPMSystemData<N>, std::shared_ptr<MPMSystemData<N>>,
                            ParticleSystemData<N>>;

template <size_t N>
py::array_t<double> MatrixView(SnowDeformationState<N>& state,
                               Matrix<double, N, N>& matrix)
{
    return py::array_t<double>(
        { static_cast<py::ssize_t>(N), static_cast<py::ssize_t>(N) },
        { static_cast<py::ssize_t>(sizeof(double) * N),
          static_cast<py::ssize_t>(sizeof(double)) },
        matrix.data(), py::cast(&state));
}

template <size_t N>
void AddSnowDeformationState(py::module& m, const char* name)
{
    using State = SnowDeformationState<N>;
    py::class_<State>(m, name)
        .def(py::init<>())
        .def_property_readonly(
            "elastic",
            [](State& state) { return MatrixView(state, state.elastic); })
        .def_property_readonly("plastic", [](State& state) {
            return MatrixView(state, state.plastic);
        });
}

template <size_t N>
py::array_t<double> ScalarView(MPMSystemData<N>& instance,
                               ArrayView1<double> view)
{
    return py::array_t<double>({ static_cast<py::ssize_t>(view.Length()) },
                               { static_cast<py::ssize_t>(sizeof(double)) },
                               view.data(), py::cast(&instance));
}

template <size_t N>
py::list DeformationStates(MPMSystemData<N>& instance)
{
    py::list result;
    auto parent = py::cast(&instance, py::return_value_policy::reference);
    for (auto& state : instance.DeformationStates())
    {
        result.append(
            py::cast(&state, py::return_value_policy::reference, parent));
    }
    return result;
}

template <size_t N>
void BindParticleState(MPMClass<N>& cls)
{
    cls.def_property_readonly("particleMasses",
                              [](MPMSystemData<N>& instance) {
                                  return ScalarView(instance,
                                                    instance.ParticleMasses());
                              })
        .def_property_readonly("initialVolumes",
                               [](MPMSystemData<N>& instance) {
                                   return ScalarView(instance,
                                                     instance.InitialVolumes());
                               })
        .def_property_readonly("deformationStates", &DeformationStates<N>);
}

template <size_t N>
void BindGridState(MPMClass<N>& cls)
{
    cls.def("ResizeGrid", &MPMSystemData<N>::ResizeGrid)
        .def_property_readonly(
            "gridMass",
            [](MPMSystemData<N>& instance) -> auto& {
                return instance.GridMass();
            },
            py::return_value_policy::reference_internal)
        .def_property_readonly(
            "gridVelocities",
            [](MPMSystemData<N>& instance) -> auto& {
                return instance.GridVelocities();
            },
            py::return_value_policy::reference_internal)
        .def_property_readonly(
            "gridVelocitiesBeforeUpdate",
            [](MPMSystemData<N>& instance) -> auto& {
                return instance.GridVelocitiesBeforeUpdate();
            },
            py::return_value_policy::reference_internal)
        .def_property("flipBlendingFactor",
                      &MPMSystemData<N>::FLIPBlendingFactor,
                      &MPMSystemData<N>::SetFLIPBlendingFactor)
        .def("TransferFromParticlesToGrid",
             &MPMSystemData<N>::TransferFromParticlesToGrid)
        .def("TransferFromGridToParticles",
             &MPMSystemData<N>::TransferFromGridToParticles);
}

template <size_t N>
void AddMPMSystemData(py::module& m, const char* className,
                      const char* stateName)
{
    AddSnowDeformationState<N>(m, stateName);
    MPMClass<N> cls(m, className);
    cls.def(py::init<const Vector<size_t, N>&, const Vector<double, N>&,
                     const Vector<double, N>&, size_t>(),
            py::arg("resolution") = Vector<size_t, N>::MakeConstant(1),
            py::arg("gridSpacing") = Vector<double, N>::MakeConstant(1.0),
            py::arg("gridOrigin") = Vector<double, N>{},
            py::arg("numberOfParticles") = 0);
    BindParticleState(cls);
    BindGridState(cls);
}
}  // namespace

void AddMPMSystemData2(pybind11::module& m)
{
    AddMPMSystemData<2>(m, "MPMSystemData2", "SnowDeformationState2");
}

void AddMPMSystemData3(pybind11::module& m)
{
    AddMPMSystemData<3>(m, "MPMSystemData3", "SnowDeformationState3");
}
