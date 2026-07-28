// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <API/Python/Geometry/Collider.hpp>
#include <API/Python/Utils/pybind11Utils.hpp>
#include <Core/Geometry/Collider.hpp>

#include <pybind11/pybind11.h>

using namespace CubbyFlow;

void AddCollider2(pybind11::module& m)
{
    pybind11::class_<Collider2, Collider2Ptr>(m, "Collider2",
                                              R"pbdoc(
			Abstract base class for generic 2-D collider object.

			This class contains basic interfaces for colliders. Most of the
			functionalities are implemented within this class, except the member
			function Collider2::VelocityAt. This class also let the subclasses to
			provide a Surface2 instance to define collider surface using
			Collider2::SetSurface function.
		)pbdoc")
        .def_property("frictionCoefficient", &Collider2::GetFrictionCoefficient,
                      &Collider2::SetFrictionCoefficient,
                      R"pbdoc(
			The friction coefficient.

			This property specifies the friction coefficient to the collider. Any
			negative inputs will be clamped to zero.
		)pbdoc")
        .def_property_readonly("surface", &Collider2::GetSurface,
                               R"pbdoc(
			The surface instance.
		)pbdoc")
        .def(
            "VelocityAt",
            [](const Collider2& instance, pybind11::object obj) {
                return instance.VelocityAt(ObjectToVector2D(obj));
            },
            R"pbdoc(Returns the velocity of the collider at given point.)pbdoc",
            pybind11::arg("point"));
}

void AddCollider3(pybind11::module& m)
{
    pybind11::class_<Collider3, Collider3Ptr>(m, "Collider3",
                                              R"pbdoc(
			Abstract base class for generic 3-D collider object.

			This class contains basic interfaces for colliders. Most of the
			functionalities are implemented within this class, except the member
			function Collider3::VelocityAt. This class also let the subclasses to
			provide a Surface3 instance to define collider surface using
			Collider3::SetSurface function.
		)pbdoc")
        .def_property("frictionCoefficient", &Collider3::GetFrictionCoefficient,
                      &Collider3::SetFrictionCoefficient,
                      R"pbdoc(
			The friction coefficient.

			This property specifies the friction coefficient to the collider. Any
			negative inputs will be clamped to zero.
		)pbdoc")
        .def_property_readonly("surface", &Collider3::GetSurface,
                               R"pbdoc(
			The surface instance.
		)pbdoc")
        .def(
            "VelocityAt",
            [](const Collider3& instance, pybind11::object obj) {
                return instance.VelocityAt(ObjectToVector3D(obj));
            },
            R"pbdoc(Returns the velocity of the collider at given point.)pbdoc",
            pybind11::arg("point"));
}