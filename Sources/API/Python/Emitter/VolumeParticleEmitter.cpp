// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <API/Python/Emitter/VolumeParticleEmitter.hpp>
#include <API/Python/Utils/pybind11Utils.hpp>
#include <Core/Emitter/VolumeParticleEmitter2.hpp>
#include <Core/Emitter/VolumeParticleEmitter3.hpp>
#include <Core/Geometry/SurfaceToImplicit.hpp>

#include <pybind11/pybind11.h>

using namespace CubbyFlow;

void AddVolumeParticleEmitter2(pybind11::module& m)
{
    pybind11::class_<VolumeParticleEmitter2, VolumeParticleEmitter2Ptr,
                     ParticleEmitter2>(m, "VolumeParticleEmitter2")
        .def(
            "__init__",
            [](VolumeParticleEmitter2& instance, pybind11::args args,
               pybind11::kwargs kwargs) {
                ImplicitSurface2Ptr implicitSurface;
                BoundingBox2D maxRegion;
                double spacing = 0.1;
                Vector2D initialVel;
                Vector2D linearVel;
                double angularVel = 0.0;
                size_t maxNumberOfParticles =
                    std::numeric_limits<size_t>::max();
                double jitter = 0.0;
                bool isOneShot = true;
                bool allowOverlapping = false;
                uint32_t seed = 0;

                const auto parseImplicitSurface =
                    [&implicitSurface, &maxRegion](pybind11::object arg) {
                    if (pybind11::isinstance<ImplicitSurface2>(arg))
                    {
                        implicitSurface = arg.cast<ImplicitSurface2Ptr>();
                    }
                    else if (pybind11::isinstance<Surface2>(arg))
                    {
                        auto surface = arg.cast<Surface2Ptr>();
                        implicitSurface =
                            std::make_shared<SurfaceToImplicit2>(surface);
                    }
                    else
                    {
                        throw std::invalid_argument(
                            "Unknown type for ImplicitSurface.");
                    }

                    // Get initial val for the max region.
                    if (implicitSurface->IsBounded())
                    {
                        maxRegion = implicitSurface->GetBoundingBox();
                    }
                };

                if (args.size() >= 3 && args.size() <= 11)
                {
                    parseImplicitSurface(args[0]);

                    maxRegion = args[1].cast<BoundingBox2D>();
                    spacing = args[2].cast<double>();

                    if (args.size() > 3)
                    {
                        initialVel =
                            ObjectToVector2D(pybind11::object(args[3]));
                    }
                    if (args.size() > 4)
                    {
                        linearVel = ObjectToVector2D(pybind11::object(args[4]));
                    }
                    if (args.size() > 3)
                    {
                        angularVel = args[5].cast<double>();
                    }
                    if (args.size() > 6)
                    {
                        maxNumberOfParticles = args[6].cast<size_t>();
                    }
                    if (args.size() > 7)
                    {
                        jitter = args[7].cast<double>();
                    }
                    if (args.size() > 8)
                    {
                        isOneShot = args[8].cast<bool>();
                    }
                    if (args.size() > 9)
                    {
                        allowOverlapping = args[9].cast<bool>();
                    }
                    if (args.size() > 10)
                    {
                        seed = args[8].cast<uint32_t>();
                    }
                }
                else if (args.size() > 0)
                {
                    throw std::invalid_argument("Too few/many arguments.");
                }

                if (kwargs.contains("implicitSurface"))
                {
                    parseImplicitSurface(kwargs["implicitSurface"]);
                }
                if (kwargs.contains("maxRegion"))
                {
                    maxRegion = kwargs["maxRegion"].cast<BoundingBox2D>();
                }
                if (kwargs.contains("spacing"))
                {
                    spacing = kwargs["spacing"].cast<double>();
                }
                if (kwargs.contains("initialVelocity"))
                {
                    initialVel = ObjectToVector2D(kwargs["initialVelocity"]);
                }
                if (kwargs.contains("linearVelocity"))
                {
                    linearVel = ObjectToVector2D(kwargs["linearVelocity"]);
                }
                if (kwargs.contains("angularVelocity"))
                {
                    angularVel = kwargs["angularVelocity"].cast<double>();
                }
                if (kwargs.contains("maxNumberOfParticles"))
                {
                    maxNumberOfParticles =
                        kwargs["maxNumberOfParticles"].cast<size_t>();
                }
                if (kwargs.contains("jitter"))
                {
                    jitter = kwargs["jitter"].cast<double>();
                }
                if (kwargs.contains("isOneShot"))
                {
                    isOneShot = kwargs["isOneShot"].cast<bool>();
                }
                if (kwargs.contains("allowOverlapping"))
                {
                    allowOverlapping = kwargs["allowOverlapping"].cast<bool>();
                }
                if (kwargs.contains("seed"))
                {
                    seed = kwargs["seed"].cast<uint32_t>();
                }

                new (&instance) VolumeParticleEmitter2(
                    implicitSurface, maxRegion, spacing, initialVel, linearVel,
                    angularVel, maxNumberOfParticles, jitter, isOneShot,
                    allowOverlapping, seed);
            },
            R"pbdoc(
			Constructs VolumeParticleEmitter2

			This method constructs VolumeParticleEmitter2 with implicit
			surface, bounding box, particle spacing, initial velocity
			(optional), max number of particles (optional), jitter
			(optional), whether it's one shot or not (optional), whether it
			should allow overlapping or not (optional), and random seed
			(optional).

            Parameters
            ----------
            - implicitSurface : The implicit surface.
            - maxRegion: The max region.
            - spacing: The spacing between particles.
            - initialVel: The initial velocity.
            - maxNumberOfParticles: The max number of particles to be emitted.
            - jitter: The jitter amount between 0 and 1.
            - isOneShot: Set true if particles are emitted just once.
            - allowOverlapping: True if particles can be overlapped.
            - seed: The random seed.
             )pbdoc")
        .def_property("surface", &VolumeParticleEmitter2::GetSurface,
                      &VolumeParticleEmitter2::SetSurface, R"pbdoc(
             Source surface.
             )pbdoc")
        .def_property("maxRegion", &VolumeParticleEmitter2::GetMaxRegion,
                      &VolumeParticleEmitter2::SetMaxRegion, R"pbdoc(
             Max particle gen region.
             )pbdoc")
        .def_property("jitter", &VolumeParticleEmitter2::GetJitter,
                      &VolumeParticleEmitter2::SetJitter, R"pbdoc(
             Jitter amount between 0 and 1.
             )pbdoc")
        .def_property("isOneShot", &VolumeParticleEmitter2::GetIsOneShot,
                      &VolumeParticleEmitter2::SetIsOneShot, R"pbdoc(
             True if particles should be emitted just once.
             )pbdoc")
        .def_property("allowOverlapping",
                      &VolumeParticleEmitter2::GetAllowOverlapping,
                      &VolumeParticleEmitter2::SetAllowOverlapping, R"pbdoc(
             True if particles can be overlapped.
             )pbdoc")
        .def_property("maxNumberOfParticles",
                      &VolumeParticleEmitter2::GetMaxNumberOfParticles,
                      &VolumeParticleEmitter2::SetMaxNumberOfParticles,
                      R"pbdoc(
             Max number of particles to be emitted.
             )pbdoc")
        .def_property("spacing", &VolumeParticleEmitter2::GetSpacing,
                      &VolumeParticleEmitter2::SetSpacing, R"pbdoc(
             The spacing between particles.
             )pbdoc")
        .def_property(
            "initialVelocity", &VolumeParticleEmitter2::GetInitialVelocity,
            [](VolumeParticleEmitter2& instance,
               pybind11::object newInitialVel) {
                instance.SetInitialVelocity(ObjectToVector2D(newInitialVel));
            },
            R"pbdoc(
             The initial velocity of the particles.
             )pbdoc")
        .def_property(
            "linearVelocity", &VolumeParticleEmitter2::GetLinearVelocity,
            [](VolumeParticleEmitter2& instance,
               pybind11::object newLinearVel) {
                instance.SetLinearVelocity(ObjectToVector2D(newLinearVel));
            },
            R"pbdoc(
             The linear velocity of the emitter.
             )pbdoc")
        .def_property(
            "angularVelocity", &VolumeParticleEmitter2::GetAngularVelocity,
            [](VolumeParticleEmitter2& instance, double newAngularVel) {
                instance.SetAngularVelocity(newAngularVel);
            },
            R"pbdoc(
             The angular velocity of the emitter.
             )pbdoc");
}

void AddVolumeParticleEmitter3(pybind11::module& m)
{
    pybind11::class_<VolumeParticleEmitter3, VolumeParticleEmitter3Ptr,
                     ParticleEmitter3>(m, "VolumeParticleEmitter3")
        .def(
            "__init__",
            [](VolumeParticleEmitter3& instance, pybind11::args args,
               pybind11::kwargs kwargs) {
                ImplicitSurface3Ptr implicitSurface;
                BoundingBox3D maxRegion;
                double spacing = 0.1;
                Vector3D initialVel;
                Vector3D linearVel;
                Vector3D angularVel;
                size_t maxNumberOfParticles =
                    std::numeric_limits<size_t>::max();
                double jitter = 0.0;
                bool isOneShot = true;
                bool allowOverlapping = false;
                uint32_t seed = 0;

                const auto parseImplicitSurface =
                    [&implicitSurface, &maxRegion](pybind11::object arg) {
                    if (pybind11::isinstance<ImplicitSurface3>(arg))
                    {
                        implicitSurface = arg.cast<ImplicitSurface3Ptr>();
                    }
                    else if (pybind11::isinstance<Surface3>(arg))
                    {
                        auto surface = arg.cast<Surface3Ptr>();
                        implicitSurface =
                            std::make_shared<SurfaceToImplicit3>(surface);
                    }
                    else
                    {
                        throw std::invalid_argument(
                            "Unknown type for ImplicitSurface.");
                    }

                    // Get initial val for the max region.
                    if (implicitSurface->IsBounded())
                    {
                        maxRegion = implicitSurface->GetBoundingBox();
                    }
                };

                if (args.size() >= 3 && args.size() <= 11)
                {
                    parseImplicitSurface(args[0]);

                    maxRegion = args[1].cast<BoundingBox3D>();
                    spacing = args[2].cast<double>();

                    if (args.size() > 3)
                    {
                        initialVel =
                            ObjectToVector3D(pybind11::object(args[3]));
                    }
                    if (args.size() > 4)
                    {
                        linearVel = ObjectToVector3D(pybind11::object(args[4]));
                    }
                    if (args.size() > 5)
                    {
                        angularVel =
                            ObjectToVector3D(pybind11::object(args[5]));
                    }
                    if (args.size() > 6)
                    {
                        maxNumberOfParticles = args[6].cast<size_t>();
                    }
                    if (args.size() > 7)
                    {
                        jitter = args[7].cast<double>();
                    }
                    if (args.size() > 8)
                    {
                        isOneShot = args[8].cast<bool>();
                    }
                    if (args.size() > 9)
                    {
                        allowOverlapping = args[9].cast<bool>();
                    }
                    if (args.size() > 10)
                    {
                        seed = args[10].cast<uint32_t>();
                    }
                }
                else if (args.size() > 0)
                {
                    throw std::invalid_argument("Too few/many arguments.");
                }

                if (kwargs.contains("implicitSurface"))
                {
                    parseImplicitSurface(kwargs["implicitSurface"]);
                }
                if (kwargs.contains("maxRegion"))
                {
                    maxRegion = kwargs["maxRegion"].cast<BoundingBox3D>();
                }
                if (kwargs.contains("spacing"))
                {
                    spacing = kwargs["spacing"].cast<double>();
                }
                if (kwargs.contains("initialVelocity"))
                {
                    initialVel = ObjectToVector3D(kwargs["initialVelocity"]);
                }
                if (kwargs.contains("linearVelocity"))
                {
                    linearVel = ObjectToVector3D(kwargs["linearVelocity"]);
                }
                if (kwargs.contains("angularVelocity"))
                {
                    angularVel = ObjectToVector3D(kwargs["angularVelocity"]);
                }
                if (kwargs.contains("maxNumberOfParticles"))
                {
                    maxNumberOfParticles =
                        kwargs["maxNumberOfParticles"].cast<size_t>();
                }
                if (kwargs.contains("jitter"))
                {
                    jitter = kwargs["jitter"].cast<double>();
                }
                if (kwargs.contains("isOneShot"))
                {
                    isOneShot = kwargs["isOneShot"].cast<bool>();
                }
                if (kwargs.contains("allowOverlapping"))
                {
                    allowOverlapping = kwargs["allowOverlapping"].cast<bool>();
                }
                if (kwargs.contains("seed"))
                {
                    seed = kwargs["seed"].cast<uint32_t>();
                }

                new (&instance) VolumeParticleEmitter3(
                    implicitSurface, maxRegion, spacing, initialVel, linearVel,
                    angularVel, maxNumberOfParticles, jitter, isOneShot,
                    allowOverlapping, seed);
            },
            R"pbdoc(
			Constructs VolumeParticleEmitter3

			This method constructs VolumeParticleEmitter3 with implicit
			surface, bounding box, particle spacing, initial velocity
			(optional), max number of particles (optional), jitter
			(optional), whether it's one shot or not (optional), whether it
			should allow overlapping or not (optional), and random seed
			(optional).

            Parameters
            ----------
            - implicitSurface : The implicit surface.
            - maxRegion: The max region.
            - spacing: The spacing between particles.
            - initialVel: The initial velocity.
            - maxNumberOfParticles: The max number of particles to be emitted.
            - jitter: The jitter amount between 0 and 1.
            - isOneShot: Set true if particles are emitted just once.
            - allowOverlapping: True if particles can be overlapped.
            - seed: The random seed.
             )pbdoc")
        .def_property("surface", &VolumeParticleEmitter3::GetSurface,
                      &VolumeParticleEmitter3::SetSurface, R"pbdoc(
             Source surface.
             )pbdoc")
        .def_property("maxRegion", &VolumeParticleEmitter3::GetMaxRegion,
                      &VolumeParticleEmitter3::SetMaxRegion, R"pbdoc(
             Max particle gen region.
             )pbdoc")
        .def_property("jitter", &VolumeParticleEmitter3::GetJitter,
                      &VolumeParticleEmitter3::SetJitter, R"pbdoc(
             Jitter amount between 0 and 1.
             )pbdoc")
        .def_property("isOneShot", &VolumeParticleEmitter3::GetIsOneShot,
                      &VolumeParticleEmitter3::SetIsOneShot, R"pbdoc(
             True if particles should be emitted just once.
             )pbdoc")
        .def_property("allowOverlapping",
                      &VolumeParticleEmitter3::GetAllowOverlapping,
                      &VolumeParticleEmitter3::SetAllowOverlapping, R"pbdoc(
             True if particles can be overlapped.
             )pbdoc")
        .def_property("maxNumberOfParticles",
                      &VolumeParticleEmitter3::GetMaxNumberOfParticles,
                      &VolumeParticleEmitter3::SetMaxNumberOfParticles,
                      R"pbdoc(
             Max number of particles to be emitted.
             )pbdoc")
        .def_property("spacing", &VolumeParticleEmitter3::GetSpacing,
                      &VolumeParticleEmitter3::SetSpacing, R"pbdoc(
             The spacing between particles.
             )pbdoc")
        .def_property(
            "initialVelocity", &VolumeParticleEmitter3::GetInitialVelocity,
            [](VolumeParticleEmitter3& instance,
               pybind11::object newInitialVel) {
                instance.SetInitialVelocity(ObjectToVector3D(newInitialVel));
            },
            R"pbdoc(
             The initial velocity of the particles.
             )pbdoc")
        .def_property(
            "linearVelocity", &VolumeParticleEmitter3::GetLinearVelocity,
            [](VolumeParticleEmitter3& instance,
               pybind11::object newLinearVel) {
                instance.SetLinearVelocity(ObjectToVector3D(newLinearVel));
            },
            R"pbdoc(
             The linear velocity of the emitter.
             )pbdoc")
        .def_property(
            "angularVelocity", &VolumeParticleEmitter3::GetAngularVelocity,
            [](VolumeParticleEmitter3& instance,
               pybind11::object newAngularVel) {
                instance.SetAngularVelocity(ObjectToVector3D(newAngularVel));
            },
            R"pbdoc(
             The angular velocity of the emitter.
             )pbdoc");
}
