// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <API/Python/Solver/Grid/GridForwardEulerDiffusionSolver.hpp>
#include <Core/Solver/Grid/GridForwardEulerDiffusionSolver2.hpp>
#include <Core/Solver/Grid/GridForwardEulerDiffusionSolver3.hpp>

#include <pybind11/pybind11.h>

using namespace CubbyFlow;

void AddGridForwardEulerDiffusionSolver2(pybind11::module& m)
{
    pybind11::class_<GridForwardEulerDiffusionSolver2,
                     GridForwardEulerDiffusionSolver2Ptr, GridDiffusionSolver2>(
        static_cast<pybind11::handle>(m), "GridForwardEulerDiffusionSolver2",
        R"pbdoc(
			2-D grid-based forward Euler diffusion solver.

			This class implements 2-D grid-based forward Euler diffusion solver using
			second-order central differencing spatially. Since the method is relying on
			explicit time-integration (i.e. forward Euler), the diffusion coefficient is
			limited by the time interval and grid spacing such as:
			\f$\mu < \frac{h}{12\Delta t} \f$ where \f$\mu\f$, \f$h\f$, and
			\f$\Delta t\f$ are the diffusion coefficient, grid spacing, and time
			interval, respectively.
		)pbdoc")
        .def(pybind11::init<>())
        .def(
            "Solver",
            [](GridForwardEulerDiffusionSolver2& instance, Grid2Ptr source,
               double diffusionCoefficient, double timeIntervalInSeconds,
               Grid2Ptr dest, pybind11::kwargs kwargs) {
                auto sourceSG = std::dynamic_pointer_cast<ScalarGrid2>(source);
                auto sourceCG =
                    std::dynamic_pointer_cast<CollocatedVectorGrid2>(source);
                auto sourceFG =
                    std::dynamic_pointer_cast<FaceCenteredGrid2>(source);

                auto destSG = std::dynamic_pointer_cast<ScalarGrid2>(dest);
                auto destCG =
                    std::dynamic_pointer_cast<CollocatedVectorGrid2>(dest);
                auto destFG =
                    std::dynamic_pointer_cast<FaceCenteredGrid2>(dest);

                ScalarField2Ptr boundarySDF =
                    ConstantScalarField2::GetBuilder()
                        .WithValue(std::numeric_limits<double>::max())
                        .MakeShared();
                ScalarField2Ptr fluidSDF =
                    ConstantScalarField2::GetBuilder()
                        .WithValue(-std::numeric_limits<double>::max())
                        .MakeShared();

                if (kwargs.contains("boundarySDF"))
                {
                    boundarySDF = kwargs["boundarySDF"].cast<ScalarField2Ptr>();
                }
                if (kwargs.contains("fluidSDF"))
                {
                    fluidSDF = kwargs["fluidSDF"].cast<ScalarField2Ptr>();
                }

                if (sourceSG != nullptr && destSG != nullptr)
                {
                    instance.Solve(*sourceSG, diffusionCoefficient,
                                   timeIntervalInSeconds, destSG.get(),
                                   *boundarySDF, *fluidSDF);
                }
                else if (sourceCG != nullptr && destCG != nullptr)
                {
                    instance.Solve(*sourceCG, diffusionCoefficient,
                                   timeIntervalInSeconds, destCG.get(),
                                   *boundarySDF, *fluidSDF);
                }
                else if (sourceFG != nullptr && destFG != nullptr)
                {
                    instance.Solve(*sourceFG, diffusionCoefficient,
                                   timeIntervalInSeconds, destFG.get(),
                                   *boundarySDF, *fluidSDF);
                }
                else
                {
                    throw std::invalid_argument(
                        "Grids source and dest must have same type.");
                }
            },
            R"pbdoc(
			Solves diffusion equation for a scalar field.

			Parameters
			----------
			- source : Input grid.
			- diffusionCoefficient : Amount of diffusion.
			- timeIntervalInSeconds : Small time-interval that diffusion occur.
			- dest : Output grid.
			- `**kwargs` :
				- Key `boundarySDF` : Shape of the solid boundary that is empty by default.
				- Key `fluidSDF` : Shape of the fluid boundary that is full by default.
		)pbdoc",
            pybind11::arg("source"), pybind11::arg("diffusionCoefficient"),
            pybind11::arg("timeIntervalInSeconds"), pybind11::arg("dest"));
}

void AddGridForwardEulerDiffusionSolver3(pybind11::module& m)
{
    pybind11::class_<GridForwardEulerDiffusionSolver3,
                     GridForwardEulerDiffusionSolver3Ptr, GridDiffusionSolver3>(
        static_cast<pybind11::handle>(m), "GridForwardEulerDiffusionSolver3",
        R"pbdoc(
			3-D grid-based forward Euler diffusion solver.

			This class implements 3-D grid-based forward Euler diffusion solver using
			second-order central differencing spatially. Since the method is relying on
			explicit time-integration (i.e. forward Euler), the diffusion coefficient is
			limited by the time interval and grid spacing such as:
			\f$\mu < \frac{h}{12\Delta t} \f$ where \f$\mu\f$, \f$h\f$, and
			\f$\Delta t\f$ are the diffusion coefficient, grid spacing, and time
			interval, respectively.
		)pbdoc")
        .def(pybind11::init<>())
        .def(
            "Solver",
            [](GridForwardEulerDiffusionSolver3& instance, Grid3Ptr source,
               double diffusionCoefficient, double timeIntervalInSeconds,
               Grid3Ptr dest, pybind11::kwargs kwargs) {
                auto sourceSG = std::dynamic_pointer_cast<ScalarGrid3>(source);
                auto sourceCG =
                    std::dynamic_pointer_cast<CollocatedVectorGrid3>(source);
                auto sourceFG =
                    std::dynamic_pointer_cast<FaceCenteredGrid3>(source);

                auto destSG = std::dynamic_pointer_cast<ScalarGrid3>(dest);
                auto destCG =
                    std::dynamic_pointer_cast<CollocatedVectorGrid3>(dest);
                auto destFG =
                    std::dynamic_pointer_cast<FaceCenteredGrid3>(dest);

                ScalarField3Ptr boundarySDF =
                    ConstantScalarField3::GetBuilder()
                        .WithValue(std::numeric_limits<double>::max())
                        .MakeShared();
                ScalarField3Ptr fluidSDF =
                    ConstantScalarField3::GetBuilder()
                        .WithValue(-std::numeric_limits<double>::max())
                        .MakeShared();

                if (kwargs.contains("boundarySDF"))
                {
                    boundarySDF = kwargs["boundarySDF"].cast<ScalarField3Ptr>();
                }
                if (kwargs.contains("fluidSDF"))
                {
                    fluidSDF = kwargs["fluidSDF"].cast<ScalarField3Ptr>();
                }

                if (sourceSG != nullptr && destSG != nullptr)
                {
                    instance.Solve(*sourceSG, diffusionCoefficient,
                                   timeIntervalInSeconds, destSG.get(),
                                   *boundarySDF, *fluidSDF);
                }
                else if (sourceCG != nullptr && destCG != nullptr)
                {
                    instance.Solve(*sourceCG, diffusionCoefficient,
                                   timeIntervalInSeconds, destCG.get(),
                                   *boundarySDF, *fluidSDF);
                }
                else if (sourceFG != nullptr && destFG != nullptr)
                {
                    instance.Solve(*sourceFG, diffusionCoefficient,
                                   timeIntervalInSeconds, destFG.get(),
                                   *boundarySDF, *fluidSDF);
                }
                else
                {
                    throw std::invalid_argument(
                        "Grids source and dest must have same type.");
                }
            },
            R"pbdoc(
			Solves diffusion equation for a scalar field.

			Parameters
			----------
			- source : Input grid.
			- diffusionCoefficient : Amount of diffusion.
			- timeIntervalInSeconds : Small time-interval that diffusion occur.
			- dest : Output grid.
			- `**kwargs` :
				- Key `boundarySDF` : Shape of the solid boundary that is empty by default.
				- Key `fluidSDF` : Shape of the fluid boundary that is full by default.
		)pbdoc",
            pybind11::arg("source"), pybind11::arg("diffusionCoefficient"),
            pybind11::arg("timeIntervalInSeconds"), pybind11::arg("dest"));
}
