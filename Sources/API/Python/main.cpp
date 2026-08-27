// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <API/Python/Animation/Animation.hpp>
#include <API/Python/Animation/Frame.hpp>
#include <API/Python/Animation/PhysicsAnimation.hpp>
#include <API/Python/Array/ArrayView.hpp>
#include <API/Python/Emitter/GridEmitter.hpp>
#include <API/Python/Emitter/ParticleEmitter.hpp>
#include <API/Python/Emitter/ParticleEmitterSet.hpp>
#include <API/Python/Emitter/PointParticleEmitter.hpp>
#include <API/Python/Emitter/VolumeGridEmitter.hpp>
#include <API/Python/Emitter/VolumeParticleEmitter.hpp>
#include <API/Python/Field/ConstantScalarField.hpp>
#include <API/Python/Field/ConstantVectorField.hpp>
#include <API/Python/Field/CustomScalarField.hpp>
#include <API/Python/Field/CustomVectorField.hpp>
#include <API/Python/Field/Field.hpp>
#include <API/Python/Field/ScalarField.hpp>
#include <API/Python/Field/VectorField.hpp>
#include <API/Python/Geometry/BoundingBox.hpp>
#include <API/Python/Geometry/Box.hpp>
#include <API/Python/Geometry/Collider.hpp>
#include <API/Python/Geometry/ColliderSet.hpp>
#include <API/Python/Geometry/Cylinder.hpp>
#include <API/Python/Geometry/ImplicitSurface.hpp>
#include <API/Python/Geometry/ImplicitTriangleMesh.hpp>
#include <API/Python/Geometry/MarchingCubes.hpp>
#include <API/Python/Geometry/Plane.hpp>
#include <API/Python/Geometry/Ray.hpp>
#include <API/Python/Geometry/RigidbodyCollider.hpp>
#include <API/Python/Geometry/Sphere.hpp>
#include <API/Python/Geometry/Surface.hpp>
#include <API/Python/Geometry/SurfaceSet.hpp>
#include <API/Python/Geometry/SurfaceToImplicit.hpp>
#include <API/Python/Geometry/Transform.hpp>
#include <API/Python/Geometry/Triangle.hpp>
#include <API/Python/Geometry/TriangleMesh.hpp>
#include <API/Python/Grid/CellCenteredScalarGrid.hpp>
#include <API/Python/Grid/CellCenteredVectorGrid.hpp>
#include <API/Python/Grid/CollocatedVectorGrid.hpp>
#include <API/Python/Grid/FaceCenteredGrid.hpp>
#include <API/Python/Grid/Grid.hpp>
#include <API/Python/Grid/GridSystemData.hpp>
#include <API/Python/Grid/ScalarGrid.hpp>
#include <API/Python/Grid/VectorGrid.hpp>
#include <API/Python/Grid/VertexCenteredScalarGrid.hpp>
#include <API/Python/Grid/VertexCenteredVectorGrid.hpp>
#include <API/Python/Math/Quaternion.hpp>
#include <API/Python/Particle/MPM/MPMSystemData.hpp>
#include <API/Python/Particle/ParticleSystemData.hpp>
#include <API/Python/Particle/SPH/SPHSystemData.hpp>
#include <API/Python/PointsToImplicit/AnisotropicPointsToImplicit.hpp>
#include <API/Python/PointsToImplicit/PointsToImplicit.hpp>
#include <API/Python/PointsToImplicit/SPHPointsToImplicit.hpp>
#include <API/Python/PointsToImplicit/SphericalPointsToImplicit.hpp>
#include <API/Python/PointsToImplicit/ZhuBridsonPointsToImplicit.hpp>
#include <API/Python/Solver/Advection/AdvectionSolver.hpp>
#include <API/Python/Solver/Advection/CubicSemiLagrangian.hpp>
#include <API/Python/Solver/Advection/SemiLagrangian.hpp>
#include <API/Python/Solver/FDM/FDMCGSolver.hpp>
#include <API/Python/Solver/FDM/FDMGaussSeidelSolver.hpp>
#include <API/Python/Solver/FDM/FDMICCGSolver.hpp>
#include <API/Python/Solver/FDM/FDMJacobiSolver.hpp>
#include <API/Python/Solver/FDM/FDMLinearSystemSolver.hpp>
#include <API/Python/Solver/FDM/FDMMGPCGSolver.hpp>
#include <API/Python/Solver/FDM/FDMMGSolver.hpp>
#include <API/Python/Solver/Grid/GridBackwardEulerDiffusionSolver.hpp>
#include <API/Python/Solver/Grid/GridBlockedBoundaryConditionSolver.hpp>
#include <API/Python/Solver/Grid/GridBoundaryConditionSolver.hpp>
#include <API/Python/Solver/Grid/GridDiffusionSolver.hpp>
#include <API/Python/Solver/Grid/GridFluidSolver.hpp>
#include <API/Python/Solver/Grid/GridForwardEulerDiffusionSolver.hpp>
#include <API/Python/Solver/Grid/GridFractionalBoundaryConditionSolver.hpp>
#include <API/Python/Solver/Grid/GridFractionalSinglePhasePressureSolver.hpp>
#include <API/Python/Solver/Grid/GridPressureSolver.hpp>
#include <API/Python/Solver/Grid/GridSinglePhasePressureSolver.hpp>
#include <API/Python/Solver/Grid/GridSmokeSolver.hpp>
#include <API/Python/Solver/Hybrid/APIC/APICSolver.hpp>
#include <API/Python/Solver/Hybrid/FLIP/FLIPSolver.hpp>
#include <API/Python/Solver/Hybrid/PIC/PICSolver.hpp>
#include <API/Python/Solver/LevelSet/ENOLevelSetSolver.hpp>
#include <API/Python/Solver/LevelSet/FMMLevelSetSolver.hpp>
#include <API/Python/Solver/LevelSet/IterativeLevelSetSolver.hpp>
#include <API/Python/Solver/LevelSet/LevelSetLiquidSolver.hpp>
#include <API/Python/Solver/LevelSet/LevelSetSolver.hpp>
#include <API/Python/Solver/LevelSet/UpwindLevelSetSolver.hpp>
#include <API/Python/Solver/Particle/MPM/MPMFluidSolver.hpp>
#include <API/Python/Solver/Particle/MPM/SnowMPMSolver.hpp>
#include <API/Python/Solver/Particle/PCISPH/PCISPHSolver.hpp>
#include <API/Python/Solver/Particle/ParticleSystemSolver.hpp>
#include <API/Python/Solver/Particle/SPH/SPHSolver.hpp>
#include <API/Python/Utils/Constants.hpp>
#include <API/Python/Utils/Logging.hpp>
#include <API/Python/Utils/Serializable.hpp>
#include <API/Python/Vector/Vector.hpp>

#include <pybind11/pybind11.h>

PYBIND11_MODULE(pyCubbyFlow, m)
{
    m.doc() =
        R"pbdoc(
			Voxel-based fluid simulation engine for computer games
		)pbdoc";

    // Constants
    AddConstants(m);

    // Serialization
    AddSerializable(m);

    // Trivial basic types
    AddVectors(m);

    // Rays
    AddRay2D(m);
    AddRay2F(m);
    AddRay3D(m);
    AddRay3F(m);

    // Bounding boxes
    AddBoundingBoxRayIntersectionF(m);
    AddBoundingBoxRayIntersectionD(m);
    AddBoundingBox2D(m);
    AddBoundingBox2F(m);
    AddBoundingBox3D(m);
    AddBoundingBox3F(m);

    // Frames
    AddFrame(m);

    // Quaternions
    AddQuaternionD(m);
    AddQuaternionF(m);

    // Transforms
    AddTransform2(m);
    AddTransform3(m);

    // Containers/helpers
    AddArrayView1(m);
    AddArrayView2(m);
    AddArrayView3(m);

    // Trivial APIs
    AddLogging(m);

    // Fields
    AddField2(m);
    AddField3(m);
    AddScalarField2(m);
    AddScalarField3(m);
    AddVectorField2(m);
    AddVectorField3(m);
    AddConstantScalarField2(m);
    AddConstantScalarField3(m);
    AddConstantVectorField2(m);
    AddConstantVectorField3(m);
    AddCustomScalarField2(m);
    AddCustomScalarField3(m);
    AddCustomVectorField2(m);
    AddCustomVectorField3(m);

    // Grids
    AddGrid2(m);
    AddGrid3(m);
    AddScalarGrid2(m);
    AddScalarGrid3(m);
    AddVectorGrid2(m);
    AddVectorGrid3(m);
    AddCollocatedVectorGrid2(m);
    AddCollocatedVectorGrid3(m);
    AddCellCenteredScalarGrid2(m);
    AddCellCenteredScalarGrid3(m);
    AddCellCenteredVectorGrid2(m);
    AddCellCenteredVectorGrid3(m);
    AddVertexCenteredScalarGrid2(m);
    AddVertexCenteredScalarGrid3(m);
    AddVertexCenteredVectorGrid2(m);
    AddVertexCenteredVectorGrid3(m);
    AddFaceCenteredGrid2(m);
    AddFaceCenteredGrid3(m);

    // Surfaces
    AddSurface2(m);
    AddSurface3(m);
    AddSurfaceSet2(m);
    AddSurfaceSet3(m);

    // Geometries, part 1
    AddBox2(m);
    AddBox3(m);
    AddCylinder3(m);
    AddPlane2(m);
    AddPlane3(m);
    AddSphere2(m);
    AddSphere3(m);
    AddTriangle3(m);
    AddTriangleMesh3(m);

    // Implicit surfaces
    AddImplicitSurface2(m);
    AddImplicitSurface3(m);
    AddSurfaceToImplicit2(m);
    AddSurfaceToImplicit3(m);

    // Geometries, part 2
    AddImplicitTriangleMesh3(m);
    AddMarchingCubes(m);

    // Data models
    AddGridSystemData2(m);
    AddGridSystemData3(m);
    AddParticleSystemData2(m);
    AddParticleSystemData3(m);
    AddMPMSystemData2(m);
    AddMPMSystemData3(m);
    AddMPMFluidSystemData2(m);
    AddMPMFluidSystemData3(m);
    AddSPHSystemData2(m);
    AddSPHSystemData3(m);

    // Emitters
    AddGridEmitter2(m);
    AddGridEmitter3(m);
    AddVolumeGridEmitter2(m);
    AddVolumeGridEmitter3(m);
    AddParticleEmitter2(m);
    AddParticleEmitter3(m);
    AddParticleEmitterSet2(m);
    AddParticleEmitterSet3(m);
    AddPointParticleEmitter2(m);
    AddPointParticleEmitter3(m);
    AddVolumeParticleEmitter2(m);
    AddVolumeParticleEmitter3(m);

    // Colliders
    AddCollider2(m);
    AddCollider3(m);
    AddColliderSet2(m);
    AddColliderSet3(m);
    AddRigidBodyCollider2(m);
    AddRigidBodyCollider3(m);

    // Advection solvers
    AddAdvectionSolver2(m);
    AddAdvectionSolver3(m);

    // Semi-lagrangians
    AddSemiLagrangian2(m);
    AddSemiLagrangian3(m);
    AddCubicSemiLagrangian2(m);
    AddCubicSemiLagrangian3(m);

    // Solvers, part 1
    AddFDMLinearSystemSolver2(m);
    AddFDMLinearSystemSolver3(m);
    AddFDMJacobiSolver2(m);
    AddFDMJacobiSolver3(m);
    AddFDMGaussSeidelSolver2(m);
    AddFDMGaussSeidelSolver3(m);
    AddFDMCGSolver2(m);
    AddFDMCGSolver3(m);
    AddFDMICCGSolver2(m);
    AddFDMICCGSolver3(m);
    AddFDMMGSolver2(m);
    AddFDMMGSolver3(m);
    AddFDMMGPCGSolver2(m);
    AddFDMMGPCGSolver3(m);
    AddGridDiffusionSolver2(m);
    AddGridDiffusionSolver3(m);
    AddGridForwardEulerDiffusionSolver2(m);
    AddGridForwardEulerDiffusionSolver3(m);
    AddGridBackwardEulerDiffusionSolver2(m);
    AddGridBackwardEulerDiffusionSolver3(m);
    AddGridBoundaryConditionSolver2(m);
    AddGridBoundaryConditionSolver3(m);
    AddGridFractionalBoundaryConditionSolver2(m);
    AddGridFractionalBoundaryConditionSolver3(m);
    AddGridBlockedBoundaryConditionSolver2(m);
    AddGridBlockedBoundaryConditionSolver3(m);
    AddGridPressureSolver2(m);
    AddGridPressureSolver3(m);
    AddGridSinglePhasePressureSolver2(m);
    AddGridSinglePhasePressureSolver3(m);
    AddGridFractionalSinglePhasePressureSolver2(m);
    AddGridFractionalSinglePhasePressureSolver3(m);
    AddLevelSetSolver2(m);
    AddLevelSetSolver3(m);
    AddIterativeLevelSetSolver2(m);
    AddIterativeLevelSetSolver3(m);
    AddUpwindLevelSetSolver2(m);
    AddUpwindLevelSetSolver3(m);
    AddENOLevelSetSolver2(m);
    AddENOLevelSetSolver3(m);
    AddFMMLevelSetSolver2(m);
    AddFMMLevelSetSolver3(m);

    // Points to implicit functions
    AddPointsToImplicit2(m);
    AddPointsToImplicit3(m);
    AddAnisotropicPointsToImplicit2(m);
    AddAnisotropicPointsToImplicit3(m);
    AddSphericalPointsToImplicit2(m);
    AddSphericalPointsToImplicit3(m);
    AddSPHPointsToImplicit2(m);
    AddSPHPointsToImplicit3(m);
    AddZhuBridsonPointsToImplicit2(m);
    AddZhuBridsonPointsToImplicit3(m);

    // Animations
    AddAnimation(m);
    AddPhysicsAnimation(m);

    // Solvers, part 2
    AddGridFluidSolver2(m);
    AddGridFluidSolver3(m);
    AddGridSmokeSolver2(m);
    AddGridSmokeSolver3(m);
    AddLevelSetLiquidSolver2(m);
    AddLevelSetLiquidSolver3(m);
    AddPICSolver2(m);
    AddPICSolver3(m);
    AddFLIPSolver2(m);
    AddFLIPSolver3(m);
    AddAPICSolver2(m);
    AddAPICSolver3(m);
    AddParticleSystemSolver2(m);
    AddParticleSystemSolver3(m);
    AddMPMFluidSolver2(m);
    AddMPMFluidSolver3(m);
    AddSnowMPMSolver2(m);
    AddSnowMPMSolver3(m);
    AddSPHSolver2(m);
    AddSPHSolver3(m);
    AddPCISPHSolver2(m);
    AddPCISPHSolver3(m);

#ifdef VERSION_INFO
    m.attr("__version__") = pybind11::str(VERSION_INFO);
#else
    m.attr("__version__") = pybind11::str("Dev");
#endif
}
