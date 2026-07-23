#include "gtest/gtest.h"

#include <Core/Emitter/GridEmitterSet2.hpp>
#include <Core/Emitter/GridEmitterSet3.hpp>
#include <Core/Emitter/ParticleEmitterSet2.hpp>
#include <Core/Emitter/ParticleEmitterSet3.hpp>
#include <Core/Emitter/PointParticleEmitter2.hpp>
#include <Core/Emitter/PointParticleEmitter3.hpp>
#include <Core/Field/ConstantScalarField.hpp>
#include <Core/Field/ConstantVectorField.hpp>
#include <Core/Geometry/Cylinder3.hpp>
#include <Core/Geometry/Triangle3.hpp>
#include <Core/Searcher/PointHashGridSearcher.hpp>
#include <Core/Searcher/PointKdTreeSearcher.hpp>
#include <Core/Searcher/PointSimpleListSearcher.hpp>
#include <Core/Solver/Grid/GridSmokeSolver2.hpp>
#include <Core/Solver/Grid/GridSmokeSolver3.hpp>
#include <Core/Solver/Hybrid/APIC/APICSolver2.hpp>
#include <Core/Solver/Hybrid/APIC/APICSolver3.hpp>
#include <Core/Solver/Hybrid/FLIP/FLIPSolver2.hpp>
#include <Core/Solver/Hybrid/FLIP/FLIPSolver3.hpp>
#include <Core/Solver/Hybrid/PIC/PICSolver2.hpp>
#include <Core/Solver/Hybrid/PIC/PICSolver3.hpp>
#include <Core/Solver/LevelSet/LevelSetLiquidSolver2.hpp>
#include <Core/Solver/LevelSet/LevelSetLiquidSolver3.hpp>
#include <Core/Solver/Particle/PCISPH/PCISPHSolver2.hpp>
#include <Core/Solver/Particle/PCISPH/PCISPHSolver3.hpp>
#include <Core/Solver/Particle/ParticleSystemSolver2.hpp>
#include <Core/Solver/Particle/ParticleSystemSolver3.hpp>
#include <Core/Solver/Particle/SPH/SPHSolver2.hpp>
#include <Core/Solver/Particle/SPH/SPHSolver3.hpp>

using namespace CubbyFlow;

template <typename T>
void ExpectMakeShared()
{
    EXPECT_NE(nullptr, T::GetBuilder().MakeShared());
}

TEST(Builder, MakeSharedEmitters)
{
    ExpectMakeShared<GridEmitterSet2>();
    ExpectMakeShared<GridEmitterSet3>();
    ExpectMakeShared<ParticleEmitterSet2>();
    ExpectMakeShared<ParticleEmitterSet3>();
    ExpectMakeShared<PointParticleEmitter2>();
    ExpectMakeShared<PointParticleEmitter3>();
}

TEST(Builder, MakeSharedFieldsGeometryAndSearchers)
{
    ExpectMakeShared<ConstantScalarField2>();
    ExpectMakeShared<ConstantScalarField3>();
    ExpectMakeShared<ConstantVectorField2>();
    ExpectMakeShared<ConstantVectorField3>();
    ExpectMakeShared<Cylinder3>();
    ExpectMakeShared<Triangle3>();
    ExpectMakeShared<PointHashGridSearcher2>();
    ExpectMakeShared<PointHashGridSearcher3>();
    ExpectMakeShared<PointKdTreeSearcher2>();
    ExpectMakeShared<PointKdTreeSearcher3>();
    ExpectMakeShared<PointSimpleListSearcher2>();
    ExpectMakeShared<PointSimpleListSearcher3>();
}

TEST(Builder, MakeSharedGridSolvers)
{
    ExpectMakeShared<GridSmokeSolver2>();
    ExpectMakeShared<GridSmokeSolver3>();
    ExpectMakeShared<APICSolver2>();
    ExpectMakeShared<APICSolver3>();
    ExpectMakeShared<FLIPSolver2>();
    ExpectMakeShared<FLIPSolver3>();
    ExpectMakeShared<PICSolver2>();
    ExpectMakeShared<PICSolver3>();
    ExpectMakeShared<LevelSetLiquidSolver2>();
    ExpectMakeShared<LevelSetLiquidSolver3>();
}

TEST(Builder, MakeSharedParticleSolvers)
{
    ExpectMakeShared<ParticleSystemSolver2>();
    ExpectMakeShared<ParticleSystemSolver3>();
    ExpectMakeShared<PCISPHSolver2>();
    ExpectMakeShared<PCISPHSolver3>();
    ExpectMakeShared<SPHSolver2>();
    ExpectMakeShared<SPHSolver3>();
}
