#include "gtest/gtest.h"

#include <Core/Geometry/Box.hpp>
#include <Core/Geometry/Cylinder3.hpp>
#include <Core/Geometry/Plane.hpp>
#include <Core/Geometry/Sphere.hpp>
#include <Core/Geometry/Triangle3.hpp>
#include <Core/Geometry/TriangleMesh3.hpp>
#include <Core/PointsToImplicit/AnisotropicPointsToImplicit2.hpp>
#include <Core/PointsToImplicit/AnisotropicPointsToImplicit3.hpp>
#include <Core/PointsToImplicit/SPHPointsToImplicit2.hpp>
#include <Core/PointsToImplicit/SPHPointsToImplicit3.hpp>
#include <Core/PointsToImplicit/SphericalPointsToImplicit2.hpp>
#include <Core/PointsToImplicit/SphericalPointsToImplicit3.hpp>
#include <Core/PointsToImplicit/ZhuBridsonPointsToImplicit2.hpp>
#include <Core/PointsToImplicit/ZhuBridsonPointsToImplicit3.hpp>

using namespace CubbyFlow;

template <typename T>
void ExpectCopyListDefaultConstruction()
{
    [[maybe_unused]] T value = {};
}

TEST(DefaultConstruction, PreservesCopyListInitialization)
{
    ExpectCopyListDefaultConstruction<Box2>();
    ExpectCopyListDefaultConstruction<Box3>();
    ExpectCopyListDefaultConstruction<Cylinder3>();
    ExpectCopyListDefaultConstruction<Plane2>();
    ExpectCopyListDefaultConstruction<Plane3>();
    ExpectCopyListDefaultConstruction<Sphere2>();
    ExpectCopyListDefaultConstruction<Sphere3>();
    ExpectCopyListDefaultConstruction<Triangle3>();
    ExpectCopyListDefaultConstruction<TriangleMesh3>();
    ExpectCopyListDefaultConstruction<AnisotropicPointsToImplicit2>();
    ExpectCopyListDefaultConstruction<AnisotropicPointsToImplicit3>();
    ExpectCopyListDefaultConstruction<SPHPointsToImplicit2>();
    ExpectCopyListDefaultConstruction<SPHPointsToImplicit3>();
    ExpectCopyListDefaultConstruction<SphericalPointsToImplicit2>();
    ExpectCopyListDefaultConstruction<SphericalPointsToImplicit3>();
    ExpectCopyListDefaultConstruction<ZhuBridsonPointsToImplicit2>();
    ExpectCopyListDefaultConstruction<ZhuBridsonPointsToImplicit3>();
}
