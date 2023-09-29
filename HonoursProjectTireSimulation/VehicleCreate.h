#pragma once
#include "PxPhysicsAPI.h"
#include "vehicle/PxVehicleDriveTank.h"
#include "vehicle/PxVehicleNoDrive.h"

namespace snippetvehicle
{

	using namespace physx;

	////////////////////////////////////////////////

	PxRigidStatic* createDrivablePlane(const PxFilterData& simFilterData, PxMaterial* material, PxPhysics* physics);

	////////////////////////////////////////////////

	struct ActorUserData
	{
		ActorUserData()
			: vehicle(NULL),
			actor(NULL)
		{
		}

		const PxVehicleWheels* vehicle;
		const PxActor* actor;
	};

	struct ShapeUserData
	{
		ShapeUserData()
			: isWheel(false),
			wheelId(0xffffffff)
		{
		}

		bool isWheel;
		PxU32 wheelId;
	};


	struct VehicleDesc
	{
		VehicleDesc()
			: chassisMass(0.0f),
			chassisDims(PxVec3(0.0f, 0.0f, 0.0f)),
			chassisMOI(PxVec3(0.0f, 0.0f, 0.0f)),
			chassisCMOffset(PxVec3(0.0f, 0.0f, 0.0f)),
			chassisMaterial(NULL),
			wheelMass(0.0f),
			wheelWidth(0.0f),
			wheelRadius(0.0f),
			wheelMOI(0.0f),
			wheelMaterial(NULL),
			actorUserData(NULL),
			shapeUserDatas(NULL)
		{
		}

		PxF32 chassisMass;
		PxVec3 chassisDims;
		PxVec3 chassisMOI;
		PxVec3 chassisCMOffset;
		PxMaterial* chassisMaterial;
		PxFilterData chassisSimFilterData;  //word0 = collide type, word1 = collide against types, word2 = PxPairFlags

		PxF32 wheelMass;
		PxF32 wheelWidth;
		PxF32 wheelRadius;
		PxF32 wheelMOI;
		PxMaterial* wheelMaterial;
		PxU32 numWheels;
		PxFilterData wheelSimFilterData;	//word0 = collide type, word1 = collide against types, word2 = PxPairFlags

		ActorUserData* actorUserData;
		ShapeUserData* shapeUserDatas;
	};

	PxVehicleDrive4W* createVehicle4W(const VehicleDesc& vehDesc, PxPhysics* physics, PxCooking* cooking);

	PxVehicleDriveTank* createVehicleTank(const VehicleDesc& vehDesc, PxPhysics* physics, PxCooking* cooking);

	PxVehicleNoDrive* createVehicleNoDrive(const VehicleDesc& vehDesc, PxPhysics* physics, PxCooking* cooking);

	////////////////////////////////////////////////

	PxConvexMesh* createChassisMesh(const PxVec3 dims, PxPhysics& physics, PxCooking& cooking);

	PxConvexMesh* createWheelMesh(const PxF32 width, const PxF32 radius, PxPhysics& physics, PxCooking& cooking);

	////////////////////////////////////////////////

	void customizeVehicleToLengthScale(const PxReal lengthScale, PxRigidDynamic* rigidDynamic, PxVehicleWheelsSimData* wheelsSimData, PxVehicleDriveSimData* driveSimData);

	void customizeVehicleToLengthScale(const PxReal lengthScale, PxRigidDynamic* rigidDynamic, PxVehicleWheelsSimData* wheelsSimData, PxVehicleDriveSimData4W* driveSimData);

	////////////////////////////////////////////////

	PxRigidDynamic* createVehicleActor
	(const PxVehicleChassisData& chassisData,
		PxMaterial** wheelMaterials, PxConvexMesh** wheelConvexMeshes, const PxU32 numWheels, const PxFilterData& wheelSimFilterData,
		PxMaterial** chassisMaterials, PxConvexMesh** chassisConvexMeshes, const PxU32 numChassisMeshes, const PxFilterData& chassisSimFilterData,
		PxPhysics& physics);

	void configureUserData(PxVehicleWheels* vehicle, ActorUserData* actorUserData, ShapeUserData* shapeUserDatas);

	////////////////////////////////////////////////

} // namespace snippetvehicle

