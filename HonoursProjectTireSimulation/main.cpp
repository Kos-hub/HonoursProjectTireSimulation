#include "PxPhysicsAPI.h"
#include "vehicle/PxVehicleUtil.h"
#include "VehicleSceneQuery.h"
#include "VehicleFilterShader.h"
#include "VehicleCreate.h"
#include "VehicleTireFriction.h"
#include "MagicFormulaTireModel.h"
#include "DefaultTireModel.h"
#include <stdio.h>
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <functional>

// TODO: Instead of writing to a string each time the wheel function is called, what I can simply do is save at the end of the step.
// Therefore, I'd have to store long slip, lat slip and all the forces in temp values that will then be passed to a csv string.


using namespace physx;
using namespace snippetvehicle;

PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;

PxFoundation* gFoundation = NULL;
PxPhysics* gPhysics = NULL;

PxDefaultCpuDispatcher* gDispatcher = NULL;
PxScene* gScene = NULL;

PxCooking* gCooking = NULL;

PxMaterial* gMaterial = NULL;

PxPvd* gPvd = NULL;

VehicleSceneQueryData* gVehicleSceneQueryData = NULL;
PxBatchQuery* gBatchQuery = NULL;

PxVehicleDrivableSurfaceToTireFrictionPairs* gFrictionPairs = NULL;

PxRigidStatic* gGroundPlane = NULL;
PxVehicleNoDrive* gVehicleNoDriveMagic = NULL;
PxVehicleNoDrive* gVehicleNoDriveDefault = NULL;

MagicFormulaTireModel* magicFormulaTireModel;
DefaultTireModel* defaultTireModel;

enum DriveMode
{
	eDRIVE_MODE_ACCEL_FORWARDS = 0,
	eDRIVE_MODE_ACCEL_REVERSE,
	eDRIVE_MODE_HARD_TURN_LEFT,
	eDRIVE_MODE_HANDBRAKE_TURN_LEFT,
	eDRIVE_MODE_HARD_TURN_RIGHT,
	eDRIVE_MODE_HANDBRAKE_TURN_RIGHT,
	eDRIVE_MODE_BRAKE,
	eDRIVE_MODE_NONE
};

// DRIVE PATTERN
DriveMode gDriveModeOrder[] =
{
	eDRIVE_MODE_BRAKE,
	eDRIVE_MODE_ACCEL_FORWARDS,
	eDRIVE_MODE_BRAKE,
	eDRIVE_MODE_ACCEL_REVERSE,
	eDRIVE_MODE_BRAKE,
	eDRIVE_MODE_HARD_TURN_LEFT,
	eDRIVE_MODE_BRAKE,
	eDRIVE_MODE_HARD_TURN_RIGHT,
	eDRIVE_MODE_ACCEL_FORWARDS,
	eDRIVE_MODE_HANDBRAKE_TURN_LEFT,
	eDRIVE_MODE_ACCEL_FORWARDS,
	eDRIVE_MODE_HANDBRAKE_TURN_RIGHT,
	eDRIVE_MODE_NONE
};

PxF32					gVehicleModeLifetime = 4.0f;
PxF32					gVehicleModeTimer = 0.0f;
bool					gVehicleOrderComplete = false;
PxU32					gVehicleOrderProgress = 0;

typedef void (*PxVehicleComputeTireForce)
(const void* shaderData,
	const PxF32 tireFriction,
	const PxF32 longSlip, const PxF32 latSlip, const PxF32 camber,
	const PxF32 wheelOmega, const PxF32 wheelRadius, const PxF32 recipWheelRadius,
	const PxF32 restTireLoad, const PxF32 normalisedTireLoad, const PxF32 tireLoad,
	const PxF32 gravity, const PxF32 recipGravity,
	PxF32& wheelTorque, PxF32& tireLongForceMag, PxF32& tireLatForceMag, PxF32& tireAlignMoment);

VehicleDesc initVehicleDesc()
{
	//Set up the chassis mass, dimensions, moment of inertia, and center of mass offset.
	//The moment of inertia is just the moment of inertia of a cuboid but modified for easier steering.
	//Center of mass offset is 0.65m above the base of the chassis and 0.25m towards the front.
	const PxF32 chassisMass = 1500.0f;
	const PxVec3 chassisDims(2.5f, 2.0f, 5.0f);
	const PxVec3 chassisMOI
	((chassisDims.y * chassisDims.y + chassisDims.z * chassisDims.z) * chassisMass / 12.0f,
		(chassisDims.x * chassisDims.x + chassisDims.z * chassisDims.z) * 0.8f * chassisMass / 12.0f,
		(chassisDims.x * chassisDims.x + chassisDims.y * chassisDims.y) * chassisMass / 12.0f);
	const PxVec3 chassisCMOffset(0.0f, -chassisDims.y * 0.5f + 0.65f, 0.25f);

	//Set up the wheel mass, radius, width, moment of inertia, and number of wheels.
	//Moment of inertia is just the moment of inertia of a cylinder.
	const PxF32 wheelMass = 20.0f;
	const PxF32 wheelRadius = 0.5f;
	const PxF32 wheelWidth = 0.4f;
	const PxF32 wheelMOI = 0.5f * wheelMass * wheelRadius * wheelRadius;
	const PxU32 nbWheels = 4;

	VehicleDesc vehicleDesc;

	vehicleDesc.chassisMass = chassisMass;
	vehicleDesc.chassisDims = chassisDims;
	vehicleDesc.chassisMOI = chassisMOI;
	vehicleDesc.chassisCMOffset = chassisCMOffset;
	vehicleDesc.chassisMaterial = gMaterial;
	vehicleDesc.chassisSimFilterData = PxFilterData(COLLISION_FLAG_CHASSIS, COLLISION_FLAG_CHASSIS_AGAINST, 0, 0);

	vehicleDesc.wheelMass = wheelMass;
	vehicleDesc.wheelRadius = wheelRadius;
	vehicleDesc.wheelWidth = wheelWidth;
	vehicleDesc.wheelMOI = wheelMOI;
	vehicleDesc.numWheels = nbWheels;
	vehicleDesc.wheelMaterial = gMaterial;
	vehicleDesc.chassisSimFilterData = PxFilterData(COLLISION_FLAG_WHEEL, COLLISION_FLAG_WHEEL_AGAINST, 0, 0);

	return vehicleDesc;
}


void startAccelerateForwardsMode()
{
	gVehicleNoDriveMagic->setDriveTorque(0, 1000.0f);
	gVehicleNoDriveDefault->setDriveTorque(0, 1000.0f);

	gVehicleNoDriveMagic->setDriveTorque(1, 1000.0f);
	gVehicleNoDriveDefault->setDriveTorque(1, 1000.0f);
}

void startAccelerateReverseMode()
{
	gVehicleNoDriveMagic->setDriveTorque(0, -1000.0f);
	gVehicleNoDriveDefault->setDriveTorque(0, -1000.0f);

	gVehicleNoDriveMagic->setDriveTorque(1, -1000.0f);
	gVehicleNoDriveDefault->setDriveTorque(1, -1000.0f);

}

void startBrakeMode()
{
	gVehicleNoDriveMagic->setBrakeTorque(0, 1000.0f);
	gVehicleNoDriveDefault->setBrakeTorque(0, 1000.0f);

	gVehicleNoDriveMagic->setBrakeTorque(1, 1000.0f);
	gVehicleNoDriveDefault->setBrakeTorque(1, 1000.0f);

	gVehicleNoDriveMagic->setBrakeTorque(2, 1000.0f);
	gVehicleNoDriveDefault->setBrakeTorque(2, 1000.0f);

	gVehicleNoDriveMagic->setBrakeTorque(3, 1000.0f);
	gVehicleNoDriveDefault->setBrakeTorque(3, 1000.0f);
}

void startTurnHardLeftMode()
{
	gVehicleNoDriveMagic->setDriveTorque(0, 1000.0f);
	gVehicleNoDriveDefault->setDriveTorque(0, 1000.0f);

	gVehicleNoDriveMagic->setDriveTorque(1, 1000.0f);
	gVehicleNoDriveDefault->setDriveTorque(1, 1000.0f);

	gVehicleNoDriveMagic->setSteerAngle(0, 1.0f);
	gVehicleNoDriveDefault->setSteerAngle(0, 1.0f);

	gVehicleNoDriveMagic->setSteerAngle(1, 1.0f);
	gVehicleNoDriveDefault->setSteerAngle(1, 1.0f);
}

void startTurnHardRightMode()
{
	gVehicleNoDriveMagic->setDriveTorque(0, 1000.0f);
	gVehicleNoDriveDefault->setDriveTorque(0, 1000.0f);

	gVehicleNoDriveMagic->setDriveTorque(1, 1000.0f);
	gVehicleNoDriveDefault->setDriveTorque(1, 1000.0f);

	gVehicleNoDriveMagic->setSteerAngle(0, -1.0f);
	gVehicleNoDriveDefault->setSteerAngle(0, -1.0f);

	gVehicleNoDriveMagic->setSteerAngle(1, -1.0f);
	gVehicleNoDriveDefault->setSteerAngle(1, -1.0f);
}

void startHandbrakeTurnLeftMode()
{
	gVehicleNoDriveMagic->setBrakeTorque(2, 1000.0f);
	gVehicleNoDriveDefault->setBrakeTorque(2, 1000.0f);

	gVehicleNoDriveMagic->setBrakeTorque(3, 1000.0f);
	gVehicleNoDriveDefault->setBrakeTorque(3, 1000.0f);

	gVehicleNoDriveMagic->setDriveTorque(0, 1000.0f);
	gVehicleNoDriveDefault->setDriveTorque(0, 1000.0f);

	gVehicleNoDriveMagic->setDriveTorque(1, 1000.0f);
	gVehicleNoDriveDefault->setDriveTorque(1, 1000.0f);

	gVehicleNoDriveMagic->setSteerAngle(0, 1.0f);
	gVehicleNoDriveDefault->setSteerAngle(0, 1.0f);

	gVehicleNoDriveMagic->setSteerAngle(1, 1.0f);
	gVehicleNoDriveDefault->setSteerAngle(1, 1.0f);
}

void startHandbrakeTurnRightMode()
{
	gVehicleNoDriveMagic->setBrakeTorque(2, 1000.0f);
	gVehicleNoDriveDefault->setBrakeTorque(2, 1000.0f);

	gVehicleNoDriveMagic->setBrakeTorque(3, 1000.0f);
	gVehicleNoDriveDefault->setBrakeTorque(3, 1000.0f);

	gVehicleNoDriveMagic->setDriveTorque(0, 1000.0f);
	gVehicleNoDriveDefault->setDriveTorque(0, 1000.0f);

	gVehicleNoDriveMagic->setDriveTorque(1, 1000.0f);
	gVehicleNoDriveDefault->setDriveTorque(1, 1000.0f);
	
	gVehicleNoDriveMagic->setSteerAngle(0, -1.0f);
	gVehicleNoDriveDefault->setSteerAngle(0, -1.0f);

	gVehicleNoDriveMagic->setSteerAngle(1, -1.0f);
	gVehicleNoDriveDefault->setSteerAngle(1, -1.0f);
}

void releaseAllControls()
{
	gVehicleNoDriveMagic->setDriveTorque(0, 0.0f);
	gVehicleNoDriveDefault->setDriveTorque(0, 0.0f);

	gVehicleNoDriveMagic->setDriveTorque(1, 0.0f);
	gVehicleNoDriveDefault->setDriveTorque(1, 0.0f);

	gVehicleNoDriveMagic->setDriveTorque(2, 0.0f);
	gVehicleNoDriveDefault->setDriveTorque(2, 0.0f);

	gVehicleNoDriveMagic->setDriveTorque(3, 0.0f);
	gVehicleNoDriveDefault->setDriveTorque(3, 0.0f);

	gVehicleNoDriveMagic->setBrakeTorque(0, 0.0f);
	gVehicleNoDriveDefault->setBrakeTorque(0, 0.0f);

	gVehicleNoDriveMagic->setBrakeTorque(1, 0.0f);
	gVehicleNoDriveDefault->setBrakeTorque(1, 0.0f);

	gVehicleNoDriveMagic->setBrakeTorque(2, 0.0f);
	gVehicleNoDriveDefault->setBrakeTorque(2, 0.0f);

	gVehicleNoDriveMagic->setBrakeTorque(3, 0.0f);
	gVehicleNoDriveDefault->setBrakeTorque(3, 0.0f);


	gVehicleNoDriveMagic->setSteerAngle(0, 0.0f);
	gVehicleNoDriveDefault->setSteerAngle(0, 0.0f);

	gVehicleNoDriveMagic->setSteerAngle(1, 0.0f);
	gVehicleNoDriveDefault->setSteerAngle(1, 0.0f);

	gVehicleNoDriveMagic->setSteerAngle(2, 0.0f);
	gVehicleNoDriveDefault->setSteerAngle(2, 0.0f);

	gVehicleNoDriveMagic->setSteerAngle(3, 0.0f);
	gVehicleNoDriveDefault->setSteerAngle(3, 0.0f);
}

void initPhysics()
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10);
	gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

	PxTolerancesScale scale;
	scale.length = 100;
	scale.speed = 981;

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);

	PxU32 numWorkers = 1;
	gDispatcher = PxDefaultCpuDispatcherCreate(numWorkers);
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = VehicleFilterShader;

	gScene = gPhysics->createScene(sceneDesc);

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if (pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	gCooking = PxCreateCooking(PX_PHYSICS_VERSION, *gFoundation, PxCookingParams(PxTolerancesScale()));

	/////////////////////////////////////////////

	PxInitVehicleSDK(*gPhysics);
	PxVehicleSetBasisVectors(PxVec3(0, 1, 0), PxVec3(0, 0, 1));
	PxVehicleSetUpdateMode(PxVehicleUpdateMode::eVELOCITY_CHANGE);

	//Create the batched scene queries for the suspension raycasts.
	gVehicleSceneQueryData = VehicleSceneQueryData::allocate(1, PX_MAX_NB_WHEELS, 1, 1, WheelSceneQueryPreFilterBlocking, NULL, gAllocator);
	gBatchQuery = VehicleSceneQueryData::setUpBatchedSceneQuery(0, *gVehicleSceneQueryData, gScene);

	//Create the friction table for each combination of tire and surface type.
	gFrictionPairs = createFrictionPairs(gMaterial);

	//Create a plane to drive on.
	PxFilterData groundPlaneSimFilterData(COLLISION_FLAG_GROUND, COLLISION_FLAG_GROUND_AGAINST, 0, 0);
	gGroundPlane = createDrivablePlane(groundPlaneSimFilterData, gMaterial, gPhysics);
	gScene->addActor(*gGroundPlane);

	//Create a vehicle that will drive on the plane.
	VehicleDesc vehicleDesc = initVehicleDesc();

	gVehicleNoDriveMagic = createVehicleNoDrive(vehicleDesc, gPhysics, gCooking);
	gVehicleNoDriveDefault = createVehicleNoDrive(vehicleDesc, gPhysics, gCooking);

	PxTransform startTransformMagic(PxVec3(0, (vehicleDesc.chassisDims.y * 0.5f + vehicleDesc.wheelRadius + 1.0f), 0), PxQuat(PxIdentity));
	PxTransform startTransformDefault(PxVec3(15.f, (vehicleDesc.chassisDims.y * 0.5f + vehicleDesc.wheelRadius + 1.0f), 0), PxQuat(PxIdentity));

	gVehicleNoDriveMagic->getRigidDynamicActor()->setGlobalPose(startTransformMagic);
	gVehicleNoDriveDefault->getRigidDynamicActor()->setGlobalPose(startTransformDefault);

	gScene->addActor(*gVehicleNoDriveMagic->getRigidDynamicActor());
	gScene->addActor(*gVehicleNoDriveDefault->getRigidDynamicActor());


	magicFormulaTireModel = new MagicFormulaTireModel();
	defaultTireModel = new DefaultTireModel();

	gVehicleNoDriveDefault->mWheelsDynData.setTireForceShaderFunction(&defaultTireModel->Function);
	gVehicleNoDriveMagic->mWheelsDynData.setTireForceShaderFunction(&magicFormulaTireModel->Function);


	//Set the vehicle to rest in first gear.
	//Set the vehicle to use auto-gears.
	gVehicleNoDriveMagic->setToRestState();
	gVehicleNoDriveDefault->setToRestState();


	gVehicleModeTimer = 0.0f;
	gVehicleOrderProgress = 0;
	startBrakeMode();
}

void incrementDrivingMode(const PxF32 timestep)
{
	gVehicleModeTimer += timestep;
	if (gVehicleModeTimer > gVehicleModeLifetime)
	{
		//Increment to next driving mode.
		gVehicleModeTimer = 0.0f;
		gVehicleOrderProgress++;
		releaseAllControls();

		//If we are at the end of the list of driving modes then start again.
		if (eDRIVE_MODE_NONE == gDriveModeOrder[gVehicleOrderProgress])
		{
			gVehicleOrderProgress = 0;
			gVehicleOrderComplete = true;
		}

		//Start driving in the selected mode.
		DriveMode eDriveMode = gDriveModeOrder[gVehicleOrderProgress];
		switch (eDriveMode)
		{
		case eDRIVE_MODE_ACCEL_FORWARDS:
			startAccelerateForwardsMode();
			break;
		case eDRIVE_MODE_ACCEL_REVERSE:
			startAccelerateReverseMode();
			break;
		case eDRIVE_MODE_HARD_TURN_LEFT:
			startTurnHardLeftMode();
			break;
		case eDRIVE_MODE_HANDBRAKE_TURN_LEFT:
			startHandbrakeTurnLeftMode();
			break;
		case eDRIVE_MODE_HARD_TURN_RIGHT:
			startTurnHardRightMode();
			break;
		case eDRIVE_MODE_HANDBRAKE_TURN_RIGHT:
			startHandbrakeTurnRightMode();
			break;
		case eDRIVE_MODE_BRAKE:
			startBrakeMode();
			break;
		case eDRIVE_MODE_NONE:
			break;
		};
	}
}

void stepPhysics()
{

	//printf("Init step\n");

	const PxF32 timestep = 1.0f / 144.0f;

	//Cycle through the driving modes to demonstrate how to accelerate/reverse/brake/turn etc.
	incrementDrivingMode(timestep);

	//Raycasts.
	PxVehicleWheels* vehicles[2] = { gVehicleNoDriveMagic, gVehicleNoDriveDefault };
	PxRaycastQueryResult* raycastResults = gVehicleSceneQueryData->getRaycastQueryResultBuffer(0);
	const PxU32 raycastResultsSize = gVehicleSceneQueryData->getQueryResultBufferSize();
	PxVehicleSuspensionRaycasts(gBatchQuery, 2, vehicles, raycastResultsSize, raycastResults);

	//Vehicle update.
	const PxVec3 grav = gScene->getGravity();
	PxWheelQueryResult wheelQueryResultsMagic[PX_MAX_NB_WHEELS];
	PxWheelQueryResult wheelQueryResultsDefault[PX_MAX_NB_WHEELS];
	PxVehicleWheelQueryResult vehicleQueryResults[2] = { 
		{wheelQueryResultsMagic, gVehicleNoDriveMagic->mWheelsSimData.getNbWheels()}, 
		{wheelQueryResultsDefault, gVehicleNoDriveDefault->mWheelsSimData.getNbWheels()} 
	};
	PxVehicleUpdates(timestep, grav, *gFrictionPairs, 2, vehicles, vehicleQueryResults);

	
	//Scene update
	gScene->simulate(timestep);
	gScene->fetchResults(true);

	magicFormulaTireModel->_csvHelper->FlushTemp();
	defaultTireModel->_csvHelper->FlushTemp();

	//printf("Finishing step\n\n");
}

void cleanupPhysics()
{

	gVehicleNoDriveMagic->getRigidDynamicActor()->release();
	gVehicleNoDriveMagic->free();

	gVehicleNoDriveDefault->getRigidDynamicActor()->release();
	gVehicleNoDriveDefault->free();
	gGroundPlane->release();
	gBatchQuery->release();
	gVehicleSceneQueryData->free(gAllocator);
	gFrictionPairs->release();
	PxCloseVehicleSDK();

	gMaterial->release();
	gCooking->release();
	gScene->release();
	gDispatcher->release();
	gPhysics->release();

	if (gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();
		gPvd->release();	gPvd = NULL;
		transport->release();
	}

	gFoundation->release();
	delete magicFormulaTireModel;
	delete defaultTireModel;

	system("Rscript C:/Users/giana/Documents/HonsProjectScript.R");

	printf("SnippetVehicleNoDrive done.\n");

}

void keyPress(unsigned char key, const PxTransform& camera)
{
	PX_UNUSED(camera);
	PX_UNUSED(key);
}

int main()
{
	initPhysics();

	while (!gVehicleOrderComplete)
	{
		stepPhysics();
	}

	cleanupPhysics();
	return 0;
}
int snippetMain(int, const char* const*)
{
#ifdef RENDER_SNIPPET
	extern void renderLoop();
	renderLoop();
#else
	initPhysics();
	while (!gVehicleOrderComplete)
	{
		stepPhysics();
	}
	cleanupPhysics();
#endif


	return 0;
}
