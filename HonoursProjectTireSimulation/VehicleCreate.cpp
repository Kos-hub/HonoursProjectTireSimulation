#include <new>
#include "VehicleCreate.h"
#include "VehicleSceneQuery.h"
#include "VehicleFilterShader.h"
#include "VehicleTireFriction.h"
#include "PxPhysicsAPI.h"

namespace snippetvehicle
{

	using namespace physx;

	void computeWheelCenterActorOffsets4W(const PxF32 wheelFrontZ, const PxF32 wheelRearZ, const PxVec3& chassisDims, const PxF32 wheelWidth, const PxF32 wheelRadius, const PxU32 numWheels, PxVec3* wheelCentreOffsets)
	{
		//chassisDims.z is the distance from the rear of the chassis to the front of the chassis.
		//The front has z = 0.5*chassisDims.z and the rear has z = -0.5*chassisDims.z.
		//Compute a position for the front wheel and the rear wheel along the z-axis.
		//Compute the separation between each wheel along the z-axis.
		const PxF32 numLeftWheels = numWheels / 2.0f;
		const PxF32 deltaZ = (wheelFrontZ - wheelRearZ) / (numLeftWheels - 1.0f);
		//Set the outside of the left and right wheels to be flush with the chassis.
		//Set the top of the wheel to be just touching the underside of the chassis.
		//Begin by setting the rear-left/rear-right/front-left,front-right wheels.
		wheelCentreOffsets[PxVehicleDrive4WWheelOrder::eREAR_LEFT] = PxVec3((-chassisDims.x + wheelWidth) * 0.5f, -(chassisDims.y / 2 + wheelRadius), wheelRearZ + 0 * deltaZ * 0.5f);
		wheelCentreOffsets[PxVehicleDrive4WWheelOrder::eREAR_RIGHT] = PxVec3((+chassisDims.x - wheelWidth) * 0.5f, -(chassisDims.y / 2 + wheelRadius), wheelRearZ + 0 * deltaZ * 0.5f);
		wheelCentreOffsets[PxVehicleDrive4WWheelOrder::eFRONT_LEFT] = PxVec3((-chassisDims.x + wheelWidth) * 0.5f, -(chassisDims.y / 2 + wheelRadius), wheelRearZ + (numLeftWheels - 1) * deltaZ);
		wheelCentreOffsets[PxVehicleDrive4WWheelOrder::eFRONT_RIGHT] = PxVec3((+chassisDims.x - wheelWidth) * 0.5f, -(chassisDims.y / 2 + wheelRadius), wheelRearZ + (numLeftWheels - 1) * deltaZ);
		//Set the remaining wheels.
		for (PxU32 i = 2, wheelCount = 4; i < numWheels - 2; i += 2, wheelCount += 2)
		{
			wheelCentreOffsets[wheelCount + 0] = PxVec3((-chassisDims.x + wheelWidth) * 0.5f, -(chassisDims.y / 2 + wheelRadius), wheelRearZ + i * deltaZ * 0.5f);
			wheelCentreOffsets[wheelCount + 1] = PxVec3((+chassisDims.x - wheelWidth) * 0.5f, -(chassisDims.y / 2 + wheelRadius), wheelRearZ + i * deltaZ * 0.5f);
		}
	}

	void setupWheelsSimulationData
	(const PxF32 wheelMass, const PxF32 wheelMOI, const PxF32 wheelRadius, const PxF32 wheelWidth,
		const PxU32 numWheels, const PxVec3* wheelCenterActorOffsets,
		const PxVec3& chassisCMOffset, const PxF32 chassisMass,
		PxVehicleWheelsSimData* wheelsSimData)
	{
		//Set up the wheels.
		PxVehicleWheelData wheels[PX_MAX_NB_WHEELS];
		{
			//Set up the wheel data structures with mass, moi, radius, width.
			for (PxU32 i = 0; i < numWheels; i++)
			{
				wheels[i].mMass = wheelMass;
				wheels[i].mMOI = wheelMOI;
				wheels[i].mRadius = wheelRadius;
				wheels[i].mWidth = wheelWidth;
			}

			//Enable the handbrake for the rear wheels only.
			wheels[PxVehicleDrive4WWheelOrder::eREAR_LEFT].mMaxHandBrakeTorque = 4000.0f;
			wheels[PxVehicleDrive4WWheelOrder::eREAR_RIGHT].mMaxHandBrakeTorque = 4000.0f;
			//Enable steering for the front wheels only.
			wheels[PxVehicleDrive4WWheelOrder::eFRONT_LEFT].mMaxSteer = PxPi * 0.3333f;
			wheels[PxVehicleDrive4WWheelOrder::eFRONT_RIGHT].mMaxSteer = PxPi * 0.3333f;
		}

		//Set up the tires.
		PxVehicleTireData tires[PX_MAX_NB_WHEELS];
		{
			//Set up the tires.
			for (PxU32 i = 0; i < numWheels; i++)
			{
				tires[i].mType = TIRE_TYPE_NORMAL;
			}
		}

		//Set up the suspensions
		PxVehicleSuspensionData suspensions[PX_MAX_NB_WHEELS];
		{
			//Compute the mass supported by each suspension spring.
			PxF32 suspSprungMasses[PX_MAX_NB_WHEELS];
			PxVehicleComputeSprungMasses
			(numWheels, wheelCenterActorOffsets,
				chassisCMOffset, chassisMass, 1, suspSprungMasses);

			//Set the suspension data.
			for (PxU32 i = 0; i < numWheels; i++)
			{
				suspensions[i].mMaxCompression = 0.3f;
				suspensions[i].mMaxDroop = 0.1f;
				suspensions[i].mSpringStrength = 35000.0f;
				suspensions[i].mSpringDamperRate = 4500.0f;
				suspensions[i].mSprungMass = suspSprungMasses[i];
			}

			//Set the camber angles.
			const PxF32 camberAngleAtRest = 0.0;
			const PxF32 camberAngleAtMaxDroop = 0.01f;
			const PxF32 camberAngleAtMaxCompression = -0.01f;
			for (PxU32 i = 0; i < numWheels; i += 2)
			{
				suspensions[i + 0].mCamberAtRest = camberAngleAtRest;
				suspensions[i + 1].mCamberAtRest = -camberAngleAtRest;
				suspensions[i + 0].mCamberAtMaxDroop = camberAngleAtMaxDroop;
				suspensions[i + 1].mCamberAtMaxDroop = -camberAngleAtMaxDroop;
				suspensions[i + 0].mCamberAtMaxCompression = camberAngleAtMaxCompression;
				suspensions[i + 1].mCamberAtMaxCompression = -camberAngleAtMaxCompression;
			}
		}

		//Set up the wheel geometry.
		PxVec3 suspTravelDirections[PX_MAX_NB_WHEELS];
		PxVec3 wheelCentreCMOffsets[PX_MAX_NB_WHEELS];
		PxVec3 suspForceAppCMOffsets[PX_MAX_NB_WHEELS];
		PxVec3 tireForceAppCMOffsets[PX_MAX_NB_WHEELS];
		{
			//Set the geometry data.
			for (PxU32 i = 0; i < numWheels; i++)
			{
				//Vertical suspension travel.
				suspTravelDirections[i] = PxVec3(0, -1, 0);

				//Wheel center offset is offset from rigid body center of mass.
				wheelCentreCMOffsets[i] =
					wheelCenterActorOffsets[i] - chassisCMOffset;

				//Suspension force application point 0.3 metres below 
				//rigid body center of mass.
				suspForceAppCMOffsets[i] =
					PxVec3(wheelCentreCMOffsets[i].x, -0.3f, wheelCentreCMOffsets[i].z);

				//Tire force application point 0.3 metres below 
				//rigid body center of mass.
				tireForceAppCMOffsets[i] =
					PxVec3(wheelCentreCMOffsets[i].x, -0.3f, wheelCentreCMOffsets[i].z);
			}
		}

		//Set up the filter data of the raycast that will be issued by each suspension.
		PxFilterData qryFilterData;
		setupNonDrivableSurface(qryFilterData);

		//Set the wheel, tire and suspension data.
		//Set the geometry data.
		//Set the query filter data
		for (PxU32 i = 0; i < numWheels; i++)
		{
			wheelsSimData->setWheelData(i, wheels[i]);
			wheelsSimData->setTireData(i, tires[i]);
			wheelsSimData->setSuspensionData(i, suspensions[i]);
			wheelsSimData->setSuspTravelDirection(i, suspTravelDirections[i]);
			wheelsSimData->setWheelCentreOffset(i, wheelCentreCMOffsets[i]);
			wheelsSimData->setSuspForceAppPointOffset(i, suspForceAppCMOffsets[i]);
			wheelsSimData->setTireForceAppPointOffset(i, tireForceAppCMOffsets[i]);
			wheelsSimData->setSceneQueryFilterData(i, qryFilterData);
			wheelsSimData->setWheelShapeMapping(i, PxI32(i));
		}

		//Add a front and rear anti-roll bar
		PxVehicleAntiRollBarData barFront;
		barFront.mWheel0 = PxVehicleDrive4WWheelOrder::eFRONT_LEFT;
		barFront.mWheel1 = PxVehicleDrive4WWheelOrder::eFRONT_RIGHT;
		barFront.mStiffness = 10000.0f;
		wheelsSimData->addAntiRollBarData(barFront);
		PxVehicleAntiRollBarData barRear;
		barRear.mWheel0 = PxVehicleDrive4WWheelOrder::eREAR_LEFT;
		barRear.mWheel1 = PxVehicleDrive4WWheelOrder::eREAR_RIGHT;
		barRear.mStiffness = 10000.0f;
		wheelsSimData->addAntiRollBarData(barRear);
	}


	PxRigidStatic* createDrivablePlane(const PxFilterData& simFilterData, PxMaterial* material, PxPhysics* physics)
	{
		//Add a plane to the scene.
		PxRigidStatic* groundPlane = PxCreatePlane(*physics, PxPlane(0, 1, 0, 0), *material);

		//Get the plane shape so we can set query and simulation filter data.
		PxShape* shapes[1];
		groundPlane->getShapes(shapes, 1);

		//Set the query filter data of the ground plane so that the vehicle raycasts can hit the ground.
		PxFilterData qryFilterData;
		setupDrivableSurface(qryFilterData);
		shapes[0]->setQueryFilterData(qryFilterData);

		//Set the simulation filter data of the ground plane so that it collides with the chassis of a vehicle but not the wheels.
		shapes[0]->setSimulationFilterData(simFilterData);

		return groundPlane;
	}

	static PxConvexMesh* createConvexMesh(const PxVec3* verts, const PxU32 numVerts, PxPhysics& physics, PxCooking& cooking)
	{
		// Create descriptor for convex mesh
		PxConvexMeshDesc convexDesc;
		convexDesc.points.count = numVerts;
		convexDesc.points.stride = sizeof(PxVec3);
		convexDesc.points.data = verts;
		convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

		PxConvexMesh* convexMesh = NULL;
		PxDefaultMemoryOutputStream buf;
		if (cooking.cookConvexMesh(convexDesc, buf))
		{
			PxDefaultMemoryInputData id(buf.getData(), buf.getSize());
			convexMesh = physics.createConvexMesh(id);
		}

		return convexMesh;
	}

	PxVehicleNoDrive* createVehicleNoDrive(const VehicleDesc& vehDesc, PxPhysics* physics, PxCooking* cooking)
	{

		PxConvexMesh* chassisConvexMesh = createChassisMesh(vehDesc.chassisDims, *physics, *cooking);
		PxConvexMesh* wheelMesh = createWheelMesh(vehDesc.wheelWidth, vehDesc.wheelRadius, *physics, *cooking);
		
		PxConvexMesh* wheelConvexMeshes[PX_MAX_NB_WHEELS];
		PxMaterial* wheelMaterials[PX_MAX_NB_WHEELS];

		for (PxU32 i = PxVehicleDrive4WWheelOrder::eFRONT_LEFT; i <= PxVehicleDrive4WWheelOrder::eREAR_RIGHT; i++)
		{
			wheelConvexMeshes[i] = wheelMesh;
			wheelMaterials[i] = vehDesc.wheelMaterial;
		}

		for (PxU32 i = PxVehicleDrive4WWheelOrder::eREAR_RIGHT + 1; i < vehDesc.numWheels; i++)
		{
			wheelConvexMeshes[i] = wheelMesh;
			wheelMaterials[i] = vehDesc.wheelMaterial;
		}

		PxConvexMesh* chassisConvexMeshes[1] = { chassisConvexMesh };
		PxMaterial* chassisMaterials[1] = { vehDesc.chassisMaterial };

		PxVehicleChassisData rigidBodyData;
		rigidBodyData.mMOI = vehDesc.chassisMOI;
		rigidBodyData.mMass = vehDesc.chassisMass;
		rigidBodyData.mCMOffset = vehDesc.chassisCMOffset;

		PxRigidDynamic* veh4WActor = createVehicleActor(rigidBodyData, wheelMaterials, wheelConvexMeshes, 
			vehDesc.numWheels, vehDesc.wheelSimFilterData, chassisMaterials, chassisConvexMeshes, 1, vehDesc.chassisSimFilterData, *physics);

		PxVehicleWheelsSimData* wheelsSimData = PxVehicleWheelsSimData::allocate(vehDesc.numWheels);
		PxVec3 wheelCenterActorOffset[PX_MAX_NB_WHEELS];
		const PxF32 frontZ = vehDesc.chassisDims.z * 0.3f;
		const PxF32 rearZ = -vehDesc.chassisDims.z * 0.3f;

		computeWheelCenterActorOffsets4W(frontZ, rearZ, vehDesc.chassisDims, vehDesc.wheelWidth, vehDesc.wheelRadius, vehDesc.numWheels, wheelCenterActorOffset);

		setupWheelsSimulationData(vehDesc.wheelMass, vehDesc.wheelMOI, vehDesc.wheelRadius, vehDesc.wheelWidth,
			vehDesc.numWheels, wheelCenterActorOffset,
			vehDesc.chassisCMOffset, vehDesc.chassisMass, wheelsSimData);

		// Diff
		PxVehicleDriveSimData4W driveSimData;
		PxVehicleDifferential4WData diff;
		diff.mType = PxVehicleDifferential4WData::eDIFF_TYPE_LS_4WD;
		driveSimData.setDiffData(diff);

		// Engine
		PxVehicleEngineData engine;
		engine.mPeakTorque = 500.f;
		engine.mMaxOmega = 600.f; // approx 6000 rpm
		driveSimData.setEngineData(engine);

		// Clutch
		PxVehicleClutchData clutch;
		clutch.mStrength = 10.0f;
		driveSimData.setClutchData(clutch);

		// Ackermann steer accuracy
		PxVehicleAckermannGeometryData ackermann;
		ackermann.mAccuracy = 1.0f;
		ackermann.mAxleSeparation =
			wheelsSimData->getWheelCentreOffset(PxVehicleDrive4WWheelOrder::eFRONT_LEFT).z -
			wheelsSimData->getWheelCentreOffset(PxVehicleDrive4WWheelOrder::eREAR_LEFT).z;
		ackermann.mFrontWidth =
			wheelsSimData->getWheelCentreOffset(PxVehicleDrive4WWheelOrder::eFRONT_RIGHT).x -
			wheelsSimData->getWheelCentreOffset(PxVehicleDrive4WWheelOrder::eFRONT_LEFT).x;
		ackermann.mRearWidth =
			wheelsSimData->getWheelCentreOffset(PxVehicleDrive4WWheelOrder::eREAR_RIGHT).x -
			wheelsSimData->getWheelCentreOffset(PxVehicleDrive4WWheelOrder::eREAR_LEFT).x;
		driveSimData.setAckermannGeometryData(ackermann);

		PxVehicleNoDrive* vehNoDrive = PxVehicleNoDrive::allocate(vehDesc.numWheels);
		vehNoDrive->setup(physics, veh4WActor, *wheelsSimData);

		configureUserData(vehNoDrive, vehDesc.actorUserData, vehDesc.shapeUserDatas);

		wheelsSimData->free();
		return vehNoDrive;
	}

	PxConvexMesh* createChassisMesh(const PxVec3 dims, PxPhysics& physics, PxCooking& cooking)
	{
		const PxF32 x = dims.x * 0.5f;
		const PxF32 y = dims.y * 0.5f;
		const PxF32 z = dims.z * 0.5f;
		PxVec3 verts[8] =
		{
			PxVec3(x,y,-z),
			PxVec3(x,y,z),
			PxVec3(x,-y,z),
			PxVec3(x,-y,-z),
			PxVec3(-x,y,-z),
			PxVec3(-x,y,z),
			PxVec3(-x,-y,z),
			PxVec3(-x,-y,-z)
		};

		return createConvexMesh(verts, 8, physics, cooking);
	}

	PxConvexMesh* createWheelMesh(const PxF32 width, const PxF32 radius, PxPhysics& physics, PxCooking& cooking)
	{
		PxVec3 points[2 * 16];
		for (PxU32 i = 0; i < 16; i++)
		{
			const PxF32 cosTheta = PxCos(i * PxPi * 2.0f / 16.0f);
			const PxF32 sinTheta = PxSin(i * PxPi * 2.0f / 16.0f);
			const PxF32 y = radius * cosTheta;
			const PxF32 z = radius * sinTheta;
			points[2 * i + 0] = PxVec3(-width / 2.0f, y, z);
			points[2 * i + 1] = PxVec3(+width / 2.0f, y, z);
		}

		return createConvexMesh(points, 32, physics, cooking);
	}

	PxRigidDynamic* createVehicleActor
	(const PxVehicleChassisData& chassisData,
		PxMaterial** wheelMaterials, PxConvexMesh** wheelConvexMeshes, const PxU32 numWheels, const PxFilterData& wheelSimFilterData,
		PxMaterial** chassisMaterials, PxConvexMesh** chassisConvexMeshes, const PxU32 numChassisMeshes, const PxFilterData& chassisSimFilterData,
		PxPhysics& physics)
	{
		//We need a rigid body actor for the vehicle.
		//Don't forget to add the actor to the scene after setting up the associated vehicle.
		PxRigidDynamic* vehActor = physics.createRigidDynamic(PxTransform(PxIdentity));

		//Wheel and chassis query filter data.
		//Optional: cars don't drive on other cars.
		PxFilterData wheelQryFilterData;
		setupNonDrivableSurface(wheelQryFilterData);
		PxFilterData chassisQryFilterData;
		setupNonDrivableSurface(chassisQryFilterData);

		//Add all the wheel shapes to the actor.
		for (PxU32 i = 0; i < numWheels; i++)
		{
			PxConvexMeshGeometry geom(wheelConvexMeshes[i]);
			PxShape* wheelShape = PxRigidActorExt::createExclusiveShape(*vehActor, geom, *wheelMaterials[i]);
			wheelShape->setQueryFilterData(wheelQryFilterData);
			wheelShape->setSimulationFilterData(wheelSimFilterData);
			wheelShape->setLocalPose(PxTransform(PxIdentity));
		}

		//Add the chassis shapes to the actor.
		for (PxU32 i = 0; i < numChassisMeshes; i++)
		{
			PxShape* chassisShape = PxRigidActorExt::createExclusiveShape(*vehActor, PxConvexMeshGeometry(chassisConvexMeshes[i]), *chassisMaterials[i]);
			chassisShape->setQueryFilterData(chassisQryFilterData);
			chassisShape->setSimulationFilterData(chassisSimFilterData);
			chassisShape->setLocalPose(PxTransform(PxIdentity));
		}

		vehActor->setMass(chassisData.mMass);
		vehActor->setMassSpaceInertiaTensor(chassisData.mMOI);
		vehActor->setCMassLocalPose(PxTransform(chassisData.mCMOffset, PxQuat(PxIdentity)));

		return vehActor;
	}

	void configureUserData(PxVehicleWheels* vehicle, ActorUserData* actorUserData, ShapeUserData* shapeUserDatas)
	{
		if (actorUserData)
		{
			vehicle->getRigidDynamicActor()->userData = actorUserData;
			actorUserData->vehicle = vehicle;
		}

		if (shapeUserDatas)
		{
			PxShape* shapes[PX_MAX_NB_WHEELS + 1];
			vehicle->getRigidDynamicActor()->getShapes(shapes, PX_MAX_NB_WHEELS + 1);
			for (PxU32 i = 0; i < vehicle->mWheelsSimData.getNbWheels(); i++)
			{
				const PxI32 shapeId = vehicle->mWheelsSimData.getWheelShapeMapping(i);
				shapes[shapeId]->userData = &shapeUserDatas[i];
				shapeUserDatas[i].isWheel = true;
				shapeUserDatas[i].wheelId = i;
			}
		}
	}

	void customizeVehicleToLengthScale(const PxReal lengthScale, PxRigidDynamic* rigidDynamic, PxVehicleWheelsSimData* wheelsSimData, PxVehicleDriveSimData* driveSimData)
	{
		//Rigid body center of mass and moment of inertia.
		{
			PxTransform t = rigidDynamic->getCMassLocalPose();
			t.p *= lengthScale;
			rigidDynamic->setCMassLocalPose(t);

			PxVec3 moi = rigidDynamic->getMassSpaceInertiaTensor();
			moi *= (lengthScale * lengthScale);
			rigidDynamic->setMassSpaceInertiaTensor(moi);
		}

		//Wheels, suspensions, wheel centers, tire/susp force application points.
		{
			for (PxU32 i = 0; i < wheelsSimData->getNbWheels(); i++)
			{
				PxVehicleWheelData wheelData = wheelsSimData->getWheelData(i);
				wheelData.mRadius *= lengthScale;
				wheelData.mWidth *= lengthScale;
				wheelData.mDampingRate *= lengthScale * lengthScale;
				wheelData.mMaxBrakeTorque *= lengthScale * lengthScale;
				wheelData.mMaxHandBrakeTorque *= lengthScale * lengthScale;
				wheelData.mMOI *= lengthScale * lengthScale;
				wheelsSimData->setWheelData(i, wheelData);

				PxVehicleSuspensionData suspData = wheelsSimData->getSuspensionData(i);
				suspData.mMaxCompression *= lengthScale;
				suspData.mMaxDroop *= lengthScale;
				wheelsSimData->setSuspensionData(i, suspData);

				PxVec3 v = wheelsSimData->getWheelCentreOffset(i);
				v *= lengthScale;
				wheelsSimData->setWheelCentreOffset(i, v);

				v = wheelsSimData->getSuspForceAppPointOffset(i);
				v *= lengthScale;
				wheelsSimData->setSuspForceAppPointOffset(i, v);

				v = wheelsSimData->getTireForceAppPointOffset(i);
				v *= lengthScale;
				wheelsSimData->setTireForceAppPointOffset(i, v);
			}
		}

		//Slow forward speed correction.
		{
			wheelsSimData->setSubStepCount(5.0f * lengthScale, 3, 1);
			wheelsSimData->setMinLongSlipDenominator(4.0f * lengthScale);
		}

		//Engine
		if (driveSimData)
		{
			PxVehicleEngineData engineData = driveSimData->getEngineData();
			engineData.mMOI *= lengthScale * lengthScale;
			engineData.mPeakTorque *= lengthScale * lengthScale;
			engineData.mDampingRateFullThrottle *= lengthScale * lengthScale;
			engineData.mDampingRateZeroThrottleClutchEngaged *= lengthScale * lengthScale;
			engineData.mDampingRateZeroThrottleClutchDisengaged *= lengthScale * lengthScale;
			driveSimData->setEngineData(engineData);
		}

		//Clutch.
		if (driveSimData)
		{
			PxVehicleClutchData clutchData = driveSimData->getClutchData();
			clutchData.mStrength *= lengthScale * lengthScale;
			driveSimData->setClutchData(clutchData);
		}

		//Scale the collision meshes too.
		{
			PxShape* shapes[16];
			const PxU32 nbShapes = rigidDynamic->getShapes(shapes, 16);
			for (PxU32 i = 0; i < nbShapes; i++)
			{
				switch (shapes[i]->getGeometryType())
				{
				case PxGeometryType::eSPHERE:
				{
					PxSphereGeometry sphere;
					shapes[i]->getSphereGeometry(sphere);
					sphere.radius *= lengthScale;
					shapes[i]->setGeometry(sphere);
				}
				break;
				case PxGeometryType::ePLANE:
					PX_ASSERT(false);
					break;
				case PxGeometryType::eCAPSULE:
				{
					PxCapsuleGeometry capsule;
					shapes[i]->getCapsuleGeometry(capsule);
					capsule.radius *= lengthScale;
					capsule.halfHeight *= lengthScale;
					shapes[i]->setGeometry(capsule);
				}
				break;
				case PxGeometryType::eBOX:
				{
					PxBoxGeometry box;
					shapes[i]->getBoxGeometry(box);
					box.halfExtents *= lengthScale;
					shapes[i]->setGeometry(box);
				}
				break;
				case PxGeometryType::eCONVEXMESH:
				{
					PxConvexMeshGeometry convexMesh;
					shapes[i]->getConvexMeshGeometry(convexMesh);
					convexMesh.scale.scale *= lengthScale;
					shapes[i]->setGeometry(convexMesh);
				}
				break;
				case PxGeometryType::eTRIANGLEMESH:
				{
					PxTriangleMeshGeometry triMesh;
					shapes[i]->getTriangleMeshGeometry(triMesh);
					triMesh.scale.scale *= lengthScale;
					shapes[i]->setGeometry(triMesh);
				}
				break;
				case PxGeometryType::eHEIGHTFIELD:
				{
					PxHeightFieldGeometry hf;
					shapes[i]->getHeightFieldGeometry(hf);
					hf.columnScale *= lengthScale;
					hf.heightScale *= lengthScale;
					hf.rowScale *= lengthScale;
					shapes[i]->setGeometry(hf);
				}
				break;
				case PxGeometryType::eINVALID:
				case PxGeometryType::eGEOMETRY_COUNT:
					break;
				}
			}
		}
	}

	void customizeVehicleToLengthScale(const PxReal lengthScale, PxRigidDynamic* rigidDynamic, PxVehicleWheelsSimData* wheelsSimData, PxVehicleDriveSimData4W* driveSimData)
	{
		customizeVehicleToLengthScale(lengthScale, rigidDynamic, wheelsSimData, static_cast<PxVehicleDriveSimData*>(driveSimData));

		//Ackermann geometry.
		if (driveSimData)
		{
			PxVehicleAckermannGeometryData ackermannData = driveSimData->getAckermannGeometryData();
			ackermannData.mAxleSeparation *= lengthScale;
			ackermannData.mFrontWidth *= lengthScale;
			ackermannData.mRearWidth *= lengthScale;
			driveSimData->setAckermannGeometryData(ackermannData);
		}
	}

} // namespace snippetvehicle