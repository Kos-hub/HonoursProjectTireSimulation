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

std::string allWheels;
std::vector<std::string> csvLinesLong;
std::vector<std::string> csvLinesLat;
std::vector<std::string> csvLinesAlg;
int numStepCount = 0;

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
PxVehicleNoDrive* gVehicleNoDrive = NULL;

shared_ptr<MagicFormulaTireModel> magicFormulaTireModel;
shared_ptr<DefaultTireModel> defaultTireModel;
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
	//eDRIVE_MODE_ACCEL_REVERSE,
	//eDRIVE_MODE_BRAKE,
	//eDRIVE_MODE_HARD_TURN_LEFT,
	//eDRIVE_MODE_BRAKE,
	//eDRIVE_MODE_HARD_TURN_RIGHT,
	//eDRIVE_MODE_ACCEL_FORWARDS,
	//eDRIVE_MODE_HANDBRAKE_TURN_LEFT,
	//eDRIVE_MODE_ACCEL_FORWARDS,
	//eDRIVE_MODE_HANDBRAKE_TURN_RIGHT,
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


// MAGIC FORMULA CONSTANTS
//const float b0 = 1.50018802672136;
//const float b1 = -15.7761466722458;
//const float b2 = 1022.11238546683;
//const float b3 = -2.55317715303733;
//const float b4 = 208.777316195246;
//const float b5 = 0.0073134908964823;
//const float b6 = -0.00376410345674027;
//const float b7 = 0.156330736057758;
//const float b8 = -1.15310023217878;
//const float b9 = 0.0;
//const float b10 = 0.0;
//
//const float a0 = 1.49975356208205;
//const float a1 = -4.84987524731462;
//const float a2 = 812.449795340733;
//const float a3 = 2613.92367840654;
//const float a4 = 48.857910109076;
//const float a5 = 0.0;
//const float a6 = -0.00879541881020228;
//const float a7 = 0.376999015041155;
//const float a8 = 0.0f;
//const float a9 = 0.0f;
//const float a10 = 0.0f;
//const float a11 = 0.0f;
//const float a12 = 0.0f;
//const float a13 = 0.0f;
//
//const float c0 = 2.34000;
//const float c1 = 0.990427;
//const float c2 = 2.9684;
//const float c3 = -0.277098;
//const float c4 = -0.944859;
//const float c5 = 0.0;
//const float c6 = 0.0027699;
//const float c7 = -0.0001151;
//const float c8 = 0.10;
//const float c9 = -1.3329;
//const float c10 = 0.0f;
//const float c11 =  0.0f;
//const float c12 = 0.0f;
//const float c13 =  0.0f;
//const float c14 = 0.0f;
//const float c15 = 0.0f;
//const float c16 =  0.0f;
//const float c17 =  0.0f;

const float b0 = 1.5;
const float b1 = 0.0;
const float b2 = 1100;
const float b3 = 0;
const float b4 = 300;
const float b5 = 0;
const float b6 = 0;
const float b7 = 0;
const float b8 = -2;
const float b9 = 0.0;
const float b10 = 0.0;

const float a0 = 1.4;
const float a1 = 0;
const float a2 = 1100;
const float a3 = 1100;
const float a4 = 10;
const float a5 = 0.0;
const float a6 = 0;
const float a7 = -2;
const float a8 = 0.0f;
const float a9 = 0.0f;
const float a10 = 0.0f;
const float a11 = 0.0f;
const float a12 = 0.0f;
const float a13 = 0.0f;

const float c0 = 2.34000;
const float c1 = 0.990427;
const float c2 = 2.9684;
const float c3 = -0.277098;
const float c4 = -0.944859;
const float c5 = 0.0;
const float c6 = 0.0027699;
const float c7 = -0.0001151;
const float c8 = 0.10;
const float c9 = -1.3329;
const float c10 = 0.0f;
const float c11 = 0.0f;
const float c12 = 0.0f;
const float c13 = 0.0f;
const float c14 = 0.0f;
const float c15 = 0.0f;
const float c16 = 0.0f;
const float c17 = 0.0f;

//const float b0 = 1.50018802672136;
//const float b1 =  0.0f;
//const float b2 = 1100.f;
//const float b3 =  0.0f;
//const float b4 = 300.f;
//const float b5 = 0.0f;
//const float b6 =  0.0f;
//const float b7 = 0.0f;
//const float b8 = -2;
//const float b9 = 0.0;
//const float b10 = 0.0;
//
//const float a0 = 1.49975356208205;
//const float a1 = 0.f;
//const float a2 = 1100;
//const float a3 = 1100;
//const float a4 = 10;
//const float a5 = 0.0;
//const float a6 = 0.f;
//const float a7 = -2;
//const float a8 = 0.0f;
//const float a9 = 0.0f;
//const float a10 = 0.0f;
//const float a11 = 0.0f;
//const float a12 = 0.0f;
//const float a13 = 0.0f;
//
//const float c0 = 2.34000;
//const float c1 = 0.990427;
//const float c2 = 2.9684;
//const float c3 = -0.277098;
//const float c4 = -0.944859;
//const float c5 = 0.0;
//const float c6 = 0.0027699;
//const float c7 = -0.0001151;
//const float c8 = 0.10;
//const float c9 = -1.3329;
//const float c10 = 0.0f;
//const float c11 = 0.0f;
//const float c12 = 0.0f;
//const float c13 = 0.0f;
//const float c14 = 0.0f;
//const float c15 = 0.0f;
//const float c16 = 0.0f;
//const float c17 = 0.0f;

//const float B = 10.0f;     // Longitudinal stiffness factor
//const float C = 1.65f;      // Longitudinal shape factor
//const float D = 1.0f;      // Longitudinal peak factor
//const float E = 0.97f;     // Lateral stiffness factor
//const float S_HY = 0.0f;   // Horizontal shift factor
//const float S_VY = 0.0f;   // Vertical shift factor
//const float S_VY2 = 0.0f;  // Vertical shift factor squared

int numWheel = 0;
//
//float gMinimumSlipThreshold = 1e-5f;
//
//#define ONE_TWENTYSEVENTH 0.037037f
//#define ONE_THIRD 0.33333f
//
//PX_FORCE_INLINE PxF32 smoothingFunction1(const PxF32 K)
//{
//	//Equation 20 in CarSimEd manual Appendix F.
//	//Looks a bit like a curve of sqrt(x) for 0<x<1 but reaching 1.0 on y-axis at K=3. 
//	PX_ASSERT(K >= 0.0f);
//	return PxMin(1.0f, K - ONE_THIRD * K * K + ONE_TWENTYSEVENTH * K * K * K);
//}
//PX_FORCE_INLINE PxF32 smoothingFunction2(const PxF32 K)
//{
//	//Equation 21 in CarSimEd manual Appendix F.
//	//Rises to a peak at K=0.75 and falls back to zero by K=3
//	PX_ASSERT(K >= 0.0f);
//	return (K - K * K + ONE_THIRD * K * K * K - ONE_TWENTYSEVENTH * K * K * K * K);
//}
//
//void TireForceDefault
//(const void* tireShaderData,
//	const PxF32 tireFriction,
//	const PxF32 longSlipUnClamped, const PxF32 latSlipUnClamped, const PxF32 camberUnclamped,
//	const PxF32 wheelOmega, const PxF32 wheelRadius, const PxF32 recipWheelRadius,
//	const PxF32 restTireLoad, const PxF32 normalisedTireLoad, const PxF32 tireLoad,
//	const PxF32 gravity, const PxF32 recipGravity,
//	PxF32& wheelTorque, PxF32& tireLongForceMag, PxF32& tireLatForceMag, PxF32& tireAlignMoment)
//{
//	
//	PX_UNUSED(wheelOmega);
//	PX_UNUSED(recipWheelRadius);
//
//	const PxVehicleTireData& tireData = *reinterpret_cast<const PxVehicleTireData*>(tireShaderData);
//
//	PX_ASSERT(tireFriction > 0);
//	PX_ASSERT(tireLoad > 0);
//
//	wheelTorque = 0.0f;
//	tireLongForceMag = 0.0f;
//	tireLatForceMag = 0.0f;
//	tireAlignMoment = 0.0f;
//
//	//Clamp the slips to a minimum value.
//	const PxF32 latSlip = PxAbs(latSlipUnClamped) >= gMinimumSlipThreshold ? latSlipUnClamped : 0.0f;
//	const PxF32 longSlip = PxAbs(longSlipUnClamped) >= gMinimumSlipThreshold ? longSlipUnClamped : 0.0f;
//	const PxF32 camber = PxAbs(camberUnclamped) >= gMinimumSlipThreshold ? camberUnclamped : 0.0f;
//
//
//	csvLinesLong[numWheel].append(std::to_string(longSlip) + ",");
//	csvLinesLat[numWheel].append(std::to_string(latSlip * (180.f / PxPi)) + ",");
//	csvLinesAlg[numWheel].append(std::to_string(latSlip * (180.f / PxPi)) + ",");
//
//	//If long slip/lat slip/camber are all zero than there will be zero tire force.
//	if ((0 == latSlip) && (0 == longSlip) && (0 == camber))
//	{
//		return;
//	}
//
//	//Compute the lateral stiffness
//	const PxF32 latStiff = restTireLoad * tireData.mLatStiffY * smoothingFunction1(normalisedTireLoad * 3.0f / tireData.mLatStiffX);
//
//	//Get the longitudinal stiffness
//	const PxF32 longStiff = tireData.mLongitudinalStiffnessPerUnitGravity * gravity;
//	const PxF32 recipLongStiff = tireData.getRecipLongitudinalStiffnessPerUnitGravity() * recipGravity;
//
//	//Get the camber stiffness.
//	const PxF32 camberStiff = tireData.mCamberStiffnessPerUnitGravity * gravity;
//
//	//Carry on and compute the forces.
//	const PxF32 TEff = PxTan(latSlip - camber * camberStiff / latStiff);
//	const PxF32 K = PxSqrt(latStiff * TEff * latStiff * TEff + longStiff * longSlip * longStiff * longSlip) / (tireFriction * tireLoad);
//	//const PxF32 KAbs=PxAbs(K);
//	PxF32 FBar = smoothingFunction1(K);//K - ONE_THIRD*PxAbs(K)*K + ONE_TWENTYSEVENTH*K*K*K;
//	PxF32 MBar = smoothingFunction2(K); //K - KAbs*K + ONE_THIRD*K*K*K - ONE_TWENTYSEVENTH*KAbs*K*K*K;
//	//Mbar = PxMin(Mbar, 1.0f);
//	PxF32 nu = 1;
//	if (K <= 2.0f * PxPi)
//	{
//		const PxF32 latOverlLong = latStiff * recipLongStiff;
//		nu = 0.5f * (1.0f + latOverlLong - (1.0f - latOverlLong) * PxCos(K * 0.5f));
//	}
//	const PxF32 FZero = tireFriction * tireLoad / (PxSqrt(longSlip * longSlip + nu * TEff * nu * TEff));
//	const PxF32 fz = longSlip * FBar * FZero;
//	const PxF32 fx = -nu * TEff * FBar * FZero;
//	//TODO: pneumatic trail.
//	const PxF32 pneumaticTrail = 1.0f;
//	const PxF32	fMy = nu * pneumaticTrail * TEff * MBar * FZero;
//
//
//	//We can add the torque to the wheel.
//	wheelTorque = -fz * wheelRadius;
//	tireLongForceMag = fz;
//	tireLatForceMag = fx;
//	tireAlignMoment = fMy;
//
//	csvLinesLong[numWheel].append(std::to_string(tireLongForceMag) + "\n");
//	csvLinesLat[numWheel].append(std::to_string(tireLatForceMag) + "\n");
//	csvLinesAlg[numWheel].append(std::to_string(tireAlignMoment) + "\n");
//
//	std::cout << numWheel << std::endl;
//	numWheel++;
//	numStepCount++;
//	if (numWheel > 3)
//		numWheel = 0;
//
//	//if (numWheel > 4)
//	//	std::cout << "STOP" << std::endl;
//
//
//	
//}
//
//void tireModelMagicFormula(const void* shaderData,
//	const PxF32 tireFriction,
//	const PxF32 longSlip, const PxF32 latSlip, const PxF32 camber,
//	const PxF32 wheelOmega, const PxF32 wheelRadius, const PxF32 recipWheelRadius,
//	const PxF32 restTireLoad, const PxF32 normalisedTireLoad, const PxF32 tireLoad,
//	const PxF32 gravity, const PxF32 recipGravity,
//	PxF32& wheelTorque, PxF32& tireLongForceMag, PxF32& tireLatForceMag, PxF32& tireAlignMoment)
//{
//
//	if ((0 == latSlip) && (0 == longSlip) && (0 == camber))
//	{
//		return;
//	}
//	// CLAMPING SLIPS
//	/*float clampLongSlip = PxClamp(longSlip, -1.f, 1.f);
//	float clampLatSlip = PxClamp(latSlip, (float)((-PxPi / 2.f) + 0.001), (float)((-PxPi / 2.f) - 0.001));*/
//
//	//std::cout << "Wheel " << numWheel << " Long Slip: " << longSlip << std::endl;
//	//std::cout << "Wheel " << numWheel << " Lat Slip: " << latSlip << std::endl;
//	csvLinesLong[numWheel].append(std::to_string(longSlip) + ",");
//
//
//
//	//printf("Calculating Tire\n");
//
//
//	// ADJUSTING VALUES
//	float Fz = tireLoad / 1000.f;
//	float longSlipP = longSlip * 100.f;
//	float camberDeg = 90.f - std::acos(camber) * (180.f / PxPi);
//	//float camberDeg = camber * (180.f / PxPi);
//	float latSlipDeg = -latSlip * (180.f / PxPi);
//
//	// Getting the lateral slip in degrees, not in radians
//	csvLinesLat[numWheel].append(std::to_string(latSlipDeg) + ",");
//	csvLinesAlg[numWheel].append(std::to_string(latSlipDeg) + ",");
//
//	// LONG FORCE CALC
//	float C = b0;
//
//
//
//	float D = (b1 * Fz * Fz + b2 * Fz);
//
//
//	float BCD = (b3 * Fz * Fz + b4 * Fz) * std::exp(-b5 * Fz);
//	float B = BCD / (C * D);
//
//	float H = b9 * Fz + b10;
//	float V = 0.0f;
//	float x1 = longSlipP + H;
//	float E = (b6 * Fz * Fz + b7 * Fz + b8);
//
//	float Fx = tireFriction * (D * std::sin(C * std::atan(B * x1 - E * (B * x1 - std::atan(B * x1))))) + V;
//
//
//	// LAT FORCE CALC
//	C = a0;
//	D = (a1 * Fz * Fz + a2 * Fz);
//	BCD = a3 * std::sin(std::atan(Fz / a4) * 2.0) * (1.0 - a5 * std::abs(camberDeg));
//	B = BCD / (C * D);
//	H = a9 * Fz + a10 + a8 * camberDeg;
//	V = a11 * Fz * camberDeg + a12 * Fz + a13;
//	x1 = latSlipDeg + H;
//	E = a6 * Fz + a7;
//
//	PxClamp(x1, (float)-89.5, (float)89.5);
//	float Fy = tireFriction * D * std::sin(C * std::atan(B * x1 - E * (B * x1 - std::atan(B * x1)))) + V;
//
//
//	// ALIGN MOMENT CALCULATION
//	C = c0;
//	D = c1 * Fz * Fz + c2 * Fz;
//	BCD = (c3 * Fz * Fz + c4 * Fz) * (1 - c6 * std::abs(camberDeg)) * std::exp(-c5 * Fz);
//	B = BCD / (C * D);
//	H = c11 * camberDeg + c12 * Fz + c13;
//	V = (c14 * Fz * Fz + c15 * Fz) * camberDeg + c16 * Fz + c17;
//	x1 = (latSlipDeg + H);
//	E = (c7 * Fz * Fz + c8 * Fz + c9) * (1 - c10 * std::abs(camberDeg));
//
//	float Mz = tireFriction * D * std::sin(C * std::atan(B * x1 - E * (B * x1 - std::atan(B * x1)))) + V;
//
//	//float deflection = Fy / 261065.0;
//
//	//float Mx = -(Fz * 1000) * deflection;
//	//Mz = Mz + Fx * deflection;
//
//	//float My = 0.01 * Fz * wheelRadius;
//
//	// Set the output variables
//	wheelTorque = 0.f;        // Magic Formula doesn't directly compute wheel torque
//	tireLongForceMag = Fx;
//	tireLatForceMag = Fy;
//	tireAlignMoment = Mz;
//
//
//	csvLinesLong[numWheel].append(std::to_string(tireLongForceMag) + "\n");
//	csvLinesLat[numWheel].append(std::to_string(tireLatForceMag) + "\n");
//	csvLinesAlg[numWheel].append(std::to_string(tireAlignMoment) + "\n");
//
//	
//	numWheel++;
//	numStepCount++;
//
//	if (numWheel > 3)
//		numWheel = 0;
//
//}

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
	gVehicleNoDrive->setDriveTorque(0, 1000.0f);
	gVehicleNoDrive->setDriveTorque(1, 1000.0f);
}

void startAccelerateReverseMode()
{
	gVehicleNoDrive->setDriveTorque(0, -1000.0f);
	gVehicleNoDrive->setDriveTorque(1, -1000.0f);
}

void startBrakeMode()
{
	gVehicleNoDrive->setBrakeTorque(0, 1000.0f);
	gVehicleNoDrive->setBrakeTorque(1, 1000.0f);
	gVehicleNoDrive->setBrakeTorque(2, 1000.0f);
	gVehicleNoDrive->setBrakeTorque(3, 1000.0f);
}

void startTurnHardLeftMode()
{
	gVehicleNoDrive->setDriveTorque(0, 1000.0f);
	gVehicleNoDrive->setDriveTorque(1, 1000.0f);
	gVehicleNoDrive->setSteerAngle(0, 1.0f);
	gVehicleNoDrive->setSteerAngle(1, 1.0f);
}

void startTurnHardRightMode()
{
	gVehicleNoDrive->setDriveTorque(0, 1000.0f);
	gVehicleNoDrive->setDriveTorque(1, 1000.0f);
	gVehicleNoDrive->setSteerAngle(0, -1.0f);
	gVehicleNoDrive->setSteerAngle(1, -1.0f);
}

void startHandbrakeTurnLeftMode()
{
	gVehicleNoDrive->setBrakeTorque(2, 1000.0f);
	gVehicleNoDrive->setBrakeTorque(3, 1000.0f);
	gVehicleNoDrive->setDriveTorque(0, 1000.0f);
	gVehicleNoDrive->setDriveTorque(1, 1000.0f);
	gVehicleNoDrive->setSteerAngle(0, 1.0f);
	gVehicleNoDrive->setSteerAngle(1, 1.0f);
}

void startHandbrakeTurnRightMode()
{
	gVehicleNoDrive->setBrakeTorque(2, 1000.0f);
	gVehicleNoDrive->setBrakeTorque(3, 1000.0f);
	gVehicleNoDrive->setDriveTorque(0, 1000.0f);
	gVehicleNoDrive->setDriveTorque(1, 1000.0f);
	gVehicleNoDrive->setSteerAngle(0, -1.0f);
	gVehicleNoDrive->setSteerAngle(1, -1.0f);
}

void releaseAllControls()
{
	gVehicleNoDrive->setDriveTorque(0, 0.0f);
	gVehicleNoDrive->setDriveTorque(1, 0.0f);
	gVehicleNoDrive->setDriveTorque(2, 0.0f);
	gVehicleNoDrive->setDriveTorque(3, 0.0f);

	gVehicleNoDrive->setBrakeTorque(0, 0.0f);
	gVehicleNoDrive->setBrakeTorque(1, 0.0f);
	gVehicleNoDrive->setBrakeTorque(2, 0.0f);
	gVehicleNoDrive->setBrakeTorque(3, 0.0f);

	gVehicleNoDrive->setSteerAngle(0, 0.0f);
	gVehicleNoDrive->setSteerAngle(1, 0.0f);
	gVehicleNoDrive->setSteerAngle(2, 0.0f);
	gVehicleNoDrive->setSteerAngle(3, 0.0f);
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
	gVehicleNoDrive = createVehicleNoDrive(vehicleDesc, gPhysics, gCooking);
	PxTransform startTransform(PxVec3(0, (vehicleDesc.chassisDims.y * 0.5f + vehicleDesc.wheelRadius + 1.0f), 0), PxQuat(PxIdentity));
	gVehicleNoDrive->getRigidDynamicActor()->setGlobalPose(startTransform);
	gScene->addActor(*gVehicleNoDrive->getRigidDynamicActor());


	magicFormulaTireModel = make_shared<MagicFormulaTireModel>();
	defaultTireModel = make_shared<DefaultTireModel>();
	//gVehicleNoDrive->mWheelsDynData.setTireForceShaderFunction(&TireForceDefault);
	gVehicleNoDrive->mWheelsDynData.setTireForceShaderFunction(&defaultTireModel->Function);
	//gVehicleNoDrive->mWheelsDynData.setTireForceShaderFunction(&magicFormulaTireModel->Function);

	for (int i = 0; i < 4; i++)
	{
		std::string csvLine;
		csvLinesLong.push_back(csvLine);
		csvLinesLat.push_back(csvLine);
		csvLinesAlg.push_back(csvLine);
	}


	//Set the vehicle to rest in first gear.
	//Set the vehicle to use auto-gears.
	gVehicleNoDrive->setToRestState();


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

	const PxF32 timestep = 1.0f / 60.0f;

	//Cycle through the driving modes to demonstrate how to accelerate/reverse/brake/turn etc.
	incrementDrivingMode(timestep);

	//Raycasts.
	PxVehicleWheels* vehicles[1] = { gVehicleNoDrive };
	PxRaycastQueryResult* raycastResults = gVehicleSceneQueryData->getRaycastQueryResultBuffer(0);
	const PxU32 raycastResultsSize = gVehicleSceneQueryData->getQueryResultBufferSize();
	PxVehicleSuspensionRaycasts(gBatchQuery, 1, vehicles, raycastResultsSize, raycastResults);

	//Vehicle update.
	const PxVec3 grav = gScene->getGravity();
	PxWheelQueryResult wheelQueryResults[PX_MAX_NB_WHEELS];
	PxVehicleWheelQueryResult vehicleQueryResults[1] = { {wheelQueryResults, gVehicleNoDrive->mWheelsSimData.getNbWheels()} };
	PxVehicleUpdates(timestep, grav, *gFrictionPairs, 1, vehicles, vehicleQueryResults);


	//PxVehicleUpdateSingleVehicleAndStoreTelemetryData(timestep, grav, *gFrictionPairs, gVehicleNoDrive, vehicleQueryResults, *myTelemetryData);

	//PxF32 xy[2 * PxVehicleGraph::eMAX_NB_SAMPLES];
	//PxVec3 color[PxVehicleGraph::eMAX_NB_SAMPLES];
	//char title[PxVehicleGraph::eMAX_NB_TITLE_CHARS];
	//
	//for (int i = 0; i < 4; i++)
	//{
	//	myTelemetryData->getWheelGraph(i).computeGraphChannel(PxVehicleWheelGraphChannel::eTIRE_LONG_SLIP,
	//		xy, color, title);		
	//	myTelemetryData->getWheelGraph(i).computeGraphChannel(PxVehicleWheelGraphChannel::eNORM_TIRE_LONG_FORCE,
	//		xy, color, title);

	//	csvLinesLong[i].append(std::to_string(myTelemetryData->getWheelGraph(i).getLatestValue(PxVehicleWheelGraphChannel::eTIRE_LONG_SLIP)) + ",");
	//	csvLinesLong[i].append(std::to_string(myTelemetryData->getWheelGraph(i).getLatestValue(PxVehicleWheelGraphChannel::eNORM_TIRE_LONG_FORCE)) + "\n");

	//	allWheels.append(std::to_string(myTelemetryData->getWheelGraph(i).getLatestValue(PxVehicleWheelGraphChannel::eTIRE_LONG_SLIP)) + ",");
	//	allWheels.append(std::to_string(myTelemetryData->getWheelGraph(i).getLatestValue(PxVehicleWheelGraphChannel::eNORM_TIRE_LONG_FORCE)) + "\n");
	//	
	//	
	//}

	
	
	
	//Scene update
	gScene->simulate(timestep);
	gScene->fetchResults(true);

	
	numStepCount = 0;
	//printf("Finishing step\n\n");
}

void cleanupPhysics()
{

	gVehicleNoDrive->getRigidDynamicActor()->release();
	gVehicleNoDrive->free();
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
	system("Rscript C:/Users/giana/Documents/HonsProjectScript.R");
	printf("SnippetVehicleNoDrive done.\n");

	// WRITING TO CSV
	/*for (int i = 0; i < 4; i++)
	{
		std::string filenameLong = "dataWheelLong" + std::to_string(i);
		std::string filenameLat = "dataWheelLat" + std::to_string(i);
		std::string filenameAlg = "dataWheelAlg" + std::to_string(i);
		filenameLong.append(".csv");
		filenameLat.append(".csv");
		filenameAlg.append(".csv");

		std::ofstream outputFileLong(filenameLong, std::ofstream::out);
		std::ofstream outputFileLat(filenameLat, std::ofstream::out);
		std::ofstream outputFileAlg(filenameAlg, std::ofstream::out);


		if (!outputFileLong.is_open())
		{
			std::cerr << "Error file Long" << std::endl;
		}

		if (!outputFileLat.is_open())
		{
			std::cerr << "Error file Lat" << std::endl;
		}
		if (!outputFileAlg.is_open())
		{
			std::cerr << "Error file Alg" << std::endl;
		}
		outputFileLong << csvLinesLong[i];
		outputFileLat << csvLinesLat[i];
		outputFileAlg << csvLinesAlg[i];

		outputFileLong.close();
		outputFileLat.close();
		outputFileAlg.close();
	}*/

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
