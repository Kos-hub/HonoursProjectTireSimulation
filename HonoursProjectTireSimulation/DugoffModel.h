#pragma once
#include "PxPhysicsAPI.h"
#include "CsvHelper.h"
#include <cmath>

using namespace physx;

class DugoffTireModel
{
private:
	float gMinimumSlipThreshold = 1e-5f;
public:
	DugoffTireModel();

	static CsvHelper* _csvHelper;

	typedef void (*PxVehicleComputeTireForce)
		(const void* shaderData,
			const PxF32 tireFriction,
			const PxF32 longSlip, const PxF32 latSlip, const PxF32 camber,
			const PxF32 wheelOmega, const PxF32 wheelRadius, const PxF32 recipWheelRadius,
			const PxF32 restTireLoad, const PxF32 normalisedTireLoad, const PxF32 tireLoad,
			const PxF32 gravity, const PxF32 recipGravity,
			PxF32& wheelTorque, PxF32& tireLongForceMag, PxF32& tireLatForceMag, PxF32& tireAlignMoment);

	static void Function(const void* shaderData,
		const PxF32 tireFriction,
		const PxF32 longSlip, const PxF32 latSlip, const PxF32 camber,
		const PxF32 wheelOmega, const PxF32 wheelRadius, const PxF32 recipWheelRadius,
		const PxF32 restTireLoad, const PxF32 normalisedTireLoad, const PxF32 tireLoad,
		const PxF32 gravity, const PxF32 recipGravity,
		PxF32& wheelTorque, PxF32& tireLongForceMag, PxF32& tireLatForceMag, PxF32& tireAlignMoment);

	~DugoffTireModel();
};



