#pragma once

#include "PxPhysicsAPI.h"

namespace snippetvehicle
{

	using namespace physx;

	//Drivable surface types.
	enum
	{
		SURFACE_TYPE_TARMAC,
		MAX_NUM_SURFACE_TYPES
	};

	//Tire types.
	enum
	{
		TIRE_TYPE_NORMAL = 0,
		TIRE_TYPE_WORN,
		MAX_NUM_TIRE_TYPES
	};

	PxVehicleDrivableSurfaceToTireFrictionPairs* createFrictionPairs(const PxMaterial* defaultMaterial);

} // namespace snippetvehicle

