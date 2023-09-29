#include <new>
#include "VehicleTireFriction.h"
#include "PxPhysicsAPI.h"

namespace snippetvehicle
{

	using namespace physx;

	//Tire model friction for each combination of drivable surface type and tire type.
	static PxF32 gTireFrictionMultipliers[MAX_NUM_SURFACE_TYPES][MAX_NUM_TIRE_TYPES] =
	{
		//NORMAL,	WORN
		{1.00f,		0.1f}//TARMAC
	};

	PxVehicleDrivableSurfaceToTireFrictionPairs* createFrictionPairs(const PxMaterial* defaultMaterial)
	{
		PxVehicleDrivableSurfaceType surfaceTypes[1];
		surfaceTypes[0].mType = SURFACE_TYPE_TARMAC;

		const PxMaterial* surfaceMaterials[1];
		surfaceMaterials[0] = defaultMaterial;

		PxVehicleDrivableSurfaceToTireFrictionPairs* surfaceTirePairs =
			PxVehicleDrivableSurfaceToTireFrictionPairs::allocate(MAX_NUM_TIRE_TYPES, MAX_NUM_SURFACE_TYPES);

		surfaceTirePairs->setup(MAX_NUM_TIRE_TYPES, MAX_NUM_SURFACE_TYPES, surfaceMaterials, surfaceTypes);

		for (PxU32 i = 0; i < MAX_NUM_SURFACE_TYPES; i++)
		{
			for (PxU32 j = 0; j < MAX_NUM_TIRE_TYPES; j++)
			{
				surfaceTirePairs->setTypePairFriction(i, j, gTireFrictionMultipliers[i][j]);
			}
		}
		return surfaceTirePairs;
	}

} // namespace snippetvehicle