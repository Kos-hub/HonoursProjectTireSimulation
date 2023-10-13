#include "MagicFormulaTireModel.h"
#include "DefaultTireModel.h"

#define ONE_TWENTYSEVENTH 0.037037f
#define ONE_THIRD 0.33333f

static int _numWheel;
float gMinimumSlipThreshold = 1e-5f;
CsvHelper* DefaultTireModel::_csvHelper = new CsvHelper("DefaultModel");


PX_FORCE_INLINE PxF32 smoothingFunction1(const PxF32 K)
{
	//Equation 20 in CarSimEd manual Appendix F.
	//Looks a bit like a curve of sqrt(x) for 0<x<1 but reaching 1.0 on y-axis at K=3. 
	PX_ASSERT(K >= 0.0f);
	return PxMin(1.0f, K - ONE_THIRD * K * K + ONE_TWENTYSEVENTH * K * K * K);
}
PX_FORCE_INLINE PxF32 smoothingFunction2(const PxF32 K)
{
	//Equation 21 in CarSimEd manual Appendix F.
	//Rises to a peak at K=0.75 and falls back to zero by K=3
	PX_ASSERT(K >= 0.0f);
	return (K - K * K + ONE_THIRD * K * K * K - ONE_TWENTYSEVENTH * K * K * K * K);
}

DefaultTireModel::DefaultTireModel()
{
}

void DefaultTireModel::Function(const void* shaderData, const PxF32 tireFriction, const PxF32 longSlipUnclamped, const PxF32 latSlipUnclamped, const PxF32 camberUnclamped, const PxF32 wheelOmega, const PxF32 wheelRadius, const PxF32 recipWheelRadius, const PxF32 restTireLoad, const PxF32 normalisedTireLoad, const PxF32 tireLoad, const PxF32 gravity, const PxF32 recipGravity, PxF32& wheelTorque, PxF32& tireLongForceMag, PxF32& tireLatForceMag, PxF32& tireAlignMoment)
{
	PX_UNUSED(wheelOmega);
	PX_UNUSED(recipWheelRadius);

	const PxVehicleTireData& tireData = *reinterpret_cast<const PxVehicleTireData*>(shaderData);

	PX_ASSERT(tireFriction > 0);
	PX_ASSERT(tireLoad > 0);

	wheelTorque = 0.0f;
	tireLongForceMag = 0.0f;
	tireLatForceMag = 0.0f;
	tireAlignMoment = 0.0f;

	//Clamp the slips to a minimum value.
	const PxF32 latSlip = PxAbs(latSlipUnclamped) >= gMinimumSlipThreshold ? latSlipUnclamped : 0.0f;
	const PxF32 longSlip = PxAbs(longSlipUnclamped) >= gMinimumSlipThreshold ? longSlipUnclamped : 0.0f;
	const PxF32 camber = PxAbs(camberUnclamped) >= gMinimumSlipThreshold ? camberUnclamped : 0.0f;



	//If long slip/lat slip/camber are all zero than there will be zero tire force.
	if ((0 == latSlip) && (0 == longSlip) && (0 == camber))
	{
		return;
	}

	//Compute the lateral stiffness
	const PxF32 latStiff = restTireLoad * tireData.mLatStiffY * smoothingFunction1(normalisedTireLoad * 3.0f / tireData.mLatStiffX);

	//Get the longitudinal stiffness
	const PxF32 longStiff = tireData.mLongitudinalStiffnessPerUnitGravity * gravity;
	const PxF32 recipLongStiff = tireData.getRecipLongitudinalStiffnessPerUnitGravity() * recipGravity;

	//Get the camber stiffness.
	const PxF32 camberStiff = tireData.mCamberStiffnessPerUnitGravity * gravity;

	//Carry on and compute the forces.
	const PxF32 TEff = PxTan(latSlip - camber * camberStiff / latStiff);
	const PxF32 K = PxSqrt(latStiff * TEff * latStiff * TEff + longStiff * longSlip * longStiff * longSlip) / (tireFriction * tireLoad);
	//const PxF32 KAbs=PxAbs(K);
	PxF32 FBar = smoothingFunction1(K);//K - ONE_THIRD*PxAbs(K)*K + ONE_TWENTYSEVENTH*K*K*K;
	PxF32 MBar = smoothingFunction2(K); //K - KAbs*K + ONE_THIRD*K*K*K - ONE_TWENTYSEVENTH*KAbs*K*K*K;
	//Mbar = PxMin(Mbar, 1.0f);
	PxF32 nu = 1;
	if (K <= 2.0f * PxPi)
	{
		const PxF32 latOverlLong = latStiff * recipLongStiff;
		nu = 0.5f * (1.0f + latOverlLong - (1.0f - latOverlLong) * PxCos(K * 0.5f));
	}
	const PxF32 FZero = tireFriction * tireLoad / (PxSqrt(longSlip * longSlip + nu * TEff * nu * TEff));
	const PxF32 fz = longSlip * FBar * FZero;
	const PxF32 fx = -nu * TEff * FBar * FZero;
	//TODO: pneumatic trail.
	const PxF32 pneumaticTrail = 1.0f;
	const PxF32	fMy = nu * pneumaticTrail * TEff * MBar * FZero;


	//We can add the torque to the wheel.
	wheelTorque = -fz * wheelRadius;
	tireLongForceMag = fz;
	tireLatForceMag = fx;
	tireAlignMoment = fMy;

	_csvHelper->StoreValues(_numWheel, std::to_string(longSlip), std::to_string(latSlip * (180/PxPi)), std::to_string(tireLongForceMag), std::to_string(tireLatForceMag), std::to_string(tireAlignMoment));

	_numWheel++;
	if (_numWheel > 3)
		_numWheel = 0;
}

DefaultTireModel::~DefaultTireModel()
{
	_csvHelper->WriteCSV();
	delete _csvHelper;
}
