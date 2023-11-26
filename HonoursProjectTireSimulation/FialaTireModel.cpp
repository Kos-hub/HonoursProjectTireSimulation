#include "FialaTireModel.h"

static int _numWheel;

CsvHelper* FialaTireModel::_csvHelper = new CsvHelper("Fiala");

FialaTireModel::FialaTireModel()
{
}

void FialaTireModel::Function(const void* shaderData, const PxF32 tireFriction, const PxF32 longSlip, const PxF32 latSlip, const PxF32 camber, const PxF32 wheelOmega, const PxF32 wheelRadius, const PxF32 recipWheelRadius, const PxF32 restTireLoad, const PxF32 normalisedTireLoad, const PxF32 ftireLoad, const PxF32 gravity, const PxF32 recipGravity, PxF32& wheelTorque, PxF32& tireLongForceMag, PxF32& tireLatForceMag, PxF32& tireAlignMoment)
{
	float tireLoad = ftireLoad;
	// Casting tire data to get the stiffness values
	const PxVehicleTireData& tireData = *reinterpret_cast<const PxVehicleTireData*>(shaderData);

	// Calculating stiffness values
	const PxF32 longStiff = tireData.mLongitudinalStiffnessPerUnitGravity * gravity;

	PxF32 latStiff;
	if (tireLoad / restTireLoad >= tireData.mLatStiffX)
	{
		latStiff = tireData.mLatStiffY * restTireLoad;
	}
	else
	{
		latStiff = tireData.mLatStiffY * (tireLoad / tireData.mLatStiffX);
	}
	const PxF32 camberStiff = tireData.mCamberStiffnessPerUnitGravity * gravity;

	if ((0 == latSlip) && (0 == longSlip))
	{
		return;
	}

	float longCrit = std::abs((tireFriction * tireLoad) / (2 * longStiff));

	float fx;
	if (std::abs(longSlip) < longCrit)
	{
		fx = longSlip * longStiff;
	}
	else
	{
		float fx1 = tireFriction * tireLoad;

		float fx2 = std::abs(std::pow(tireFriction * tireLoad, 2) / (4 * longSlip * longStiff));

		fx = PxSign(longSlip) * (fx1 - fx2);
	}

	float latCrit = std::atan((3 * tireFriction * std::abs(tireLoad)) / latStiff);

	float fy;
	float mz;


	if (std::abs(latSlip) <= latCrit)
	{
		float h = 1 - (latStiff * std::abs(std::tan(latSlip)) / (3 * tireFriction * std::abs(tireLoad)));

		fy = -tireFriction * std::abs(tireLoad) * (1 - std::pow(h, 3)) * PxSign(latSlip);
		mz = tireFriction * std::abs(tireLoad) * 0.4 * (1 - h) * std::pow(h, 3) * PxSign(latSlip);
	}
	else
	{
		fy = -tireFriction * std::abs(tireLoad) * PxSign(latSlip);
		mz = 0.f;
	}



	tireLongForceMag = fx;
	tireLatForceMag = fy;
	tireAlignMoment = mz;
	wheelTorque= 0.f;

	_csvHelper->StoreValues(_numWheel, std::to_string(longSlip), std::to_string(latSlip * 180.f / PxPi), std::to_string(tireLongForceMag), std::to_string(-tireLatForceMag), std::to_string(-tireAlignMoment));
	_numWheel++;

	if (_numWheel > 3)
		_numWheel = 0;
}

FialaTireModel::~FialaTireModel()
{
	_csvHelper->WriteCSV();
	delete _csvHelper;
}
