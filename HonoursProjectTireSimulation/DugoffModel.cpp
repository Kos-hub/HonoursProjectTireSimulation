#include "DugoffModel.h"


static int _numWheel;

CsvHelper* DugoffTireModel::_csvHelper = new CsvHelper("Dugoff");


DugoffTireModel::DugoffTireModel()
{
}

void DugoffTireModel::Function(const void* shaderData, const PxF32 tireFriction, const PxF32 longSlipUnclamped, const PxF32 latSlip, const PxF32 camber, const PxF32 wheelOmega, const PxF32 wheelRadius, const PxF32 recipWheelRadius, const PxF32 restTireLoad, const PxF32 normalisedTireLoad, const PxF32 tireLoad, const PxF32 gravity, const PxF32 recipGravity, PxF32& wheelTorque, PxF32& tireLongForceMag, PxF32& tireLatForceMag, PxF32& tireAlignMoment)
{
	// Casting tire data to get the stiffness values
	const PxVehicleTireData& tireData = *reinterpret_cast<const PxVehicleTireData*>(shaderData);

	// Calculating stiffness values
	const PxF32 longStiff = tireData.mLongitudinalStiffnessPerUnitGravity * gravity;

	PxF32 latStiff;
	if (tireLoad/restTireLoad >= tireData.mLatStiffX)
	{
		latStiff = tireData.mLatStiffY * restTireLoad;
	}
	else
	{
		latStiff = tireData.mLatStiffY * (tireLoad / tireData.mLatStiffX);
	}
	const PxF32 camberStiff = tireData.mCamberStiffnessPerUnitGravity * gravity;


	PxF32 longSlip = PxClamp(longSlipUnclamped, - 1.0f, 1.f - std::numeric_limits<float>::epsilon());
	//PxF32 longSlip = std::max(-1.f, std::min(longSlipUnclamped, -(1.0f - std::numeric_limits<float>::min())));
	//std::cout << std::numeric_limits<float>::epsilon() << std::endl;

	if ((0 == latSlip) && (0 == longSlip))
	{
		return;
	}
	
	//float latSlipDeg = latSlip * (180.f / PxPi);
	//float fz = tireLoad / 1000.f;
	//float longSlipP = longSlip * 100.f;

	float zNum = (1 - longSlip) * tireFriction * tireLoad;
	float root = (std::pow(longStiff * longSlip, 2)) + (std::pow(latStiff * std::tan(-latSlip), 2));
	float zDen = 2 * std::sqrt(root);
	float z = zNum / zDen;

	float gx = (1.15f - (0.75f * tireFriction)) * std::pow(longSlip, 2) - (1.63f - (0.75 * tireFriction)) * longSlip + 1.27f;
	float gy = (tireFriction - 1.6) * std::tan(-latSlip) + 1.155f;

	float Kz = 0;
	if (z < 1)
	{
		Kz = (2 - z) * z;
	}
	else
	{
		Kz = 1;
	}

	float fx;
	float fy;



	fx = longStiff * ((longSlip) / (1 - longSlip)) * Kz ;
	fy = latStiff * ((std::tan(-latSlip) / (1 - longSlip))) * Kz;

	//std::cout << tireFriction << std::endl;
	//if (longSlip > 0)
	//{
	//	std::cout << "Test" << std::endl;
	//}
	//float tLongSlip = longSlip / (1 + longSlip);
	//float tLatSlip = std::tan(-latSlip) / (longSlip + 1);

	//float root = std::pow(longStiff * tLongSlip, 2) + std::pow(latStiff * std::tan(tLatSlip), 2);
	//float lambda = (tireFriction * tireLoad * (1 + longSlip)) / (2 * std::sqrt(root));

	//if (lambda < 1)
	//{
	//	lambda = lambda * (2 - lambda);
	//}
	//else
	//{
	//	lambda = 1;
	//}

	/*float fx = longStiff * tLongSlip * lambda;
	float fy = latStiff * tLatSlip * lambda;*/

	//float fx = longStiff * longSlip * lambda;
	//float fy = latStiff * -latSlip * lambda;

	//float muused = std::sqrt((fx * fx) + (fy * fy)) / tireLoad;
	//float murest = tireFriction * (1 - (tireFriction / 4 * muused));




	tireLongForceMag = fx;
	tireLatForceMag = fy;
	tireAlignMoment = 0.f;
	wheelTorque = 0.f;

	_csvHelper->StoreValues(_numWheel, std::to_string(longSlip), std::to_string(latSlip), std::to_string(tireLongForceMag), std::to_string(tireLatForceMag), std::to_string(tireAlignMoment));
	_numWheel++;

	if (_numWheel > 3)
		_numWheel = 0;

}

DugoffTireModel::~DugoffTireModel()
{
	_csvHelper->WriteCSV();
	delete _csvHelper;
}
