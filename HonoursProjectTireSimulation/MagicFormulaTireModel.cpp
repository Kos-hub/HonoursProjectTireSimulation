#include "MagicFormulaTireModel.h"
#include <iostream>

static MagicFormulaConstants _constants;
static int _numWheel;
CsvHelper* MagicFormulaTireModel::_csvHelper = new CsvHelper("MagicFormula");



std::string relation;
MagicFormulaTireModel::MagicFormulaTireModel()
{
	_constants.b0 = 1.50018802672136;
	_constants.b1 = 0.0f;
	_constants.b2 = 1100.f;
	_constants.b3 = 0.0f;
	_constants.b4 = 300.0f;
	_constants.b5 = 0.0f;
	_constants.b6 = 0.0f;
	_constants.b7 = 0.0f;
	_constants.b8 = -2.f;
	_constants.b9 = 0.0f;
	_constants.b10 = 0.0f;

	_constants.a0 = 1.49975356208205;
	_constants.a1 = 0.f;
	_constants.a2 = 1100.f;
	_constants.a3 = 1100.f;
	_constants.a4 = 10.f;
	_constants.a5 = 0.0f;
	_constants.a6 = 0.0f;
	_constants.a7 = -2.f;
	_constants.a8 = 0.0f;
	_constants.a9 = 0.0f;
	_constants.a10 = 0.0f;
	_constants.a11 = 0.0f;
	_constants.a12 = 0.0f;
	_constants.a13 = 0.0f;

	_constants.c0 = 2.34000;
	_constants.c1 = 0.990427;
	_constants.c2 = 2.9684;
	_constants.c3 = -0.277098;
	_constants.c4 = -0.944859;
	_constants.c5 = 0.0;
	_constants.c6 = 0.0027699;
	_constants.c7 = -0.0001151;
	_constants.c8 = 0.10;
	_constants.c9 = -1.3329;
	_constants.c10 = 0.0f;
	_constants.c11 = 0.0f;
	_constants.c12 = 0.0f;
	_constants.c13 = 0.0f;
	_constants.c14 = 0.0f;
	_constants.c15 = 0.0f;
	_constants.c16 = 0.0f;
	_constants.c17 = 0.0f;
}

void MagicFormulaTireModel::Function(const void* shaderData, const PxF32 tireFriction, const PxF32 longSlip, const PxF32 latSlip, const PxF32 camber, const PxF32 wheelOmega, const PxF32 wheelRadius, const PxF32 recipWheelRadius, const PxF32 restTireLoad, const PxF32 normalisedTireLoad, const PxF32 tireLoad, const PxF32 gravity, const PxF32 recipGravity, PxF32& wheelTorque, PxF32& tireLongForceMag, PxF32& tireLatForceMag, PxF32& tireAlignMoment)
{

	if ((0 == latSlip) && (0 == longSlip) && (0 == camber))
	{
		return;
	}
	// CLAMPING SLIPS
	/*float clampLongSlip = PxClamp(longSlip, -1.f, 1.f);
	float clampLatSlip = PxClamp(latSlip, (float)((-PxPi / 2.f) + 0.001), (float)((-PxPi / 2.f) - 0.001));*/

	//std::cout << "Wheel " << numWheel << " Long Slip: " << longSlip << std::endl;
	//std::cout << "Wheel " << numWheel << " Lat Slip: " << latSlip << std::endl;
	//csvLinesLong[numWheel].append(std::to_string(longSlip) + ",");



	//printf("Calculating Tire\n");


	// ADJUSTING VALUES
	float Fz = tireLoad / 1000.f;
	float longSlipP = longSlip * 100.f;
	float camberDeg = 90.f - std::acos(camber) * (180.f / PxPi);
	//float camberDeg = camber * (180.f / PxPi);
	float latSlipDeg = -latSlip * (180.f / PxPi);

	// Getting the lateral slip in degrees, not in radians
	//csvLinesLat[numWheel].append(std::to_string(latSlipDeg) + ",");
	//csvLinesAlg[numWheel].append(std::to_string(latSlipDeg) + ",");

	// LONG FORCE CALC
	float C = _constants.b0;



	float D = (_constants.b1 * Fz * Fz + _constants.b2 * Fz);


	float BCD = (_constants.b3 * Fz * Fz + _constants.b4 * Fz) * std::exp(-_constants.b5 * Fz);
	float B = BCD / (C * D);

	float H = _constants.b9 * Fz + _constants.b10;
	float V = 0.0f;
	float x1 = longSlipP + H;
	float E = (_constants.b6 * Fz * Fz + _constants.b7 * Fz + _constants.b8);

	float Fx = tireFriction * (D * std::sin(C * std::atan(B * x1 - E * (B * x1 - std::atan(B * x1))))) + V;


	// LAT FORCE CALC
	C = _constants.a0;
	D = (_constants.a1 * Fz * Fz + _constants.a2 * Fz);
	BCD = _constants.a3 * std::sin(std::atan(Fz / _constants.a4) * 2.0) * (1.0 - _constants.a5 * std::abs(camberDeg));
	B = BCD / (C * D);
	H = _constants.a9 * Fz + _constants.a10 + _constants.a8 * camberDeg;
	V = _constants.a11 * Fz * camberDeg + _constants.a12 * Fz + _constants.a13;
	x1 = latSlipDeg + H;
	E = _constants.a6 * Fz + _constants.a7;

	PxClamp(x1, (float)-89.5, (float)89.5);
	float Fy = tireFriction * D * std::sin(C * std::atan(B * x1 - E * (B * x1 - std::atan(B * x1)))) + V;


	// ALIGN MOMENT CALCULATION
	C = _constants.c0;
	D = _constants.c1 * Fz * Fz + _constants.c2 * Fz;
	BCD = (_constants.c3 * Fz * Fz + _constants.c4 * Fz) * (1 - _constants.c6 * std::abs(camberDeg)) * std::exp(-_constants.c5 * Fz);
	B = BCD / (C * D);
	H = _constants.c11 * camberDeg + _constants.c12 * Fz + _constants.c13;
	V = (_constants.c14 * Fz * Fz + _constants.c15 * Fz) * camberDeg + _constants.c16 * Fz + _constants.c17;
	x1 = (latSlipDeg + H);
	E = (_constants.c7 * Fz * Fz + _constants.c8 * Fz + _constants.c9) * (1 - _constants.c10 * std::abs(camberDeg));

	float Mz = tireFriction * D * std::sin(C * std::atan(B * x1 - E * (B * x1 - std::atan(B * x1)))) + V;

	//float deflection = Fy / 261065.0;

	//float Mx = -(Fz * 1000) * deflection;
	//Mz = Mz + Fx * deflection;

	//float My = 0.01 * Fz * wheelRadius;

	// Set the output variables
	wheelTorque = 0.f;        // Magic Formula doesn't directly compute wheel torque
	tireLongForceMag = Fx;
	tireLatForceMag = Fy;
	tireAlignMoment = Mz;


	//if (longSlip > -0.97 && _numWheel == 2)
	//{
	//	std::cout << "Long force is : " << tireLongForceMag << " when slip > -0.97 and load = " << tireLoad << std::endl;
	//}

	//csvLinesLong[numWheel].append(std::to_string(tireLongForceMag) + "\n");
	//csvLinesLat[numWheel].append(std::to_string(tireLatForceMag) + "\n");
	//csvLinesAlg[numWheel].append(std::to_string(tireAlignMoment) + "\n");

	_csvHelper->StoreValues(_numWheel, std::to_string(longSlip), std::to_string(latSlipDeg), std::to_string(tireLongForceMag), std::to_string(tireLatForceMag), std::to_string(tireAlignMoment));
	_numWheel++;

	if (_numWheel > 3)
		_numWheel = 0;

}

MagicFormulaTireModel::~MagicFormulaTireModel()
{
	_csvHelper->WriteCSV();
	delete _csvHelper;
}
