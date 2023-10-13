#pragma once

#include "PxPhysicsAPI.h"
#include "CsvHelper.h"
#include <cmath>

using namespace physx;
struct MagicFormulaConstants
{
	float b0;
	float b1;
	float b2;
	float b3;
	float b4;
	float b5;
	float b6;
	float b7;
	float b8;
	float b9;
	float b10;

	float a0;
	float a1;
	float a2;
	float a3;
	float a4;
	float a5;
	float a6;
	float a7;
	float a8;
	float a9;
	float a10;
	float a11;
	float a12;
	float a13;

	float c0;
	float c1;
	float c2;
	float c3;
	float c4;
	float c5;
	float c6;
	float c7;
	float c8;
	float c9;
	float c10;
	float c11;
	float c12;
	float c13;
	float c14;
	float c15;
	float c16;
	float c17;
};

class MagicFormulaTireModel
{
private:
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


	//_constants.b0 = 1.5;
	//_constants.b1 = 0.0;
	//_constants.b2 = 1100;
	//_constants.b3 = 0;
	//_constants.b4 = 300;
	//_constants.b5 = 0;
	//_constants.b6 = 0;
	//_constants.b7 = 0;
	//_constants.b8 = -2;
	//_constants.b9 = 0.0;
	//_constants.b10 = 0.0;

	//_constants.a0 = 1.4;
	//_constants.a1 = 0;
	//_constants.a2 = 1100;
	//_constants.a3 = 1100;
	//_constants.a4 = 10;
	//_constants.a5 = 0.0;
	//_constants.a6 = 0;
	//_constants.a7 = -2;
	//_constants.a8 = 0.0f;
	//_constants.a9 = 0.0f;
	//_constants.a10 = 0.0f;
	//_constants.a11 = 0.0f;
	//_constants.a12 = 0.0f;
	//_constants.a13 = 0.0f;

	//_constants.c0 = 2.34000;
	//_constants.c1 = 0.990427;
	//_constants.c2 = 2.9684;
	//_constants.c3 = -0.277098;
	//_constants.c4 = -0.944859;
	//_constants.c5 = 0.0;
	//_constants.c6 = 0.0027699;
	//_constants.c7 = -0.0001151;
	//_constants.c8 = 0.10;
	//_constants.c9 = -1.3329;
	//_constants.c10 = 0.0f;
	//_constants.c11 = 0.0f;
	//_constants.c12 = 0.0f;
	//_constants.c13 = 0.0f;
	//_constants.c14 = 0.0f;
	//_constants.c15 = 0.0f;
	//_constants.c16 = 0.0f;
	//_constants.c17 = 0.0f;
	// 
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


public:
	static CsvHelper* _csvHelper;

	MagicFormulaTireModel();
	
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
		const PxF32 longSlipUnclamped, const PxF32 latSlipUnclamped, const PxF32 camberUnclamped,
		const PxF32 wheelOmega, const PxF32 wheelRadius, const PxF32 recipWheelRadius,
		const PxF32 restTireLoad, const PxF32 normalisedTireLoad, const PxF32 tireLoad,
		const PxF32 gravity, const PxF32 recipGravity,
		PxF32& wheelTorque, PxF32& tireLongForceMag, PxF32& tireLatForceMag, PxF32& tireAlignMoment);

	~MagicFormulaTireModel();
};