#include "CsvHelper.h"

CsvHelper::CsvHelper(string tireModelName)
{
	_tireModelName = tireModelName;
	for (int i = 0; i < 4; i++)
	{
		std::string csvLine;
		_csvLinesLong.push_back(csvLine);
		_csvLinesLat.push_back(csvLine);
		_csvLinesAlg.push_back(csvLine);
	}

}

void CsvHelper::WriteCSV()
{
	for (int i = 0; i < 4; i++)
	{
		std::string filenameLong = _tireModelName + "dataWheelLong" + std::to_string(i);
		std::string filenameLat = _tireModelName + "dataWheelLat" + std::to_string(i);
		std::string filenameAlg = _tireModelName + "dataWheelAlg" + std::to_string(i);
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
		outputFileLong << _csvLinesLong[i];
		outputFileLat << _csvLinesLat[i];
		outputFileAlg << _csvLinesAlg[i];

		outputFileLong.close();
		outputFileLat.close();
		outputFileAlg.close();
	}
}

void CsvHelper::StoreValues(const int numWheel, const string longSlip, const string latSlip, const string longForce, const string latForce, const string algMoment)
{
	_csvLinesLong[numWheel].append(longSlip + "," + longForce + "\n");
	_csvLinesLat[numWheel].append(latSlip + "," + latForce + "\n");
	_csvLinesAlg[numWheel].append(latSlip + "," + algMoment + "\n");
}
