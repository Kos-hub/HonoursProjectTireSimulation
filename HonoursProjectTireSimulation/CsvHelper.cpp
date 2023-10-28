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

		_tempCsvLinesLong.push_back(csvLine);
		_tempCsvLinesLat.push_back(csvLine);
		_tempCsvLinesAlg.push_back(csvLine);
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

void CsvHelper::FlushTemp()
{
	for (int i = 0; i < _tempCsvLinesLong.size(); i++)
	{
		if (_tempCsvLinesLong[i].empty())
			return;
	}

	for (int i = 0; i < _tempCsvLinesLong.size(); i++)
	{
		stringstream ss(_tempCsvLinesLong[i]);
		vector<string> tokens;
		string temp;

		while (getline(ss, temp, '\n'))
		{
			tokens.push_back(temp);
		}

		_csvLinesLong[i].append(tokens[tokens.size() - 1] + "\n");
			
	}

	for (int i = 0; i < _tempCsvLinesLat.size(); i++)
	{
		stringstream ss(_tempCsvLinesLat[i]);
		vector<string> tokens;
		string temp;

		while (getline(ss, temp, '\n'))
		{
			tokens.push_back(temp);
		}

		_csvLinesLat[i].append(tokens[tokens.size() - 1] + "\n");

	}

	for (int i = 0; i < _tempCsvLinesAlg.size(); i++)
	{
		stringstream ss(_tempCsvLinesAlg[i]);
		vector<string> tokens;
		string temp;

		while (getline(ss, temp, '\n'))
		{
			tokens.push_back(temp);
		}

		_csvLinesAlg[i].append(tokens[tokens.size() - 1] + "\n");

	}

	for (int i = 0; i < 4; i++)
	{
		_tempCsvLinesLong[i].clear();
		_tempCsvLinesLat[i].clear();
		_tempCsvLinesAlg[i].clear();
	}

	

}

void CsvHelper::StoreValues(const int numWheel, const string longSlip, const string latSlip, const string longForce, const string latForce, const string algMoment)
{
	//cout << "Storing values for wheel number: " << numWheel << " for tire model " << _tireModelName << endl;
	_tempCsvLinesLong[numWheel].append(longSlip + "," + longForce + "\n");
	_tempCsvLinesLat[numWheel].append(latSlip + "," + latForce + "\n");
	_tempCsvLinesAlg[numWheel].append(latSlip + "," + algMoment + "\n");
}
