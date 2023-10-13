#pragma once
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
using namespace std;

class CsvHelper
{
private:
	string _tireModelName;
	vector<string> _csvLinesLong;
	vector<string> _csvLinesLat;
	vector<string> _csvLinesAlg;

	vector<string> _tempCsvLinesLong;
	vector<string> _tempCsvLinesLat;
	vector<string> _tempCsvLinesAlg;

public:
	CsvHelper(string tireModelName);

	void StoreValues(const int numWheel, const string longSlip, const string latSlip, const string longForce, const string latForce, const string algMoment);

	void WriteCSV();

	void FlushTemp();
};