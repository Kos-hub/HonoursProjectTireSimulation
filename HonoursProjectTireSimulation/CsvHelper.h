#pragma once
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
using namespace std;

class CsvHelper
{
private:
	string _tireModelName;
	vector<string> _csvLinesLong;
	vector<string> _csvLinesLat;
	vector<string> _csvLinesAlg;

public:
	CsvHelper(string tireModelName);

	void StoreValues(const int numWheel, const string longSlip, const string latSlip, const string longForce, const string latForce, const string algMoment);

	void WriteCSV();
};