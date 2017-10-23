#include <fstream>
#include <string>
#include <iostream>
#include <map>
#include "Eigen/Eigen"
using namespace std;
class Signal
{
public:
	string color;
	string FeaLabel;
	int startFrame;
	Eigen::Vector3f worldPosition;
	Eigen::Vector2f imgPosition;
	unsigned char r, g, b, a;

};

class ParameterReader
{
public:
	ParameterReader(string pFile = "E:/CrossingReconstruct/parameter.txt")
	{
		ifstream fin(pFile.c_str());
		if (!fin)
		{
			cerr << "parameter file missed" << endl;
			return;
		}
		while (!fin.eof())
		{
			string str;
			getline(fin, str);
			if (str[0] == '#')
			{
				// 以‘＃’开头的是注释
				continue;
			}
			size_t pos = str.find("=");
			if (pos == -1)
				continue;
			// 找到"="的位置，以此为界，前面是键，后面是值 
			string key = str.substr(0, pos);
			string value = str.substr(pos + 1, str.length());
			data[key] = value;
			if (key == "signal")
			{
				Signal sig;
				string colorLine,RangeLine, labelLine, xLine, yLine, pline;
				getline(fin, colorLine);
				size_t pos0 = colorLine.find("=");
				string Lcolor = colorLine.substr(pos0 + 1, colorLine.length());
				if (Lcolor == "green")
				{
					sig.r = 0; sig.g = 255; sig.b = 0; sig.a = 255;
				}
				if (Lcolor == "red")
				{
					sig.r = 255; sig.g = 0; sig.b = 0; sig.a = 255;
				}
				if (Lcolor == "yellow")
				{
					sig.r = 255; sig.g = 255; sig.b = 0; sig.a = 255;
				}
				//getline(fin, RangeLine);
				//getline(fin, labelLine);
				//getline(fin, xLine);
				//getline(fin, yLine);
				getline(fin, pline);
				size_t posit = pline.find('=');
				stringstream ss;
				string ps = pline.substr(posit + 1, pline.length());
				ss.clear();
				ss.str(ps);
				string x, y, z;
				ss >> x >> y >> z;
				sig.worldPosition = Eigen::Vector3f (stof(x), stof(y), stof(z));
				sigPoints.push_back(sig);
			}
			//include all bad situations
			if (!fin.good())
				break;
		}
	}
	//接口函数，提供“键”，对象调用这个方法就可以获得“值”
	string getData(string key)
	{
		map <string, string>::iterator iter = data.find(key);
		if (iter == data.end())
		{
			cerr << "Parameter name " << key << "not found!" << endl;
			return string("NOT FOUND!");
		}
		return iter->second; //second is the value, first is the key
	}

public:
	map <string, string> data;
	std::vector<Signal> sigPoints;

};