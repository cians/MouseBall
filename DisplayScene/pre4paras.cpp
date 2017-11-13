#include <fstream>
#include <string>
#include <iostream>
#include <map>
#include "Eigen/Eigen"
using namespace std;
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
class Signal
{
public:
	Signal() {
		color = "white";
		MapPointId = -1;
		startFrame = -1;
		endFrame = -1;
		trackStatus = "untracked";
		worldPosition = Eigen::Vector3f::Zero();
		r = g = b = a = 255;
	}
public:
	string name;
	string color;
	int MapPointId;
	int startFrame;
	int endFrame;
	string trackStatus;
	Eigen::Vector3f worldPosition;
	unsigned char r, g, b, a;

};
struct Point
{

	Eigen::Vector3f worldPosition;
	unsigned char r, g, b, a;
};

struct Label
{
	size_t frameIndex;
	cv::Rect position;
	string color;
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
				sig.name = value;
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
				if (Lcolor == "black")
				{
					sig.r = 0; sig.g = 0; sig.b = 0; sig.a = 255;
				}
				getline(fin, pline);
				size_t posit = pline.find('=');
				stringstream ss;
				string fId = pline.substr(posit + 1, pline.length());
				sig.MapPointId = stoi(fId);
				inputSigInfo.push_back(sig);
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
	void readLabel(string path)
	{
		ifstream iter(path.c_str());
		if (!iter)
			cerr << "can not find label txt\n";
		int start = stoi(getData("rec_start"));
		int end = stoi(getData("rec_end"));
		while (!iter.eof())
		{
			string str;
			getline(iter, str);
			int length = str.length();
			if (length < 12)
				continue;
			size_t posit = 6;
			int fId = stoi(str.substr(posit + 1, 6));
			if (fId < start || fId > end)
				continue;
			stringstream ss;
			ss.str(str);
			string name, x, y, width, height, color;
			ss >> name >> x >> y >> width >> height >> color;
			Label label;
			label.frameIndex = fId;
			label.position = cv::Rect(stoi(x), stoi(y), stoi(width), stoi(height));
			label.color = color;
			Labelstack.push_back(label);
		}
	}
public:
	map <string, string> data;//parameter.txt 其他参数
	std::vector<Signal> inputSigInfo;//result.act 结合map读取的act文件完善signal信息
	std::vector< Label> Labelstack;//label.txt
};


