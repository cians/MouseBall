#pragma once
#include <GL/freeglut.h>
#include <vector>
#include "ArcBall.h"
#include "Map.h"
#include "pre4paras.cpp"
#include <math.h> //sqrt

struct SignalPoint
{
	Eigen::Vector3f worldPosition;
	unsigned char r, g, b, a;
	string name;
	int MID;
	Visualization::MapPoint mapPoint;
};
using namespace std;
//用于鼠标交互显示的参数
Matrix4fT   Transform;
//用于累计旋转变换
Matrix3fT   LastRot, ThisRot;
ArcBallT    ArcBall(1280.0f, 800.0f);                                // NEW: same as win size
Point2fT    MousePt, LastMousePt;                                             // NEW: Current Mouse Point
bool        rightClicked = false;                                        // NEW: Clicking The Mouse?
bool        midClicked = false;
bool        leftClicked = false;                                       // NEW: Dragging The Mouse?
bool     midDragging = false;
bool     rightDragging = false;
bool     leftDragging = false;
//用于累计平移变换。
float LastTX = 0;   float LastTY = 0;
float ThisTX = 0;   float ThisTY = 0;
//用于累计缩放变换
float LastScale = 1;
float ThisScale = 1;
//鼠标平移移动因子
float translateScale = 5;
//鼠标缩放移动因子
float ScaleFactor = 0.04f;

//要显示的灯
std::vector< SignalPoint> sigPoints;
int drawPointsNum = 0;//动态绘制的点数
Eigen::Matrix3f INTRINSIC;

Visualization::Map maps;
cv::VideoCapture imgShowSeque;
std::vector< Point > points;//camera center,feature points
ParameterReader pd;

void drawPlane(Eigen::Vector3f v, Eigen::Vector4f Plane, float gridSize, int gridNumb);
void drawLine(Eigen::Vector3f M0, Eigen::Vector3f M1);
void drawCameraModel(Eigen::Vector3f Cam_center, Eigen::Matrix3f Cam_M, float length, float Xoffset, float Yoffset);
void getPlaneParas(Eigen::Vector4f &Plane4paras, Eigen::Vector3f &v3Mean);
void getParameters(int &ViewRange, float &pSize, float &lightSize, int& gridNumb, float& gridSize, float& distanceP2P, float& M_len, float& M_X, float& M_Y, Eigen::Vector4f &Plane4paras, Eigen::Vector3f &v3Mean);
void display(void);
void MultiviewTriangulation(std::vector < Eigen::Vector2f > &img_coords, std::vector<Eigen::Matrix<float, 3, 4> > &cam_P_ext, Eigen::Vector3f &XYZ);
void drawSignal(size_t frameIndex);
void reshape(int w, int h);
void mouse(int btn, int state, int x, int y);
void mouseMotion(int x, int y);
void keyboard(unsigned char key, int x, int y);
void color2rgb(string color, cv::Scalar &RGB);
void RenderUpdate(int i);