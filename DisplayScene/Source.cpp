#include <GL/glut.h>
#include <vector>
#include <cstdlib>
#include <Eigen/Core>
#include "ArcBall.h"
#include "Map.h"
#include <fstream>
#include "pre.cpp"
#include <math.h>
using namespace std;
//#include <GL\glfw3.h>
struct Point
{
	float x, y, z;
	unsigned char r, g, b, a;
};
//全局变量，要显示的点
std::vector< Point > points;
std::vector<Eigen::Vector3f> rectPoints;
//quads color

Matrix4fT   Transform = { 1.0f,  0.0f,  0.0f,  0.0f,                // NEW: Final Transform
0.0f,  1.0f,  0.0f,  0.0f,
0.0f,  0.0f,  1.0f,  0.0f,
0.0f,  0.0f,  0.0f,  1.0f };

//用于累计旋转变换
Matrix3fT   LastRot = { 1.0f,  0.0f,  0.0f,                    // NEW: Last Rotation
0.0f,  1.0f,  0.0f,
0.0f,  0.0f,  1.0f };

Matrix3fT   ThisRot = { 1.0f,  0.0f,  0.0f,                    // NEW: This Rotation
0.0f,  1.0f,  0.0f,
0.0f,  0.0f,  1.0f };
ArcBallT    ArcBall(1280.0f, 800.0f);                                // NEW: same as win size
Point2fT    MousePt;                                                // NEW: Current Mouse Point
Point2fT    LastMousePt;
bool        rightClicked = false;                                        // NEW: Clicking The Mouse?
bool        midClicked = false;
bool        leftClicked = false;
//bool        isDragging = false;                                        // NEW: Dragging The Mouse?
bool     midDragging = false;
bool     rightDragging = false;
bool      leftDragging = false;
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
ParameterReader pd;
Visualization::Map maps;

void getPlaneParas(Eigen::Vector4f &Plane4paras, Eigen::Vector3f &v3Mean)
{
	size_t n_cameras = maps.CameraCount();
	// Get mean of all camera center
	using namespace Eigen;
	v3Mean.setZero(3);
	for (uint i = 0; i < n_cameras; ++i)
	{
		const Vector3f& pos = maps.GetCamera(i).GetPose().GetCenter();
		v3Mean += pos;
	}
	v3Mean /= maps.CameraCount();//法向量起点
								//set matrix A[n,3]
	Eigen::MatrixXf A(n_cameras, 3);
	for (uint i = 0; i < n_cameras; ++i)
	{
		const Vector3f& pos = maps.GetCamera(i).GetPose().GetCenter();
		A.row(i) = pos - v3Mean;
	}

	//SVD of matrix A, solve A * x = 0, and x is the normal of the plane
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::VectorXf Normal3 = svd.matrixV().col(2);
	//Eigen::Vector3f Normal3(normal(0)/normal(3), normal(1)/normal(3), normal(2)/normal(3));
	Normal3.normalize();
	float line_d = -(Normal3.dot(v3Mean));
	Plane4paras = Vector4f(Normal3(0),Normal3(1),Normal3(2), line_d);
}
void getParameters(int &ViewRange, float &pSize, float &lightSize, float& gridNumb,float& gridSize,float& distanceP2P, vector<Signal> &sPoints,Eigen::Vector4f &Plane4paras, Eigen::Vector3f &v3Mean)
{
	 ViewRange = stoi(pd.getData("glOrtho"));
	 pSize = stof(pd.getData("pointSize"));
	 lightSize = stof(pd.getData("lightSize"));
	 gridSize = stof(pd.getData("gridSize"));
	 gridNumb = stof(pd.getData("gridNumb"));
	 distanceP2P = stof(pd.getData("distanceP2P"));
	 sPoints = pd.sigPoints;
	 getPlaneParas(Plane4paras,v3Mean);
}
void drawPlane(Eigen::Vector3f v, Eigen::Vector4f Plane,float gridSize,float gridNumb)
{
	//draw lines
	//for (int i = -gridNumb; i < gridNumb+1; i++)
	//{
	//	float x = gridSize*i + v(0);//固定x得到一条直线
	//	float z0 = - gridNumb*gridSize;
	//	float y0 = -(Plane(0)* x + Plane(2)* z0 + Plane(3)) / Plane(1);
	//	float z1 = gridNumb*gridSize;
	//	float y1 = -(Plane(0)* x + Plane(2)* z1 + Plane(3)) / Plane(1);
	//	glBegin(GL_LINES);
	//	glVertex3f(x, y0, z0);
	//	glVertex3f(x, y1, z1);
	//	glEnd();
	//}
	//for (int j = -gridNumb; j < gridNumb+1; j++)
	//{
	//	float z = gridSize*j + v(2);//固定x得到一条直线
	//	float x0 = - gridNumb*gridSize;
	//	float y0 = -(Plane(0)* x0 + Plane(2)* z + Plane(3)) / Plane(1);
	//	float x1 = gridNumb*gridSize;
	//	float y1 = -(Plane(0)* x1 + Plane(2)* z + Plane(3)) / Plane(1);
	//	glBegin(GL_LINES);
	//	glVertex3f(x0, y0, z);
	//	glVertex3f(x1, y1, z);
	//	glEnd();
	//}

	//draw quads
	for (int j = -gridNumb; j < gridNumb + 1; j++)
	{
		for (int i = -gridNumb; i < gridNumb + 1; i++)
		{
			//固定x
			float dy = gridSize*Plane(2) / sqrt(Plane(1)*Plane(1) + Plane(2)*Plane(2)); //dC/sqrt(B2+C2)
			float dz = -gridSize*Plane(1) / sqrt(Plane(1)*Plane(1) + Plane(2)*Plane(2));
			//固定y
			float dx = gridSize*Plane(2) / sqrt(Plane(0)*Plane(0) + Plane(2)*Plane(2));//dC/sqrt(A2+C2)
			float dz2 = -gridSize*Plane(0) / sqrt(Plane(0)*Plane(0) + Plane(2)*Plane(2));

			float x = v(0) + dx*i;	   float y = v(1) + dy*j;		float z = v(2) + dz2*i + dz*j;
			glColor4ub(135, 206, 235, 200);//skyblue
			glBegin(GL_QUADS);
			glVertex3f(x, y, z);
			glVertex3f(x, y + dy, z + dz);
			glVertex3f(x + dx, y + dy, z + dz + dz2);
			glVertex3f(x + dx, y, z + dz2);
			glEnd();
			if (j == -gridNumb)
			{
				glColor4ub(255, 255, 255, 255);
				glBegin(GL_LINES);
				glVertex3f(x, y, z);
				glVertex3f(x, y + 2*dy*gridNumb, z + 2*dz*gridNumb);
				glEnd();
			}
			if (i == -gridNumb)
			{
				glColor4ub(255, 255, 255, 255);
				glBegin(GL_LINES);
				glVertex3f(x, y, z);
				glVertex3f(x + 2*dx*gridNumb, y, z + 2*dz2*gridNumb);
				glEnd();
			}
		}
	}
}
void display(void)
{
	int ViewRange;
	float pSize, lightSize, gridSize,gridNumb,distanceP2P;
	std::vector<Signal> sigPoints;
	Eigen::Vector4f Plane4paras;
	Eigen::Vector3f v3Mean;
	getParameters(ViewRange, pSize, lightSize, gridNumb, gridSize,distanceP2P, sigPoints, Plane4paras,v3Mean);

	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glEnable(GL_DEPTH_TEST);
	// Accept fragment if it closer to the camera than the former one
	glDepthFunc(GL_LESS);

	glMatrixMode(GL_PROJECTION);//Applies subsequent matrix operations to the projection matrix stack
	glLoadIdentity();
	glOrtho(-ViewRange, ViewRange, -ViewRange, ViewRange, -ViewRange, ViewRange);//glOrtho(left, right, bottom, top, near, far)

	glMatrixMode(GL_MODELVIEW);//Applies subsequent matrix operations to the modelview matrix stack.
	glLoadIdentity();//重置当前Modelview矩阵
	glPushMatrix();                                                    // NEW: Prepare Dynamic Transform
	glMultMatrixf(Transform.M);                                        // NEW: Apply Dynamic Transform
	//glRotatef(4, 1.0f, 0.0f, 0.0f);
	// draw
	glColor3ub(255, 255, 255);  
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);

	glVertexPointer(3, GL_FLOAT, sizeof(Point), &points[0].x);//依次是(坐标个数，type，stride，首元素地址)，后面坐标是3！
	glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(Point), &points[0].r);
	glLineWidth(pSize-1);
	glPointSize(pSize);
	glDrawArrays(GL_POINTS, 0, points.size());

	glVertexPointer(3, GL_FLOAT, sizeof(Signal), &sigPoints[0].worldPosition(0));//依次是(坐标个数，type，stride，首元素地址)，后面坐标是3！
	glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(Signal), &sigPoints[0].r);
	glPointSize(lightSize);
	glDrawArrays(GL_POINTS, 0, sigPoints.size());
	
	//draw plane
	//平面方程Ax + By + Cz + D= 0,
	Eigen::Vector4f Plane2 = Plane4paras;
	Plane2(3) -= distanceP2P;
	float dealt = -distanceP2P / sqrt(Plane4paras(0)*Plane4paras(0) + Plane4paras(1)*Plane4paras(1) + Plane4paras(2)*Plane4paras(2));
	Eigen::Vector3f v_new = Eigen::Vector3f(v3Mean(0) - dealt*Plane4paras(0), v3Mean(1) - dealt*Plane4paras(1), v3Mean(2) - dealt*Plane4paras(2));
	drawPlane(v_new, Plane2, gridSize, gridNumb);
	

	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);
	glPopMatrix();

	glFlush();
	glutSwapBuffers();
}

void reshape(int w, int h)
{
	glViewport(0, 0, w, h);    glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	if (w <= h)
		glOrtho(-2.0, 2.0, -2.0 * (GLfloat)h / (GLfloat)w, 2.0 * (GLfloat)h / (GLfloat)w, -10.0, 10.0);
	else
		glOrtho(-2.0 * (GLfloat)w / (GLfloat)h, 2.0 * (GLfloat)w / (GLfloat)h, -2.0, 2.0, -10.0, 10.0);
	glMatrixMode(GL_MODELVIEW);

}
void mouse(int btn, int state, int x, int y)
{
	if (btn == GLUT_RIGHT_BUTTON)
	{
		switch (state)
		{
		case GLUT_DOWN:
			rightClicked = true;
			printf_s("startmotion: x = %d, y = %d\n", x, y);
			break;
		case GLUT_UP:
			rightClicked = false;
			rightDragging = false;
			printf_s("stopmotion: x = %d, y = %d\n", x, y);
			break;
		}
	}

	if (btn == GLUT_LEFT_BUTTON )
	{
		switch (state)
		{
		case GLUT_DOWN:
			leftClicked = true;
			printf_s("startmotion: x = %d, y = %d\n", x, y);
			break;
		case GLUT_UP:
			leftClicked = false;
			leftDragging = false;
			printf_s("stopmotion: x = %d, y = %d\n", x, y);
			break;
		}
	}
	if (btn == GLUT_MIDDLE_BUTTON)
	{
		switch (state)
		{
		case GLUT_DOWN:
			midClicked = true;
			printf_s("startmotion: x = %d, y = %d\n", x, y);
			break;
		case GLUT_UP:
			midClicked = false;
			midDragging = false;
			printf_s("stopmotion: x = %d, y = %d\n", x, y);
			break;
		}
	}

}
void mouseMotion(int x, int y)
{
	MousePt.s.X = x;
	MousePt.s.Y = y;

	if (!rightDragging)                                                // Not Dragging
	{
		if (rightClicked)                                                // First Click
		{
			rightDragging = true;                                        // Prepare For Dragging
			LastRot = ThisRot;                                        // Set Last Static Rotation To Last Dynamic One           
			ArcBall.click(&MousePt);                                // Update Start Vector And Prepare For Dragging
		}
	}
	else
	{
		if (rightClicked)                                                // Still Clicked, So Still Dragging
		{
			Quat4fT     ThisQuat;
			ArcBall.drag(&MousePt, &ThisQuat);                        // Update End Vector And Get Rotation As Quaternion
			Matrix3fSetRotationFromQuat4f(&ThisRot, &ThisQuat);        // Convert Quaternion Into Matrix3fT
			Matrix3fMulMatrix3f(&ThisRot, &LastRot);                // Accumulate Last Rotation Into This One
			Matrix4fSetRotationFromMatrix3f(&Transform, &ThisRot);    // Set Our Final Transform's Rotation From This One
		}
	}
	if (!midDragging)
	{
		if (midClicked)
		{
			midDragging = true;
			LastMousePt = MousePt;
			LastTX = ThisTX;
			LastTY = ThisTY;
		}
	}
	else
	{
		if (midClicked)
		{
			ThisTX = LastTX + MousePt.s.X - LastMousePt.s.X;
			ThisTY = LastTY + MousePt.s.Y - LastMousePt.s.Y;
			Transform.s.TX =   translateScale * ThisTX;
			Transform.s.TY = -translateScale * ThisTY;
		}
	}
	if (!leftDragging)
	{
		if (leftClicked)
		{
			leftDragging = true;
			LastMousePt = MousePt;
			LastScale = ThisScale;	
		}
	}
	else
	{
		if (leftClicked)
		{
            ThisScale = LastScale * ( 1 + ScaleFactor* ( MousePt.s.Y - LastMousePt.s.Y));//缩放比例是（1+X）比较合适
			Transform.s.SW  = ThisScale;

		}
	}
	glutPostRedisplay();
}
void keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 27: // VK_ESCAPE
		points.clear();
		exit(0);
		break;
	case 82: //Press "R" to Reset view Matrix
		Matrix3fSetIdentity(&LastRot);                                // Reset Rotation
		Matrix3fSetIdentity(&ThisRot);                                // Reset Rotation
		Matrix4fSetRotationFromMatrix3f(&Transform, &ThisRot);        // Reset Rotation
		glutPostRedisplay();
		break;
	}
}


int main(int argc, char **argv)
{

	string filename = pd.getData("act_dir");
	maps.LoadAct(filename);
	//drawPlane(maps);
	translateScale = stof(pd.getData("translateScale"));
	ScaleFactor = stof(pd.getData("ScaleFactor"));
	int width = stoi(pd.getData("width"));
	int height = stoi(pd.getData("height"));
	ArcBall.setBounds(width, height);
	unsigned char cam_r =(unsigned char) stoi(pd.getData("cam_r"));
	unsigned char cam_g = (unsigned char)stoi(pd.getData("cam_g"));
	unsigned char cam_b = (unsigned char)stoi(pd.getData("cam_b"));
	unsigned char cam_a = (unsigned char)stoi(pd.getData("cam_a"));
	string drawFeatures = pd.getData("drawFeatures");
	// populate points,GLfloat vertices[][3]
	//
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
	//创建窗口
	glutInitWindowSize(width, height);
	glutCreateWindow("Crossing");
	//是否绘制其它特征点
	if (drawFeatures == "yes")
	{
		for (size_t i = 0; i < maps.MapPointCount(); ++i)
		{
			Point pt;
			pt.x = maps.GetMapPoint(i).m_3d_point(0);
			pt.y = maps.GetMapPoint(i).m_3d_point(1);
			pt.z = maps.GetMapPoint(i).m_3d_point(2);
			pt.r = 255;
			pt.g = 255;
			pt.b = 255;
			pt.a = 220;
			points.push_back(pt);
		}
	}
	//相机轨迹
	for (size_t i = 0; i < maps.CameraCount(); ++i)
	{
		Point cam;
		cam.x = maps.GetCamera(i).GetPose().center(0);
		cam.y = maps.GetCamera(i).GetPose().center(1);
		cam.z = maps.GetCamera(i).GetPose().center(2);
		cam.r = cam_r;
		cam.g = cam_g;
		cam.b = cam_b;
		cam.a = cam_a;
		points.push_back(cam);
	}



	glutDisplayFunc(display);
	glutReshapeFunc(reshape);//reshape 		
	glutMouseFunc(mouse);
	glutMotionFunc(mouseMotion);
	glutKeyboardFunc(keyboard);
	glutMainLoop();
	return 0;
}