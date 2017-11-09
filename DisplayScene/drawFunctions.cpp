#include "drawFuncs.h"
void drawPlane(Eigen::Vector3f v, Eigen::Vector4f Plane, float gridSize, int gridNumb)
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
			float dyz = -gridSize*Plane(1) / sqrt(Plane(1)*Plane(1) + Plane(2)*Plane(2));
			//固定y
			float dx = gridSize*Plane(2) / sqrt(Plane(0)*Plane(0) + Plane(2)*Plane(2));//dC/sqrt(A2+C2)
			float dxz = -gridSize*Plane(0) / sqrt(Plane(0)*Plane(0) + Plane(2)*Plane(2));

			float x = v(0) + dx*i;	   float y = v(1) + dy*j;		float z = v(2) + dxz*i + dyz*j;
			glColor4ub(135, 206, 235, 200);//skyblue
			glBegin(GL_QUADS);
			glVertex3f(x, y, z);
			glVertex3f(x, y + dy, z + dyz);
			glVertex3f(x + dx, y + dy, z + dyz + dxz);
			glVertex3f(x + dx, y, z + dxz);
			glEnd();
			//画四边形边框
			if (j == -gridNumb)
			{
				glColor4ub(255, 255, 255, 255);
				glBegin(GL_LINES);
				glVertex3f(x, y, z);
				glVertex3f(x, y + 2 * dy*gridNumb, z + 2 * dyz*gridNumb);
				glEnd();
			}
			if (i == -gridNumb)
			{
				glColor4ub(255, 255, 255, 255);
				glBegin(GL_LINES);
				glVertex3f(x, y, z);
				glVertex3f(x + 2 * dx*gridNumb, y, z + 2 * dxz*gridNumb);
				glEnd();
			}
		}
	}
}
void drawLine(Eigen::Vector3f M0, Eigen::Vector3f M1)
{
	glLineWidth(2.0);
	glColor4ub(123, 104, 238, 255);
	glBegin(GL_LINES);
	glVertex3f(M0(0), M0(1), M0(2));
	glVertex3f(M1(0), M1(1), M1(2));
	glEnd();
}
//draw camera trajectory and plane
void drawCameraModel(Eigen::Vector3f Cam_center, Eigen::Matrix3f Cam_M, float length, float Xoffset, float Yoffset)
{
	//rotation的逆才是 transform matrix
	Eigen::Vector3f Cam_R = Cam_M.row(0) / Cam_M.row(0).norm();
	Eigen::Vector3f Cam_UP = -Cam_M.row(1) / Cam_M.row(1).norm();
	Eigen::Vector3f Cam_BK = -Cam_M.row(2) / Cam_M.row(2).norm();
	Eigen::Vector3f Model_center;
	Model_center = Cam_center - length *Cam_BK / Cam_BK.norm();

	Eigen::Vector3f M00, M01, M11, M10;
	M00 = Model_center - Xoffset*Cam_R - Yoffset*Cam_UP;
	M01 = Model_center - Xoffset*Cam_R + Yoffset*Cam_UP;
	M11 = Model_center + Xoffset*Cam_R + Yoffset*Cam_UP;
	M10 = Model_center + Xoffset*Cam_R - Yoffset*Cam_UP;

	drawLine(M00, M01);
	drawLine(M01, M11);
	drawLine(M11, M10);
	drawLine(M10, M00);
	drawLine(M10, M00);
	drawLine(M10, Cam_center);
	drawLine(M11, Cam_center);
	drawLine(M00, Cam_center);
	drawLine(M01, Cam_center);
}
