#include <GL/glut.h>
#include <vector>
#include <cstdlib>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "ArcBall.h"
using namespace std;
//#include <GL\glfw3.h>
struct Point
{
	float x, y,z;
	unsigned char r, g, b, a;
};
//全局变量，要显示的点
std::vector< Point > points;
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
ArcBallT    ArcBall(640.0f, 480.0f);                                // NEW: ArcBall Instance
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
void display(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);//Applies subsequent matrix operations to the projection matrix stack
	glLoadIdentity();
	glOrtho(-5, 5, -5, 5, -3, 3);//glOrtho(left, right, bottom, top, near, far)

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
	glPointSize(3.0);
	glDrawArrays(GL_POINTS, 0, points.size());

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
			Transform.s.TX =  0.05 * ThisTX;
			Transform.s.TY = - 0.05 * ThisTY;
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
            ThisScale = LastScale * ( 1 + 0.01* ( MousePt.s.Y - LastMousePt.s.Y));//缩放比例是（1+X）比较合适
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
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
	//创建窗口
	glutInitWindowSize(640, 480);
	glutCreateWindow("Random Points");

	glutDisplayFunc(display);
	glutReshapeFunc(reshape);//reshape 是？？
	//鼠标控制
	//glutMouseFunc(MouseCon);
	glutMouseFunc(mouse);
	glutMotionFunc(mouseMotion);
	glutKeyboardFunc(keyboard);

	// populate points,GLfloat vertices[][3]
	for (size_t i = 0; i < 100; ++i)
	{
		Point pt;
		pt.x = -5 + (rand() % 10);//-50~50
		pt.y = -5 + (rand() % 10);
		pt.z = -3 + (rand()% 6);
		pt.r = rand() % 255;
		pt.g = rand() % 255;
		pt.b = rand() % 255;
		pt.a = 255;
		points.push_back(pt);
	}

	glutMainLoop();
	return 0;
}