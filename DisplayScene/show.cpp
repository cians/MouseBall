#include <stdlib.h>
#include <GL/glut.h>
#include <stdio.h>
#include "math.h"
#include "ArcBall.h"

// object
GLfloat vertices[] = { -1.0,-1.0,-1.0,1.0,-1.0,-1.0,
1.0,1.0,-1.0, -1.0,1.0,-1.0, -1.0,-1.0,1.0,
1.0,-1.0,1.0, 1.0,1.0,1.0, -1.0,1.0,1.0 };

GLfloat colors[] = { 0.0,0.0,0.0,1.0,0.0,0.0,
1.0,1.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0,
1.0,0.0,1.0, 1.0,1.0,1.0, 0.0,1.0,1.0 };

GLubyte cubeIndices[] = { 0,3,2,1,2,3,7,6,0,4,7,3,1,2,6,5,4,5,6,7,0,1,5,4 };

// mouse control
Matrix4fT   Transform = { 1.0f,  0.0f,  0.0f,  0.0f,                // NEW: Final Transform
0.0f,  1.0f,  0.0f,  0.0f,
0.0f,  0.0f,  1.0f,  0.0f,
0.0f,  0.0f,  0.0f,  1.0f };

Matrix3fT   LastRot = { 1.0f,  0.0f,  0.0f,                    // NEW: Last Rotation
0.0f,  1.0f,  0.0f,
0.0f,  0.0f,  1.0f };

Matrix3fT   ThisRot = { 1.0f,  0.0f,  0.0f,                    // NEW: This Rotation
0.0f,  1.0f,  0.0f,
0.0f,  0.0f,  1.0f };

ArcBallT    ArcBall(640.0f, 480.0f);                                // NEW: ArcBall Instance
Point2fT    MousePt;                                                // NEW: Current Mouse Point
bool        isClicked = false;                                        // NEW: Clicking The Mouse?
bool        isDragging = false;                                        // NEW: Dragging The Mouse?


void display(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glLoadIdentity();

	glPushMatrix();                                                    // NEW: Prepare Dynamic Transform
	glMultMatrixf(Transform.M);                                        // NEW: Apply Dynamic Transform
	//glDrawArrays(GL_POINTS, 0, points.size());
	glDrawElements(GL_QUADS, 24, GL_UNSIGNED_BYTE, cubeIndices);
	glPopMatrix();

	glutSwapBuffers();
}

void mouse(int btn, int state, int x, int y)
{
	if (btn == GLUT_LEFT_BUTTON)
	{
		switch (state)
		{
		case GLUT_DOWN:
			isClicked = true;
			printf_s("startmotion: x = %d, y = %d\n", x, y);
			break;
		case GLUT_UP:
			isClicked = false;
			isDragging = false;
			printf_s("stopmotion: x = %d, y = %d\n", x, y);
			break;
		}
	}

	if (btn == GLUT_RIGHT_BUTTON && state == GLUT_DOWN)
	{
		Matrix3fSetIdentity(&LastRot);                                // Reset Rotation
		Matrix3fSetIdentity(&ThisRot);                                // Reset Rotation
		Matrix4fSetRotationFromMatrix3f(&Transform, &ThisRot);        // Reset Rotation

		glutPostRedisplay();
	}
}

void mouseMotion(int x, int y)
{
	MousePt.s.X = x;
	MousePt.s.Y = y;

	if (!isDragging)                                                // Not Dragging
	{
		if (isClicked)                                                // First Click
		{
			isDragging = true;                                        // Prepare For Dragging
			LastRot = ThisRot;                                        // Set Last Static Rotation To Last Dynamic One           
			ArcBall.click(&MousePt);                                // Update Start Vector And Prepare For Dragging
		}
	}
	else
	{
		if (isClicked)                                                // Still Clicked, So Still Dragging
		{
			Quat4fT     ThisQuat;

			ArcBall.drag(&MousePt, &ThisQuat);                        // Update End Vector And Get Rotation As Quaternion
			Matrix3fSetRotationFromQuat4f(&ThisRot, &ThisQuat);        // Convert Quaternion Into Matrix3fT
			Matrix3fMulMatrix3f(&ThisRot, &LastRot);                // Accumulate Last Rotation Into This One
			Matrix4fSetRotationFromMatrix3f(&Transform, &ThisRot);    // Set Our Final Transform's Rotation From This One
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
	}
}

void myReshape(int w, int h)
{
	glViewport(0, 0, w, h);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	if (w <= h)
		glOrtho(-2.0, 2.0, -2.0 * (GLfloat)h / (GLfloat)w, 2.0 * (GLfloat)h / (GLfloat)w, -10.0, 10.0);
	else
		glOrtho(-2.0 * (GLfloat)w / (GLfloat)h, 2.0 * (GLfloat)w / (GLfloat)h, -2.0, 2.0, -10.0, 10.0);
	glMatrixMode(GL_MODELVIEW);
}

void main(int argc, char **argv)
{
	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(500, 500);
	glutCreateWindow("colorcube");

	glutReshapeFunc(myReshape);
	glutDisplayFunc(display);
	glutMouseFunc(mouse);
	glutMotionFunc(mouseMotion);
	glutKeyboardFunc(keyboard);

	glEnable(GL_DEPTH_TEST);

	glEnableClientState(GL_COLOR_ARRAY);
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, vertices);
	glColorPointer(3, GL_FLOAT, 0, colors);

	glClearColor(0.0, 0.0, 0.0, 1.0);
	glColor3f(1.0, 1.0, 1.0);

	glutMainLoop();
}
      