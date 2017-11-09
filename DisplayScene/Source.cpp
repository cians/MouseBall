//#include <GL/glut.h>
#include <GL/freeglut.h>
#include <vector>
#include "ArcBall.h"
#include "Map.h"
#include "pre4paras.cpp"
#include "drawFuncs.h"
#include <math.h> //sqrt
std::vector< Point > points;//camera center,feature points
ParameterReader pd;
struct SignalPoint
{
	Eigen::Vector3f worldPosition;
	unsigned char r, g, b, a;
	string name;
	int MID;
	Visualization::MapPoint mapPoint;
};
using namespace std;
//������꽻����ʾ�Ĳ���
Matrix4fT   Transform;
//�����ۼ���ת�任
Matrix3fT   LastRot,ThisRot;
ArcBallT    ArcBall(1280.0f, 800.0f);                                // NEW: same as win size
Point2fT    MousePt, LastMousePt;                                             // NEW: Current Mouse Point
bool        rightClicked = false;                                        // NEW: Clicking The Mouse?
bool        midClicked = false;
bool        leftClicked = false;                                       // NEW: Dragging The Mouse?
bool     midDragging = false;
bool     rightDragging = false;
bool     leftDragging = false;
//�����ۼ�ƽ�Ʊ任��
float LastTX = 0;   float LastTY = 0;
float ThisTX = 0;   float ThisTY = 0;
//�����ۼ����ű任
float LastScale = 1;
float ThisScale = 1;
//���ƽ���ƶ�����
float translateScale = 5;
//��������ƶ�����
float ScaleFactor = 0.04f;

//ȫ�ֱ�����Ҫ��ʾ�ĵ�
std::vector< SignalPoint> sigPoints;
int drawPointsNum = 0;//��̬���Ƶĵ���
Eigen::Matrix3f INTRINSIC;

Visualization::Map maps;
cv::VideoCapture imgShowSeque;

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
	v3Mean /= (float)maps.CameraCount();//���������
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
void getParameters(int &ViewRange, float &pSize, float &lightSize, int& gridNumb,float& gridSize,float& distanceP2P, float& M_len,float& M_X,float& M_Y, Eigen::Vector4f &Plane4paras, Eigen::Vector3f &v3Mean)
{
	 ViewRange = stoi(pd.getData("glOrtho"));
	 pSize = stof(pd.getData("pointSize"));
	 lightSize = stof(pd.getData("lightSize"));
	 gridSize = stof(pd.getData("gridSize"));
	 gridNumb = stoi(pd.getData("gridNumb"));
	 distanceP2P = stof(pd.getData("distanceP2P"));
	 M_len = stof(pd.getData("Model_length"));
	 M_X = stof(pd.getData("Model_Xoffset"));
	 M_Y = stof(pd.getData("Model_Yoffset"));
	 getPlaneParas(Plane4paras,v3Mean);
}

void display(void)
{
	int ViewRange;
	float pSize, lightSize, gridSize, distanceP2P, Model_Len, Model_Xoffset, Model_Yoffset;
	int gridNumb;
	Eigen::Vector4f Plane4paras;
	Eigen::Vector3f v3Mean;
	getParameters(ViewRange, pSize, lightSize, gridNumb, gridSize, distanceP2P, Model_Len, Model_Xoffset, Model_Yoffset, Plane4paras, v3Mean);
	if (drawPointsNum >= points.size())
		drawPointsNum = points.size() - 1;
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//����zbuffer
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

	glMatrixMode(GL_PROJECTION);//Applies subsequent matrix operations to the projection matrix stack
	glLoadIdentity();
	glOrtho(-ViewRange , ViewRange, -ViewRange, ViewRange, -ViewRange, ViewRange);//glOrtho(left, right, bottom, top, near, far)

	glMatrixMode(GL_MODELVIEW);//Applies subsequent matrix operations to the modelview matrix stack.
	glLoadIdentity();//���õ�ǰModelview����
	glPushMatrix();                                                    // NEW: Prepare Dynamic Transform
	glMultMatrixf(Transform.M);                                        // NEW: Apply Dynamic Transform
	glRotatef(270, 1.0f, 0.0f, 0.0f);      //glRotatef(angle,x,y,z),angle�ǽǶ�ֵ��x,y,z�ǲ���ֵ
	glRotatef(90, 0.0f, 1.0f, 0.0f);
	glScalef(2.5, 2.5, 2.5);

	// draw
	glColor4ub(255, 255, 255,255);
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);

	glVertexPointer(3, GL_FLOAT, sizeof(Point), &points[0].worldPosition(0));//������(���������type��stride����Ԫ�ص�ַ)������������3��
	glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(Point), &points[0].r);
	glLineWidth(pSize - 2);
	glPointSize(pSize);
	glDrawArrays(GL_POINTS, 0, drawPointsNum);//��̬�ı���Ƶ���
	drawCameraModel(points[drawPointsNum].worldPosition, maps.GetCamera(drawPointsNum).m_pose.m_rotation,Model_Len, Model_Xoffset, Model_Yoffset);

	if (sigPoints.size() > 0)
	{
		glVertexPointer(3, GL_FLOAT, sizeof(SignalPoint), &sigPoints[0].worldPosition(0));//������(���������type��stride����Ԫ�ص�ַ)������������3��
		glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(SignalPoint), &sigPoints[0].r);
		glPointSize(lightSize);
		glDrawArrays(GL_POINTS, 0, sigPoints.size());
	}
	//draw plane
	//ƽ�淽��Ax + By + Cz + D= 0, ������ƽ�������·��ľ���
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
void MultiviewTriangulation(std::vector < Eigen::Vector2f > &img_coords, std::vector<Eigen::Matrix<float, 3, 4> > &cam_P_ext, Eigen::Vector3f &XYZ)
{
	assert(img_coords.size() == cam_P_ext.size());
	const size_t num = img_coords.size();
	Eigen::MatrixX3f T;
	Eigen::VectorXf p;
	T.resize(2*num, 3);
	p.resize(2 * num);
	for (size_t i = 0; i < num; i++)
	{
		Eigen::Matrix<float, 3, 4> CAM_P = INTRINSIC*cam_P_ext[i];
		T.block<1, 3>(2*i, 0) = CAM_P.block<1, 3>(0, 0) - img_coords[i](0) * CAM_P.block<1, 3>(2, 0);
		T.block<1, 3>(2*i+1, 0) = CAM_P.block<1, 3>(1, 0) - img_coords[i](1) * CAM_P.block<1, 3>(2, 0);
		p(2 * i) = CAM_P(0, 3) - img_coords[i](0) * CAM_P(2, 3);
		p(2*i+1) = CAM_P(1, 3) - img_coords[i](1) * CAM_P(2, 3);
	}
	XYZ = (T.transpose()*T).colPivHouseholderQr().solve(-T.transpose()*p);
	cout << XYZ << endl;
}
void drawSignal(size_t frameIndex)
{
	size_t SignalId;
	for (size_t i = 0; i < pd.inputSigInfo.size(); i++)
	{
		if (pd.inputSigInfo[i].trackStatus == "untracked")
		{
			if (frameIndex >(pd.inputSigInfo[i].startFrame + 1))//��֡���ϲ��ܹ������
			{
				SignalId = i;
				pd.inputSigInfo[i].trackStatus = "tracking";
				SignalPoint p;
				p.r = pd.inputSigInfo[i].r;
				p.g = pd.inputSigInfo[i].g;
				p.b = pd.inputSigInfo[i].b;
				p.a = pd.inputSigInfo[i].a;
				p.name = pd.inputSigInfo[i].name;
				p.MID = pd.inputSigInfo[i].MapPointId;
				p.mapPoint = maps.GetMapPoint(pd.inputSigInfo[i].MapPointId);
				sigPoints.push_back(p);//����tracking ����
			}
		}
	}
	//SignalId Ϊ Ŀǰtrack��֡����

	for (auto tsig = sigPoints.begin();tsig != sigPoints.end();tsig++)
	{
		//ֻ������ʷ֡��ϢframeIndex - measure[0].m_cam_index
		//ÿ֡��2d���� measure[i].m_img
		//camera project matrix:Getcamera(MID),r,
		//ֻ��������tracking�ĵƵ�XYZ,tracked ��ֱ����act�ļ���3d ����
		if (maps.GetMapPointMeasures(tsig->MID)[maps.GetMapPointMeasures(tsig->MID).size() - 1]->m_cam_index < frameIndex)
		{
			tsig->worldPosition = tsig->mapPoint.m_3d_point;//���track���
			continue;
		}
		std::vector < Eigen::Vector2f > img_coords;
		std::vector<Eigen::Matrix<float, 3, 4> > cam_Ps;
		auto  Measss =  maps.GetMapPointMeasures(tsig->MID);
		for (vector<const Visualization::Measurement*>::const_iterator iter = Measss.cbegin();(*iter)->m_cam_index < frameIndex && iter!= Measss.cend();iter++)
		{
			img_coords.push_back((*iter)->m_img);
			Eigen::Matrix<float, 3, 4> P;
			P.block<3, 3>(0, 0) = maps.GetCamera((*iter)->m_cam_index).GetPose().m_rotation;
			P.block<3, 1>(0, 3) = maps.GetCamera((*iter)->m_cam_index).GetPose().m_translation;
			cam_Ps.push_back(P);
		}
		Eigen::Vector3f XYZ;
		MultiviewTriangulation(img_coords, cam_Ps, XYZ);
		tsig->worldPosition = XYZ;
	}
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
	MousePt.s.X = (float)x;
	MousePt.s.Y = (float)y;

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
            ThisScale = LastScale * ( 1 + ScaleFactor* ( MousePt.s.Y - LastMousePt.s.Y));//���ű����ǣ�1+X���ȽϺ���
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
		//glutLeaveMainLoop();
		glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
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
void color2rgb(string color, cv::Scalar &RGB)
{
	if (color == "red")
		RGB = cv::Scalar(0, 0, 255);//BGRA
	if (color == "green")
		RGB = cv::Scalar(0, 255, 0);
	if (color == "yellow")
		RGB = cv::Scalar(0, 255, 255);
	if (color == "other")
		RGB = cv::Scalar(0, 0, 0);
}
void RenderUpdate(int i)
{
	if (drawPointsNum < points.size())
	{
		//һ֡֡��ʾͼ��
		cv::Mat img;	
		imgShowSeque >> img;
		if (img.empty())
			return;
		for (auto lab : pd.Labelstack)
		{
			if (lab.frameIndex - stoi(pd.getData("rec_start")) == drawPointsNum)
			{
				cv::Rect lamp = lab.position;
				cv::Scalar lampColor;
				color2rgb(lab.color, lampColor);
				cv::rectangle(img, lamp, lampColor, 3);
			}
		}
		resize(img, img, cv::Size(968, 768));

		cv::imshow("video", img);
		cv::moveWindow("video", 868, 0);
		//imshowʱ��̫�̣�movewindow����û�ã�ֻ����ͣ�£��ֶ��ƶ��ˡ�
		if (drawPointsNum < 1)
			cv::waitKey(15000);
		drawPointsNum++;
		drawSignal(drawPointsNum);
		display();//��·�������켣
		glutTimerFunc(40, RenderUpdate, 1);//��Ҫ�ں������ٵ���һ�α���ѭ��
	}
}
int main(int argc, char **argv)
{
	//�ڲ�
	INTRINSIC <<
		1941.45, 0, 957.207,
		0, 1945.9, 611.806,
		0, 0, 1;
	//������ʾ�Ĳ���
	translateScale = stof(pd.getData("translateScale"));
	ScaleFactor = stof(pd.getData("ScaleFactor"));
	int width = stoi(pd.getData("width"));
	int height = stoi(pd.getData("height"));
	ArcBall.setBounds(width, height);
	unsigned char cam_r = (unsigned char)stoi(pd.getData("cam_r"));
	unsigned char cam_g = (unsigned char)stoi(pd.getData("cam_g"));
	unsigned char cam_b = (unsigned char)stoi(pd.getData("cam_b"));
	unsigned char cam_a = (unsigned char)stoi(pd.getData("cam_a"));
	//��ȡlabel��imgSequence
	string drawFeatures = pd.getData("drawFeatures");
	string imgsPath = pd.getData("img_dir");
	string labelPath = pd.getData("label_dir");
	string resultFile = imgsPath + "result.act";
	imgShowSeque.open(imgsPath + "%05d.jpg");
	if (!imgShowSeque.isOpened())
	{
		cerr << "can not find Image sequence!\n";
		return -1;
	}
	pd.readLabel(labelPath);
	maps.LoadAct(resultFile);

	//���map��ȡ��act�ļ�����signal��Ϣ
	for (size_t i = 0; i < pd.inputSigInfo.size(); ++i)
	{
		size_t MID = pd.inputSigInfo[i].MapPointId;
		pd.inputSigInfo[i].startFrame = maps.GetMapPointMeasures(MID)[0]->m_cam_index;
		pd.inputSigInfo[i].endFrame = maps.GetMapPointMeasures(MID)[maps.GetMapPointMeasures(MID).size() - 1]->m_cam_index;
		pd.inputSigInfo[i].worldPosition = maps.GetMapPoint(MID).m_3d_point;
	}
	//�Ƿ��������������
	if (drawFeatures == "yes")
	{
		for (size_t i = 0; i < maps.MapPointCount(); ++i)
		{
			Point pt;
			pt.worldPosition = maps.GetMapPoint(i).m_3d_point;
			pt.r = 255;
			pt.g = 255;
			pt.b = 255;
			pt.a = 220;
			points.push_back(pt);
		}
	}
	//����켣
	for (size_t i = 0; i < maps.CameraCount(); ++i)
	{
		Point cam;
		cam.worldPosition = maps.GetCamera(i).GetPose().center;
		cam.r = cam_r;
		cam.g = cam_g;
		cam.b = cam_b;
		cam.a = cam_a;
		points.push_back(cam);
	}
	//
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
	//��������
	glutInitWindowSize(width, height);
	glutInitWindowPosition(0, 0);
	glutCreateWindow("CrossingReconstruction");
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);//reshape 		
	glutMouseFunc(mouse);
	glutMotionFunc(mouseMotion);
	glutKeyboardFunc(keyboard);
	glutTimerFunc(40, RenderUpdate, 1);
	glutMainLoop();
	return 0;
}