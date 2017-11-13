#include "Source.h"
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
	v3Mean /= (float)maps.CameraCount();//法向量起点
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
	//开启zbuffer
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

	glMatrixMode(GL_PROJECTION);//Applies subsequent matrix operations to the projection matrix stack
	glLoadIdentity();
	glOrtho(-ViewRange , ViewRange, -ViewRange, ViewRange, -ViewRange, ViewRange);//glOrtho(left, right, bottom, top, near, far)

	glMatrixMode(GL_MODELVIEW);//Applies subsequent matrix operations to the modelview matrix stack.
	glLoadIdentity();//重置当前Modelview矩阵
	glPushMatrix();                                                    // NEW: Prepare Dynamic Transform
	glMultMatrixf(Transform.M);                                        // NEW: Apply Dynamic Transform
	glRotatef(270, 1.0f, 0.0f, 0.0f);      //glRotatef(angle,x,y,z),angle是角度值，x,y,z是布尔值
	glRotatef(90, 0.0f, 1.0f, 0.0f);
	glScalef(2.5, 2.5, 2.5);

	// draw
	glColor4ub(255, 255, 255,255);
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);

	glVertexPointer(3, GL_FLOAT, sizeof(Point), &points[0].worldPosition(0));//依次是(坐标个数，type，stride，首元素地址)，后面坐标是3！
	glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(Point), &points[0].r);
	glLineWidth(pSize - 2);
	glPointSize(pSize);
	glDrawArrays(GL_POINTS, 0, drawPointsNum);//动态改变绘制点数
	drawCameraModel(points[drawPointsNum].worldPosition, maps.GetCamera(drawPointsNum).m_pose.m_rotation,Model_Len, Model_Xoffset, Model_Yoffset);

	if (sigPoints.size() > 0)
	{
		glVertexPointer(3, GL_FLOAT, sizeof(SignalPoint), &sigPoints[0].worldPosition(0));//依次是(坐标个数，type，stride，首元素地址)，后面坐标是3！
		glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(SignalPoint), &sigPoints[0].r);
		glPointSize(lightSize);
		glDrawArrays(GL_POINTS, 0, sigPoints.size());
	}
	//draw plane
	//平面方程Ax + By + Cz + D= 0, 但向下平移相机到路面的距离
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
			if (frameIndex >(pd.inputSigInfo[i].startFrame + 1))//两帧以上才能估出深度
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
				sigPoints.push_back(p);//加入tracking 队列
			}
		}
	}
	//SignalId 为 目前track的帧数。

	for (auto tsig = sigPoints.begin();tsig != sigPoints.end();tsig++)
	{
		//只能用历史帧信息frameIndex - measure[0].m_cam_index
		//每帧灯2d坐标 measure[i].m_img
		//camera project matrix:Getcamera(MID),r,
		//只更改正在tracking的灯的XYZ,tracked 的直接用act文件的3d 坐标
		if (maps.GetMapPointMeasures(tsig->MID)[maps.GetMapPointMeasures(tsig->MID).size() - 1]->m_cam_index < frameIndex)
		{
			tsig->worldPosition = tsig->mapPoint.m_3d_point;//如果track完成
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
		//一帧帧显示图像
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
		//imshow时间太短，movewindow好像没用，只好暂停手动调整下窗口位置便于录屏。
		if (drawPointsNum < 1)
			cv::waitKey(7000);
		drawPointsNum++;
		drawSignal(drawPointsNum);
		display();//画路面和相机轨迹
		glutTimerFunc(40, RenderUpdate, 1);//需要在函数中再调用一次保持循环
	}
}
int main(int argc, char **argv)
{
	//内参
	INTRINSIC <<
		1941.45, 0, 957.207,
		0, 1945.9, 611.806,
		0, 0, 1;
	//用于显示的参数
	translateScale = stof(pd.getData("translateScale"));
	ScaleFactor = stof(pd.getData("ScaleFactor"));
	int width = stoi(pd.getData("width"));
	int height = stoi(pd.getData("height"));
	ArcBall.setBounds(width, height);
	unsigned char cam_r = (unsigned char)stoi(pd.getData("cam_r"));
	unsigned char cam_g = (unsigned char)stoi(pd.getData("cam_g"));
	unsigned char cam_b = (unsigned char)stoi(pd.getData("cam_b"));
	unsigned char cam_a = (unsigned char)stoi(pd.getData("cam_a"));
	//读取label，imgSequence
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

	//结合map读取的act文件完善signal信息
	for (size_t i = 0; i < pd.inputSigInfo.size(); ++i)
	{
		size_t MID = pd.inputSigInfo[i].MapPointId;
		pd.inputSigInfo[i].startFrame = maps.GetMapPointMeasures(MID)[0]->m_cam_index;
		pd.inputSigInfo[i].endFrame = maps.GetMapPointMeasures(MID)[maps.GetMapPointMeasures(MID).size() - 1]->m_cam_index;
		pd.inputSigInfo[i].worldPosition = maps.GetMapPoint(MID).m_3d_point;
	}
	//是否绘制其它特征点
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
	//相机轨迹
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
	//创建窗口
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