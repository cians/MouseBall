#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <GL/glut.h>
void drawPlane(Eigen::Vector3f v, Eigen::Vector4f Plane, float gridSize, int gridNumb);
void drawLine(Eigen::Vector3f M0, Eigen::Vector3f M1);
void drawCameraModel(Eigen::Vector3f Cam_center, Eigen::Matrix3f Cam_M, float length, float Xoffset, float Yoffset);