#ifndef VISUALIZATION_MAP_DATA_H
#define VISUALIZATION_MAP_DATA_H

#include <Eigen/Eigen>
#ifdef _WIN32
#include <algorithm>
#undef min
#undef max
#endif
#include <stdio.h>
typedef unsigned int uint;
namespace Visualization
{

typedef float FLOAT;
typedef Eigen::Matrix4f Mat4f;
typedef Eigen::Matrix3f Mat3f;
typedef Eigen::Vector4f Vec4f;
typedef Eigen::Vector3f Vec3f;
typedef Eigen::Vector2f Vec2f;
typedef Eigen::VectorXf Descriptor;

typedef uint Index;

struct IntrinsicMatrix
{
    Vec4f data;

    const FLOAT& fx(){ return data[0]; }
    const FLOAT& fy(){ return data[1]; }
    const FLOAT& cx(){ return data[2]; }
    const FLOAT& cy(){ return data[3]; }

    Mat3f GetMat3x3() {
        Mat3f m3f;
        m3f << fx(), 0.0f , cx(),
               0.0f, fy(),  cy(),
               0.0f, 0.0f,  1.0f;
        return m3f;
    }

    void Load(FILE* fp)
    {
        fscanf_s(fp,
               "%f %f %f %f",
               &data[0], &data[1], &data[2], &data[3]);
    }
};

struct Measurement
{
    Vec2f m_img;//coordinate in image
    Index m_cam_index;
    Index m_map_point_index;

    void Load(FILE* fp)
    {
        fscanf_s(fp, "%u %f %f",
                &m_cam_index,
                &m_img[0],
                &m_img[1]
                );
    }
};

typedef std::vector<const Measurement*> MeasureList;

struct Pose
{
    //Mat4f m4f;
    Mat3f m_rotation;
    Vec3f m_translation;
    Vec3f center;
    void Load(FILE* fp)
    {
        fscanf_s(fp,
               "%f %f %f %f"
               "%f %f %f %f"
               "%f %f %f %f",
               &m_rotation(0,0), &m_rotation(0,1), &m_rotation(0,2), &m_translation(0),
               &m_rotation(1,0), &m_rotation(1,1), &m_rotation(1,2), &m_translation(1),
               &m_rotation(2,0), &m_rotation(2,1), &m_rotation(2,2), &m_translation(2)
               );
        center = - m_rotation.transpose() * m_translation;
    }

    Vec3f GetCenter() const
    {
        return center;
    }

    void Print()
    {
        fprintf(stdout,
               "%g %g %g %g\n"
               "%g %g %g %g\n"
               "%g %g %g %g\n",
               m_rotation(0,0), m_rotation(0,1), m_rotation(0,2), m_translation(0),
               m_rotation(1,0), m_rotation(1,1), m_rotation(1,2), m_translation(1),
               m_rotation(2,0), m_rotation(2,1), m_rotation(2,2), m_translation(2)
               );
    }
};

struct Camera
{
    FLOAT m_focal;
    Index m_index;
    Pose m_pose;
    MeasureList m_measures;
    void Load(FILE* fp)
    {
        fscanf_s(fp, "%f", &m_focal);
        m_pose.Load(fp);
    }

    const Pose& GetPose() const {return m_pose;}
};

struct MapPoint
{
    int flag1, flag2;
    int mn_frame;
    MeasureList m_measures;
    Vec3f m_3d_point;
    Descriptor m_desc;

    void Load(FILE* fp)
    {
        //Load flag
        fscanf_s(fp, "%d %d %d",
                &mn_frame,
                &flag1,
                &flag2
                );

        //Load 3D position
        fscanf_s(fp, "%f %f %f",
                &m_3d_point[0],
                &m_3d_point[1],
                &m_3d_point[2]
                );

    }

    void Load(FILE* fp, int desc_len)
    {
        //Load desc
        if (desc_len > 0)
        {
            m_desc.resize(desc_len);
            for (int i = 0; i < desc_len; ++i)
                fscanf_s(fp, "%f", &m_desc[i]);
        }
    }
};

}

#endif
