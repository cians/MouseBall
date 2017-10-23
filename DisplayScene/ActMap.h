#ifndef VISUALZATION_ACT_MAP_H
#define VISUALZATION_ACT_MAP_H

#include "MapData.h"
#include "Utility.h"

namespace Visualization {

const int MAX_LINE_LENGTH = 512;
struct ActMap
{
protected:
    char buffer[MAX_LINE_LENGTH];

    std::string m_base_dir;
    std::string m_base_image;//full path
    //
    std::vector<std::string> mv_image_file_names;

    //core data about the map
    IntrinsicMatrix K;

    std::vector<Camera> mv_cameras;
    std::vector<MapPoint> mv_map_points;
    std::vector<Measurement> mv_measurements;


    //Locate some XML label
    int LocateContext(FILE* fp, const std::string& label)
    {
        const size_t label_len = label.length();
        while(fgets(buffer, MAX_LINE_LENGTH - 1, fp))
        {
            size_t len = strlen(buffer);
            if (len < label_len)
                continue;
            buffer[label_len] = '\0';//remove the EOL of \n
            if (label == buffer)
                return 1;
        }
        return 0;
    }

    int LoadCameras(FILE* fp)
    {
        //Read all frame parameters
        LocateContext(fp, "<Camera Track>");
        while(!feof(fp))
        {
            LocateContext(fp, "<FRAME");
            Camera frame;
            frame.Load(fp);
            mv_cameras.push_back(frame);
        }
        return mv_cameras.size();
    }

    int LoadMapPoints(FILE* fp)
    {
        //Locate to Feature Track
        LocateContext(fp, "<Feature Tracks>");

        //Read feature descriptor tag such as (kpts: 64)
        fgets(buffer, MAX_LINE_LENGTH - 1, fp);
        int total_3d_points = 0;
        int desc_len = 0;//len of descriptor feature, 0 means no descriptor
        if (buffer[0] == 'k')//kpts: 64
        {
            sscanf_s(buffer, "kpts: %d", &desc_len);
            fscanf_s(fp, "%d", &total_3d_points);
        }
        else
            sscanf_s(buffer, "%d", &total_3d_points);

        //Read all 3d points
        for (int i_pt = 0; i_pt < total_3d_points; ++i_pt)
        {
            MapPoint pt;
            pt.Load(fp);//Load 3D coordinate

            //Load all measurement of this MapPoint
            for (int i_meas = 0; i_meas < pt.mn_frame; ++i_meas)
            {
                Measurement meas;
                meas.Load(fp);
                meas.m_map_point_index = i_pt;
                mv_measurements.push_back(meas);
            }

            //Load descriptor
            pt.Load(fp, desc_len);
            mv_map_points.push_back(pt);
        }
        return mv_map_points.size();
    }

    // assignment measure for Cameras and MapPoints
    int AssignmentMeasurement()
    {
        for (uint i_meas = 0; i_meas < mv_measurements.size();
             ++i_meas)
        {
            const Measurement& meas = mv_measurements[i_meas];
            mv_cameras[meas.m_cam_index].m_measures.push_back(&meas);
            mv_map_points[meas.m_map_point_index].m_measures.push_back(&meas);
        }
        return 0;
    }

    int LoadImageNames(FILE* fp)
    {
        LocateContext(fp, "<Image Sequence>");
        fscanf_s(fp, "Sequence:%s\n", buffer);
        std::string file_name = IO::RemoveFileDirectory(buffer);
        m_base_image = m_base_dir + file_name;

        int i_start = 0, i_step = 0, i_end = 0;
        fscanf_s(fp, "start:%d\n", &i_start);
        fscanf_s(fp, "step:%d\n", &i_step);
        fscanf_s(fp, "end:%d\n", &i_end);

        mv_image_file_names.clear();
        for (int i_img = i_start; i_img <= i_end; i_img += i_step)
        {
             mv_image_file_names.push_back(
                  IO::IncreaseFileNumber(m_base_image, i_img));
        }
    }

    int LoadAct(FILE* fp)
    {
        //LoadImageNames(fp);
        LoadMapPoints(fp);
        LoadCameras(fp);
        AssignmentMeasurement();

        printf("Act Load done\n%d Frames %d 3D MapPonits Loaded\n",
               (int)mv_cameras.size(),
               (int)mv_map_points.size());

        return 0;
    }

public:

    size_t CameraCount() const {return mv_cameras.size();}

    size_t MapPointCount() const {return mv_map_points.size();}

    const Camera& GetCamera(const int index) const {return mv_cameras[index];}

    const MapPoint& GetMapPoint(const int index) const {return mv_map_points[index];}

    const MeasureList& GetCameraMeasures(const int index) const {return mv_cameras[index].m_measures;}

    const MeasureList& GetMapPointMeasures(const int index) const {return mv_map_points[index].m_measures;}

    const std::string& GetImagePath(const int index) const { return mv_image_file_names[index];}

    int LoadAct(const std::string& file_path)
    {
        m_base_dir = IO::ExtractFileDirectory(file_path);
        FILE* fp = fopen(file_path.c_str(), "r");
        LoadAct(fp);
        fclose(fp);
        return 0;
    }

    //convert to obj file format
    int SaveObjFile(const std::string& file_name)
    {
        FILE* fp = fopen(file_name.c_str(), "w");

//        for (uint i = 0; i < CameraCount(); ++i)
//        {
//            const Vec3f& center = GetCamera(i).GetPose().GetCenter();
//            fprintf(fp, "v %f %f %f\n", center[0], center[1], center[2]);
//        }
        for (uint i = 0; i < mv_map_points.size(); ++i)
        {
            const Vec3f& v3_pos = mv_map_points[i].m_3d_point;
            fprintf(fp, "v %f %f %f\n", v3_pos[0], v3_pos[1], v3_pos[2]);
        }

        fclose(fp);
    }

};

}

#endif
