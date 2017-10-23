#ifndef VISUALZATION_MAP_H
#define VISUALZATION_MAP_H

#include "ActMap.h"

namespace Visualization
{

class Map
{
protected:

    //maybe multiple act files, default we only process the first one
    std::vector<ActMap*> mv_acts;

public:
    Map() {}

    ~Map() {
        for (uint i = 0; i < mv_acts.size(); ++i)
            delete mv_acts[i];
    }

    size_t CameraCount(const int seq_index = 0) const {return mv_acts[seq_index]->CameraCount();}

    size_t MapPointCount(const int seq_index = 0) const {return mv_acts[seq_index]->MapPointCount();}

    const Camera& GetCamera(const int index, const int seq_index = 0) const {return mv_acts[seq_index]->GetCamera(index);}

    const MapPoint& GetMapPoint(const int index, const int seq_index = 0) const {return mv_acts[seq_index]->GetMapPoint(index);}

    const MeasureList& GetCameraMeasures(const int index, const int seq_index = 0) const {return mv_acts[seq_index]->GetCameraMeasures(index);}

    const MeasureList& GetMapPointMeasures(const int index, const int seq_index = 0) const {return mv_acts[seq_index]->GetMapPointMeasures(index);}

    const std::string& GetImagePath(const int index, const int seq_index = 0) const { return mv_acts[seq_index]->GetImagePath(index);}

    int LoadAct(const std::string& file_name)
    {
        ActMap* new_act = new ActMap;
        new_act->LoadAct(file_name);
        mv_acts.push_back(new_act);
        return 0;
    }

    int LoadAct(const std::vector<std::string>& file_names)
    {
        for (uint i = 0; i < file_names.size(); ++i)
        {
            ActMap* new_act = new ActMap;
            new_act->LoadAct(file_names[i]);
            mv_acts.push_back(new_act);
        }
        return 0;
    }

    size_t SeqCount() const {return mv_acts.size();}

};
}
#endif
