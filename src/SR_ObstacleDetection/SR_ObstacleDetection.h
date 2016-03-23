#ifndef _SR_ObstacleDetection_H
#define _SR_ObstacleDetection_H

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace adtf;
using namespace std;
using namespace cv;
#include "../misc/PinWrapper.h"
#include <map_data.h>

#define OID_SR_ObstacleDetection "sr.obstacledetection"


class SR_ObstacleDetection : public adtf::cFilter
{
        ADTF_FILTER(OID_SR_ObstacleDetection, "SpaceRacer Obstacle Detection", adtf::OBJCAT_DataFilter)

public:
        SR_ObstacleDetection(const tChar *info);
        virtual ~SR_ObstacleDetection();

private:
        float valueFilter(int *counter, std::vector<float> *data, float value);
        void calcUnderground();
private:
        typedef struct{
            int counter_front_left;
            int counter_center_left;
        } us_counter;

        typedef struct{
            int start_x;
            int start_y;
            int width;
            int height;
        } depth_areas;
        depth_areas straight_area,right_area;
        tBitmapFormat rgbFormat,depthFormat;
        bool m_bFirstFrameDepth,m_bFirstFrameRGB,debug;
        double avgsum_right;
        double avgsum_straight;
        double offset_depth;
        double range_front_left;
        double range_center_left;
        Scalar color_straight,color_right;
        BlockedSpots blspots;
        us_counter counters;
        std::vector<std::vector<float> > us_datas;

protected:
    WrappedSVInputPin us_front_left, us_center_left;
    adtf::cVideoPin camera_depth,depth_output, rgb_input,rgb_output;
    adtf::cOutputPin outputBlockedSpots;
    tResult Init(tInitStage stage, __exception);
    tResult Shutdown(tInitStage stage, __exception);
    tResult ProcessDepth(adtf::IMediaSample *pSample);
    tResult PropertyChanged(const tChar *strName);
    tResult OnPinEvent(adtf::IPin *source, tInt eventCode, tInt param1, tInt param2, adtf::IMediaSample *mediaSample);
};


#endif
