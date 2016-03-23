#include "SR_ObstacleDetection.h"

#include <adtf_plugin_sdk.h>
using namespace adtf;

// Create filter shell
ADTF_FILTER_PLUGIN("SpaceRacer Obstacle Detection", OID_SR_ObstacleDetection, SR_ObstacleDetection)

#define PROP_TOPLEFT_SUM "Avg sum  Straight"
#define PROP_TOPRIGHT_SUM "Avg sum right"
#define PROP_DEBUG  "Debug output"
#define PROP_DEPTH "Offset for depth camera"
#define PROP_START_LEFT_X "Straight area start x"
#define PROP_START_LEFT_Y "Straight area start y"
#define PROP_LEFT_WIDTH "Straight area width"
#define PROP_LEFT_HEIGHT "Straight area height"

#define PROP_START_RIGHT_X "Right area start x"
#define PROP_START_RIGHT_Y "Right area start y"
#define PROP_RIGHT_WIDTH "Right area width"
#define PROP_RIGHT_HEIGHT "Right area height"

SR_ObstacleDetection::SR_ObstacleDetection(const tChar *info) : cFilter(info)
{
    counters.counter_center_left = 0;
    counters.counter_front_left = 0;

    offset_depth = 200;
    us_datas.push_back(std::vector<float>());
    us_datas.push_back(std::vector<float>());


	blspots.straight_lane = false;
	blspots.right_lane = false;
    blspots.left_lane = false;
    straight_area.start_x = 80;
    straight_area.start_y = 20;
    straight_area.height = 80;
    straight_area.width = 150;

    color_right = Scalar(255,255,255);
    color_straight = Scalar(255,255,255);
    right_area.start_x = 460;
    right_area.start_y = 20;
    right_area.height = 100;
    right_area.width = 180;
    range_center_left = 100;
    range_front_left = 100;

    m_bFirstFrameDepth = true;
    m_bFirstFrameRGB = true;
    debug = false;
    avgsum_straight = 1750;
    avgsum_right = 1550;

    SetPropertyFloat(PROP_TOPLEFT_SUM,avgsum_straight);
    SetPropertyBool(PROP_TOPLEFT_SUM NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyFloat(PROP_TOPRIGHT_SUM,avgsum_right);
    SetPropertyBool(PROP_TOPRIGHT_SUM NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyBool(PROP_DEBUG,debug);
    SetPropertyBool(PROP_DEBUG NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyFloat(PROP_DEPTH,offset_depth);
    SetPropertyBool(PROP_DEPTH NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt(PROP_START_RIGHT_X,right_area.start_x);
    SetPropertyBool(PROP_START_RIGHT_X NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt(PROP_START_RIGHT_Y,right_area.start_y);
    SetPropertyBool(PROP_START_RIGHT_Y NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt(PROP_RIGHT_HEIGHT,right_area.height);
    SetPropertyBool(PROP_RIGHT_HEIGHT NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt(PROP_RIGHT_WIDTH,right_area.width);
    SetPropertyBool(PROP_RIGHT_WIDTH NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt(PROP_START_LEFT_X,straight_area.start_x);
    SetPropertyBool(PROP_START_LEFT_X NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt(PROP_START_LEFT_Y,straight_area.start_y);
    SetPropertyBool(PROP_START_LEFT_Y NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt(PROP_LEFT_HEIGHT,straight_area.height);
    SetPropertyBool(PROP_LEFT_HEIGHT NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt(PROP_LEFT_WIDTH,straight_area.width);
    SetPropertyBool(PROP_LEFT_WIDTH NSSUBPROP_ISCHANGEABLE, tTrue);

}

SR_ObstacleDetection::~SR_ObstacleDetection()
{

}

tResult SR_ObstacleDetection::PropertyChanged(const tChar *strName)
{
    if(cString::IsEqual(strName, PROP_TOPLEFT_SUM)){
        avgsum_straight = GetPropertyFloat(PROP_TOPLEFT_SUM);
    }else if(cString::IsEqual(strName, PROP_TOPRIGHT_SUM)){
        avgsum_right = GetPropertyFloat(PROP_TOPRIGHT_SUM);
    }else if(cString::IsEqual(strName,PROP_DEBUG)){
        debug = GetPropertyBool(PROP_DEBUG);
    }else if(cString::IsEqual(strName,PROP_DEPTH)){
        offset_depth = GetPropertyFloat(PROP_DEPTH);
    }else if(cString::IsEqual(strName,PROP_START_RIGHT_X)){
        right_area.start_x = GetPropertyInt(PROP_START_RIGHT_X);
    }else if(cString::IsEqual(strName,PROP_START_RIGHT_Y)){
        right_area.start_y = GetPropertyInt(PROP_START_RIGHT_Y);
    } else if(cString::IsEqual(strName,PROP_RIGHT_HEIGHT)){
        right_area.height = GetPropertyInt(PROP_RIGHT_HEIGHT);
    }else if(cString::IsEqual(strName,PROP_RIGHT_WIDTH)){
        right_area.width = GetPropertyInt(PROP_RIGHT_WIDTH);
    }else if(cString::IsEqual(strName,PROP_START_LEFT_X)){
        straight_area.start_x = GetPropertyInt(PROP_START_LEFT_X);
    }else if(cString::IsEqual(strName,PROP_START_LEFT_Y)){
        straight_area.start_y = GetPropertyInt(PROP_START_LEFT_Y);
    }else if(cString::IsEqual(strName,PROP_LEFT_HEIGHT)){
        straight_area.height = GetPropertyInt(PROP_LEFT_HEIGHT);
    }else if(cString::IsEqual(strName,PROP_LEFT_WIDTH)){
        straight_area.width = GetPropertyInt(PROP_LEFT_WIDTH);
    }
    RETURN_NOERROR;
}

float SR_ObstacleDetection::valueFilter(int *counter,std::vector<float> *data, float value)
{
    if (*counter < 4) {
        data->push_back(value);
    } else {
        data->at(*counter % 4) =  value;
    }
    *counter+=1;
    float sum = 0;
    for (size_t i = 0; i < data->size(); ++i) {
        sum += data->at(i);
    }
    return sum / static_cast<float>(data->size());
}



void SR_ObstacleDetection::calcUnderground()
{

}

tResult SR_ObstacleDetection::Init(adtf::cFilter::tInitStage stage, ucom::IException **__exception_ptr)
{
    // call base implementation
    RETURN_IF_FAILED(cFilter::Init(stage, __exception_ptr))

    if (stage == StageFirst) {
        INIT_WRAPPED_PIN(us_front_left,"us_front_left");
        INIT_WRAPPED_PIN(us_center_left,"us_center_left");
        REGISTER_MEDIA_PIN(outputBlockedSpots,MEDIA_TYPE_MAPDATA, MEDIA_SUBTYPE_MAPDATA_BLOCKEDSPOTS,"blocked_spots");
        RETURN_IF_FAILED(depth_output.Create("Camera_depth_output", IPin::PD_Output,static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(camera_depth.Create("Camera_depth_input",IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&depth_output));
        RETURN_IF_FAILED(RegisterPin(&camera_depth));

        RETURN_IF_FAILED(rgb_output.Create("rgb_output", IPin::PD_Output,static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(rgb_input.Create("rgb_input",IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&rgb_output));
        RETURN_IF_FAILED(RegisterPin(&rgb_input));

    } else if (stage == StageNormal) {
        avgsum_straight = GetPropertyFloat(PROP_TOPLEFT_SUM,avgsum_straight);
        avgsum_right = GetPropertyFloat(PROP_TOPRIGHT_SUM,avgsum_right);
        offset_depth = GetPropertyFloat(PROP_DEPTH,offset_depth);
        debug = GetPropertyBool(PROP_DEBUG,debug);
        straight_area.start_x = GetPropertyInt(PROP_START_LEFT_X,straight_area.start_x);
        straight_area.start_y = GetPropertyInt(PROP_START_LEFT_Y,straight_area.start_y);
        straight_area.height = GetPropertyInt(PROP_LEFT_HEIGHT,straight_area.height);
        straight_area.width = GetPropertyInt(PROP_LEFT_WIDTH,straight_area.width);

        right_area.start_x = GetPropertyInt(PROP_START_RIGHT_X,right_area.start_x);
        right_area.start_y = GetPropertyInt(PROP_START_RIGHT_Y,right_area.start_y);
        right_area.height = GetPropertyInt(PROP_RIGHT_HEIGHT,right_area.height);
        right_area.width = GetPropertyInt(PROP_RIGHT_WIDTH,right_area.width);

    } else if (stage == StageGraphReady) {

    }
    RETURN_NOERROR;
}

tResult SR_ObstacleDetection::Shutdown(adtf::cFilter::tInitStage stage, ucom::IException **__exception_ptr)
{
    if (stage == StageGraphReady) {

    } else if (stage == StageNormal) {

    } else if (stage == StageFirst) {

    }

    // call base implementation
    return cFilter::Shutdown(stage, __exception_ptr);
}

tResult SR_ObstacleDetection::ProcessDepth(IMediaSample *pSample)
{
    RETURN_IF_POINTER_NULL(pSample);

    const tVoid* l_pSrcBuffer;
    IplImage* oImg = cvCreateImageHeader(cvSize(depthFormat.nWidth, depthFormat.nHeight), IPL_DEPTH_16U,1);
    RETURN_IF_FAILED(pSample->Lock(&l_pSrcBuffer));
    oImg->imageData = (char*)l_pSrcBuffer;
    Mat image(cvarrToMat(oImg));
    circle(image,Point(120,120),10,Scalar(65000),10);
    cvReleaseImage(&oImg);
    pSample->Unlock(l_pSrcBuffer);
    Mat top_left = image( Rect(straight_area.start_x/2, straight_area.start_y/2, straight_area.width/2, straight_area.height/2));

    Mat top_right = image(Rect(right_area.start_x/2, right_area.start_y/2, right_area.width/2,right_area.height/2));


    if(countNonZero(top_left) == 0 || countNonZero(top_right) == 0)
        RETURN_NOERROR;

    double sum_top_left = sum(top_left)[0] /  countNonZero(top_left);
    double sum_top_right = sum(top_right)[0] / countNonZero(top_right);
    if(debug)
        LOG_INFO(cString::Format("Top_Left: %g , Top_Right: %g ",sum_top_left,sum_top_right));

    if((avgsum_straight - offset_depth) > sum_top_left ){
        if(debug)
            LOG_INFO(" straight blocked");
		blspots.straight_lane = true;
        color_straight = Scalar(0,0,255);
    }else{
		blspots.straight_lane = false;
        color_straight = Scalar(255,255,255);
    }
    if((avgsum_right - offset_depth) > sum_top_right ){
		blspots.right_lane = true;
        color_right = Scalar(0,0,255);
        if(debug)
            LOG_INFO(" right blocked");
    }else{
		blspots.right_lane = false;
        color_right = Scalar(255,255,255);
    }
	SEND_MEDIA_SAMPLE(outputBlockedSpots, BlockedSpots ,blspots, pSample->GetTime());
    RETURN_NOERROR;
}

tResult SR_ObstacleDetection::OnPinEvent(adtf::IPin *source, tInt eventCode, tInt param1, tInt param2, adtf::IMediaSample *mediaSample)
{
    if (eventCode == IPinEventSink::PE_MediaSampleReceived) {
        RETURN_IF_POINTER_NULL(mediaSample);
        tFloat32 temp = 0;
        if(source == &camera_depth){
            if (m_bFirstFrameRGB)
            {
                cObjectPtr<IMediaType> pType;
                RETURN_IF_FAILED(camera_depth.GetMediaType(&pType));
                cObjectPtr<IMediaTypeVideo> pTypeVideo;
                RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));
                const tBitmapFormat* pFormat = pTypeVideo->GetFormat();
                if (pFormat == NULL){
                    LOG_ERROR("No Bitmap information found on pin \"input\"");
                    RETURN_ERROR(ERR_NOT_SUPPORTED);
                }
                depthFormat.nPixelFormat = pFormat->nPixelFormat;
                depthFormat.nWidth = pFormat->nWidth;
                depthFormat.nHeight =  pFormat->nHeight;
                depthFormat.nBitsPerPixel = pFormat->nBitsPerPixel;
                depthFormat.nBytesPerLine = pFormat->nBytesPerLine;
                depthFormat.nSize = pFormat->nSize;
                depthFormat.nPaletteSize = pFormat->nPaletteSize;
                m_bFirstFrameDepth = false;
            }
             ProcessDepth(mediaSample);
        }
        if(source == &rgb_input){
            if (m_bFirstFrameRGB)
            {
                cObjectPtr<IMediaType> pType;
                RETURN_IF_FAILED(rgb_input.GetMediaType(&pType));
                cObjectPtr<IMediaTypeVideo> pTypeVideo;
                RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));
                const tBitmapFormat* pFormat = pTypeVideo->GetFormat();
                if (pFormat == NULL){
                    LOG_ERROR("No Bitmap information found on pin \"input\"");
                    RETURN_ERROR(ERR_NOT_SUPPORTED);
                }
                rgbFormat.nPixelFormat = pFormat->nPixelFormat;
                rgbFormat.nWidth = pFormat->nWidth;
                rgbFormat.nHeight =  pFormat->nHeight;
                rgbFormat.nBitsPerPixel = pFormat->nBitsPerPixel;
                rgbFormat.nBytesPerLine = pFormat->nBytesPerLine;
                rgbFormat.nSize = pFormat->nSize;
                rgbFormat.nPaletteSize = pFormat->nPaletteSize;
                m_bFirstFrameRGB = false;
                rgb_output.SetFormat(&rgbFormat,NULL);
            }

            const tVoid* l_pSrcBuffer;
            IplImage* oImg = cvCreateImageHeader(cvSize(rgbFormat.nWidth, rgbFormat.nHeight), IPL_DEPTH_8U, 3);
            RETURN_IF_FAILED(mediaSample->Lock(&l_pSrcBuffer));
            oImg->imageData = (char*)l_pSrcBuffer;
            Mat image(cvarrToMat(oImg));
            Rect a(straight_area.start_x,straight_area.start_y,straight_area.width,straight_area.height);
            Rect b(right_area.start_x,right_area.start_y,right_area.width,right_area.height);
            rectangle(image,a,color_straight);
            rectangle(image,b,color_right);
            IMediaSample *pNewSample;
            RETURN_IF_FAILED(_runtime->CreateInstance(OID_ADTF_MEDIA_SAMPLE, IID_ADTF_MEDIA_SAMPLE, (tVoid**) &pNewSample));
            RETURN_IF_FAILED(pNewSample->AllocBuffer(image.elemSize() * image.total()));
            RETURN_IF_FAILED(pNewSample->Update(0, image.data, image.elemSize() * image.total(), 0));
            rgb_output.Transmit(pNewSample);
            pNewSample->Unref();
            cvReleaseImage(&oImg);
            mediaSample->Unlock(l_pSrcBuffer);

        }else if(source == us_front_left.ppin) {
            us_front_left.getValue(mediaSample,&temp);
            range_front_left = valueFilter(&(counters.counter_front_left),&us_datas.at(0),temp);
            if(debug)
                LOG_INFO(cString::Format("US Front left: %g US Center Left: %g",range_front_left,range_center_left));
        }else if(source == us_center_left.ppin){
            us_center_left.getValue(mediaSample,&temp);
            range_center_left = valueFilter(&(counters.counter_center_left),&us_datas.at(1),temp);
        }
        if(range_center_left <1.4 || range_front_left < 1.4){
            blspots.left_lane = true;
        }else{
            blspots.left_lane = false;
        }

    }

    RETURN_NOERROR;
}
