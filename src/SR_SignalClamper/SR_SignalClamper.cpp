#include "SR_SignalClamper.h"
#include "../misc/PinWrapper.h"
#include <adtf_plugin_sdk.h>
using namespace adtf;

// Create filter shell
ADTF_FILTER_PLUGIN("SpaceRacer Signal Clamper", OID_SR_SIGNALCLAMPER, SR_SignalClamper)

SR_SignalClamper::SR_SignalClamper(const tChar *info) : cFilter(info),max_limit(180),min_limit(0)
{

    SetPropertyFloat("Max limit value", 180);
    SetPropertyFloat("Min limit value", 0);

}

SR_SignalClamper::~SR_SignalClamper()
{

}

tResult SR_SignalClamper::Init(adtf::cFilter::tInitStage stage, ucom::IException **__exception_ptr)
{
	// call base implementation
	RETURN_IF_FAILED(cFilter::Init(stage, __exception_ptr))

	if (stage == StageFirst) {
		INIT_SIGNAL_VALUE_PIN(&inPin,"Input");
		INIT_SIGNAL_VALUE_PIN(&outPin,"Clamped_output");
	} else if (stage == StageNormal) {
        max_limit = GetPropertyFloat("Max limit value",180);
        min_limit = GetPropertyFloat("Min limit value", 0);
	} else if (stage == StageGraphReady) {

	}
	RETURN_NOERROR;
}

tResult SR_SignalClamper::Shutdown(adtf::cFilter::tInitStage stage, ucom::IException **__exception_ptr)
{
	if (stage == StageGraphReady) {

	} else if (stage == StageNormal) {

	} else if (stage == StageFirst) {

	}

	// call base implementation
	return cFilter::Shutdown(stage, __exception_ptr);
}

tResult SR_SignalClamper::OnPinEvent(adtf::IPin *source, tInt eventCode, tInt nParam1, tInt nParam2, adtf::IMediaSample *mediaSample)
{
    RETURN_IF_POINTER_NULL(source);
    RETURN_IF_POINTER_NULL(mediaSample);
    if(eventCode == IPinEventSink::PE_MediaSampleReceived && source == &inPin){
		tFloat32 input = 0;
        tUInt32 timestamp;
        RETURN_IF_FAILED(SignalValuePin::getValue(&inPin,mediaSample,&input,&timestamp));

        if(input > max_limit)
            input = max_limit;
        if(input < min_limit)
            input = min_limit;

		SEND_SIGNAL_VALUE_SAMPLE(&outPin,input,timestamp);
    }

	RETURN_NOERROR;
}
