#ifndef _SR_SIGNAL_CLAMPER_H
#define _SR_SIGNAL_CLAMPER_H

#include <adtf_plugin_sdk.h>

#define OID_SR_SIGNALCLAMPER "sr.watchdog.signalclamper"


class SR_SignalClamper : public adtf::cFilter
{
	ADTF_FILTER(OID_SR_SIGNALCLAMPER, "SpaceRacer Signal Clamper", adtf::OBJCAT_DataFilter)

public:
	SR_SignalClamper(const tChar *info);
	virtual ~SR_SignalClamper();

private:
    adtf::cInputPin inPin;
    adtf::cOutputPin outPin;
    tFloat32 max_limit,min_limit;

protected:
	tResult Init(tInitStage stage, __exception);
	tResult Shutdown(tInitStage stage, __exception);

	tResult OnPinEvent(adtf::IPin *source, tInt eventCode, tInt nParam1, tInt nParam2, adtf::IMediaSample *mediaSample);
};


#endif
