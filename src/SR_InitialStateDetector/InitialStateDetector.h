#ifndef _EMERGENCY_BREAKING_H_
#define _EMERGENCY_BREAKING_H_

#include <adtf_plugin_sdk.h>
#include "../misc/PinWrapper.h"

#define OID_ISD "sr.startup.initialstatedetector"

class InitialStateDetector : public adtf::cFilter
{
	ADTF_FILTER(OID_ISD, "SpaceRacer Initial State Detector", adtf::OBJCAT_DataFilter)

public:
	InitialStateDetector(const tChar *info);
	virtual ~InitialStateDetector();

protected:
	tResult Init(tInitStage eStage, IException **__exception_ptr);
	tResult Shutdown(tInitStage eStage, IException **__exception_ptr);
	tResult OnPinEvent(adtf::IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, adtf::IMediaSample *pMediaSample);

private:
	tResult checkState();

	adtf::cOutputPin outputPin;
	WrappedSVInputPin frontPin;
	WrappedSVInputPin rearPin;
	WrappedSVInputPin leftPin;
	WrappedSVInputPin rightPin;
	adtf::cInputPin resetPin;

	size_t bufferDepth;
	size_t samplesToEvaluate;

	typedef struct {
		WrappedSVInputPin *pin;
		vector<tFloat32> hist;
		size_t pos;
		size_t sampleCount;
		tFloat32 threshold;
		double currAvg;

		bool occupied()
		{
			return currAvg < threshold;
		}
	} Filter;
	vector<InitialStateDetector::Filter> inputs;

	bool done;
};



#endif
