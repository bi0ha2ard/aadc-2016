#include <cmath>
#include "InitialStateDetector.h"
#include "../misc/PinWrapper.h"
#include <maneuverdata.h>

using namespace adtf;


ADTF_FILTER_PLUGIN("SpaceRacer Initial State Detector", OID_ISD, InitialStateDetector)

vector<cString> names;

InitialStateDetector::InitialStateDetector(const tChar *info)
{
	done = false;
	bufferDepth = 16;
	SetPropertyInt("buffer_depth", bufferDepth);
	SetPropertyInt("buffer_depth" NSSUBPROP_MIN, 2);
	samplesToEvaluate = 32;
	SetPropertyInt("Samples to evaluate", samplesToEvaluate);
	SetPropertyInt("Samples to evaluate" NSSUBPROP_MIN, 16);
	names.insert(names.end(), "Front");
	names.insert(names.end(), "Rear");
	names.insert(names.end(), "Left");
	names.insert(names.end(), "Right");
	for (int i = 0; i < 4; ++i) {
		SetPropertyFloat(cString::Format("Threshold %s", names[i].GetPtr()), 0.4f);
	}
}

InitialStateDetector::~InitialStateDetector()
{

}

tResult InitialStateDetector::Init(cFilter::tInitStage eStage, IException **__exception_ptr)
{
	// call base implementation
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

	if (eStage == StageFirst) {
		// Output pin
		REGISTER_MEDIA_PIN(outputPin, MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_UINT8, "Initial_State");

		//create pins for US input
		inputs.resize(4);
		inputs[0].pin = &frontPin;
		inputs[1].pin = &rearPin;
		inputs[2].pin = &leftPin;
		inputs[3].pin = &rightPin;
		for (size_t i = 0; i < inputs.size(); ++i) {
			INIT_WRAPPED_PIN((*inputs[i].pin), names[i]);
		}
		REGISTER_MEDIA_PIN(resetPin, MEDIA_TYPE_USER, 0, "Reset");
	} else if (eStage == StageNormal) {
		bufferDepth = static_cast<size_t>(GetPropertyInt("buffer_depth", bufferDepth));
		samplesToEvaluate = static_cast<size_t>(GetPropertyInt("Samples to evaluate", samplesToEvaluate));
		for (size_t i = 0; i < inputs.size(); ++i) {
			inputs[i].hist.resize(bufferDepth);
			inputs[i].pos = 0;
			inputs[i].sampleCount = 0;
			inputs[i].threshold = GetPropertyFloat(cString::Format("Threshold %s", names[i].GetPtr()), 10.0f);
		}
		done = false;
	} else if (eStage == StageGraphReady) {
		done = false;
	}
	RETURN_NOERROR;
}

tResult InitialStateDetector::Shutdown(cFilter::tInitStage eStage, IException **__exception_ptr)
{
	return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult InitialStateDetector::OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample)
{
	// Check pointers
	RETURN_IF_POINTER_NULL(pSource);
	RETURN_IF_POINTER_NULL(pMediaSample);

	if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
		if (pSource == &resetPin) {
			done = false;
			for (size_t i = 0; i < inputs.size(); ++i) {
				inputs[i].sampleCount = 0;
				inputs[i].pos = 0;
			}
		} else if (!done) {
			for (size_t i = 0; i < inputs.size(); ++i) {
				if (pSource == inputs[i].pin->ppin) {
					tFloat32 theValue = 0;
					inputs[i].pin->getValue(pMediaSample, &theValue);
					++inputs[i].sampleCount;
					inputs[i].hist[inputs[i].pos % bufferDepth] = theValue;
					// evaluate here
					tFloat32 sum = 0;
					for (size_t j = 0; j < bufferDepth; ++j) {
						sum += inputs[i].hist[j];
					}
					++inputs[i].pos;
					inputs[i].pos %= bufferDepth;
					inputs[i].currAvg = sum / fmin(inputs[i].sampleCount, bufferDepth);
					RETURN_IF_FAILED(checkState());
				}
			}
		}
	}
	RETURN_NOERROR;
}

tResult InitialStateDetector::checkState()
{
	for (size_t i = 0; i < inputs.size(); ++i) {
		if (inputs[i].sampleCount < samplesToEvaluate) {
			RETURN_NOERROR;
		}
	}
	done = true;
	// Get current state and send it
	InitialCarConfiguration c = InitialConfigurationTrack;
	if (!inputs[2].occupied()) {
		c = InitialConfigurationParkedParallel;
	} else if (!inputs[0].occupied()) {
		c = InitialConfigurationParkedCross;
	}
	uint8_t val = static_cast<uint8_t>(c);
	SEND_MEDIA_SAMPLE(outputPin, uint8_t, val, _clock->GetStreamTime());
	RETURN_NOERROR;
}
