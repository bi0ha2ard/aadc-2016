#include "SR_ParkingFinder.h"

using namespace adtf;


// Create filter shell
ADTF_FILTER_PLUGIN("SpaceRacer Parking Finder", OID_SR_ParkingFinder, SR_ParkingFinder)

#define PROP_SLOT_PAR "Parallel slot length"
#define PROP_SLOT_CROSS "Cross slot length"
#define PROP_FREE_DIST "Min distance considered free"

#undef LOG_INFO
#define LOG_INFO(str) logger.GLogTrap((str));


SR_ParkingFinder::SR_ParkingFinder(const tChar *info) : cFilter(info),logger(OID_SR_ParkingFinder)
{
	currMode = Ready;
	currPosition = Point2d(0,0);
	active = false;
    counter = 0;
    dist_to_object = 0;
	currSlotLength = 0;
	parallelSlotLength = .8;
	crossSlotLength = .5;
	depthLimit = .4;
	SetPropertyFloat(PROP_SLOT_CROSS, crossSlotLength);
	SetPropertyFloat(PROP_SLOT_PAR, parallelSlotLength);
	SetPropertyFloat(PROP_FREE_DIST, depthLimit);
}

SR_ParkingFinder::~SR_ParkingFinder()
{

}

tResult SR_ParkingFinder::final_calc(double dist)
{
	LOG_INFO(cString::Format("Distance was %g", dist));
	double temp = dist;
	if (temp < .3) {
		currMode = FindStart;
		LOG_INFO("Garbage, restarting");
		RETURN_NOERROR;
	}
	if ((temp > crossSlotLength + .1 && typeToFind == ParkingTypeParallel) || (temp > crossSlotLength - .1 && typeToFind == ParkingTypeCross)) {
		currMode = Ready;
		SEND_WRAPPED_SAMPLE(searchResultPin, (temp / 2.0), _clock->GetStreamTime());
		SEND_MEDIA_SAMPLE(referencePoseOut, OdometryPose, currPose, _clock->GetStreamTime());
	}
	RETURN_NOERROR;
}

tResult SR_ParkingFinder::Init(adtf::cFilter::tInitStage stage, ucom::IException **__exception_ptr)
{
	// call base implementation
	RETURN_IF_FAILED(cFilter::Init(stage, __exception_ptr));

	if (stage == StageFirst) {
		INIT_WRAPPED_PIN(ultrasonic_input,"usonic_input");
		INIT_WRAPPED_PIN(abortPin,"Abort");
		REGISTER_MEDIA_PIN(searchRequestPin,MEDIA_TYPE_MAPDATA, MEDIA_SUBTYPE_MAPDATA_PARKING_SEARCH_REQ,"Parking_Input");
		INIT_WRAPPED_PIN(searchResultPin,"Parking_Output");
		INIT_WRAPPED_PIN(blinkerRequestPin, "Blinker_On");
        REGISTER_MEDIA_PIN(pose_input,MEDIA_TYPE_ODOMETRY, MEDIA_SUBTYPE_ODOMETRY_POSE,"Pose_Input");
		REGISTER_MEDIA_PIN(referencePoseOut,MEDIA_TYPE_ODOMETRY, MEDIA_SUBTYPE_ODOMETRY_POSE,"Reference_Pose");
	} else if (stage == StageNormal) {
		currMode = Ready;
		counter = 0;
		uSonicFilter.clear();
		parallelSlotLength = GetPropertyFloat(PROP_SLOT_PAR, parallelSlotLength);
		crossSlotLength = GetPropertyFloat(PROP_SLOT_CROSS, crossSlotLength);
		depthLimit = GetPropertyFloat(PROP_FREE_DIST, depthLimit);
	} else if (stage == StageGraphReady) {

	}
	RETURN_NOERROR;
}

tResult SR_ParkingFinder::Shutdown(adtf::cFilter::tInitStage stage, ucom::IException **__exception_ptr)
{
	if (stage == StageGraphReady) {

	} else if (stage == StageNormal) {

	} else if (stage == StageFirst) {

	}

	// call base implementation
	return cFilter::Shutdown(stage, __exception_ptr);
}

tResult SR_ParkingFinder::OnPinEvent(adtf::IPin *source, tInt eventCode, tInt param1, tInt param2, adtf::IMediaSample *mediaSample)
{
	if (eventCode == IPinEventSink::PE_MediaSampleReceived) {
		RETURN_IF_POINTER_NULL(mediaSample);
		if(source == &searchRequestPin){
			ParkingSearchRequest s;
			{
				__adtf_sample_read_lock(mediaSample, ParkingSearchRequest, data);
				s = *data;
			}
			distanceToStart = s.dist;
			typeToFind = s.type;
			currMode = GoToStart;
			startPoint = currPosition;
			currSlotLength = typeToFind == ParkingTypeParallel ? parallelSlotLength : crossSlotLength;
			LOG_INFO("Going to start");
        }else if(source == &pose_input){
            __sample_read_lock(mediaSample, OdometryPose, data);
			currPosition = Point2d(data->x, data->y);
			currPose = *data;

			if (currMode == GoToStart) {
				//LOG_INFO("Going to start");
				double distTraveled = norm(currPosition - startPoint);
				if (distTraveled > distanceToStart) {
					LOG_INFO("Start reached");
					SEND_WRAPPED_SAMPLE(blinkerRequestPin, true, _clock->GetStreamTime());
					currMode = FindStart;
					startPoint = currPosition;
				}
			}
		}else if(source == ultrasonic_input.ppin){
			if (currMode != FindStart && currMode != FindEnd) {
				RETURN_NOERROR;
			}
			tFloat32 temp = 0;
			ultrasonic_input.getValue(mediaSample, &temp);
			if (counter < 4) {
				uSonicFilter.push_back(temp);
			} else {
				uSonicFilter[counter % 4] = temp;
			}
			counter++;
			float sum = 0;
			for (size_t i = 0; i < uSonicFilter.size(); ++i) {
				sum += uSonicFilter[i];
			}
			dist_to_object = sum / static_cast<float>(uSonicFilter.size());
			//LOG_INFO(cString::Format("US: %g",dist_to_object));
			if (currMode == FindStart) {
				//LOG_INFO("Searching start");
				if (dist_to_object > depthLimit) {
					LOG_INFO("Start found");
					startPoint = currPosition;
					currMode = FindEnd;
				}
			} else if (currMode == FindEnd) {
				//LOG_INFO("Searching end");
				if (dist_to_object < depthLimit || norm(currPosition - startPoint) >= currSlotLength) {
					if (dist_to_object < depthLimit) {
						LOG_INFO("End found with ultrasonics");
					} else {
						LOG_INFO("End found with distance");
					}
					// updates state
					final_calc(norm(currPosition - startPoint));
				}
			}
		}else if(source == abortPin.ppin){
			currMode = Ready;
			LOG_INFO("Parking search aborted");

        }
	}

	RETURN_NOERROR;
}
