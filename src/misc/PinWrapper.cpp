#include "PinWrapper.h"

using namespace adtf;
bool SignalValuePin::haveDesc = false;
cObjectPtr<adtf::IMediaTypeDescription> SignalValuePin::signalValueDescription = NULL;
cObjectPtr<adtf::IMediaType> SignalValuePin::mediaType = NULL;
adtf::tBufferID SignalValuePin::valueID = 0;
adtf::tBufferID SignalValuePin::timeStampID = 0;

tResult SignalValuePin::initPin(adtf::IPin *pin, IException **__exception_ptr)
{
	RETURN_IF_POINTER_NULL(pin);

	if (!signalValueDescription) {
		cObjectPtr<IMediaDescriptionManager> descManager;
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&descManager,__exception_ptr));
		//get description
		const tChar *strDescription = descManager->GetMediaDescription("tSignalValue");

		//check if exists
		RETURN_IF_POINTER_NULL(strDescription);

		// get media type
		mediaType = new cMediaType(0, 0, 0, "tSignalValue", strDescription, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// get the actual description
		RETURN_IF_FAILED(mediaType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&signalValueDescription));
	}

	RETURN_NOERROR;
}

tResult SignalValuePin::getValue(adtf::cInputPin *pin, adtf::IMediaSample *sample, tFloat32 *value, tUInt32 *timeStamp)
{
	RETURN_IF_POINTER_NULL(pin);
	RETURN_IF_POINTER_NULL(sample);
	__adtf_sample_read_lock_mediadescription(signalValueDescription, sample, coderInput);
	if (!haveDesc) {
		coderInput->GetID("f32Value", valueID);
		coderInput->GetID("ui32ArduinoTimestamp", timeStampID);
		haveDesc = true;
	}
	if (value) {
		coderInput->Get(valueID, (tVoid*)value);
	}
	if (timeStamp) {
		coderInput->Get(timeStampID, (tVoid*)timeStamp);
	}
	RETURN_NOERROR;
}

tResult SignalValuePin::setValue(cOutputPin *pin, cObjectPtr<IMediaSample> outSample, tFloat32 value, tUInt32 timestamp)
{
	cObjectPtr<IMediaSerializer> serializer;
	signalValueDescription->GetMediaSampleSerializer(&serializer);
	outSample->AllocBuffer(serializer->GetDeserializedSize());
	//write date to the media sample with the coder of the descriptor
	{
		__adtf_sample_write_lock_mediadescription(signalValueDescription, outSample, outCoder);

		// set the id if not already done
		if (!haveDesc) {
			outCoder->GetID("f32Value", valueID);
			outCoder->GetID("ui32ArduinoTimestamp", timeStampID);
			haveDesc = true;
		}

		// set value
		outCoder->Set(valueID, (tVoid*)&value);
		outCoder->Set(timeStampID,(tVoid*)&timestamp);
	}

	pin->Transmit(outSample);

	RETURN_NOERROR;
}

cObjectPtr<IMediaType> SignalValuePin::getMediaType()
{
	return mediaType;
}


bool BoolValuePin::haveDesc = false;
cObjectPtr<adtf::IMediaTypeDescription> BoolValuePin::boolValueDescription = NULL;
cObjectPtr<adtf::IMediaType> BoolValuePin::mediaType = NULL;
adtf::tBufferID BoolValuePin::boolID = 0;
adtf::tBufferID BoolValuePin::timeStampID = 0;

tResult BoolValuePin::initPin(IPin *pin, IException **__exception_ptr)
{
	RETURN_IF_POINTER_NULL(pin);

	if (!boolValueDescription) {
		cObjectPtr<IMediaDescriptionManager> descManager;
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&descManager,__exception_ptr));
		//get description
		const tChar *strDescription = descManager->GetMediaDescription("tBoolSignalValue");

		//check if exists
		RETURN_IF_POINTER_NULL(strDescription);

		// get media type
		mediaType = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescription, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// get the actual description
		RETURN_IF_FAILED(mediaType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&boolValueDescription));
	}

	RETURN_NOERROR;
}

tResult BoolValuePin::getValue(cInputPin *pin, IMediaSample *sample, tBool *value, tUInt32 *timeStamp)
{
	RETURN_IF_POINTER_NULL(pin);
	RETURN_IF_POINTER_NULL(sample);
	__adtf_sample_read_lock_mediadescription(boolValueDescription, sample, coderInput);
	if (!haveDesc) {
		coderInput->GetID("bValue", boolID);
		coderInput->GetID("ui32ArduinoTimestamp", timeStampID);
		haveDesc = true;
	}
	if (value) {
		coderInput->Get(boolID, (tVoid*)value);
	}
	if (timeStamp) {
		coderInput->Get(timeStampID, (tVoid*)timeStamp);
	}
	RETURN_NOERROR;
}

tResult BoolValuePin::setValue(cOutputPin *pin, cObjectPtr<IMediaSample> outSample, tBool value, tUInt32 timestamp)
{
	cObjectPtr<IMediaSerializer> serializer;
	boolValueDescription->GetMediaSampleSerializer(&serializer);
	outSample->AllocBuffer(serializer->GetDeserializedSize());
	//write date to the media sample with the coder of the descriptor
	{
		__adtf_sample_write_lock_mediadescription(boolValueDescription, outSample, outCoder);

		// set the id if not already done
		if (!haveDesc) {
			outCoder->GetID("bValue", boolID);
			outCoder->GetID("ui32ArduinoTimestamp", timeStampID);
			haveDesc = true;
		}

		// set value
		outCoder->Set(boolID, (tVoid*)&value);
		outCoder->Set(timeStampID,(tVoid*)&timestamp);
	 }

	pin->Transmit(outSample);

	RETURN_NOERROR;
}

cObjectPtr<IMediaType> BoolValuePin::getMediaType()
{
	return mediaType;
}

tResult WrappedPin::initPin(const char *name, IException **__exception_ptr)
{
	RETURN_IF_POINTER_NULL(ppin);

	if (!mediaTypeDescription) {
		cObjectPtr<IMediaDescriptionManager> descManager;
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&descManager,__exception_ptr));
		//get description
		const tChar *strDescription = descManager->GetMediaDescription(name);

		//check if exists
		RETURN_IF_POINTER_NULL(strDescription);

		// get media type
		mediaType = new cMediaType(0, 0, 0, name, strDescription, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// get the actual description
		RETURN_IF_FAILED(mediaType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&mediaTypeDescription));
	}

	RETURN_NOERROR;
}

cObjectPtr<IMediaType> WrappedPin::getMediaType()
{
	return mediaType;
}


tResult WrappedSVPin::initPin(IException **__exception_ptr)
{
	return WrappedPin::initPin("tSignalValue", __exception_ptr);
}

WrappedSVInputPin::WrappedSVInputPin()
{
	this->ppin = &pin;
}

tResult WrappedSVInputPin::getValue(IMediaSample *sample, tFloat32 *value, tUInt32 *timeStamp)
{
	RETURN_IF_POINTER_NULL(sample);
	__adtf_sample_read_lock_mediadescription(mediaTypeDescription, sample, coderInput);
	if (!haveDesc) {
		coderInput->GetID("f32Value", valueID);
		coderInput->GetID("ui32ArduinoTimestamp", timeStampID);
		haveDesc = true;
	}
	if (value) {
		coderInput->Get(valueID, (tVoid*)value);
	}
	if (timeStamp) {
		coderInput->Get(timeStampID, (tVoid*)timeStamp);
	}
	RETURN_NOERROR;
}

WrappedSVOutputPin::WrappedSVOutputPin()
{
	this->ppin = &pin;
}

tResult WrappedSVOutputPin::setValue(cObjectPtr<IMediaSample> outSample, tFloat32 value, tUInt32 timestamp)
{
	cObjectPtr<IMediaSerializer> serializer;
	mediaTypeDescription->GetMediaSampleSerializer(&serializer);
	outSample->AllocBuffer(serializer->GetDeserializedSize());
	//write date to the media sample with the coder of the descriptor
	{
		__adtf_sample_write_lock_mediadescription(mediaTypeDescription, outSample, outCoder);

		// set the id if not already done
		if (!haveDesc) {
			outCoder->GetID("f32Value", valueID);
			outCoder->GetID("ui32ArduinoTimestamp", timeStampID);
			haveDesc = true;
		}

		// set value
		outCoder->Set(valueID, (tVoid*)&value);
		outCoder->Set(timeStampID,(tVoid*)&timestamp);
	}

	ppin->Transmit(outSample);

	RETURN_NOERROR;
}

tResult WrappedBVPin::initPin(IException **__exception_ptr)
{
	return WrappedPin::initPin("tBoolSignalValue", __exception_ptr);
}

WrappedBVInputPin::WrappedBVInputPin()
{
	this->ppin = &pin;
}

tResult WrappedBVInputPin::getValue(IMediaSample *sample, tBool *value, tUInt32 *timeStamp)
{
	RETURN_IF_POINTER_NULL(ppin);
	RETURN_IF_POINTER_NULL(sample);
	__adtf_sample_read_lock_mediadescription(mediaTypeDescription, sample, coderInput);
	if (!haveDesc) {
		coderInput->GetID("bValue", boolID);
		coderInput->GetID("ui32ArduinoTimestamp", timeStampID);
		haveDesc = true;
	}
	if (value) {
		coderInput->Get(boolID, (tVoid*)value);
	}
	if (timeStamp) {
		coderInput->Get(timeStampID, (tVoid*)timeStamp);
	}
	RETURN_NOERROR;
}

WrappedBVOutputPin::WrappedBVOutputPin()
{
	this->ppin = &pin;
}

tResult WrappedBVOutputPin::setValue(cObjectPtr<IMediaSample> outSample, tBool value, tUInt32 timestamp)
{
	cObjectPtr<IMediaSerializer> serializer;
	mediaTypeDescription->GetMediaSampleSerializer(&serializer);
	outSample->AllocBuffer(serializer->GetDeserializedSize());
	//write date to the media sample with the coder of the descriptor
	{
		__adtf_sample_write_lock_mediadescription(mediaTypeDescription, outSample, outCoder);

		// set the id if not already done
		if (!haveDesc) {
			outCoder->GetID("bValue", boolID);
			outCoder->GetID("ui32ArduinoTimestamp", timeStampID);
			haveDesc = true;
		}

		// set value
		outCoder->Set(boolID, (tVoid*)&value);
		outCoder->Set(timeStampID,(tVoid*)&timestamp);
	 }

	ppin->Transmit(outSample);

	RETURN_NOERROR;
}

WrappedPin::WrappedPin()
{
	haveDesc = false;
}

WrappedPin::~WrappedPin() {}
