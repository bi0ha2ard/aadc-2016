#ifndef _PIN_WRAPPER_H_
#define _PIN_WRAPPER_H_

#include <adtf_plugin_sdk.h>

#define REGISTER_MEDIA_PIN(pin, mediatype, mediasubtype, name) \
{\
	cObjectPtr<adtf::IMediaType> _mediaType;\
	RETURN_IF_FAILED(AllocMediaType(&_mediaType, (mediatype), (mediasubtype)))\
	RETURN_IF_FAILED((pin).Create((name), _mediaType, static_cast<adtf::IPinEventSink*>(this)));\
	RETURN_IF_FAILED(RegisterPin(&(pin)));\
}

#define SEND_MEDIA_SAMPLE(pin, type, value, timestamp) \
{\
	cObjectPtr<IMediaSample> sample;\
	if (IS_OK(AllocMediaSample(&sample))) {\
		sample->Update((timestamp), &(value), sizeof(type), 0);\
		RETURN_IF_FAILED((pin).Transmit(sample));\
	}\
}

#define READ_MEDIA_PIN(mediaSample, type, value) {\
	__sample_read_lock(mediaSample, type, data);\
	(value) = *data;\
}

class SignalValuePin
{
public:
	static tResult initPin(adtf::IPin *pin, IException **__exception_ptr);
	static tResult getValue(adtf::cInputPin *pin, adtf::IMediaSample *sample, tFloat32 *value, tUInt32 *timeStamp = NULL);
    static tResult setValue(adtf::cOutputPin *pin, cObjectPtr<adtf::IMediaSample> outSample, tFloat32 value, tUInt32 timestamp);
	static cObjectPtr<adtf::IMediaType> getMediaType();

private:
	static bool haveDesc;
	static cObjectPtr<adtf::IMediaTypeDescription> signalValueDescription;
	static cObjectPtr<adtf::IMediaType> mediaType;
	static adtf::tBufferID valueID;
	static adtf::tBufferID timeStampID;
};

#define INIT_SIGNAL_VALUE_FILTER_PIN(filter, pin, name, exception) RETURN_IF_FAILED(SignalValuePin::initPin(pin, exception)); \
	RETURN_IF_FAILED((pin)->Create(name, SignalValuePin::getMediaType(), static_cast<IPinEventSink*>(filter))); \
	RETURN_IF_FAILED(filter->RegisterPin(pin));

#define INIT_SIGNAL_VALUE_PIN(pin, name) INIT_SIGNAL_VALUE_FILTER_PIN(this, pin, name, __exception_ptr);

#define SEND_SIGNAL_VALUE_SAMPLE(pin, value, timestamp) {\
	cObjectPtr<IMediaSample> outSample;\
	AllocMediaSample((tVoid**)&outSample);\
\
	RETURN_IF_FAILED(SignalValuePin::setValue((pin), outSample, (value), (timestamp)));}

class WrappedPin
{
public:
	cObjectPtr<adtf::IMediaType> getMediaType();
	tResult initPin(const char *name, IException **__exception_ptr);
	adtf::IPin *ppin;

protected:
	WrappedPin();
	virtual ~WrappedPin();
	bool haveDesc;
	cObjectPtr<adtf::IMediaTypeDescription> mediaTypeDescription;
	cObjectPtr<adtf::IMediaType> mediaType;
};

class WrappedSVPin : public WrappedPin
{
public:
	tResult initPin(IException **__exception_ptr);

protected:
	adtf::tBufferID valueID;
	adtf::tBufferID timeStampID;
};

class WrappedSVInputPin : public WrappedSVPin
{
public:
	WrappedSVInputPin();
	tResult getValue(adtf::IMediaSample *sample, tFloat32 *value, tUInt32 *timeStamp = NULL);
	adtf::cInputPin pin;
};

class WrappedSVOutputPin : public WrappedSVPin
{
public:
	WrappedSVOutputPin();
	tResult setValue(cObjectPtr<adtf::IMediaSample> outSample, tFloat32 value, tUInt32 timestamp);
	adtf::cOutputPin pin;
};


class BoolValuePin
{
public:
	static tResult initPin(adtf::IPin *pin, IException **__exception_ptr);
	static tResult getValue(adtf::cInputPin *pin, adtf::IMediaSample *sample, tBool *value, tUInt32 *timeStamp = NULL);
	static tResult setValue(adtf::cOutputPin *pin, cObjectPtr<adtf::IMediaSample> outSample, tBool value, tUInt32 timestamp);
	static cObjectPtr<adtf::IMediaType> getMediaType();

private:
	static bool haveDesc;
	static cObjectPtr<adtf::IMediaTypeDescription> boolValueDescription;
	static cObjectPtr<adtf::IMediaType> mediaType;
	static adtf::tBufferID boolID;
	static adtf::tBufferID timeStampID;
};

#define INIT_BOOL_FILTER_PIN(filter, pin, name, exception) RETURN_IF_FAILED(BoolValuePin::initPin(pin, exception)); \
	RETURN_IF_FAILED((pin)->Create(name, BoolValuePin::getMediaType(), static_cast<IPinEventSink*>(filter))); \
	RETURN_IF_FAILED(filter->RegisterPin(pin));

#define INIT_BOOL_PIN(pin, name) INIT_BOOL_FILTER_PIN(this, pin, name, __exception_ptr);

#define SEND_BOOL_SAMPLE(pin, value, timestamp) {\
	cObjectPtr<IMediaSample> outSample;\
	AllocMediaSample((tVoid**)&outSample);\
\
	RETURN_IF_FAILED(BoolValuePin::setValue((pin), outSample, (value), (timestamp)));}


class WrappedBVPin : public WrappedPin
{
public:
	tResult initPin(IException **__exception_ptr);

protected:
	adtf::tBufferID boolID;
	adtf::tBufferID timeStampID;
};

class WrappedBVInputPin : public WrappedBVPin
{
public:
	WrappedBVInputPin();
	tResult getValue(adtf::IMediaSample *sample, tBool *value, tUInt32 *timeStamp = NULL);
	adtf::cInputPin pin;
};

class WrappedBVOutputPin : public WrappedBVPin
{
public:
	WrappedBVOutputPin();
	tResult setValue(cObjectPtr<adtf::IMediaSample> outSample, tBool value, tUInt32 timestamp);
	adtf::cOutputPin pin;
};

#define INIT_WRAPPED_PIN(wrappedpin, name) (wrappedpin).initPin(__exception_ptr);\
	RETURN_IF_FAILED((wrappedpin).pin.Create(name, (wrappedpin).getMediaType(), static_cast<IPinEventSink*>(this)));\
	RETURN_IF_FAILED(this->RegisterPin(&(wrappedpin).pin));

#define SEND_WRAPPED_SAMPLE(wrappedpin, value, timestamp) {\
	cObjectPtr<IMediaSample> outSample;\
	AllocMediaSample((tVoid**)&outSample);\
	RETURN_IF_FAILED((wrappedpin).setValue(outSample, (value), (timestamp)));}


#endif
