#ifndef _SR_LOGGER_H_
#define _SR_LOGGER_H_

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
#include <adtf_graphics.h>
#include <adtf_utils.h>

using namespace adtf;

class SR_InputPin
{
public:
    SR_InputPin(cString vName);

    cString myName;
    adtf::cInputPin myPin;
    cObjectPtr<IMediaTypeDescription> myMediaTypeDescription;
    tBool myBufferIDGet;
    tBufferID myBufferID;
protected:
    tResult Init(cFilter *vParentFilter, IPinEventSink *vParent, __exception);
};

#endif
