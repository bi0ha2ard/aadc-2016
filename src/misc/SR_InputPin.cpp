#include "SR_InputPin.h"

SR_InputPin::SR_InputPin(cString vName)
{
    myName=vName;

}

tResult SR_InputPin::Init(cFilter* vParentFilter,IPinEventSink* vParent,__exception = NULL)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    // get media type
    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    // set member media description
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&myMediaTypeDescription));

    // create pin
    RETURN_IF_FAILED(myPin.Create(myName, pTypeSignalValue, vParent));
    RETURN_IF_FAILED(RegisterPin(&myPin));
}






