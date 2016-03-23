#ifndef _SR_DRIVERMODULE_HEADER_H_
#define _SR_DRIVERMODULE_HEADER_H_


#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
#include <adtf_graphics.h>
#include <adtf_utils.h>
#include "juryEnums.h"
#include "../misc/PinWrapper.h"
#include "../misc/SR_Maneuvers_List.h"
#include "../misc/SR_Logger.h"
#include "ManeuverList.h"
#include "maneuverdata.h"

#include <map_data.h>

#define OID_ST_DRIVER_MODULE "adtf.aadc.sr_driverModule"

/*! /brief SR_DriverModule
This filter was developed as a prototype to explain the interaction with the Jury Module.
It receives the structs from the Jury module and shows them in the middle of the gui. The user can respond the the received messages with the four buttons Ready to Start, Running, Error and Complete. 
After clicking on the button the sample is transmitted on the output pin Driver_Struct an contains the following struct:

typedef struct
{
tInt8 i8StateID;
tInt16 i16ManeuverEntry;
} tDriverStruct;

The possible i8StateID are:
	stateCar_Error: This is sent if some error occurred on the car.
	stateCar_Ready: If the car is ready to start a maneuver ID this state is sent including the maneuver ID in i16ManeuverEntry.
	stateCar_Running: Sent during running the maneuver contained in i16ManeuverEntry 
	stateCar_Complete: Sent if the car finished the whole maneuver list.
	stateCar_Startup: Sent at the initial phase to indicate that car is working properly

The struct tDriverStruct is defined in aadc_structs.h in src\aadcBase\include and the used enums are defined in juryEnums.h in src\aadcBase\include

The teams must not implement any filter containing a Qt GUI because there is no opportunity to control the car with a GUI in the competition. The only way to interact with the car is through the jury module which is controlled by the jury.

*/
class SR_DriverModule : public adtf::cFilter
{
    ADTF_FILTER(OID_ST_DRIVER_MODULE, "SR AADC Driver Module", OBJCAT_Tool);




public: // construction
    SR_DriverModule(const tChar *);
    virtual ~SR_DriverModule();

	tHandle m_hTimer;
	cCriticalSection m_oCriticalSectionTimerSetup;
	cCriticalSection m_oCriticalSectionTransmit;

	/*! creates the timer for the cyclic transmits*/
	tResult createTimer();

   /*! destroys the timer for the cyclic transmits*/
	tResult destroyTimer(__exception = NULL);

	/*! overrides cFilter */
	tResult Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr=NULL);

	/*! signal for sending the state
	@param i8StateID state to be sent; -1: error, 0: Ready, 1: Running
	@param i16ManeuverEntry current entry to be sent
	*/
	tResult SendState(stateCar state, tInt16 i16ManeuverEntry);

protected:
    /*! overrides cFilter */
    tResult Init(tInitStage eStage, IException **__exception_ptr);
    /*! overrides cFilter */
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
    /*! overrides cFilter */
     tResult Start(IException **__exception_ptr);
    /*! overrides cFilter */
    tResult Stop(IException **__exception_ptr);

    tResult Shutdown(tInitStage eStage, ucom::IException **__exception_ptr);
private:
    /*! function which transmits the state      
    @param i8StateID state to be sent; -1: error, 0: Ready, 1: Running
    @param i16ManeuverEntry current entry to be sent
    */
    tTimeStamp lastTimeStampRun;
    tResult OnSendState(stateCar stateID, tInt16 i16ManeuverEntry);
    /*! this functions loads the maneuver list given in the properties*/
    tResult LoadManeuverList();    
	/*! these functions are called as a result of receiving DriverStruct*/
	tResult ReadyReceived(int entryId);
	tResult RunReceived(int entryId);
	tResult StopReceived(int entryId);
	/*! this function is called update:
	 * myActiveSensor
	 * myActiveManeuver*/
	tResult ProcessActiveManeuver();
	tResult ProcessManeuverComplete(ManeuverResponse vData);

	int sendReadyEntryID;
    /*! signal to the gui to show the command "run" from the jury
    @param entryId current entry to be sent 
    */
    tResult SendRun(int entryId);

    /*! signal to the gui to show the command "stop" from the jury 
    @param entryId current entry to be sent 
    */
    tResult SendStop(int entryId);

    /*! signal to the gui to show the command "request ready" from the jury 
    @param entryId current entry to be sent 
    */
    tResult SendRequestReady(int entryId);


	bool continueAfterSection;
	bool sendStateError;
	bool sendStateReady;
	bool sendStateRunning;
	bool sendStateComplete;
	bool sendStateStartup;

	SR_Logger logger;

    /*! Coder Descriptor */
    cObjectPtr<IMediaTypeDescription> m_pDescJuryStruct;
    /*! the id for the i8StateID of the media description */
    tBufferID m_szIDJuryStructI8ActionID; 
    /*! the id for the i16ManeuverEntry of the media description data */
    tBufferID m_szIDJuryStructI16ManeuverEntry;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsJuryStructSet;

    /*! Coder Descriptor */
    cObjectPtr<IMediaTypeDescription> m_pDescDriverStruct;
    /*! the id for the i8StateID of the media description */
    tBufferID m_szIDDriverStructI8StateID; 
    /*! the id for the i16ManeuverEntry of the media description data */
    tBufferID m_szIDDriverStructI16ManeuverEntry;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsDriverStructSet;

    /* Coder description */
    cObjectPtr<IMediaTypeDescription> m_pDescManeuverList;


    /*! input pin for the run command*/
    cInputPin        m_JuryStructInputPin;
    /*! input pin for the maneuver list*/
    cInputPin        m_ManeuverListInputPin;

	cInputPin        inPinManeuverComplete;
	WrappedBVInputPin        inPinManeuverError;


    /*! output pin for state from driver*/
    cOutputPin        m_DriverStructOutputPin;
	/*! output pin for general start, true as long as car should run*/
    /*! output pin for active command */
	cOutputPin        m_ActiveCommandOutputPin;
	WrappedBVOutputPin        outPinRestartSection;




    bool myGlobalFinishedSection;

    cString     m_strManeuverFileString;
    /*! this is the filename of the maneuver list*/
    cFilename m_maneuverListFile;
    /*! this is the list with all the loaded sections from the maneuver list*/
    std::vector<tSector> m_sectorList;

	//User members
	int myLastEntryID;
	int myActiveSector;
	int myActiveManeuver;
    int myLastManeuver;
    bool myInManeuver;

    tResult WriteOutManeuver(int vData);
	tResult WriteBoolToPin(WrappedBVOutputPin &vOutputPin, bool vData);
    tResult WriteFloatToPin(cOutputPin *vOutputPin, float vData);
	tResult SendManeuverStop();
	tResult SendManeuverComplete(int vReason);
	bool getSectorAndIndexForID(int vEntryID,int &vSector,int &vIndex);

	cString GetActionStringForActiveManeuver();
	ManeuverType GetLocalManeuverForActiveManeuverString(cString vString);



	bool checkReachedEndOfParcour(int vEntryID);
	bool checkReachedEndOfSection(int vEntryID);
	// cConfiguration interface
public:
	tResult PropertyChanged(const tChar *strName);

};

#endif
