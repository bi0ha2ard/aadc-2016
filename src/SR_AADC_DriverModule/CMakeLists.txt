# add the include directory of the filter
include_directories(../include)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})
set(FILTER_NAME SR_DriverModule)

  
    adtf_add_filter(${FILTER_NAME}
		    SR_DriverModule.h
                    SR_DriverModule.cpp
		    ../misc/PinWrapper.h
                    ../misc/PinWrapper.cpp
		    ../misc/SR_Maneuvers_List.h
                    ../misc/SR_Logger.h
                    ../misc/SR_Logger.cpp
    )

    
    
    adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
    




