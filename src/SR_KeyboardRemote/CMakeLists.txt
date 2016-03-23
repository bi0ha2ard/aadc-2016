# add the include directory of the filter
include_directories(../include)
# declare our filter
set(FILTER_NAME SR_KeyboardRemote)
adtf_add_filter(${FILTER_NAME}
	SR_KeyboardRemote.h
	SR_KeyboardRemote.cpp
        ../misc/PinWrapper.h
        ../misc/PinWrapper.cpp
        ../misc/SR_Logger.h
        ../misc/SR_Logger.cpp
)
# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
