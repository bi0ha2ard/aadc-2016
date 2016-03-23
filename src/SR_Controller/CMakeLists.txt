# add the include directory of the filter
include_directories(../include)
# declare our filter
set(FILTER_NAME SR_Controller)
adtf_add_filter(${FILTER_NAME}
	SR_Controller.h
	SR_Controller.cpp
        SR_ParallelParkingController.cpp
        SR_ParallelParkingController.h
        SR_PullOutParallelParkingController.cpp
        SR_PullOutParallelParkingController.h
        SR_CrossParkingController.cpp
        SR_CrossParkingController.h
        SR_PullOutCrossParkingController.cpp
        SR_PullOutCrossParkingController.h
	SR_PullOutLeftCrossParkingController.cpp
	SR_PullOutLeftCrossParkingController.h
		SR_AbstractParkingController.h
		SR_AbstractParkingController.cpp
        ../misc/PinWrapper.h
        ../misc/PinWrapper.cpp
        ../misc/SR_Logger.h
        ../misc/SR_Logger.cpp
)
# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
