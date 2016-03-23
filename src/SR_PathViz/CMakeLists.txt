# add the include directory of the filter
include_directories(../include)
# declare our filter
set(FILTER_NAME sr_pathViz)

adtf_use_qt(QtCore QtGui)

adtf_add_moc(${FILTER_NAME}
	PathViz.h
)

adtf_add_filter(${FILTER_NAME}
	PathViz.h
	PathViz.cpp
)

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
