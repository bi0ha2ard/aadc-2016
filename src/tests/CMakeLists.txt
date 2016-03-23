adtf_use_qt(QtCore QtTest)
INCLUDE_DIRECTORIES(${QT_INCLUDES})
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})
INCLUDE_DIRECTORIES(../../include)
SET( TEST_LIBRARIES ${OpenCV_LIBRARIES} ${QT_QTTEST_LIBRARY} ${QT_LIBRARIES} )
SET(CMAKE_AUTOMOC ON)

SET( LineHelpersTest_SRCS tst_linehelperstest.h
	tst_linehelperstest.cpp
	../misc/LineHelpers.cpp
	../misc/LineHelpers.h
	../misc/FastRot.h)
ADD_EXECUTABLE( LineHelpersTest ${LineHelpersTest_SRCS} )
TARGET_LINK_LIBRARIES( LineHelpersTest ${TEST_LIBRARIES} )
ADD_TEST( NAME LineHelpersTest COMMAND LineHelpersTest )

SET (DrivingPathTest_SRCS DrivingPathTest.h
	DrivingPathTest.cpp
	../misc/LineHelpers.cpp
	../misc/LineHelpers.h
	../SR_PathPlanner/DrivingPath.cpp
	../SR_PathPlanner/DrivingPath.h
	../misc/OdometryHelpers.h
	../misc/OdometryHelpers.cpp
	)

ADD_EXECUTABLE( DrivingPathTest ${DrivingPathTest_SRCS})
TARGET_LINK_LIBRARIES( DrivingPathTest ${TEST_LIBRARIES})
ADD_TEST( NAME DrivingPathTest COMMAND DrivingPathTest )
