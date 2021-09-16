HEADERS = \
    include/odometry_calibration/CalibrationData.h \
    include/odometry_calibration/OdometryCalibration.h \
    include/odometry_calibration/FileIO.h \

SOURCES = \
    ../gtest/src/gtest-all.cc \
    test/test_odometry_calibration.cpp \
    src/FileIO.cpp \
    src/OdometryCalibration.cpp


INCLUDEPATH += include
INCLUDEPATH += ../includes
INCLUDEPATH += ../gtest/include
INCLUDEPATH += ../gtest
TEMPLATE = app
TARGET = odometry_calibration-test
DEFINES += PROJECT_SOURCE_DIR=\\\"$$absolute_path(".")\\\"
unix:QMAKE_LFLAGS += -pthread
windows:{
    QMAKE_LFLAGS += -static
    CONFIG += windows console
}
