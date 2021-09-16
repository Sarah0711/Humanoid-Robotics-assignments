HEADERS = \
    include/odometry_calibration/CalibrationData.h \
    include/odometry_calibration/OdometryCalibration.h \
    include/odometry_calibration/FileIO.h \

SOURCES = \
    src/main.cpp \
    src/FileIO.cpp \
    src/OdometryCalibration.cpp \

INCLUDEPATH += include
INCLUDEPATH += ../includes
TEMPLATE = app
TARGET = odometry_calibration_node
DEFINES += PROJECT_SOURCE_DIR=\\\"$$absolute_path(".")\\\"
windows:{
    QMAKE_LFLAGS += -static
    CONFIG += windows console
}
