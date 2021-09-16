HEADERS = \
	include/inverse_kinematics/InverseKinematics.h

SOURCES = \
    ../gtest/src/gtest-all.cc \
	src/InverseKinematics.cpp \
    test/test_inverse_kinematics.cpp

INCLUDEPATH += include
INCLUDEPATH += ../includes
INCLUDEPATH += ../gtest/include
INCLUDEPATH += ../gtest
TEMPLATE = app
CONFIG -= qt
CONFIG -= app_bundle
TARGET = inverse_kinematics-test
DEFINES += PROJECT_SOURCE_DIR=\\\"$$absolute_path(".")\\\"
unix:QMAKE_LFLAGS += -pthread
windows:{
    QMAKE_LFLAGS += -static
    CONFIG += windows console
}
