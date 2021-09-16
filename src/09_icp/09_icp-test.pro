HEADERS = \
	include/icp/ICP.h \
	include/icp/FileIO.h

SOURCES = \
    ../gtest/src/gtest-all.cc \
	src/ICP.cpp \
	src/FileIO.cpp \
	test/test_icp.cpp

INCLUDEPATH += include
INCLUDEPATH += ../includes
INCLUDEPATH += ../gtest/include
INCLUDEPATH += ../gtest
TEMPLATE = app
CONFIG -= qt
CONFIG -= app_bundle
TARGET = icp-test
DEFINES += PROJECT_SOURCE_DIR=\\\"$$absolute_path(".")\\\"
unix:QMAKE_LFLAGS += -pthread
windows:{
    QMAKE_LFLAGS += -static
    CONFIG += windows console
}
