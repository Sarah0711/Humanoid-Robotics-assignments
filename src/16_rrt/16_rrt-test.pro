HEADERS = \
	include/rrt/GridNode.h \
	include/rrt/RRT.h \
	include/rrt/AbstractNode.h \
	include/rrt/GridMap.h \
	include/rrt/Logger.h \
	include/rrt/FileIO.h

SOURCES = \
    ../gtest/src/gtest-all.cc \
	src/RRT.cpp \
	src/GridNode.cpp \
	src/FileIO.cpp \
	src/Logger.cpp \
    test/test_rrt.cpp

INCLUDEPATH += include
INCLUDEPATH += ../includes
INCLUDEPATH += ../gtest/include
INCLUDEPATH += ../gtest
TEMPLATE = app
CONFIG -= qt
CONFIG -= app_bundle
TARGET = rrt-test
DEFINES += PROJECT_SOURCE_DIR=\\\"$$absolute_path(".")\\\"
unix:QMAKE_LFLAGS += -pthread
windows:{
    QMAKE_LFLAGS += -static
    CONFIG += windows console
}
