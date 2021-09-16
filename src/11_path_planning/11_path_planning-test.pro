HEADERS = \
	include/path_planning/OpenList.h \
	include/path_planning/GridNode.h \
	include/path_planning/ClosedList.h \
	include/path_planning/AbstractNode.h \
	include/path_planning/GridMap.h \
	include/path_planning/PathPlanning.h \
	include/path_planning/Heuristic.h \
	include/path_planning/FileIO.h

SOURCES = \
    ../gtest/src/gtest-all.cc \
	src/GridNode.cpp \
	src/PathPlanning.cpp \
	src/OpenList.cpp \
	src/ClosedList.cpp \
	src/FileIO.cpp \
	test/test_path_planning.cpp

INCLUDEPATH += include
INCLUDEPATH += ../includes
INCLUDEPATH += ../gtest/include
INCLUDEPATH += ../gtest
TEMPLATE = app
CONFIG -= qt
CONFIG -= app_bundle
TARGET = path_planning-test
DEFINES += PROJECT_SOURCE_DIR=\\\"$$absolute_path(".")\\\"
unix:QMAKE_LFLAGS += -pthread
windows:{
    QMAKE_LFLAGS += -static
    CONFIG += windows console
}
