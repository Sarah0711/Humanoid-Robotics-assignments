HEADERS = \
	include/ara_star/AbstractNode.h \
	include/ara_star/ARAStar.h \
	include/ara_star/ClosedList.h \
	include/ara_star/FileIO.h \
	include/ara_star/GridMap.h \
	include/ara_star/GridNode.h \
	include/ara_star/Heuristic.h \
	include/ara_star/Logger.h \
	include/ara_star/OpenList.h \
	include/ara_star/PathPlanning.h

SOURCES = \
	../gtest/src/gtest-all.cc \
	src/ARAStar.cpp \
	src/ClosedList.cpp \
	src/FileIO.cpp \
	src/GridNode.cpp \
	src/Logger.cpp \
	src/OpenList.cpp \
	src/PathPlanning.cpp \
	test/test_ara_star.cpp

INCLUDEPATH += include
INCLUDEPATH += ../11_path_planning/include
INCLUDEPATH += ../includes
INCLUDEPATH += ../gtest/include
INCLUDEPATH += ../gtest
TEMPLATE = app
CONFIG -= qt
CONFIG -= app_bundle
TARGET = ara_star-test
DEFINES += PROJECT_SOURCE_DIR=\\\"$$absolute_path(".")\\\"
unix:QMAKE_LFLAGS += -pthread
windows:{
    QMAKE_LFLAGS += -static
    CONFIG += windows console
}

