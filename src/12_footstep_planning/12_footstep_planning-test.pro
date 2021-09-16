HEADERS = \
	include/footstep_planning/OpenList.h \
	include/footstep_planning/ClosedList.h \
	include/footstep_planning/AbstractNode.h \
	include/footstep_planning/GridMap.h \
	include/footstep_planning/PathPlanning.h \
	include/footstep_planning/Heuristic.h \
	include/footstep_planning/FileIO.h \
	include/footstep_planning/FootstepNode.h \
	include/footstep_planning/FootstepPlanning.h \
	include/footstep_planning/FootstepMap.h

SOURCES = \
    ../gtest/src/gtest-all.cc \
	src/FootstepMap.cpp \
	src/FootstepPlanning.cpp \
	src/FootstepNode.cpp \
	src/PathPlanning.cpp \
	src/OpenList.cpp \
	src/ClosedList.cpp \
	src/FileIO.cpp \	
	test/test_footstep_planning.cpp

INCLUDEPATH += include
INCLUDEPATH += ../includes
INCLUDEPATH += ../gtest/include
INCLUDEPATH += ../gtest
TEMPLATE = app
CONFIG -= qt
CONFIG -= app_bundle
TARGET = footstep_planning-test
DEFINES += PROJECT_SOURCE_DIR=\\\"$$absolute_path(".")\\\"
unix:QMAKE_LFLAGS += -pthread
windows:{
    QMAKE_LFLAGS += -static
    CONFIG += windows console
}
