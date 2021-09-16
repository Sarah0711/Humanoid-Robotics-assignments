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
	src/FootstepMap.cpp \
	src/FootstepPlanning.cpp \
	src/FootstepNode.cpp \
	src/PathPlanning.cpp \
	src/OpenList.cpp \
	src/ClosedList.cpp \
	src/FileIO.cpp \	
	src/main.cpp

INCLUDEPATH += include
INCLUDEPATH += ../includes
TEMPLATE = app
CONFIG -= qt
CONFIG -= app_bundle
TARGET = footstep_planning_node
DEFINES += PROJECT_SOURCE_DIR=\\\"$$absolute_path(".")\\\"
windows:{
    QMAKE_LFLAGS += -static
    CONFIG += windows console
}
