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
	src/ARAStar.cpp \
	src/ClosedList.cpp \
	src/FileIO.cpp \
	src/GridNode.cpp \
	src/Logger.cpp \
	src/main.cpp \
	src/OpenList.cpp \
	src/PathPlanning.cpp

INCLUDEPATH += include
INCLUDEPATH += ../includes
TEMPLATE = app
CONFIG -= qt
CONFIG -= app_bundle
TARGET = ara_star_node
DEFINES += PROJECT_SOURCE_DIR=\\\"$$absolute_path(".")\\\"
windows:{
    QMAKE_LFLAGS += -static
    CONFIG += windows console
}

