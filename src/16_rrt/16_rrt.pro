HEADERS = \
	include/rrt/GridNode.h \
	include/rrt/RRT.h \
	include/rrt/AbstractNode.h \
	include/rrt/GridMap.h \
	include/rrt/Logger.h \
	include/rrt/FileIO.h

SOURCES = \
	src/RRT.cpp \
	src/GridNode.cpp \
	src/main.cpp \
	src/FileIO.cpp \
	src/Logger.cpp

INCLUDEPATH += include
INCLUDEPATH += ../includes
TEMPLATE = app
CONFIG -= qt
CONFIG -= app_bundle
TARGET = rrt_node
DEFINES += PROJECT_SOURCE_DIR=\\\"$$absolute_path(".")\\\"
windows:{
    QMAKE_LFLAGS += -static
    CONFIG += windows console
}
