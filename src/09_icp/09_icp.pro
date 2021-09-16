HEADERS = \
	include/icp/ICP.h \
	include/icp/FileIO.h

SOURCES = \
	src/ICP.cpp \
	src/main.cpp \
	src/FileIO.cpp

INCLUDEPATH += include
INCLUDEPATH += ../includes
TEMPLATE = app
CONFIG -= qt
CONFIG -= app_bundle
TARGET = icp_node
DEFINES += PROJECT_SOURCE_DIR=\\\"$$absolute_path(".")\\\"
windows:{
    QMAKE_LFLAGS += -static
    CONFIG += windows console
}
