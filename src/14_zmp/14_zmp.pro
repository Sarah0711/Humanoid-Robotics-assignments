HEADERS = \
    include/zmp/ZMP.h

SOURCES = \
	src/ZMP.cpp \
	src/main.cpp

INCLUDEPATH += include
INCLUDEPATH += ../includes
TEMPLATE = app
CONFIG -= qt
CONFIG -= app_bundle
TARGET = zmp_node
DEFINES += PROJECT_SOURCE_DIR=\\\"$$absolute_path(".")\\\"
windows:{
    QMAKE_LFLAGS += -static
    CONFIG += windows console
}
