HEADERS = \
	include/irm/AbstractIRM.h \
	include/irm/IRM.h \
	include/irm/FileIO.h

SOURCES = \
	src/AbstractIRM.cpp \
	src/IRM.cpp \
	src/main.cpp \
	src/FileIO.cpp

INCLUDEPATH += include
INCLUDEPATH += ../includes
TEMPLATE = app
CONFIG -= qt
CONFIG -= app_bundle
TARGET = irm_node
DEFINES += PROJECT_SOURCE_DIR=\\\"$$absolute_path(".")\\\"
windows:{
    QMAKE_LFLAGS += -static
    CONFIG += windows console
}
