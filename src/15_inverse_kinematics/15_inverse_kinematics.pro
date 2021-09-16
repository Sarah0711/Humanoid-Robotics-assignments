HEADERS = \
	include/inverse_kinematics/InverseKinematics.h

SOURCES = \
	src/main.cpp \
	src/InverseKinematics.cpp

INCLUDEPATH += include
INCLUDEPATH += ../includes
TEMPLATE = app
CONFIG -= qt
CONFIG -= app_bundle
TARGET = inverse_kinematics_node
DEFINES += PROJECT_SOURCE_DIR=\\\"$$absolute_path(".")\\\"
windows:{
    QMAKE_LFLAGS += -static
    CONFIG += windows console
}
