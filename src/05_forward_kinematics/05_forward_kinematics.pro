HEADERS = \
    include/forward_kinematics/ForwardKinematics.h \
    include/forward_kinematics/FileIO.h

SOURCES = \
    src/ForwardKinematics.cpp \
    src/main.cpp \
    src/FileIO.cpp

INCLUDEPATH += include
INCLUDEPATH += ../includes
TEMPLATE = app
CONFIG -= qt
CONFIG -= app_bundle
TARGET = forward_kinematics_node
DEFINES += PROJECT_SOURCE_DIR=\\\"$$absolute_path(".")\\\"
windows:{
    QMAKE_LFLAGS += -static
    CONFIG += windows console
}
