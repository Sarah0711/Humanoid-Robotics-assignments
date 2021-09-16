HEADERS = \
    include/signed_distance_function/SignedDistanceFunction.h \
    include/signed_distance_function/FileIO.h

SOURCES = \
    src/SignedDistanceFunction.cpp \
    src/main.cpp \
    src/FileIO.cpp

INCLUDEPATH += include
INCLUDEPATH += ../includes
TEMPLATE = app
CONFIG -= qt
CONFIG -= app_bundle
TARGET = signed_distance_function_node
DEFINES += PROJECT_SOURCE_DIR=\\\"$$absolute_path(".")\\\"
windows:{
    QMAKE_LFLAGS += -static
    CONFIG += windows console
}
