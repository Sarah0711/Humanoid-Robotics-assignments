HEADERS = \
    include/octree/Octree.h \
    include/octree/FileIO.h

SOURCES = \
    src/Octree.cpp \
    src/main.cpp \
    src/FileIO.cpp

INCLUDEPATH += include
INCLUDEPATH += ../includes
TEMPLATE = app
CONFIG -= qt
CONFIG -= app_bundle
TARGET = octree
DEFINES += PROJECT_SOURCE_DIR=\\\"$$absolute_path(".")\\\"
windows:{
    QMAKE_LFLAGS += -static
    CONFIG += windows console
}
