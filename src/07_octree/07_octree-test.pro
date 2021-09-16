HEADERS = \
    include/octree/Octree.h \
    include/octree/FileIO.h


SOURCES = \
    ../gtest/src/gtest-all.cc \
    src/Octree.cpp \
    src/FileIO.cpp \
    test/test_octree.cpp

INCLUDEPATH += include
INCLUDEPATH += ../includes
INCLUDEPATH += ../gtest/include
INCLUDEPATH += ../gtest
TEMPLATE = app
CONFIG -= qt
CONFIG -= app_bundle
TARGET = octree-test
DEFINES += PROJECT_SOURCE_DIR=\\\"$$absolute_path(".")\\\"
unix:QMAKE_LFLAGS += -pthread
windows:{
    QMAKE_LFLAGS += -static
    CONFIG += windows console
}
