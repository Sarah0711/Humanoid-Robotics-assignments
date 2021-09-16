HEADERS = \
    include/signed_distance_function/SignedDistanceFunction.h \
    include/signed_distance_function/FileIO.h

SOURCES = \
    ../gtest/src/gtest-all.cc \
    src/SignedDistanceFunction.cpp \
    src/FileIO.cpp \
    test/test_signed_distance_function.cpp

INCLUDEPATH += include
INCLUDEPATH += ../includes
INCLUDEPATH += ../gtest/include
INCLUDEPATH += ../gtest
TEMPLATE = app
CONFIG -= qt
CONFIG -= app_bundle
TARGET = signed_distance_function-test
DEFINES += PROJECT_SOURCE_DIR=\\\"$$absolute_path(".")\\\"
unix:QMAKE_LFLAGS += -pthread
windows:{
    QMAKE_LFLAGS += -static
    CONFIG += windows console
}
