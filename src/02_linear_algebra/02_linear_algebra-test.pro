HEADERS = include/linear_algebra/LinearAlgebra.h

SOURCES = ../gtest/src/gtest-all.cc \
          src/LinearAlgebra.cpp \
          test/test_linear_algebra.cpp

INCLUDEPATH += include
INCLUDEPATH += ../includes
INCLUDEPATH += ../gtest/include
INCLUDEPATH += ../gtest
TEMPLATE = app
TARGET = linear_algebra-test
DEFINES += PROJECT_SOURCE_DIR=\\\"$$absolute_path(".")\\\"
unix:QMAKE_LFLAGS += -pthread
windows:{
    QMAKE_LFLAGS += -static
    CONFIG += windows console
}
