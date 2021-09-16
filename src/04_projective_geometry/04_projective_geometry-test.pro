HEADERS = \
    include/projective_geometry/ProjectiveGeometry.h

SOURCES = \
    ../gtest/src/gtest-all.cc \
    test/test_projective_geometry.cpp \
    src/ProjectiveGeometry.cpp

INCLUDEPATH += include
INCLUDEPATH += ../includes
INCLUDEPATH += ../gtest/include
INCLUDEPATH += ../gtest
TEMPLATE = app
TARGET = projective_geometry-test
DEFINES += PROJECT_SOURCE_DIR=\\\"$$absolute_path(".")\\\"
unix:QMAKE_LFLAGS += -pthread
windows:{
    QMAKE_LFLAGS += -static
    CONFIG += windows console
}
