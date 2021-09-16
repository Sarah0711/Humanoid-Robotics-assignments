HEADERS = \
    include/projective_geometry/ProjectiveGeometry.h

SOURCES = \
    src/main.cpp \
    src/ProjectiveGeometry.cpp

INCLUDEPATH += include
INCLUDEPATH += ../includes
TEMPLATE = app
TARGET = projective_geometry_node
DEFINES += PROJECT_SOURCE_DIR=\\\"$$absolute_path(".")\\\"
windows:{
    QMAKE_LFLAGS += -static
    CONFIG += windows console
}

