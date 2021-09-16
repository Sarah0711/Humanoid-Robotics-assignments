HEADERS = \
	include/particle_filter/ParticleFilter.h \
	include/particle_filter/FileIO.h

SOURCES = \
	src/ParticleFilter.cpp \
	src/main.cpp \
	src/FileIO.cpp

INCLUDEPATH += include
INCLUDEPATH += ../includes
TEMPLATE = app
CONFIG -= qt
CONFIG -= app_bundle
TARGET = particle_filter_node
DEFINES += PROJECT_SOURCE_DIR=\\\"$$absolute_path(".")\\\"
windows:{
    QMAKE_LFLAGS += -static
    CONFIG += windows console
}
