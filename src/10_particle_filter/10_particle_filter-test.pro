HEADERS = \
	include/particle_filter/ParticleFilter.h \
	include/particle_filter/FileIO.h

SOURCES = \
    ../gtest/src/gtest-all.cc \
	src/ParticleFilter.cpp \
	src/FileIO.cpp \
	test/test_particle_filter.cpp 

INCLUDEPATH += include
INCLUDEPATH += ../includes
INCLUDEPATH += ../gtest/include
INCLUDEPATH += ../gtest
TEMPLATE = app
CONFIG -= qt
CONFIG -= app_bundle
TARGET = particle_filter-test
DEFINES += PROJECT_SOURCE_DIR=\\\"$$absolute_path(".")\\\"
unix:QMAKE_LFLAGS += -pthread
windows:{
    QMAKE_LFLAGS += -static
    CONFIG += windows console
}
