HEADERS = \
	include/irm/AbstractIRM.h \
	include/irm/IRM.h \
	include/irm/FileIO.h

SOURCES = \
    ../gtest/src/gtest-all.cc \
	src/AbstractIRM.cpp \
	src/IRM.cpp \
	src/FileIO.cpp \
    test/test_irm.cpp

INCLUDEPATH += include
INCLUDEPATH += ../includes
INCLUDEPATH += ../gtest/include
INCLUDEPATH += ../gtest
TEMPLATE = app
CONFIG -= qt
CONFIG -= app_bundle
TARGET = irm-test
DEFINES += PROJECT_SOURCE_DIR=\\\"$$absolute_path(".")\\\"
unix:QMAKE_LFLAGS += -pthread
windows:{
    QMAKE_LFLAGS += -static
    CONFIG += windows console
}
