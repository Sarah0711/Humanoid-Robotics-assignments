HEADERS = include/linear_algebra/LinearAlgebra.h

SOURCES = src/main.cpp \
          src/LinearAlgebra.cpp

INCLUDEPATH += include
INCLUDEPATH += ../includes
TEMPLATE = app
TARGET = linear_algebra_node
DEFINES += PROJECT_SOURCE_DIR=\\\"$$absolute_path(".")\\\"
