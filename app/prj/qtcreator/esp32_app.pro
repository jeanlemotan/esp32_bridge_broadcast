#-------------------------------------------------
#
# Project created by QtCreator 2014-06-04T18:33:20
#
#-------------------------------------------------

TARGET = esp32_app
TEMPLATE = app

target.path = esp32_app
INSTALLS = target

CONFIG -= qt
CONFIG += c++11

INCLUDEPATH += =/usr/local/include
INCLUDEPATH += ../../
INCLUDEPATH += ../../../lib
INCLUDEPATH += ../../../lib/utils


QMAKE_CXXFLAGS += -Wno-unused-variable -Wno-unused-parameter
QMAKE_CFLAGS += -Wno-unused-variable -Wno-unused-parameter

#PRECOMPILED_HEADER = ../../src/stdafx.h
#CONFIG *= precompile_header

rpi {
    DEFINES+=RASPBERRY_PI
    QMAKE_MAKEFILE = "Makefile.rpi"
    MAKEFILE = "Makefile.rpi"
    CONFIG(debug, debug|release) {
        DEST_FOLDER = rpi/debug
    }
    CONFIG(release, debug|release) {
        DEST_FOLDER = rpi/release
        DEFINES += NDEBUG
    }
} else {
    QMAKE_MAKEFILE = "Makefile"
    CONFIG(debug, debug|release) {
        DEST_FOLDER = pc/debug
    }
    CONFIG(release, debug|release) {
        DEST_FOLDER = pc/release
        DEFINES += NDEBUG
    }
}

LIBS += -lz -lpthread

OBJECTS_DIR = ./.obj/$${DEST_FOLDER}
MOC_DIR = ./.moc/$${DEST_FOLDER}
RCC_DIR = ./.rcc/$${DEST_FOLDER}
UI_DIR = ./.ui/$${DEST_FOLDER}
DESTDIR = ../../bin

HEADERS += \
    ../../../lib/Phy.h \
    ../../../lib/utils/pigpio.h \
    ../../../lib/utils/command.h

SOURCES += \
    ../../main.cpp \
    ../../../lib/Phy.cpp \
    ../../../lib/utils/pigpio.c \
    ../../../lib/utils/command.c

