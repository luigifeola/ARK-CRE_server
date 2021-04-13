#-------------------------------------------------
#
# Project created by QtCreator 2018-01-01T07:35:48
#
#-------------------------------------------------

QT       -= gui

QT += widgets network

TARGET = CRES
TEMPLATE = lib

DEFINES += CRESEXP_LIBRARY

SOURCES += \
    kilobot.cpp \
    serverStuff.cpp \
    cresEnvironment.cpp \
    cresExperiment.cpp

HEADERS +=\
    kilobot.h \
    serverStuff.h \
    kilobotexperiment.h \
    kilobotenvironment.h \
    global.h \
    cresEnvironment.h \
    cresExperiment.h \
    area.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}

INCLUDEPATH += /usr/local/include/
LIBS += -L/usr/local/lib \
        -lopencv_core
