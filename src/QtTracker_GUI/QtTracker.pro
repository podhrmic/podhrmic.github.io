#-------------------------------------------------
#
# Project created by QtCreator 2015-02-19T14:14:35
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = QtTracker
TEMPLATE = app


SOURCES += main.cpp\
        dialog.cpp \
    Dot.cpp \
    serverthread.cpp \
    Agent.cpp \
    CollectionAgent.cpp \
    ControlAgent.cpp \
    Event.cpp \
    StrategyAgent.cpp

HEADERS  += dialog.h \
    Dot.h \
    serverthread.h \
    Agent.h \
    CollectionAgent.h \
    ControlAgent.h \
    Event.h \
    StrategyAgent.h

FORMS    += dialog.ui

#INCLUDEPATH += -I/usr/local/include/opencv -I/usr/local/include/opencv2 -I/usr/local/include/opencv2/highgui
INCLUDEPATH += -I/usr/local/

LIBS += -L/usr/local/lib \
-lopencv_contrib \
-lopencv_nonfree \
-lopencv_legacy \
-lopencv_flann \
-lopencv_core \
-lopencv_imgproc \
-lopencv_highgui \
-lopencv_ml \
-lopencv_video \
-lopencv_features2d \
-lopencv_calib3d \
-lopencv_objdetect
