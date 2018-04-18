#-------------------------------------------------
#
# Project created by QtCreator 2018-04-16T10:43:23
#
#-------------------------------------------------

QT       -= gui

TARGET = grabberRS
TEMPLATE = lib

DEFINES += GRABBERLIB_LIBRARY

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

INCLUDEPATH +=          C:/GITROOT/CPPCommon \
                        C:/GITROOT/QTCommon \
                        "C:/dev/IntelRealSenseSDK2/include/librealsense2/"

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += grabber.cpp \
           C:/GITROOT/CPPCommon/IniFile.cpp \
           C:/GITROOT/CPPCommon/Logger.cpp \
           C:/GITROOT/CPPCommon/LTime.cpp





HEADERS += grabber.h\
        GrabberLib_global.h \
        C:/GITROOT/CPPCommon/QConsoleDebugStream.h \
        "C:/dev/IntelRealSenseSDK2/include/librealsense2/rs.hpp" \

CONFIG( debug, debug|release ) {
    LIBS+=             "C:/BUILDS/IntelRealSense_SDK/Debug/realsense2.lib"
}else {
    LIBS+= "C:/BUILDS/IntelRealSense_SDK/Release/realsense2.lib"
}
