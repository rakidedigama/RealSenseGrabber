QT += core
QT -= gui

CONFIG += c++11

include(../grabberRS/grabberRS.pri)

TARGET = RSGrabberTester
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \



# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

INCLUDEPATH += ../grabberRS \
 C:/GITROOT/CPPCommon \
  "C:/dev/IntelRealSenseSDK2/include/librealsense2/" \
     C:/BUILDS/openCV/install/include

HEADERS += ../grabberRS/grabber.h \
              "C:/dev/IntelRealSenseSDK2/include/librealsense2/rs.hpp" \
            ../grabberRS/GrabberLib_global.h

CONFIG( debug, debug|release ) {
    LIBS+=             "C:/BUILDS/IntelRealSense_SDK/Debug/realsense2.lib"
    LIBS += C:/BUILDS/openCV/lib/Debug/opencv_core330d.lib
    LIBS += C:/BUILDS/openCV/lib/Debug/opencv_highgui330d.lib
    LIBS +=C:/BUILDS/openCV/lib/Debug/opencv_imgcodecs330d.lib
    LIBS += C:/BUILDS/openCV/lib/Debug/opencv_imgproc330d.lib
    LIBS += C:/BUILDS/openCV/lib/Debug/opencv_calib3d330d.lib
    LIBS += C:/BUILDS/openCV/lib/Debug/opencv_video330d.lib
    LIBS += C:/BUILDS/openCV/lib/Debug/opencv_videoio330d.lib
    LIBS += C:/BUILDS/openCV/lib/Debug/opencv_videostab330d.lib
    LIBS +=  "C:/BUILDS/glfw-3.2.1/src/Debug/glfw3dll.lib"

}else {
    LIBS+= "C:/BUILDS/IntelRealSense_SDK/Release/realsense2.lib"
    LIBS += C:/BUILDS/openCV/lib/Release/opencv_core330.lib
    LIBS += C:/BUILDS/openCV/lib/Release/opencv_highgui330.lib
    LIBS +=C:/BUILDS/openCV/lib/Release/opencv_imgcodecs330.lib
    LIBS += C:/BUILDS/openCV/lib/Release/opencv_imgproc330.lib
    LIBS += C:/BUILDS/openCV/lib/Release/opencv_calib3d330.lib
    LIBS += C:/BUILDS/openCV/lib/Release/opencv_video330.lib
    LIBS += C:/BUILDS/openCV/lib/Release/opencv_videoio330.lib
    LIBS += C:/BUILDS/openCV/lib/Release/opencv_videostab330.lib
    LIBS +=  "C:/BUILDS/glfw-3.2.1/src/Release/glfw3dll.lib"
}


LIBS+= ../grabberRS/grabberRS.lib

