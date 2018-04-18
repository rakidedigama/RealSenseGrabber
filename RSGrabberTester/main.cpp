#include <QCoreApplication>
#include "GrabberLib_global.h"
#include "grabber.h"
#include "QConsoleDebugStream.h"

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgproc.hpp"
#include "qDebug"

using namespace cv;
const String  WINDOW_NAME_1("Depth video");
int main(int argc, char *argv[])
{
    qDebug()<<"Starting Client ";
    QCoreApplication a(argc, argv);
    std::string qsIniFile = "CameraThreadViewer.ini";
    GrabberRS grabber(qsIniFile);
    grabber.start();
    //grabber.startGrabbing();

    int width = 640;
    int height = 480;
    int count = 0;
    int bufferSize = width*height*2; // 16 bit pixels
    qDebug()<<"Buffer size" << bufferSize;
    while(true){
        Mat openDepth((Size(width, height)), CV_16UC1);
        unsigned short *depthImgPtr = (unsigned short*)openDepth.data;
        int out = grabber.getImage(depthImgPtr,bufferSize,1000);
        if(out==1){
            count = count + 1;
        }
         qDebug()<<count;
         if(!openDepth.empty())
             cv::imshow(WINDOW_NAME_1,openDepth);
        cv::waitKey(30);

    }
    return a.exec();
}
