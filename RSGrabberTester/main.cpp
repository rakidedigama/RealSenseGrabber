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


    int width = 640;
    int height = 480;
    int count = 0;
    int depthBufferSize = width*height*2; // 16 bit pixels
    int colorBufferSize = width*height*3; // 24 bit color pixels BGR
    bool depthImage = 1;

    while(true){
        if(depthImage){

            Mat openDepth((Size(width, height)), CV_16UC1);
            Mat openDepthResized8(height,width,CV_8UC1);
            unsigned short *depthImgPtr = (unsigned short*)openDepth.data;
            int out = grabber.getImage(depthImgPtr,depthBufferSize,1000);
            qDebug()<<out;
            if(out==1){
                count = count + 1;
                //qDebug()<<count;
                if(!openDepth.empty()){
                     openDepth.convertTo(openDepthResized8,CV_8UC1,0.00390625);
                    //openDepthResized8 = 255-openDepthResized8;
                    cv::imshow(WINDOW_NAME_1,openDepthResized8);
                 cv::waitKey(30);

                 }
             }
            if(out==-1){
                qDebug()<<"Reconnect";
            }
        }




        if(!depthImage){
            Mat openColor((Size(width, height)), CV_8UC3);
            unsigned short *colorImgPtr = (unsigned short*)openColor.data;
            int out = grabber.getImage(colorImgPtr,colorBufferSize,1000);
            if(out==1){
                count = count + 1;
                 qDebug()<<count;
                 if(!openColor.empty())
                       cv::cvtColor(openColor,openColor,COLOR_BGR2RGB);
                       cv::imshow(WINDOW_NAME_1,openColor);
                       cv::waitKey(30);

                 }
            }


     }


    return a.exec();
}
