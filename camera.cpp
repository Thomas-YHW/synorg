#include "mainwindow.h"
#include "ui_mainwindow_touch.h"
#include "zmotion.h"
#include "zmcaux.h"
#include "lib/include/MvCameraControl.h"
#include "fgt_SDK.h"
#include "fgt_SDK_Cpp.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <QSettings>
#include <chrono>
#include <QDebug>
#include <QObject>
#include <QButtonGroup>
#include <QMessageBox>
#include <thread>
#include <iostream>
#include <cstring>
#include <cstdlib>
#include<vector>

using namespace cv;
using namespace std;

#define MAX_BUF_SIZE (2448 * 2048 * 3)

extern QImage gShow;
extern QImage gmodelFullShow;

volatile bool bResetCamera = false;
extern volatile bool bsaveModel;
extern volatile bool bgetModel;
extern volatile bool bcompareImage;

volatile bool bsetRoi = false;
int showImageCounter = 1;
int roix = 0;
int roiy = 0;
int roiw = 100;
int roih = 100;
int printNumber = 0;//实际打印微球的个数
int theNumber = 1;//用户需要打印微球的个数

Mat gmodelImage;
Mat diffImage;

QImage gdiff;


int area_center(Mat src);
int image_Processing(Mat imgOriginal);
int empaseDifferent(Mat src, int minDiff);
int mydiff(Mat src, int minDiff);
extern volatile int gMinArea;
extern volatile int gMinThresold;
extern volatile bool gbDrops;   //drops done
extern volatile bool gbArrival; //move ok
volatile bool bshowImage = false;
extern ZMC_HANDLE cg_handle; //zmotion handle for print action.

void cameraPrint();

QImage QCVMat2QImage(const cv::Mat &mat)
{
    const unsigned char *data = mat.data;

    int width = mat.cols;
    int height = mat.rows;
    int bytesPerLine = static_cast<int>(mat.step);
    switch (mat.type())
    {
    // 8 bit , ARGB
    case CV_8UC4:
    {
        QImage image(data, width, height, bytesPerLine, QImage::Format_ARGB32);
        return image;
    }
        // 8 bit BGR
    case CV_8UC3:
    {
        QImage image(data, width, height, bytesPerLine, QImage::Format_RGB888);
        // swap blue and red channel
        return image.rgbSwapped();
    }
        // 8 bit Gray shale
    case CV_8UC1:
    {
        QImage image(data, width, height, bytesPerLine, QImage::Format_Grayscale8);
        return image;
    }
        //
    default:
    {
        // Unsupported format
        qWarning() << "Unsupported cv::Mat type:" << mat.type()
                   << ", Empty QImage will be returned!";
        return QImage();
    }
    }
}


void cameraMain() {
    int nRet = -1;
    void *m_handle = NULL;

    //枚举子网内指定的传输协议对应的所有设备
    unsigned int nTLayerType = MV_GIGE_DEVICE;
    MV_CC_DEVICE_INFO_LIST m_stDevList = {0};
    nRet = MV_CC_EnumDevices(nTLayerType, &m_stDevList);
    if (nRet != 0) {
        printf("error: EnumDevices fail [%x]\n", nRet);
    }

    if (m_stDevList.nDeviceNum == 0) {
        printf("no camera found!\n");
    }

    //选择查找到的第一台在线设备，创建设备句柄
    int nDeviceIndex = 0;
    nRet = MV_CC_CreateHandle(&m_handle, m_stDevList.pDeviceInfo[nDeviceIndex]);

    if (nRet != 0) {
        printf("error: CreateHandle fail [%x]\n", nRet);
    }

    //连接设备
    unsigned int nAccessMode = MV_ACCESS_Exclusive;
    unsigned short nSwitchoverKey = 0;
    nRet = MV_CC_OpenDevice(m_handle, nAccessMode, nSwitchoverKey);
    if (nRet != 0) {
        printf("error: OpenDevice fail [%x]\n", nRet);
    }

    if(bResetCamera)
    {
        //恢复ROI
        MVCC_INTVALUE_EX nWidthMaxValue= {0},nHeightMaxValue = {0};
        nRet = MV_CC_GetIntValueEx(m_handle, "WidthMax", &nWidthMaxValue);
        if (MV_OK != nRet)
        {
            printf("Get WidthMax fail! nRet [0x%x]\n", nRet);
        }
        nRet = MV_CC_GetIntValueEx(m_handle, "HeightMax", &nHeightMaxValue);
        if (MV_OK != nRet)
        {
            printf("Get HeightMax fail! nRet [0x%x]\n", nRet);
        }
        //一定要先还原相机的偏移值，再去设置相机宽高
        nRet = MV_CC_SetIntValue(m_handle,"OffsetX",0);
        nRet = MV_CC_SetIntValue(m_handle,"OffsetY",0);
        nRet = MV_CC_SetIntValue(m_handle,"Width",nWidthMaxValue.nCurValue);//设置为最大值
        if(nRet != MV_OK)
        {
            printf("Warning: Set Width  fail nRet [0x%x]!", nRet);
        }
        nRet = MV_CC_SetIntValue(m_handle,"Height",nHeightMaxValue.nCurValue);//设置为最大值
        if(nRet != MV_OK)
        {
            printf("Warning: Set Height  fail nRet [0x%x]!", nRet);
        }

        bResetCamera = false;
    }



    //开始采集图像
    nRet = MV_CC_StartGrabbing(m_handle);
    if (nRet != 0) {
        printf("error: StartGrabbing fail [%x]\n", nRet);
    }

    //获取一帧数据的大小
    MVCC_INTVALUE stIntvalue = {0};
    nRet = MV_CC_GetIntValue(m_handle, "PayloadSize", &stIntvalue);
    if (nRet != 0) {
        printf("Get PayloadSize failed! nRet [%x]\n", nRet);
    }
    // int nBufSize = stIntvalue.nCurValue; //一帧数据大小
    int nBufSize = MAX_BUF_SIZE;
    unsigned char *pFrameBuf = NULL;
    pFrameBuf = (unsigned char *)malloc(nBufSize);

    MV_FRAME_OUT_INFO_EX stInfo;
    memset(&stInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));

    //上层应用程序需要根据帧率，控制好调用该接口的频率
    //此次代码仅供参考，实际应用建议另建线程进行图像帧采集和处理
    while (true) {

        if(bResetCamera)
        {
            MV_CC_StopGrabbing(m_handle);

            //恢复ROI
            MVCC_INTVALUE_EX nWidthMaxValue= {0},nHeightMaxValue = {0};
            nRet = MV_CC_GetIntValueEx(m_handle, "WidthMax", &nWidthMaxValue);
            if (MV_OK != nRet)
            {
                printf("Get WidthMax fail! nRet [0x%x]\n", nRet);
            }
            nRet = MV_CC_GetIntValueEx(m_handle, "HeightMax", &nHeightMaxValue);
            if (MV_OK != nRet)
            {
                printf("Get HeightMax fail! nRet [0x%x]\n", nRet);
            }
            //一定要先还原相机的偏移值，再去设置相机宽高
            nRet = MV_CC_SetIntValue(m_handle,"OffsetX",0);
            nRet = MV_CC_SetIntValue(m_handle,"OffsetY",0);
            nRet = MV_CC_SetIntValue(m_handle,"Width",nWidthMaxValue.nCurValue);//设置为最大值
            if(nRet != MV_OK)
            {
                printf("Warning: Set Width  fail nRet [0x%x]!", nRet);
            }
            nRet = MV_CC_SetIntValue(m_handle,"Height",nHeightMaxValue.nCurValue);//设置为最大值
            if(nRet != MV_OK)
            {
                printf("Warning: Set Height  fail nRet [0x%x]!", nRet);
            }
            nRet = MV_CC_StartGrabbing(m_handle);
            bResetCamera = false;
            printf("finish reset camera full roi\n");

            //重新分配图像内存
            free(pFrameBuf);
            pFrameBuf = (unsigned char *)malloc(nBufSize);
        }

        if(bsetRoi){
            bsetRoi = false;
            MV_CC_StopGrabbing(m_handle);

            // nRet = MV_CC_SetIntValue(m_handle,"OffsetX",0);
            //nRet = MV_CC_SetIntValue(m_handle,"OffsetY",0);
            nRet = MV_CC_SetIntValue(m_handle,"Width",408);//设置为最大值
            if(nRet != MV_OK)
            {
                printf("Warning: Set Width  fail nRet [0x%x]!", nRet);
            }
            nRet = MV_CC_SetIntValue(m_handle,"Height",1276);//设置为最大值
            if(nRet != MV_OK)
            {
                printf("Warning: Set Height  fail nRet [0x%x]!", nRet);
            }
            //nRet = MV_CC_SetIntValue(m_handle,"OffsetX",924);
            //nRet = MV_CC_SetIntValue(m_handle,"OffsetY",494);

            nRet = MV_CC_SetIntValue(m_handle,"OffsetX",1036);
            nRet = MV_CC_SetIntValue(m_handle,"OffsetY",68);

            printf("finish set camera small roi, x:%d, y:%d  width:%d, heigth:%d\n",roix, roiy, roiw, roih);
            //重新分配图像内存
            free(pFrameBuf);
            //pFrameBuf = (unsigned char *)malloc(roiw*roih*3);
            //pFrameBuf = (unsigned char *)malloc(456*672*3);
            pFrameBuf = (unsigned char *)malloc(408*1276*3);

            nRet = MV_CC_StartGrabbing(m_handle);
        }

        if (true) {
            //  printf("photo while...\n");
            nRet = MV_CC_GetImageForBGR(m_handle, pFrameBuf, nBufSize, &stInfo, 1000);
            if (nRet != 0) {
                //cout << "error:GetImageForRGB:" << setbase(16) << nRet << endl;
                printf("error:GetImageForRGB:\n");
                break;
            } else {
                //...图像数据处理
                //cout << "imagetype:" << setbase(16) << stInfo.enPixelType << endl;

                int width = stInfo.nWidth;
                int height = stInfo.nHeight;

                if (stInfo.enPixelType == PixelType_Gvsp_BGR8_Packed) {
                    cv::Mat pImg(height, width, CV_8UC3, pFrameBuf);
                    // gImage = pImg.clone();
                    if(bsaveModel){
                        bsaveModel = false;
                        cv::imwrite("model.bmp", pImg);
                        gmodelImage = pImg.clone();
                    }
                    if(bgetModel){
                        gmodelFullShow = QCVMat2QImage(pImg);
                        cv::Mat grayImagetmp;
                        cvtColor(pImg, grayImagetmp, cv::COLOR_BGR2GRAY);
                        gmodelImage = grayImagetmp.clone();
                        //保存到硬盘用于下次操作
                        imwrite("model.bmp", gmodelImage);
                        imwrite("color.bmp", pImg);
                        bgetModel = false;
                    }

                    if(bcompareImage){
                        while(!gbArrival)
                        {
                            //wait for moto arrial
                        }

                        /*cv::Mat tempImage;
                        cvtColor(pImg, tempImage, cv::COLOR_BGR2GRAY);
                        //cv::absdiff(gmodelImage, tempImage, diffImage);
                        diffImage = gmodelImage-tempImage;
                        mydiff(diffImage, gMinThresold);
                        auto resultArea =  area_center(diffImage);
                        printf("different area: %d\n", resultArea);*/


                        auto resultArea = image_Processing(pImg);
                        if(resultArea>gMinArea)
                        {
                            // check if motion stop...
                            int status;
                            while(true)
                            {
                                ZAux_Direct_GetIfIdle(cg_handle, 0, &status);
                                if(status != 0)
                                    break;
                            }

                            while(true)
                            {
                                ZAux_Direct_GetIfIdle(cg_handle, 1, &status);
                                if(status != 0)
                                    break;
                            }
                           cameraPrint();
                           printNumber++;
                           if(printNumber >= theNumber)
                           {
                               gbDrops = true;
                               gbArrival = false;
                               printNumber = 0;
                           }


                           printf("set");//
                        }
                        if(bshowImage){
                            gShow = QCVMat2QImage(pImg);
                        }
                       // empaseDifferent(diffImage, 55);
                       // gdiff = QCVMat2QImage(diffImage);
                        //gdiff = QCVMat2QImage(pImg);
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        resultArea = 0;
                    }

                    flip(pImg, pImg, -1);

                    showImageCounter++;
                    if(showImageCounter<200)
                    gShow = QCVMat2QImage(pImg);
                    // printf("photo update...\n");
                    //std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    /*cv::imwrite("2.bmp", pImg);
                    cv::imshow("1", pImg);
                    cv::waitKey(10);*/
                }
            }
        }
    }

    //停止采集图像
    nRet = MV_CC_StopGrabbing(m_handle);
    if (nRet != 0) {
        printf("error: StopGrabbing fail [%x]\n", nRet);
    }

    //关闭设备，释放资源
    nRet = MV_CC_CloseDevice(m_handle);
    if (nRet != 0) {
        printf("error: CloseDevice fail [%x]\n", nRet);
    }

    //销毁句柄，释放资源
    nRet = MV_CC_DestroyHandle(m_handle);
    if (nRet != 0) {
        printf("error: DestroyHandle fail [%x]\n", nRet);
    }

    printf("camera exit!\n");
    return;
}



int area_center(Mat src)
{
    Point2f center;
    int area;
    int pixelsCount = src.rows * src.cols;
    area = 0;
    center = Point2f(0, 0);
    float centerX = 0;
    float centerY = 0;
    int mcol = src.cols;
    for(int i=0;i<pixelsCount;i++)
    {
        if(src.data[i] > gMinThresold)
        {
            area++;

            int x = i % mcol;
            int y = i / mcol;

            centerX += x;
            centerY += y;
        }
    }
    if (area > 0)
    {
        centerX /= area;
        centerY /= area;
        center = Point2f(centerX, centerY);
    }

    return area;
}

int image_Processing(Mat imgOriginal)
{
    Mat imgHSV, imgBGR;
    Mat imgThresholded;
    //imshow("chushi", imgOriginal);
    //GaussianBlur(imgOriginal, imgOriginal, Size(7, 7), 0, 0);
    //imshow("gaosi", imgOriginal);
    vector<Mat> hsvSplit;   //创建向量容器，存放HSV的三通道数据
    cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
    split(imgHSV, hsvSplit);			//分类原图像的HSV三通道
    equalizeHist(hsvSplit[2], hsvSplit[2]);    //对HSV的亮度通道进行直方图均衡
    merge(hsvSplit, imgHSV);				   //合并三种通道
    Mat ce1;
    Mat ce2;
    Mat ce3;
    //inRange(imgHSV, Scalar(156, 43, 46), Scalar(180, 255, 255), imgThresholded); //红色
    inRange(imgHSV, Scalar(0, 83, 166), Scalar(10, 255, 255), ce1); //红色
   // inRange(imgHSV, Scalar(0, 43, 46), Scalar(3, 255, 255), ce2); //红色
    //add(ce1,ce2,ce3,Mat());
    //imshow("0-10", ce1);
   // imshow("156-180",ce2);
    auto counterArea = area_center(ce1);
    std::cout<<"Area:"<<counterArea<<std::endl;
    //imshow("add后", ce3);

    return counterArea;
}


int empaseDifferent(Mat src, int minDiff)
{

    int pixelsCount = src.rows * src.cols;
    for(int i=0;i<pixelsCount;i++)
    {
        if(src.data[i] >= minDiff)
        {
            src.data[i] = 255;
        }
        else
        {
            src.data[i] = 0;
        }
    }
}

int mydiff(Mat src, int minDiff)
{

    int pixelsCount = src.rows * src.cols;
    for(int i=0;i<pixelsCount;i++)
    {
        if(src.data[i] >= minDiff)
        {
            src.data[i] = 255;
        }
        else
        {
            src.data[i] = 0;
        }
    }
}

extern int gprintTime;

void cameraPrint()
{

    ZAux_Direct_SetOp(cg_handle, 8, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ZAux_Direct_SetOp(cg_handle, 8, 0);
}
