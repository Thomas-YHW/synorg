#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "zmotion.h"
#include "zmcaux.h"
#include <QDebug>
#include <QObject>
#include <QButtonGroup>
#include <QMessageBox>
#include <thread>
#include <iostream>
#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include "lib/include/MvCameraControl.h"
#include "fgt_SDK.h"
#include "fgt_SDK_Cpp.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include <QSettings>
#include <chrono>

using namespace cv;

#define MAX_BUF_SIZE (2448 * 2048 * 3)

int sid1, sid2, pid1, pid2;
Mat gImage;
QImage gShow;
bool takeFlag = true;

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
MV_CC_DEVICE_INFO_LIST stDeviceList = {0};
//枚举相机
void listCamera()
{
    //读取各区域
    do
    {
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            printf("Enum Devices fail! nRet [0x%x]\n", nRet);
            break;
        }

        if (stDeviceList.nDeviceNum > 0)
        {
            for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                printf("[device %d]:\n", i);
                MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (NULL == pDeviceInfo)
                {
                    break;
                }
            }
        }
        else
        {
            printf("Find No Devices!\n");
            break;
        }
    } while (0);
}

//相机线程，用于设置相机，并且回调图片生成
void photoThread()
{

    int nRet;
    void *handle = NULL;

    listCamera();
    do
    {
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]);

        if (MV_OK != nRet)
        {
            printf("Create Handle fail! nRet [0x%x]\n", nRet);
            break;
        }

        // Open device
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet)
        {
            printf("Open Device fail! nRet [0x%x]\n", nRet);
            break;
        }

        // Detection network optimal package size(It only works for the GigE camera)
        if (stDeviceList.pDeviceInfo[0]->nTLayerType == MV_GIGE_DEVICE)
        {
            int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
            if (nPacketSize > 0)
            {
                nRet = MV_CC_SetIntValue(handle, "GevSCPSPacketSize", nPacketSize);
                if (nRet != MV_OK)
                {
                    printf("Warning: Set Packet Size fail nRet [0x%x]!", nRet);
                }
            }
            else
            {
                printf("Warning: Get Packet Size fail nRet [0x%x]!", nPacketSize);
            }
        }

        // Set trigger mode as off
        nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
        // MV_CC_SetTriggerMode(handle, 1);
        // MV_CC_SetTriggerSource(handle, MV_TRIGGER_SOURCE_LINE0);
        // MV_CC_SetTriggerSource(handle, MV_TRIGGER_SOURCE_SOFTWARE);

        if (MV_OK != nRet)
        {
            printf("Set Trigger Mode fail! nRet [0x%x]\n", nRet);
            break;
        }

        MV_CC_SetExposureTime(handle, 3000);
        MV_CC_SetGamma(handle, 0.7);
        MV_CC_SetGain(handle, 8);
        MV_CC_SetPixelFormat(handle, PixelType_Gvsp_RGB8_Packed);

        /*   nRet = MV_CC_RegisterImageCallBackEx(handle, ImageCallBackEx, handle);
           if (MV_OK != nRet) {
             printf("Register Image CallBack fail! nRet [0x%x]\n", nRet);
           }*/

        // Start grab image
        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("Start Grabbing fail! nRet [0x%x]\n", nRet);
            break;
        }
    } while (0);

    //在此循环
    unsigned char *m_pBufForSaveImage = NULL;
    bool bAlloc = true;
    while (true)
    {
        // 设置相机参数

        if (takeFlag)
        {

            unsigned int nRecvBufSize = 0;
            MVCC_INTVALUE stParam;
            memset(&stParam, 0, sizeof(MVCC_INTVALUE));
            int nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
            if (nRet != 0)
            {
                return;
            }
            nRecvBufSize = stParam.nCurValue;
            unsigned char *m_pBufForDriver = NULL;

            if (bAlloc)
                m_pBufForDriver = (unsigned char *)malloc(nRecvBufSize);
            MV_FRAME_OUT_INFO_EX stImageInfo = {0};
            nRet = MV_CC_GetOneFrameTimeout(handle, m_pBufForDriver, nRecvBufSize,
                                            &stImageInfo, 1000);
            if (nRet != 0)
            {
                return;
            }
            auto m_nBufSizeForSaveImage =
                stImageInfo.nWidth * stImageInfo.nHeight * 3 + 2048;

            if (bAlloc)
                if (m_pBufForSaveImage == NULL)
                {
                    m_pBufForSaveImage = (unsigned char *)malloc(m_nBufSizeForSaveImage);
                }
            bAlloc = false;
            bool isMono; //判断是否为黑白图像
            switch (stImageInfo.enPixelType)
            {
            case PixelType_Gvsp_Mono8:
            case PixelType_Gvsp_Mono10:
            case PixelType_Gvsp_Mono10_Packed:
            case PixelType_Gvsp_Mono12:
            case PixelType_Gvsp_Mono12_Packed:
                isMono = true;
                break;
            default:
                isMono = false;
                break;
            }

            if (isMono)
            {
                gImage = Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC1,
                             m_pBufForDriver);
            }
            else
            {
                //转换图像格式为BGR8
                MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
                memset(&stConvertParam, 0, sizeof(MV_CC_PIXEL_CONVERT_PARAM));
                stConvertParam.nWidth = stImageInfo.nWidth;
                stConvertParam.nHeight = stImageInfo.nHeight;
                stConvertParam.pSrcData = m_pBufForDriver;
                stConvertParam.nSrcDataLen = stImageInfo.nFrameLen;
                stConvertParam.enSrcPixelType = stImageInfo.enPixelType;
                stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
                // stConvertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
                stConvertParam.pDstBuffer = m_pBufForSaveImage;
                stConvertParam.nDstBufferSize = m_nBufSizeForSaveImage;
                MV_CC_ConvertPixelType(handle, &stConvertParam);
                gImage = Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3,
                             m_pBufForSaveImage)
                             .clone();
            }

            // imwrite("./testx.bmp", gImage);
            takeFlag = false;
            // Sleep(1000);
            gShow = QCVMat2QImage(gImage);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }

    nRet = MV_CC_StopGrabbing(handle);
    if (MV_OK != nRet)
    {
        printf("Stop Grabbing fail! nRet [0x%x]\n", nRet);
    }

    // Close device
    nRet = MV_CC_CloseDevice(handle);
    if (MV_OK != nRet)
    {
        printf("ClosDevice fail! nRet [0x%x]\n", nRet);
    }

    // Destroy handle
    nRet = MV_CC_DestroyHandle(handle);
    if (MV_OK != nRet)
    {
        printf("Destroy Handle fail! nRet [0x%x]\n", nRet);
    }

    if (nRet != MV_OK)
    {
        if (handle != NULL)
        {
            MV_CC_DestroyHandle(handle);
            handle = NULL;
        }
    }

    return;
}


void camThread2()
{

    int nRet = -1;
    void *m_handle = NULL;

    //枚举子网内指定的传输协议对应的所有设备
    unsigned int nTLayerType = MV_GIGE_DEVICE | MV_USB_DEVICE;
    MV_CC_DEVICE_INFO_LIST m_stDevList = {0};
    nRet = MV_CC_EnumDevices(nTLayerType, &m_stDevList);
    if (MV_OK != nRet)
    {
        printf("error: EnumDevices fail [%x]\n", nRet);
        return;
    }

    int i = 0;
    if (m_stDevList.nDeviceNum == 0)
    {
        printf("no camera found!\n");
        return;
    }

    //选择查找到的第一台在线设备，创建设备句柄
    int nDeviceIndex = 0;

    MV_CC_DEVICE_INFO m_stDevInfo = {0};
    memcpy(&m_stDevInfo, m_stDevList.pDeviceInfo[nDeviceIndex], sizeof(MV_CC_DEVICE_INFO));

    nRet = MV_CC_CreateHandle(&m_handle, &m_stDevInfo);
    if (MV_OK != nRet)
    {
        printf("error: CreateHandle fail [%x]\n", nRet);
        return;
    }

    //连接设备
    // nRet = MV_CC_OpenDevice(m_handle, nAccessMode, nSwitchoverKey);
    /*
nAccessMode = MV_ACCESS_Exclusive，设备访问模式，默认独占模式 MV_ACCESS_Exclusive：独占权限，其他APP只允许读CCP寄存器。
nSwitchoverKey = 0 切换权限时的密钥，默认为无，访问模式支持权限切换（2/4/6模式）时有效。
*/
    nRet = MV_CC_OpenDevice(m_handle);
    if (MV_OK != nRet)
    {
        printf("error: OpenDevice fail [%x]\n", nRet);
        return;
    }

    //...其他处理

    // 设置连续模式
    unsigned int enMode = MV_TRIGGER_MODE_OFF; //关闭触发源
    nRet = MV_CC_SetEnumValue(m_handle, "TriggerMode", enMode);
    std::cout << "Trigger mode off\n";
    if (MV_OK != nRet)
    {
        printf("error: Trigger mode off fail [%x]\n", nRet);
        return;
    }

    /* 设置自动曝光
   0 : Off
   1 : Once
   2 ：Continuous  */
    nRet = MV_CC_SetEnumValue(m_handle, "ExposureAuto", 2);
    std::cout << "Exposure auto\n";
    if (MV_OK != nRet)
    {
        printf("error: Exposure auto fail [%x]\n", nRet);
        return;
    }

    /* 设置自动增益
   0 : Off
   1 : Once
   2 ：Continuous  */
    nRet = MV_CC_SetEnumValue(m_handle, "GainAuto", 2);
    std::cout << "Gain auto\n";
    if (MV_OK != nRet)
    {
        printf("error: Gain auto fail [%x]\n", nRet);
        return;
    }

    /* 设置自动白平衡
   0 : Off
   2 : Once
   1 ：Continuous  */
    nRet = MV_CC_SetEnumValue(m_handle, "BalanceWhiteAuto", 1);
    std::cout << "Balance white auto\n";
    if (MV_OK != nRet)
    {
        printf("error: Balance white auto fail [%x]\n", nRet);
        return;
    }

    //开始采集图像
    nRet = MV_CC_StartGrabbing(m_handle);
    std::cout << "Start grabbing...\n";
    if (MV_OK != nRet)
    {
        printf("error: StartGrabbing fail [%x]\n", nRet);
        return;
    }

    //获取一帧数据的大小
    MVCC_INTVALUE stIntvalue = {0};
    nRet = MV_CC_GetIntValue(m_handle, "PayloadSize", &stIntvalue);
    if (nRet != MV_OK)
    {
        printf("Get PayloadSize failed! nRet [%x]\n", nRet);
        return;
    }
    int nBufSize = stIntvalue.nCurValue; //一帧数据大小

    unsigned int nTestFrameSize = 0;
    unsigned char *pFrameBuf = NULL;
    pFrameBuf = (unsigned char *)malloc(nBufSize);

    MV_FRAME_OUT_INFO_EX stInfo;
    memset(&stInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));

    //上层应用程序需要根据帧率，控制好调用该接口的频率
    //此次代码仅供参考，实际应用建议另建线程进行图像帧采集和处理
    while (1)
    {
        if (nTestFrameSize > 9)
        {
           // break;
        }
        nRet = MV_CC_GetOneFrameTimeout(m_handle, pFrameBuf, nBufSize, &stInfo, 1000);
        if (MV_OK != nRet)
        {
            for (size_t i = 0; i < 10000; i++)
            {
                for (size_t j = 0; j < 100000; j++)
                {
                    /* code */
                }
            }
        }
        else
        {
            //图片数据输入输出参数
            MV_SAVE_IMAGE_PARAM_EX stParam;

            //源数据
            stParam.pData = pFrameBuf;                //原始图像数据
            stParam.nDataLen = stInfo.nFrameLen;      //原始图像数据长度
            stParam.enPixelType = stInfo.enPixelType; //原始图像数据的像素格式
            stParam.nWidth = stInfo.nWidth;           //图像宽
            stParam.nHeight = stInfo.nHeight;         //图像高
            stParam.nJpgQuality = 90;                 // JPEG图片编码质量

            // ch:jpg图像质量范围为(50-99], png图像质量范围为[0-9]

            //目标数据
            stParam.enImageType = MV_Image_Jpeg; //需要保存的图像类型，转换成JPEG格式
            // stParam.enImageType = MV_Image_Png;            //需要保存的图像类型，转换成PNG格式
            stParam.nBufferSize = nBufSize; //存储节点的大小
            unsigned char *pImage = (unsigned char *)malloc(nBufSize);
            stParam.pImageBuffer = pImage; //输出数据缓冲区，存放转换之后的图片数据

            nRet = MV_CC_SaveImageEx2(m_handle, &stParam);
            if (MV_OK != nRet)
            {
                break;
            }

            // ui.imagesetPixmap(QPixmap::fromImage(QImage::fromData((const char*)pImage)));

            //将转换之后图片数据保存成文件
            FILE *fp = fopen("test3.png", "wb"); // "wb" 只写打开或新建一个二进制文件；只允许写数据。
            fwrite(pImage, 1, stParam.nImageLen, fp);
            fclose(fp);
          //  ui->image->setPixmap(QPixmap("/home/zmotion/build-single_home-Desktop-Release/test3.png"));
            free(pImage);

            //...其他图像数据处理
            nTestFrameSize++;
        }
    }
    free(pFrameBuf);

    //...其他处理

    //停止采集图像
    nRet = MV_CC_StopGrabbing(m_handle);
    if (MV_OK != nRet)
    {
        printf("error: StopGrabbing fail [%x]\n", nRet);
        return;
    }

    //关闭设备，释放资源
    nRet = MV_CC_CloseDevice(m_handle);
    if (MV_OK != nRet)
    {
        printf("error: CloseDevice fail [%x]\n", nRet);
        return;
    }

    //销毁句柄，释放资源
    nRet = MV_CC_DestroyHandle(m_handle);
    if (MV_OK != nRet)
    {
        printf("error: DestroyHandle fail [%x]\n", nRet);
        return;
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

  // MV_CC_DEVICE_INFO m_stDevInfo = {0};
  /*memcpy(&m_stDevInfo, m_stDevList.pDeviceInfo[nDeviceIndex],
         sizeof(MV_CC_DEVICE_INFO));*/

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
  //...其他处理

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
  unsigned int nTestFrameSize = 0;
  unsigned char *pFrameBuf = NULL;
  pFrameBuf = (unsigned char *)malloc(nBufSize);

  MV_FRAME_OUT_INFO_EX stInfo;
  memset(&stInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));

  //上层应用程序需要根据帧率，控制好调用该接口的频率
  //此次代码仅供参考，实际应用建议另建线程进行图像帧采集和处理
  while (1) {
    if (true) {
      printf("photo while...\n");
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
          gShow = QCVMat2QImage(pImg);
          printf("photo update...\n");
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          /*cv::imwrite("2.bmp", pImg);
          cv::imshow("1", pImg);
          cv::waitKey(10);*/
        }
      }

      //takeFlag = false;
    }
  }

  //...其他处理

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
  system("pause");
}

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent),
                                          ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    m_units = 1;
    m_lspeed = 0;
    m_speed = 100;
    m_creep = 10;
    m_acc = 3000;
    m_dec = 3000;
    m_datumin = 0;
    m_nAxis = 0;

    ui->radio_X->setChecked(true);
    bt = new QButtonGroup(this);
    bt->addButton(ui->radio_1, 0);
    bt->addButton(ui->radio_2, 1);
    bt->addButton(ui->radio_3, 2);
    bt->addButton(ui->radio_4, 3);
    bt->addButton(ui->radio_5, 4);
    bt->addButton(ui->radio_6, 5);
    ui->radio_1->setChecked(true);

    // get ip from current list
    int32 iresult;
    // char tmp_buff[16] = {0};
    const char *tmp_buff = "192.168.0.11";
    QString str;
    QString str_title;
    if (0 != g_handle)
    {
        ZMC_Close(g_handle);
    }
    iresult = ZMC_OpenEth((char *)tmp_buff, &g_handle);
    if (0 == iresult)
    {
        str_title = ("linked:");
        str_title += tmp_buff;
        setWindowTitle(str_title);

         QSettings *configIniWrite = new QSettings("config.ini", QSettings::IniFormat);

    //x aixs
         float speed = configIniWrite->value("machineX/m_speed").toFloat();
         if(speed > 0.1)
         {
            loadSpeed();
         }

        id1 = startTimer(100);
    }

    else
    {
        setWindowTitle("no link!");
    }

    initFgtUi();
    id1 = startTimer(100);

    ui->image->setPixmap(QPixmap::fromImage(QImage("./test3.png")));
    std::thread cam(cameraMain);
    cam.detach();

    initPorts();
    gShow = QImage("./logo.bmp");
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setLimitedPos()
{
    ZAux_Direct_SetFsLimit(g_handle, 0, 10000);
    ZAux_Direct_SetRsLimit(g_handle, 0, 0);
}

void MainWindow::backZero()
{
    int status = 0;

    m_units = 100;
    m_lspeed = 20;
    m_speed = 100;
    m_acc = 100;
    m_dec = 100;
    m_creep = 20;
    // X aixs
    ZAux_Direct_GetIfIdle(g_handle, 0, &status); //判断当前轴状态
    if (status == 0)                             //已经在运动中
        return;
    //设定轴类型 7-   脉冲轴类型 +	编码器Z信号		不用EZ回零也可以设置为1
    ZAux_Direct_SetAtype(g_handle, 0, 7);
    //设定脉冲模式及逻辑方向（脉冲+方向）
    ZAux_Direct_SetInvertStep(g_handle, 0, 0);
    //设置脉冲当量	1表示以一个脉冲为单位 ，设置为1MM的脉冲个数，这度量单位为MM
    ZAux_Direct_SetUnits(g_handle, 0, m_units);
    //设定速度，加减速
    ZAux_Direct_SetLspeed(g_handle, 0, m_lspeed);
    ZAux_Direct_SetSpeed(g_handle, 0, m_speed);
    ZAux_Direct_SetAccel(g_handle, 0, m_acc);
    ZAux_Direct_SetDecel(g_handle, 0, m_dec);
    ZAux_Direct_SetCreep(g_handle, 0, m_creep);
    //设定对应轴的原点输入口信号
    ZAux_Direct_SetDatumIn(g_handle, 0, 1);
    ZAux_Direct_SetInvertIn(g_handle, 1, 1);  // ZMC系列认为OFF时碰到了原点信号（常闭） ，如果是常开传感器则需要反转输入口，ECI系列的不需要反转
    ZAux_Direct_Single_Datum(g_handle, 0, 8); //模式8，9
    // X limited
    // ZAux_Direct_SetDpos(g_handle, 0, 10000);
    ZAux_Direct_SetFsLimit(g_handle, 0, 1000);
    ZAux_Direct_SetRsLimit(g_handle, 0, 0);

    // Y aixs
    ZAux_Direct_GetIfIdle(g_handle, 1, &status); //判断当前轴状态
    if (status == 0)                             //已经在运动中
        return;
    //设定轴类型 7-   脉冲轴类型 +	编码器Z信号		不用EZ回零也可以设置为1
    ZAux_Direct_SetAtype(g_handle, 1, 7);
    //设定脉冲模式及逻辑方向（脉冲+方向）
    ZAux_Direct_SetInvertStep(g_handle, 1, 0);
    //设置脉冲当量	1表示以一个脉冲为单位 ，设置为1MM的脉冲个数，这度量单位为MM
    ZAux_Direct_SetUnits(g_handle, 1, m_units);
    //设定速度，加减速
    ZAux_Direct_SetLspeed(g_handle, 1, m_lspeed);
    ZAux_Direct_SetSpeed(g_handle, 1, m_speed);
    ZAux_Direct_SetAccel(g_handle, 1, m_acc);
    ZAux_Direct_SetDecel(g_handle, 1, m_dec);
    ZAux_Direct_SetCreep(g_handle, 1, m_creep);
    //设定对应轴的原点输入口信号
    ZAux_Direct_SetDatumIn(g_handle, 1, 2);
    ZAux_Direct_SetInvertIn(g_handle, 2, 1);  // ZMC系列认为OFF时碰到了原点信号（常闭） ，如果是常开传感器则需要反转输入口，ECI系列的不需要反转
    ZAux_Direct_Single_Datum(g_handle, 1, 8); //模式8，9
    // Y limited
    // ZAux_Direct_SetDpos(g_handle, 1, 1000);
    ZAux_Direct_SetFsLimit(g_handle, 1, 1000);
    ZAux_Direct_SetRsLimit(g_handle, 1, 0);

    // Z aixs
    ZAux_Direct_GetIfIdle(g_handle, 2, &status); //判断当前轴状态
    if (status == 0)                             //已经在运动中
        return;
    //设定轴类型 7-   脉冲轴类型 +	编码器Z信号		不用EZ回零也可以设置为1
    ZAux_Direct_SetAtype(g_handle, 2, 7);
    //设定脉冲模式及逻辑方向（脉冲+方向）
    ZAux_Direct_SetInvertStep(g_handle, 2, 0);
    //设置脉冲当量	1表示以一个脉冲为单位 ，设置为1MM的脉冲个数，这度量单位为MM
    ZAux_Direct_SetUnits(g_handle, 2, m_units);
    //设定速度，加减速
    ZAux_Direct_SetLspeed(g_handle, 2, m_lspeed);
    ZAux_Direct_SetSpeed(g_handle, 2, m_speed);
    ZAux_Direct_SetAccel(g_handle, 2, m_acc);
    ZAux_Direct_SetDecel(g_handle, 2, m_dec);
    ZAux_Direct_SetCreep(g_handle, 2, m_creep);
    //设定对应轴的原点输入口信号
    ZAux_Direct_SetDatumIn(g_handle, 2, 0);
    ZAux_Direct_SetInvertIn(g_handle, 0, 1);  // ZMC系列认为OFF时碰到了原点信号（常闭） ，如果是常开传感器则需要反转输入口，ECI系列的不需要反转
    ZAux_Direct_Single_Datum(g_handle, 2, 9); //模式8，9


    // X limited
   // ZAux_Direct_SetDpos(g_handle, 0, 0);
    ZAux_Direct_SetFsLimit(g_handle, 0, 400);
    ZAux_Direct_SetRsLimit(g_handle, 0, 0);

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    int state = 0;
    while(state == 0)
    {
        ZAux_Direct_GetIfIdle(g_handle, 0, &state);
    }

    state = 0;
    while(state == 0)
    {
        ZAux_Direct_GetIfIdle(g_handle, 1, &state);
    }

    state = 0;
    while(state == 0)
    {
        ZAux_Direct_GetIfIdle(g_handle, 2, &state);
    }

    ZAux_Direct_SetDpos(g_handle, 0, 2800);
    ZAux_Direct_SetDpos(g_handle, 1, 1800);
    ZAux_Direct_SetDpos(g_handle, 2, 0);
}

void MainWindow::xMoveTO(float distance)
{
    int run_state = 0; //轴运动状态

    int ret = ZAux_Direct_GetIfIdle(g_handle, 0, &run_state); //读取轴 0 运动状态， 0-运动， 1- 停止

    if (0 == run_state)
    {
        printf("axis 0 is running!\n");
        return;
    }
    ret = ZAux_Direct_SetUnits(g_handle, 0, 20);   //设置轴0脉冲当量20/mm
    ret = ZAux_Direct_SetSpeed(g_handle, 0, 200);  //设置轴 0 速度为 200units/s
    ret = ZAux_Direct_SetAccel(g_handle, 0, 2000); //设置轴 0 加速度为 2000units/s/s
    ret = ZAux_Direct_SetDecel(g_handle, 0, 2000); //设置轴 0 减速度为 2000units/s/s
    ret = ZAux_Direct_SetSramp(g_handle, 0, 100);  //设置轴 0 S曲线时间 100(梯形加减速)
    ret = ZAux_Trigger(g_handle);
    // ret = ZAux_Direct_Single_Move(g_handle, 0, 1000); //轴 0 相对于当前位置运动1000units
    ret = ZAux_Direct_Single_MoveAbs(g_handle, 0, distance);
}

void MainWindow::yMoveTo(float distance)
{
    int run_state = 0; //轴运动状态

    int ret = ZAux_Direct_GetIfIdle(g_handle, 1, &run_state); //读取轴 0 运动状态， 0-运动， 1- 停止

    if (0 == run_state)
    {
        printf("axis 0 is running!\n");
        return;
    }
    ret = ZAux_Direct_SetUnits(g_handle, 1, 20);   //设置轴0脉冲当量20/mm
    ret = ZAux_Direct_SetSpeed(g_handle, 1, 200);  //设置轴 0 速度为 200units/s
    ret = ZAux_Direct_SetAccel(g_handle, 1, 2000); //设置轴 0 加速度为 2000units/s/s
    ret = ZAux_Direct_SetDecel(g_handle, 1, 2000); //设置轴 0 减速度为 2000units/s/s
    ret = ZAux_Direct_SetSramp(g_handle, 1, 100);  //设置轴 0 S曲线时间 100(梯形加减速)
    ret = ZAux_Trigger(g_handle);
    // ret = ZAux_Direct_Single_Move(g_handle, 0, 1000); //轴 0 相对于当前位置运动1000units
    ret = ZAux_Direct_Single_MoveAbs(g_handle, 1, distance);
}

void MainWindow::zMoveTo(float distance)
{
    int run_state = 0; //轴运动状态

    int ret = ZAux_Direct_GetIfIdle(g_handle, 2, &run_state); //读取轴 0 运动状态， 0-运动， 1- 停止

    if (0 == run_state)
    {
        printf("axis 0 is running!\n");
        return;
    }
    ret = ZAux_Direct_SetUnits(g_handle, 2, 20);   //设置轴0脉冲当量20/mm
    ret = ZAux_Direct_SetSpeed(g_handle, 2, 200);  //设置轴 0 速度为 200units/s
    ret = ZAux_Direct_SetAccel(g_handle, 2, 2000); //设置轴 0 加速度为 2000units/s/s
    ret = ZAux_Direct_SetDecel(g_handle, 2, 2000); //设置轴 0 减速度为 2000units/s/s
    ret = ZAux_Direct_SetSramp(g_handle, 2, 100);  //设置轴 0 S曲线时间 100(梯形加减速)
    ret = ZAux_Trigger(g_handle);
    // ret = ZAux_Direct_Single_Move(g_handle, 0, 1000); //轴 0 相对于当前位置运动1000units
    ret = ZAux_Direct_Single_MoveAbs(g_handle, 2, distance);
}

void MainWindow::on_open_clicked()
{
    // get ip from current list
    int32 iresult;
    // char tmp_buff[16] = {0};
    char *tmp_buff = new char[16];
    QString str;
    QString str_title;
    // nIndex = ui->comboBox_IP->currentIndex();
    str = ui->comboBox_IP->currentText();
    qDebug() << "current ip" << str;
    QByteArray ba = str.toLatin1();
    tmp_buff = ba.data();
    qDebug() << "current ip tmp_buff" << tmp_buff;
    // delete[] tmp_buff;
    if (0 != g_handle)
    {
        ZMC_Close(g_handle);
    }
    iresult = ZMC_OpenEth(tmp_buff, &g_handle);
    if (0 == iresult)
    {
        str_title = ("linked:");
        str_title += tmp_buff;
        setWindowTitle(str_title);
    }

    else
    {
        setWindowTitle("no link!");
    }
    id1 = startTimer(100);
}

void MainWindow::on_close_clicked()
{
    if (NULL != g_handle)
    {
        ZMC_Close(g_handle);
        g_handle = NULL;
        setWindowTitle("未链接");
    }
}

void MainWindow::on_Start_clicked()
{
    // TODO: Add your control notification handler code here
    if (NULL == g_handle)
    {
        setWindowTitle("no link");
        return;
    }

    Update_Para(); //刷新参数

    int status = 0;
    ZAux_Direct_GetIfIdle(g_handle, m_nAxis, &status); //判断当前轴状态

    if (status == 0) //已经在运动中
        return;
    //设定轴类型 7-   脉冲轴类型 +	编码器Z信号		不用EZ回零也可以设置为1

    ZAux_Direct_SetAtype(g_handle, m_nAxis, 7);

    //设定脉冲模式及逻辑方向（脉冲+方向）
    ZAux_Direct_SetInvertStep(g_handle, m_nAxis, 0);

    //设置脉冲当量	1表示以一个脉冲为单位 ，设置为1MM的脉冲个数，这度量单位为MM
    ZAux_Direct_SetUnits(g_handle, m_nAxis, m_units);

    //设定速度，加减速
    ZAux_Direct_SetLspeed(g_handle, m_nAxis, m_lspeed);
    ZAux_Direct_SetSpeed(g_handle, m_nAxis, m_speed);
    ZAux_Direct_SetAccel(g_handle, m_nAxis, m_acc);
    ZAux_Direct_SetDecel(g_handle, m_nAxis, m_dec);
    ZAux_Direct_SetCreep(g_handle, m_nAxis, m_creep);

    //设定对应轴的原点输入口信号
    ZAux_Direct_SetDatumIn(g_handle, m_nAxis, m_datumin);
    ZAux_Direct_SetInvertIn(g_handle, m_datumin, 1); // ZMC系列认为OFF时碰到了原点信号（常闭） ，如果是常开传感器则需要反转输入口，ECI系列的不需要反转

    //回零
    if (m_datummode < 4)
    {
        ZAux_Direct_Single_Datum(g_handle, m_nAxis, m_datummode + 1); //模式1-4
    }
    else
    {
        ZAux_Direct_Single_Datum(g_handle, m_nAxis, m_datummode + 4); //模式8，9
    }
    update();
}

void MainWindow::Update_Para()
{
    m_acc = (ui->accel_value->text()).toFloat();
    m_creep = (ui->craw_value->text()).toFloat();
    m_datumin = (ui->iol_value->text()).toFloat();
    m_dec = (ui->decel_value->text()).toFloat();
    m_lspeed = (ui->Lspeed_value->text()).toFloat();
    m_speed = (ui->Speed_value->text()).toFloat();
    m_units = (ui->units_value->text()).toFloat();
    m_datummode = bt->checkedId();
}

void MainWindow::on_Stop_clicked()
{
    if (NULL == g_handle)
    {
        setWindowTitle("no link");
        return;
    }
    ZAux_Direct_Single_Cancel(g_handle, m_nAxis, 2); //
}

void MainWindow::on_Clear_clicked()
{
    if (NULL == g_handle)
    {
        setWindowTitle("no link");
        return;
    }
    // TODO: Add your control notification handler code here
    for (int i = 0; i < 3; i++)
    {
        ZAux_Direct_SetDpos(g_handle, i, 0); //设置零点
    }
}

void MainWindow::on_radio_X_clicked()
{
    update();
    m_nAxis = 0;
    m_datumin = 0;
    ui->iol_value->setText("1");
}

void MainWindow::on_radio_Y_clicked()
{
    update();
    m_nAxis = 1;
    m_datumin = 1;
    ui->iol_value->setText("2");
}

void MainWindow::on_radi_Z_clicked()
{
    update();
    m_nAxis = 2;
    m_datumin = 2;
    ui->iol_value->setText("0");
}

void MainWindow::on_radio_R_clicked()
{
    update();
    m_nAxis = 3;
    m_datumin = 3;
    ui->iol_value->setText("3");
}

// total water
float total1 = 0;
float total2 = 0;

// last time speed
float last1 = 0;
float last2 = 0;
bool firstTime = true;  //expected first time to count...

void MainWindow::timerEvent(QTimerEvent *event)
{
    if (NULL == g_handle)
    {
        setWindowTitle("链接断开");
        return;
    }

    if (event->timerId() == id1)
    {
        QString string;
        float position[4] = {0};
        int status[4] = {0};

        for (int i = 0; i < 4; i++)
        {
            ZAux_Direct_GetDpos(g_handle, i, &position[i]); //获取当前轴位置
            ZAux_Direct_GetIfIdle(g_handle, i, &status[i]); //判断当前轴状态
        }

        if (status[0] == -1)
        {
            string.sprintf("X  Stop  %.2f", position[0]);
            ui->label_x->setText(string);
        }
        else
        {
            string.sprintf("X  Sporting  %.2f", position[0]);
            ui->label_x->setText(string);
        }

        if (status[1] == -1)
        {
            string.sprintf("Y  Stop  %.2f", position[1]);
            ui->label_y->setText(string);
        }
        else
        {
            string.sprintf("Y  Sporting  %.2f", position[1]);
            ui->label_y->setText(string);
        }

        if (status[2] == -1)
        {
            string.sprintf("Z   Stop  %.2f", position[2]);
            ui->label_z->setText(string);
        }
        else
        {
            string.sprintf("Z  Sporting  %.2f", position[2]);
            ui->label_z->setText(string);
        }

        if (status[3] == -1)
        {
            string.sprintf("R  Stop  %.2f", position[3]);
            ui->label_r->setText(string);
        }
        else
        {
            string.sprintf("R  Sporting %.2f", position[3]);
            ui->label_r->setText(string);
        }
    }

    if (bfgt)
    {

        float v1, v2; // speed
        float p1, p2; // pressure

        fgt_get_sensorValue(sid1, &v1);
        fgt_get_sensorValue(sid2, &v2);

        fgt_get_pressure(pid1, &p1);
        fgt_get_pressure(pid2, &p2);

        ui->p1->setText(QString::number(p1));
        ui->p2->setText(QString::number(p2));

        ui->v1->setText(QString::number(v1));
        ui->v2->setText(QString::number(v2));

        if(firstTime)
        {
            last1 = v1;
            last2 = v2;
            firstTime = false;
        }else{
            total1+=0.1*(last1+v1)/120;
            total2+=0.1*(last2+v2)/120;

            last1 = v1;
            last2 = v2;
        }

        ui->fgtsT1->setText(QString::number(total1));
        ui->fgtsT2->setText(QString::number(total2));

    }

    ui->image->setPixmap(QPixmap::fromImage(gShow));
    takeFlag = true;
}

void MainWindow::on_btnOriginX_clicked()
{

    auto ret = ZAux_Direct_SetDpos(g_handle, 0, 0);   //初始设置dpos为0
    ret = ZAux_Direct_SetSpeed(g_handle, 0, 20);      //设置轴 X 速度为 200units/s
    ret = ZAux_Direct_SetAccel(g_handle, 0, 80);      //设置轴 X 加速度为 2000units/s/s
    ret = ZAux_Direct_SetDecel(g_handle, 0, 80);      //设置轴 X 减速度为 2000units/s/s
    ret = ZAux_Direct_SetCreep(g_handle, 0, 10);      //设置回X时反向爬行速
    ret = ZAux_Direct_SetDatumIn(g_handle, 0, 1);     //设置原点点开关
    ret = ZAux_Direct_SetHomeWait(g_handle, 0, 1000); //设置回零等待时间
    ret = ZAux_Direct_SetInvertIn(g_handle, 0, 1);    //设置输入口0电平反转
    ret = ZAux_Trigger(g_handle);
    ret = ZAux_Direct_Single_Datum(g_handle, 0, 9); //回零，模式9
}

void MainWindow::on_btnOriginY_clicked()
{

    auto ret = ZAux_Direct_SetDpos(g_handle, 1, 0);   //初始设置dpos为0
    ret = ZAux_Direct_SetSpeed(g_handle, 1, 20);      //设置轴 Y 速度为 200units/s
    ret = ZAux_Direct_SetAccel(g_handle, 1, 80);      //设置轴 Y 加速度为 2000units/s/s
    ret = ZAux_Direct_SetDecel(g_handle, 1, 80);      //设置轴 Y 减速度为 2000units/s/s
    ret = ZAux_Direct_SetCreep(g_handle, 1, 10);      //设置回0时反向爬行速
    ret = ZAux_Direct_SetDatumIn(g_handle, 1, 2);     //设置原点点开关
    ret = ZAux_Direct_SetHomeWait(g_handle, 1, 1000); //设置回零等待时间
    ret = ZAux_Direct_SetInvertIn(g_handle, 1, 1);    //设置输入口0电平反转
    ret = ZAux_Trigger(g_handle);
    ret = ZAux_Direct_Single_Datum(g_handle, 1, 8); //回零，模式8
}

void MainWindow::on_btnOriginZ_clicked()
{

    auto ret = ZAux_Direct_SetDpos(g_handle, 2, 0);   //初始设置dpos为0
    ret = ZAux_Direct_SetSpeed(g_handle, 2, 20);      //设置轴 z 速度为 200units/s
    ret = ZAux_Direct_SetAccel(g_handle, 2, 80);      //设置轴 z 加速度为 2000units/s/s
    ret = ZAux_Direct_SetDecel(g_handle, 2, 80);      //设置轴 Z减速度为 2000units/s/s
    ret = ZAux_Direct_SetCreep(g_handle, 2, 10);      //设置回0时反向爬行速
    ret = ZAux_Direct_SetDatumIn(g_handle, 2, 0);     //设置原点点开关
    ret = ZAux_Direct_SetHomeWait(g_handle, 2, 1000); //设置回零等待时间
    ret = ZAux_Direct_SetInvertIn(g_handle, 2, 1);    //设置输入口0电平反转
    ret = ZAux_Trigger(g_handle);
    ret = ZAux_Direct_Single_Datum(g_handle, 2, 8); //回零，模式8
}

void MainWindow::on_btnXAdd_clicked()
{
    ZAux_Direct_Single_Move(
        g_handle, 0, 100);
}
void MainWindow::on_btnYAdd_clicked()
{
    ZAux_Direct_Single_Move(
        g_handle, 1, 100);
}
void MainWindow::on_btnZAdd_clicked()
{
    ZAux_Direct_Single_Move(
        g_handle, 2, 100);
}
void MainWindow::on_btnXSub_clicked()
{
    ZAux_Direct_Single_Move(
        g_handle, 0, -100);
}
void MainWindow::on_btnYSub_clicked()
{
    ZAux_Direct_Single_Move(
        g_handle, 1, -100);
}
void MainWindow::on_btnZSub_clicked()
{
    ZAux_Direct_Single_Move(
        g_handle, 2, -100);
}

void MainWindow::on_btnClear_clicked()
{
    ZAux_BusCmd_DriveClear(g_handle, 0, 0);
    ZAux_BusCmd_DriveClear(g_handle, 1, 0);
    ZAux_BusCmd_DriveClear(g_handle, 2, 0);
}

void MainWindow::on_btnIO1_clicked()
{
    if (io1)
    {
        ZAux_Direct_SetOp(g_handle, 6, 1);
        io1 = false;
    }
    else
    {
        ZAux_Direct_SetOp(g_handle, 6, 0);
        io1 = true;
    }
}

void MainWindow::on_btnIO2_clicked()
{

    if (io2)
    {
        ZAux_Direct_SetOp(g_handle, 7, 1);
        io2 = false;
    }
    else
    {
        ZAux_Direct_SetOp(g_handle, 7, 0);
        io2 = true;
    }
}

void MainWindow::on_btnIO3_clicked()
{

    if (io3)
    {
        ZAux_Direct_SetOp(g_handle, 8, 1);
        io3 = false;
    }
    else
    {
        ZAux_Direct_SetOp(g_handle, 8, 0);
        io3 = true;
    }
}

void MainWindow::on_btnReset_clicked()
{
    std::thread backThread([&]()
                           { this->backZero(); });
    backThread.detach();
}

void MainWindow::on_btnRun_clicked()
{
}

cv::Mat gImageShow;

void MainWindow::on_btnCamera_clicked()
{
    std::thread camThread([&]()
                          {

        int nRet = -1;
        void* m_handle = NULL;

        //枚举子网内指定的传输协议对应的所有设备
        unsigned int nTLayerType = MV_GIGE_DEVICE | MV_USB_DEVICE;
        MV_CC_DEVICE_INFO_LIST m_stDevList = { 0 };
        nRet = MV_CC_EnumDevices(nTLayerType, &m_stDevList);
        if (MV_OK != nRet)
        {
            printf("error: EnumDevices fail [%x]\n", nRet);
            return;
        }

        int i = 0;
        if (m_stDevList.nDeviceNum == 0)
        {
            printf("no camera found!\n");
            return;
        }

        //选择查找到的第一台在线设备，创建设备句柄
        int nDeviceIndex = 0;

        MV_CC_DEVICE_INFO m_stDevInfo = { 0 };
        memcpy(&m_stDevInfo, m_stDevList.pDeviceInfo[nDeviceIndex], sizeof(MV_CC_DEVICE_INFO));

        nRet = MV_CC_CreateHandle(&m_handle, &m_stDevInfo);
        if (MV_OK != nRet)
        {
            printf("error: CreateHandle fail [%x]\n", nRet);
            return;
        }

        //连接设备
        //nRet = MV_CC_OpenDevice(m_handle, nAccessMode, nSwitchoverKey);
        /*
    nAccessMode = MV_ACCESS_Exclusive，设备访问模式，默认独占模式 MV_ACCESS_Exclusive：独占权限，其他APP只允许读CCP寄存器。
    nSwitchoverKey = 0 切换权限时的密钥，默认为无，访问模式支持权限切换（2/4/6模式）时有效。
    */
        nRet = MV_CC_OpenDevice(m_handle);
        if (MV_OK != nRet)
        {
            printf("error: OpenDevice fail [%x]\n", nRet);
            return;
        }

        //...其他处理

        // 设置连续模式
        unsigned int enMode = MV_TRIGGER_MODE_OFF; //关闭触发源
        nRet = MV_CC_SetEnumValue(m_handle, "TriggerMode", enMode);
        std::cout<<"Trigger mode off\n";
        if (MV_OK != nRet)
        {
            printf("error: Trigger mode off fail [%x]\n", nRet);
            return;
        }

        /* 设置自动曝光
       0 : Off
       1 : Once
       2 ：Continuous  */
        nRet = MV_CC_SetEnumValue(m_handle,"ExposureAuto", 2);
        std::cout << "Exposure auto\n";
        if (MV_OK != nRet)
        {
            printf("error: Exposure auto fail [%x]\n", nRet);
            return;
        }

        /* 设置自动增益
       0 : Off
       1 : Once
       2 ：Continuous  */
        nRet = MV_CC_SetEnumValue(m_handle, "GainAuto", 2);
        std::cout << "Gain auto\n";
        if (MV_OK != nRet)
        {
            printf("error: Gain auto fail [%x]\n", nRet);
            return;
        }

        /* 设置自动白平衡
       0 : Off
       2 : Once
       1 ：Continuous  */
        nRet = MV_CC_SetEnumValue(m_handle, "BalanceWhiteAuto", 1);
        std::cout << "Balance white auto\n";
        if (MV_OK != nRet)
        {
            printf("error: Balance white auto fail [%x]\n", nRet);
            return;
        }

        //开始采集图像
        nRet = MV_CC_StartGrabbing(m_handle);
        std::cout << "Start grabbing...\n";
        if (MV_OK != nRet)
        {
            printf("error: StartGrabbing fail [%x]\n", nRet);
            return;
        }

        //获取一帧数据的大小
        MVCC_INTVALUE stIntvalue = { 0 };
        nRet = MV_CC_GetIntValue(m_handle, "PayloadSize", &stIntvalue);
        if (nRet != MV_OK)
        {
            printf("Get PayloadSize failed! nRet [%x]\n", nRet);
            return;
        }
        int nBufSize = stIntvalue.nCurValue; //一帧数据大小

        unsigned int    nTestFrameSize = 0;
        unsigned char* pFrameBuf = NULL;
        pFrameBuf = (unsigned char*)malloc(nBufSize);

        MV_FRAME_OUT_INFO_EX stInfo;
        memset(&stInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));

        //上层应用程序需要根据帧率，控制好调用该接口的频率
        //此次代码仅供参考，实际应用建议另建线程进行图像帧采集和处理
        while (1)
        {
            if (nTestFrameSize > 9)
            {
                break;
            }
            nRet = MV_CC_GetOneFrameTimeout(m_handle, pFrameBuf, nBufSize, &stInfo, 1000);
            if (MV_OK != nRet)
            {
                //
            }
            else
            {
                //图片数据输入输出参数
                MV_SAVE_IMAGE_PARAM_EX stParam;

                //源数据
                stParam.pData = pFrameBuf;                //原始图像数据
                stParam.nDataLen = stInfo.nFrameLen;      //原始图像数据长度
                stParam.enPixelType = stInfo.enPixelType; //原始图像数据的像素格式
                stParam.nWidth = stInfo.nWidth;           //图像宽
                stParam.nHeight = stInfo.nHeight;         //图像高
                stParam.nJpgQuality = 90;			      //JPEG图片编码质量

                // ch:jpg图像质量范围为(50-99], png图像质量范围为[0-9]

                //目标数据
                stParam.enImageType = MV_Image_Jpeg;            //需要保存的图像类型，转换成JPEG格式
                //stParam.enImageType = MV_Image_Png;            //需要保存的图像类型，转换成PNG格式
                stParam.nBufferSize = nBufSize;                 //存储节点的大小
                unsigned char* pImage = (unsigned char*)malloc(nBufSize);
                stParam.pImageBuffer = pImage;                   //输出数据缓冲区，存放转换之后的图片数据

                nRet = MV_CC_SaveImageEx2(m_handle, &stParam);
                if (MV_OK != nRet)
                {
                    break;
                }


                //ui.imagesetPixmap(QPixmap::fromImage(QImage::fromData((const char*)pImage)));

                //将转换之后图片数据保存成文件
                FILE* fp = fopen("test3.png", "wb");  // "wb" 只写打开或新建一个二进制文件；只允许写数据。
                fwrite(pImage, 1, stParam.nImageLen, fp);
                fclose(fp);
                ui->image->setPixmap(QPixmap("/home/zmotion/build-single_home-Desktop-Release/test3.png"));
                free(pImage);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                //...其他图像数据处理
                nTestFrameSize++;
            }
        }
        free(pFrameBuf);

        //...其他处理

        //停止采集图像
        nRet = MV_CC_StopGrabbing(m_handle);
        if (MV_OK != nRet)
        {
            printf("error: StopGrabbing fail [%x]\n", nRet);
            return;
        }

        //关闭设备，释放资源
        nRet = MV_CC_CloseDevice(m_handle);
        if (MV_OK != nRet)
        {
            printf("error: CloseDevice fail [%x]\n", nRet);
            return;
        }

        //销毁句柄，释放资源
        nRet = MV_CC_DestroyHandle(m_handle);
        if (MV_OK != nRet)
        {
            printf("error: DestroyHandle fail [%x]\n", nRet);
            return;
        } });

    camThread.detach();
}

bool sendByte(QSerialPort *m_device, const char c)
{
    printf("enter into sendByte...\n");
    if (!m_device->isOpen())
    {
        return false;
    }
    printf("%x ", c);
    if ((m_device->write(&c, 1)) < 1)
    {
        qDebug() << m_device->errorString();

        return false;
    }
    m_device->flush();
    if (true)
    {
        // millisleep(delay);
        //  m_device->flush();
    }
    return true;
}

bool sendString(QSerialPort *m_device, const QString &s)
{
    printf("enter into sendString...\n");
    if (true) // hex
    {
        QString hex = s;
        hex.remove(QRegExp("\\s"));
        if ((hex.startsWith("0x")) || (hex.startsWith("0X")))
        {
            hex = hex.mid(2);
        }

        if (hex.length() % 2 != 0)
        {
            hex = "0" + hex;
        }

        for (int i = 0; i < hex.length() / 2; i++)
        {
            QString nextByte = hex.mid(i * 2, 2);
            bool ok = true;
            nextByte.toUInt(&ok, 16);
            if (!ok)
            {
                return false;
            }
        }

        for (int i = 0; i < hex.length() / 2; i++)
        {
            QString nextByte = hex.mid(i * 2, 2);
            unsigned int byte = nextByte.toUInt(0, 16);
            sendByte(m_device, byte & 0xff);
            // fprintf(stderr, " 0x%x d:%d ", byte & 0xff, charDelay);
        }
        return true;
    }
}
void MainWindow::switchChannal(int number)
{
    QString cmd1 = QString("010600012019");
    QString cmd2 = QString("010600026018");

    m_port1->clear();
    switch (number)
    {
    case 1:
        sendString(m_port1, cmd1);
        break;
    default:
        sendString(m_port1, cmd2);
        break;
    }
}

void MainWindow::on_btnChannal_clicked()
{
    injector("/1ZR\r\n", 6);
}

void MainWindow::injector(char *command, int length)
{
    m_port2->clear();
    m_port2->write(command, length);
}

void MainWindow::on_btnXGo_clicked()
{
    float dis = ui->xgo->text().toFloat();
    ZAux_Direct_Single_Move(
        g_handle, 0, dis);
}

void MainWindow::on_btnYGo_clicked()
{
    float dis = ui->ygo->text().toFloat();
    ZAux_Direct_Single_Move(
        g_handle, 1, dis);
}

void MainWindow::on_btnZGo_clicked()
{
    float dis = ui->zgo->text().toFloat();
    ZAux_Direct_Single_Move(
        g_handle, 2, dis);
}

void MainWindow::on_btnTestGo_clicked()
{

    std::thread gotest([&](){
    int i, j;
    i = 0;
    j = 0;

    for (i = 0; i < 8; i++)
    {
        for (j = 0; j < 12; j++)
        {
            if ((i % 2) == 0)
                ZAux_Direct_Single_Move(g_handle, 0, -90);
            else
            {
                ZAux_Direct_Single_Move(g_handle, 0, 90);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        ZAux_Direct_Single_Move(g_handle, 1, -90);
        sleep(1);
    }

    });
    gotest.detach();

}

void MainWindow::on_btnOrigin_clicked()
{
    ZAux_Direct_Single_MoveAbs(
        g_handle, 0, 2650);

    ZAux_Direct_Single_MoveAbs(
        g_handle, 1, 1715);
}

void MainWindow::on_btnFgtInit_clicked()
{
    // Variables declaration
    fgt_CHANNEL_INFO sensor_info[256];
    fgt_SENSOR_TYPE sensor_type[256];
    unsigned char sensor_number = 0;
    std::string unit = "";
    float range_min = 0;
    float range_max = 0;
    float sensor_value = 0;
    fgt_ERROR_CODE errCode = fgt_ERROR_CODE::OK;
    fgt_CHANNEL_INFO pinfo[256];
    // Initialize session with all detected Fluigent instrument(s)
    // This step is optional, if not called session will be automatically created
    errCode = Fgt_init();

    if (errCode == fgt_ERROR_CODE::OK)
    {
        // Get total number of initialized sensor channel(s)
        errCode = Fgt_get_sensorChannelCount(&sensor_number);
        std::cout << "Status: " << errCode << " ### Total number of sensor channels detected: " << int(sensor_number) << std::endl;

        // Get information about the connected sensor channel(s)
        errCode = Fgt_get_sensorChannelsInfo(sensor_info, sensor_type);
        std::cout << "Status: " << errCode << " ### Retrieved information about sensor channel(s)" << std::endl;
    }
    else
    {
        std::cout << "Please make sure that your hardware setup matches this example's requirements and that all instruments are connected to the computer" << std::endl;
    }

    sid1 = sensor_info[0].indexID;
    sid2 = sensor_info[1].indexID;

    Fgt_get_pressureChannelsInfo(pinfo);
    pid1 = pinfo[0].indexID;
    pid2 = pinfo[1].indexID;

    fgt_set_pressureResponse(pid1, 1); //设置响应时间
    fgt_set_pressureResponse(pid2, 1);

    fgt_set_sensorUnit(sid1, "µl/min");
    fgt_set_sensorUnit(sid2, "µl/min");
    fgt_set_sensorCalibration(sid1, (fgt_calibration_t)2);
    fgt_set_sensorCalibration(sid2, (fgt_calibration_t)2);

    // bind the sensors and pressures
    fgt_set_sensorRegulation(sid1, pid1, 40);
    fgt_set_sensorRegulation(sid2, pid2, 40);

    bfgt = true; // has been started...
}

void MainWindow::on_btnFgtSet_clicked()
{
    float v1 = ui->fgts1->text().toFloat();
    float v2 = ui->fgts2->text().toFloat();


    fgt_set_sensorRegulation(sid1, pid1, v1);
    fgt_set_sensorRegulation(sid2, pid2, v2);
}

void MainWindow::initFgtUi()
{
    ui->fgts1->setText("60");
    ui->fgts2->setText("15");

    ui->fgtType1->addItem("None", 0);
    ui->fgtType1->addItem("H20", 1);
    ui->fgtType1->addItem("IPA", 2);
    ui->fgtType1->addItem("HFE", 3);
    ui->fgtType1->addItem("FC40", 4);
    ui->fgtType1->addItem("OIL", 5);

    ui->fgtType2->addItem("None", 0);
    ui->fgtType2->addItem("H20", 1);
    ui->fgtType2->addItem("IPA", 2);
    ui->fgtType2->addItem("HFE", 3);
    ui->fgtType2->addItem("FC40", 4);
    ui->fgtType2->addItem("OIL", 5);
}

void MainWindow::initPorts()
{
    // open m_port1  /dev/ttyS0
    m_port1 = new QSerialPort();
    if (m_port1->isOpen())
    {
        m_port1->close();
    }

    m_port1->setPortName("/dev/ttyS0");
    m_port1->setBaudRate(QSerialPort::Baud115200);
    m_port1->setDataBits(QSerialPort::Data8);
    m_port1->setFlowControl(QSerialPort::NoFlowControl);
    m_port1->setParity(QSerialPort::NoParity);  //无校验位
    m_port1->setStopBits(QSerialPort::OneStop); //一位停止位

    if (!m_port1->open(QIODevice::ReadWrite))
    {
        qDebug() << "open failed\n";
        return;
    }
    else
    {
        qDebug() << "open serial port ttys0";
    }

    // open m_port2 /dev/ttyS1
    m_port2 = new QSerialPort();
    if (m_port2->isOpen())
    {
        m_port2->close();
    }

    m_port2->setPortName("/dev/ttyS1");

    m_port2->setBaudRate(QSerialPort::Baud9600);
    m_port2->setDataBits(QSerialPort::Data8);
    m_port2->setFlowControl(QSerialPort::NoFlowControl);
    m_port2->setParity(QSerialPort::NoParity);  //无校验位
    m_port2->setStopBits(QSerialPort::OneStop); //一位停止位

    if (!m_port2->open(QIODevice::ReadWrite))
    {
        qDebug() << "open  ttys 1failed\n";
        QMessageBox::information(this, "error", "port2 open failed!");
        return;
    }
    else
    {
        qDebug() << "open serial port ttys1";
    }
}

void MainWindow::on_btnSw1_clicked()
{
    switchChannal(1);
}

void MainWindow::on_btnSw2_clicked()
{
    switchChannal(2);
}

void MainWindow::on_btnA6000_clicked()
{
    injector("/1A6000R\r\n", 10);
    auto msg = m_port2->readAll();

    qDebug()<<QString(msg);
}

void MainWindow::on_btnA0_clicked()
{
    injector("/1A0R\r\n", 7);
    auto msg = m_port2->readAll();

    qDebug()<<QString(msg);
}

void MainWindow::on_btn1IR_clicked()
{
    injector("/1IR\r\n", 6);
    auto msg = m_port2->readAll();

    qDebug()<<QString(msg);
}

void MainWindow::on_btn1OR_clicked()
{
    injector("/1OR\r\n", 6);
    auto msg = m_port2->readAll();

    qDebug()<<QString(msg);
}

void MainWindow::on_btnExit_clicked()
{
    close();
}

void MainWindow::on_btnSpeedSet_clicked()
{
    QSettings *configIniWrite = new QSettings("config.ini", QSettings::IniFormat);

    //x aixs
    m_lspeed = ui->xInitSpeed->text().toFloat();
    m_speed = ui->xSpeed->text().toFloat();
    m_acc = ui->xAcc->text().toFloat();
    m_dec = ui->xDec->text().toFloat();

    ZAux_Direct_SetLspeed(g_handle, 0, m_lspeed);
    ZAux_Direct_SetSpeed(g_handle, 0, m_speed);
    ZAux_Direct_SetAccel(g_handle, 0, m_acc);
    ZAux_Direct_SetDecel(g_handle, 0, m_dec);
    configIniWrite->setValue("machineX/m_lspeed", QString::number(m_lspeed,'f',2));
    configIniWrite->setValue("machineX/m_speed", QString::number(m_speed,'f',2));
    configIniWrite->setValue("machineX/m_acc", QString::number(m_acc,'f',2));
    configIniWrite->setValue("machineX/m_dec", QString::number(m_dec,'f',2));

    //y aixs
    m_lspeed = ui->yInitSpeed->text().toFloat();
    m_speed = ui->ySpeed->text().toFloat();
    m_acc = ui->yAcc->text().toFloat();
    m_dec = ui->yDec->text().toFloat();

    ZAux_Direct_SetLspeed(g_handle, 1, m_lspeed);
    ZAux_Direct_SetSpeed(g_handle, 1, m_speed);
    ZAux_Direct_SetAccel(g_handle, 1, m_acc);
    ZAux_Direct_SetDecel(g_handle, 1, m_dec);
    configIniWrite->setValue("machineY/m_lspeed", QString::number(m_lspeed,'f',2));
    configIniWrite->setValue("machineY/m_speed", QString::number(m_speed,'f',2));
    configIniWrite->setValue("machineY/m_acc", QString::number(m_acc,'f',2));
    configIniWrite->setValue("machineY/m_dec", QString::number(m_dec,'f',2));

    //z aixs
    m_lspeed = ui->zInitSpeed->text().toFloat();
    m_speed = ui->zSpeed->text().toFloat();
    m_acc = ui->zAcc->text().toFloat();
    m_dec = ui->zDec->text().toFloat();

    ZAux_Direct_SetLspeed(g_handle, 2, m_lspeed);
    ZAux_Direct_SetSpeed(g_handle, 2, m_speed);
    ZAux_Direct_SetAccel(g_handle, 2, m_acc);
    ZAux_Direct_SetDecel(g_handle, 2, m_dec);
    configIniWrite->setValue("machineZ/m_lspeed", QString::number(m_lspeed,'f',2));
    configIniWrite->setValue("machineZ/m_speed", QString::number(m_speed,'f',2));
    configIniWrite->setValue("machineZ/m_acc", QString::number(m_acc,'f',2));
    configIniWrite->setValue("machineZ/m_dec", QString::number(m_dec,'f',2));

    QMessageBox::information(this, "OK", "Setting OK!");
}

void MainWindow::loadSpeed()
{
    QSettings *configIniWrite = new QSettings("config.ini", QSettings::IniFormat);

    //x aixs
    ui->xInitSpeed->setText(configIniWrite->value("machineX/m_lspeed").toString());
    ui->xSpeed->setText(configIniWrite->value("machineX/m_speed").toString());
    ui->xAcc->setText(configIniWrite->value("machineX/m_acc").toString());
    ui->xDec->setText(configIniWrite->value("machineX/m_dec").toString());

    m_lspeed = configIniWrite->value("machineX/m_lspeed").toFloat();
    m_speed = configIniWrite->value("machineX/m_speed").toFloat();
    m_acc = configIniWrite->value("machineX/m_acc").toFloat();
    m_dec = configIniWrite->value("machineX/m_dec").toFloat();
    ZAux_Direct_SetLspeed(g_handle, 0, m_lspeed);
    ZAux_Direct_SetSpeed(g_handle, 0, m_speed);
    ZAux_Direct_SetAccel(g_handle, 0, m_acc);
    ZAux_Direct_SetDecel(g_handle, 0, m_dec);

    // y axis
    ui->yInitSpeed->setText(configIniWrite->value("machineY/m_lspeed").toString());
    ui->ySpeed->setText(configIniWrite->value("machineY/m_speed").toString());
    ui->yAcc->setText(configIniWrite->value("machineY/m_acc").toString());
    ui->yDec->setText(configIniWrite->value("machineY/m_dec").toString());

    m_lspeed = configIniWrite->value("machineY/m_lspeed").toFloat();
    m_speed = configIniWrite->value("machineY/m_speed").toFloat();
    m_acc = configIniWrite->value("machineY/m_acc").toFloat();
    m_dec = configIniWrite->value("machineY/m_dec").toFloat();
    ZAux_Direct_SetLspeed(g_handle, 1, m_lspeed);
    ZAux_Direct_SetSpeed(g_handle, 1, m_speed);
    ZAux_Direct_SetAccel(g_handle, 1, m_acc);
    ZAux_Direct_SetDecel(g_handle, 1, m_dec);

    // z axis
    ui->zInitSpeed->setText(configIniWrite->value("machineZ/m_lspeed").toString());
    ui->zSpeed->setText(configIniWrite->value("machineZ/m_speed").toString());
    ui->zAcc->setText(configIniWrite->value("machineZ/m_acc").toString());
    ui->zDec->setText(configIniWrite->value("machineZ/m_dec").toString());

    m_lspeed = configIniWrite->value("machineZ/m_lspeed").toFloat();
    m_speed = configIniWrite->value("machineZ/m_speed").toFloat();
    m_acc = configIniWrite->value("machineZ/m_acc").toFloat();
    m_dec = configIniWrite->value("machineZ/m_dec").toFloat();
    ZAux_Direct_SetLspeed(g_handle, 2, m_lspeed);
    ZAux_Direct_SetSpeed(g_handle, 2, m_speed);
    ZAux_Direct_SetAccel(g_handle, 2, m_acc);
    ZAux_Direct_SetDecel(g_handle, 2, m_dec);
}


void MainWindow::on_btnStop_clicked()
{

}
