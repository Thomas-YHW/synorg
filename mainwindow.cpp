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
#include<unistd.h>

using namespace cv;

int sid1, sid2, pid1, pid2;
QImage gShow;
volatile bool bsuspend = false;
extern void cameraMain();
ZMC_HANDLE cg_handle = NULL;
int gprintTime = 150;   //camera thread print drops time....
volatile bool gbBreak = false;
volatile bool gActived = false;

void MainWindow::linkZmotion()
{
    m_units = 1;
    m_lspeed = 0;
    m_speed = 100;
    m_creep = 10;
    m_acc = 3000;
    m_dec = 3000;
    m_datumin = 0;
    m_nAxis = 0;

    // get ip from current list
    int32 iresult;
    // char tmp_buff[16] = {0};
    const char *tmp_buff = "192.168.0.11";
    QString str;
    QString str_title;
    if (NULL != g_handle)
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
            //loadSpeed();
        }

        id1 = startTimer(100);
        cg_handle = g_handle;
    }

    else
    {
        setWindowTitle("no link!");
    }
}


MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    linkZmotion();    //connection to zmotion.
    initFgtUi();
    std::thread cam(cameraMain);
    cam.detach();

    initPorts();
    //ui->cbBoardNumber->addItem("请选择", 1);
    ui->cbBoardNumber->addItem("One", 1);
    ui->cbBoardNumber->addItem("Two", 2);
    ui->cbBoardNumber->addItem("Three", 3);
    ui->cbBoardNumber->addItem("Four", 4);

    ui->cbBoardNumber->currentIndexChanged(0);

    //板孔种类选择
    ui->cbBoardtype->addItem("96孔板", 1);
    ui->cbBoardtype->addItem("24孔板", 2);

    ui->cbBoardtype->currentIndexChanged(0);

    //gShow = QImage("./logo.bmp");
    //ui->image->setPixmap(QPixmap::fromImage(QImage("./test3.png")));
    id1 = startTimer(100);
}

MainWindow::~MainWindow()
{

    try{
        m_port1->close();
        m_port2->close();
        ZMC_Close(g_handle);
    }catch(...){
        qDebug()<<"close error!";
    }
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
    m_lspeed = 0;
    m_speed = 200;
    m_acc = 3000;
    m_dec = 3000;
    m_creep = 120;
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
    ZAux_Direct_SetFsLimit(g_handle, 0, 2800);
    ZAux_Direct_SetRsLimit(g_handle, 0, -10);

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
    ZAux_Direct_SetFsLimit(g_handle, 1, 1800);
    ZAux_Direct_SetRsLimit(g_handle, 1, -10);

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
    ZAux_Direct_SetFsLimit(g_handle, 2, 400);
    ZAux_Direct_SetRsLimit(g_handle, 2, 0);

    //std::this_thread::sleep_for(std::chrono::milliseconds(200));

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


    ZAux_Direct_SetLspeed(g_handle, 0, 0);
    ZAux_Direct_SetSpeed(g_handle, 0, 800);
    ZAux_Direct_SetAccel(g_handle, 0, 3000);
    ZAux_Direct_SetDecel(g_handle, 0, 3000);

    ZAux_Direct_SetLspeed(g_handle, 1, 0);
    ZAux_Direct_SetSpeed(g_handle, 1, 800);
    ZAux_Direct_SetAccel(g_handle, 1, 3000);
    ZAux_Direct_SetDecel(g_handle, 1, 3000);

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


void MainWindow::on_Stop_clicked()
{
    if (NULL == g_handle)
    {
        setWindowTitle("no link");
        return;
    }
    ZAux_Direct_Single_Cancel(g_handle, m_nAxis, 2); //
}


// total water
float total1 = 0;
float total2 = 0;

// last time speed
float last1 = 0;
float last2 = 0;
bool firstTime = true;  //expected first time to count...
QImage gmodelFullShow;
extern QImage diffImage;
extern QImage gdiff;
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
   // ui->fullImage->setPixmap(QPixmap::fromImage(gmodelFullShow));
   // ui->modelImage->setPixmap(QPixmap::fromImage(gdiff));
    if(gActived){
        gActived = false;
        ui->btnRun->setDisabled(false);
    }
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


    ZAux_Direct_SetOp(g_handle, 8, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(m_printTime));
    ZAux_Direct_SetOp(g_handle, 8, 0);

}

void MainWindow::on_btnReset_clicked()
{
    std::thread backThread([&]()
    { this->backZero(); });
    backThread.detach();
}

volatile bool bsaveModel = false;
volatile bool bcompareImage = false;
int icompare = 0;
volatile int gMinThresold = 35;
volatile int gMinArea = 200;
extern volatile bool gbDrops =false;
extern volatile bool gbArrival = false;
extern volatile bool bsetRoi;
volatile bool bgetModel = false;

int injectCounter = 0;
int fgtInitTimes = 0;
fgt_CHANNEL_INFO sensor_info[256];
fgt_SENSOR_TYPE sensor_type[256];
unsigned char sensor_number = 0;
extern int printNumber;
extern int theNumber;
int Boradtype = 0;
float distance = 90.0;
//int xNumber = 8;
//int yNumber = 11;
int xNumber = 4;
int yNumber = 5;

float xps[4];
float yps[4];

void MainWindow::on_btnRun_clicked()
{

    ui->btnRun->setDisabled(true);


    theNumber = ui->lineNumber->text().toInt();
    Boradtype = ui->cbBoardtype->currentIndex();

    printf("boardType :%d\n", Boradtype);
    if(Boradtype == 1 ){
        distance = 193.04;
        int xNumber = 4;
        int yNumber = 5;

        float firstx = 2655;
        float firsty = 1655;

        xps[0] = firstx;
        xps[1] = firstx;
        xps[2] = firstx-1380;
        xps[3] = firstx-1380;

        yps[0] = firsty;
        yps[1] = firsty-955;
        yps[2] = firsty;
        yps[3] = firsty-955;
         printf("b4^6\n");
    }else{
        float firstx = 2680;
        float firsty = 1675;
        xps[0] = firstx;
        xps[1] = firstx;
        xps[2] = firstx-1380;
        xps[3] = firstx-1380;

        yps[0] = firsty;
        yps[1] = firsty-955;
        yps[2] = firsty;
        yps[3] = firsty-955;

        int xNumber = 8;
        int yNumber = 11;
    }

    //FGT Init
    // Variables declaration

    std::string unit = "";
    float range_min = 0;
    float range_max = 0;
    float sensor_value = 0;
    fgt_ERROR_CODE errCode = fgt_ERROR_CODE::OK;
    fgt_CHANNEL_INFO pinfo[256];
    // Initialize session with all detected Fluigent instrument(s)
    // This step is optional, if not called session will be automatically created
    if(fgtInitTimes == 0){
        errCode = Fgt_init();
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));

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
        fgtInitTimes++;
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
    fgt_set_sensorCalibration(sid1, fgt_SENSOR_CALIBRATION::HFE);
    fgt_set_sensorCalibration(sid2, fgt_SENSOR_CALIBRATION::HFE);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // bind the sensors and pressures
    fgt_set_sensorRegulation(sid1, pid1, 0);
    fgt_set_sensorRegulation(sid2, pid2, 110);

    fgt_set_sensorRegulation(sid1, pid1, 0);
    fgt_set_sensorRegulation(sid2, pid2, 110);
    bfgt = true; // has been started...


    //制备微球
    volatile bool bfgt_finished = false;//fgt速度变化动作
    bool bcarve_finished = false;//到达切割点动作/1T
    bool btestgo_finished = false;//三轴走完板子的动作
    volatile bool pleaseGo = false;
    //    fgt_set_sensorCalibration(sid1, fgt_SENSOR_CALIBRATION::HFE);//设置fgt的液体类型为HFE
    //    fgt_set_sensorCalibration(sid2, fgt_SENSOR_CALIBRATION::HFE);

    //    //设置fgt的推样泵速度为0，推油速度为110
    //    fgt_set_sensorRegulation(sid1, pid1, 0);//0
    //    fgt_set_sensorRegulation(sid2, pid2, 110);//110


    //切换阀切换到抽液模式，注射泵抽100微升的量，使100微升的样本被抽到暂存管里
    switchChannal(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));//等待切换阀完成时间
    if(injectCounter !=0){
        //injector("/1ZI1V50A1100R\r\n", 16);
        //std::this_thread::sleep_for(std::chrono::milliseconds(8000));//等待切换阀完成时间
//        injector("/1OV1000A0R\r\n", 13);
//        std::this_thread::sleep_for(std::chrono::milliseconds(8000));//等待切换阀完成时间
//        injector("/1IV50A1100R\r\n", 14);
//        std::this_thread::sleep_for(std::chrono::milliseconds(10000));//等待切换阀完成时间
        printf("before zhushebeng1\n");
        injector("/1OV1000A0IV50A1100R\r\n", 22);
         printf("after zhushebeng1\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(40000));//等待切换阀完成时间
    }
    else {
        injector("/1IV50A1100R\r\n", 14);
    }

    injectCounter++;



    std::thread GodBless([&](){

        std::this_thread::sleep_for(std::chrono::milliseconds(30000));//抽样到暂存管所需时间
        switchChannal(2);
        printf("after test chanal 2\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));//等待切换阀完成时间

        printf("test change 1\n");
        fgt_set_sensorRegulation(sid1, pid1, 140);
        fgt_set_sensorRegulation(sid2, pid2, 140);

        std::this_thread::sleep_for(std::chrono::milliseconds(17000));//推样到剪切点所需时间，时间结束切换阀切换到推样模式
        printf("test change 2\n");
        fgt_set_sensorRegulation(sid1, pid1, 20);
        fgt_set_sensorRegulation(sid2, pid2, 170);

        volatile int direct = 0;
        volatile int iDex = 0;
        volatile int jDex = 0;
        int status;



//        float firstx = 2680;
//        float firsty = 1675;
//        xps[0] = firstx;
//        xps[1] = firstx;
//        xps[2] = firstx-1380;
//        xps[3] = firstx-1380;

//        yps[0] = firsty;
//        yps[1] = firsty-955;
//        yps[2] = firsty;
//        yps[3] = firsty-955;

//        if(Boradtype == 1)
//        {
//            float firstx = 2655;
//            float firsty = 1655;

//            xps[0] = firstx;
//            xps[1] = firstx;
//            xps[2] = firstx-1380;
//            xps[3] = firstx-1380;

//            yps[0] = firsty;
//            yps[1] = firsty-955;
//            yps[2] = firsty;
//            yps[3] = firsty-955;

//        }

        printf("prepare to go...\n");

        printf("take photo\n");
        //设置识别区域
        bsetRoi = true;
        //开始识别
        bcompareImage = true;
        gMinArea = 200;

        int boxNumber = 1;
        int i = 0;

        //在某地等到花儿谢为止
        double wait_x, wait_y; // 此处花姑娘大大地有，所以在这等吧。。。
        wait_x = 2770;
        wait_y = 1800;
        do{
            while(true)
            {
                ZAux_Direct_GetIfIdle(g_handle, 0, &status);
                if(status != 0)
                    break;
            }

            while(true)
            {
                ZAux_Direct_GetIfIdle(g_handle, 1, &status);
                if(status != 0)
                    break;
            }
            //运动到流水位
            ZAux_Direct_Single_MoveAbs(
                        g_handle, 0, wait_x);
            ZAux_Direct_Single_MoveAbs(
                        g_handle, 1,  wait_y);
            while(true)
            {
                ZAux_Direct_GetIfIdle(g_handle, 0, &status);
                if(status != 0)
                    break;
            }

            while(true)
            {
                ZAux_Direct_GetIfIdle(g_handle, 1, &status);
                if(status != 0)
                    break;
            }
            gbArrival = true;//运动到了坐标点
            while(!gbDrops)
            {
                // wait for drops done!
                if(gbBreak)
                    goto BREAKEND;
            }
            gbDrops = false;

        }while (0);



        printf("xnumber and yNumber: %d, %d", xNumber, yNumber);
        for(i=0; i<=boxNumber-1; i++)
        {
            direct = 0;
            iDex = 0;
            jDex = 0;
            while(true)
            {
                ZAux_Direct_GetIfIdle(g_handle, 0, &status);
                if(status != 0)
                    break;
            }

            while(true)
            {
                ZAux_Direct_GetIfIdle(g_handle, 1, &status);
                if(status != 0)
                    break;
            }
            ////运动到第一孔位
            ZAux_Direct_Single_MoveAbs(
                        g_handle, 0, xps[i]);
            ZAux_Direct_Single_MoveAbs(
                        g_handle, 1, yps[i]);
            while(true)
            {
                ZAux_Direct_GetIfIdle(g_handle, 0, &status);
                if(status != 0)
                    break;
            }

            while(true)
            {
                ZAux_Direct_GetIfIdle(g_handle, 1, &status);
                if(status != 0)
                    break;
            }
            gbArrival = true;
            while(!gbDrops)
            {
                // wait for drops done!
                if(gbBreak)
                    goto BREAKEND;
            }
            gbDrops = false;
            //while (iDex < 8)
            while (iDex < xNumber)
            {
                direct = iDex%2;
                jDex = 0;
                printf("befor x move!\n");
                 //while (jDex < 11)
                while (jDex < yNumber)
                {
                    printf("enter x move!\n");

                    while(m_bsuspend)
                    {
                        //是否暂停
                        if(gbBreak)
                            goto BREAKEND;
                    }
                    if (direct == 0)
                        ZAux_Direct_Single_Move(g_handle, 0, 0.0-distance);
                    else
                    {
                        ZAux_Direct_Single_Move(g_handle, 0, distance);
                    }

                    while(true)
                    {
                        ZAux_Direct_GetIfIdle(g_handle, 0, &status);
                        if(status != 0)
                            break;
                    }

                    while(true)
                    {
                        ZAux_Direct_GetIfIdle(g_handle, 1, &status);
                        if(status != 0)
                            break;
                    }
                    jDex++;
                    gbArrival = true;
                    while(!gbDrops)
                    {
                        // wait for drops done!
                        if(gbBreak)
                            goto BREAKEND;
                    }
                    gbDrops = false;
                    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                jDex = 0;
                printf("befor y move!\n");
                if(iDex == (xNumber-1))
                    break;
                ZAux_Direct_Single_Move(g_handle, 1, 0.0-distance);
                while(true)
                {
                    ZAux_Direct_GetIfIdle(g_handle, 0, &status);
                    if(status != 0)
                        break;
                }

                while(true)
                {
                    ZAux_Direct_GetIfIdle(g_handle, 1, &status);
                    if(status != 0)
                        break;
                }
                printf("after y move!\n");
                iDex++;

                gbArrival = true;
                while(!gbDrops)
                {
                    // wait for drops done!
                    if(gbBreak)
                        goto BREAKEND;
                }

                gbDrops = false;
                //std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            }

        }

BREAKEND:
        gbBreak = false;
        //走完板子后返回到流水位
        while(true)
        {
            ZAux_Direct_GetIfIdle(g_handle, 0, &status);
            if(status != 0)
                break;
        }

        while(true)
        {
            ZAux_Direct_GetIfIdle(g_handle, 1, &status);
            if(status != 0)
                break;
        }
//        ZAux_Direct_Single_MoveAbs(
//                    g_handle, 0, wait_x);
//        ZAux_Direct_Single_MoveAbs(
//                    g_handle, 1,  wait_y);
        backZero();
        printf("stop!\n");
        bcompareImage = false;//停止识别
        btestgo_finished = true;

        printf("backThrea\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));


        //injector("/1I2V1500A0R\r\n", 14);
        //injector("/1ZR\r\n", 6);
        fgt_set_sensorRegulation(sid1, pid1, 2);
        fgt_set_sensorRegulation(sid2, pid2, 2);
        gActived = true;

    });
     GodBless.detach();
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

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    auto msg = m_port1->readAll();
    qDebug()<<QString(msg);
    QString printmsg;
    printmsg.prepend(msg);
    qDebug()<<QString(printmsg);
}

void MainWindow::on_btnChannal_clicked()
{
    injector("/1ZIR\r\n", 7);

}

void MainWindow::injector(char *command, int length)
{
    m_port2->clear();
    m_port2->write(command, length);

    auto msg = m_port2->readAll();
    QString printmsg;
    printmsg.prepend(msg);
    qDebug()<<QString(printmsg);
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


/*
void MainWindow::on_btnTestGo_clicked()
{

    std::thread goTask([&](){

        volatile int direct = 0;
        volatile int iDex = 0;
        volatile int jDex = 0;
        int status;
        int boxNumber = ui->cbBoardNumber->currentIndex()+1;
        int iNumber = 0;
        float xps[4] = {2740,2740,2740-1450,2740-1450};
        float yps[4] = {1775, 1775-950, 1775, 1775-950};
        for(iNumber=0; iNumber<boxNumber; iNumber++)
        {

            ui->xSpeed->setText("800");
            ui->ySpeed->setText("800");
            ui->zSpeed->setText("800");

            on_btnSpeedSet_clicked();

            ZAux_Direct_Single_MoveAbs(
                        g_handle, 0, xps[iNumber]);
            ZAux_Direct_Single_MoveAbs(
                        g_handle, 1, yps[iNumber]);

            while (iDex < 8)
            {
                direct = iDex%2;
                jDex = 0;
                printf("befor x move!\n");
                printf("board Number: %d\n", boxNumber);
                while (jDex < 11)  //11
                {
                    printf("enter x move!\n");
                    //wait for printing...
                    //if(gbBreak)
                    // goto GOEND;
                    gbArrival = true;
                    while(!gbDrops)
                    {
                        // wait for drops done!
                    }
                    gbDrops = false;

                    while(m_bsuspend)
                    {
                        //是否暂停
                        //if(gbBreak)
                        // goto GOEND;
                    }
                    if (direct == 0)
                    {
                        ZAux_Direct_Single_Move(g_handle, 0, -90);
                    }
                    else
                    {
                        ZAux_Direct_Single_Move(g_handle, 0, 90);
                    }

                    while(true)
                    {
                        ZAux_Direct_GetIfIdle(g_handle, 0, &status);
                        if(status != 0)
                            break;
                    }

                    while(true)
                    {
                        ZAux_Direct_GetIfIdle(g_handle, 1, &status);
                        if(status != 0)
                            break;
                    }
                    jDex++;

                    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                jDex = 0;
                printf("befor y move!\n");
                if(iDex != 7)
                    ZAux_Direct_Single_Move(g_handle, 1, -90);

                while(true)
                {
                    ZAux_Direct_GetIfIdle(g_handle, 0, &status);
                    if(status != 0)
                        break;
                }

                while(true)
                {
                    ZAux_Direct_GetIfIdle(g_handle, 1, &status);
                    if(status != 0)
                        break;
                }
                printf("after y move!\n");
                iDex++;

                            gbArrival = true;
                            while(!gbDrops)
                            {
                                // wait for drops done!
                            }

                            gbDrops = false;
                //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
        }

        GOEND:
        gbBreak = false;

    });
    goTask.detach();
}
*/

void MainWindow::on_btnTestGo_clicked()
{

    std::thread goTask([&](){

        volatile int direct = 0;
        volatile int iDex = 0;
        volatile int jDex = 0;
        int status;

        float xps[4];
        float yps[4];

        float firstx = 2680;
        float firsty = 1675;

        xps[0] = firstx;
        xps[1] = firstx;
        xps[2] = firstx-1380;
        xps[3] = firstx-1380;

        yps[0] = firsty;
        yps[1] = firsty-955;
        yps[2] = firsty;
        yps[3] = firsty-955;


        /*
        ui->xSpeed->setText("800");
        ui->ySpeed->setText("800");
        ui->zSpeed->setText("800");

        on_btnSpeedSet_clicked();  */

        //        ZAux_Direct_Single_MoveAbs(
        //                    g_handle, 0, 2710);
        //        ZAux_Direct_Single_MoveAbs(
        //                    g_handle, 1, 1775);

        int boxNumber = 1;
        //boxNumber = ui->cbBoardNumber->currentIndex();
        int i = 0;

        //判断有无选择板数
        //QString zone = ui->cbBoardNumber->currentData("请选择").toString();

        if(boxNumber == 0)
        {
            QMessageBox::warning(this,"注意","请选择板数!");
        }
        else
        {
            //在某地等到花儿谢为止
            double wait_x, wait_y; // 此处花姑娘大大地有，所以在这等吧。。。
            wait_x = 2770;
            wait_y = 1800;
            do{
                while(true)
                {
                    ZAux_Direct_GetIfIdle(g_handle, 0, &status);
                    if(status != 0)
                        break;
                }

                while(true)
                {
                    ZAux_Direct_GetIfIdle(g_handle, 1, &status);
                    if(status != 0)
                        break;
                }
                //运动到流水位
                ZAux_Direct_Single_MoveAbs(
                            g_handle, 0, wait_x);
                ZAux_Direct_Single_MoveAbs(
                            g_handle, 1,  wait_y);
                while(true)
                {
                    ZAux_Direct_GetIfIdle(g_handle, 0, &status);
                    if(status != 0)
                        break;
                }

                while(true)
                {
                    ZAux_Direct_GetIfIdle(g_handle, 1, &status);
                    if(status != 0)
                        break;
                }
                gbArrival = true;//运动到了坐标点
                while(!gbDrops)
                {
                    // wait for drops done!
                    if(gbBreak)
                        goto BREAKEND;
                }
                gbDrops = false;

            }while (0);



            for(i=0; i<=boxNumber-1; i++)
            {
                direct = 0;
                iDex = 0;
                jDex = 0;
                while(true)
                {
                    ZAux_Direct_GetIfIdle(g_handle, 0, &status);
                    if(status != 0)
                        break;
                }

                while(true)
                {
                    ZAux_Direct_GetIfIdle(g_handle, 1, &status);
                    if(status != 0)
                        break;
                }
                ////运动到第一孔位
                ZAux_Direct_Single_MoveAbs(
                            g_handle, 0, xps[i]);
                ZAux_Direct_Single_MoveAbs(
                            g_handle, 1, yps[i]);
                while(true)
                {
                    ZAux_Direct_GetIfIdle(g_handle, 0, &status);
                    if(status != 0)
                        break;
                }

                while(true)
                {
                    ZAux_Direct_GetIfIdle(g_handle, 1, &status);
                    if(status != 0)
                        break;
                }
                gbArrival = true;
                while(!gbDrops)
                {
                    // wait for drops done!
                    if(gbBreak)
                        goto BREAKEND;
                }
                gbDrops = false;
                while (iDex < 8)
                {
                    direct = iDex%2;
                    jDex = 0;
                    printf("befor x move!\n");
                    while (jDex < 11)
                    {
                        printf("enter x move!\n");

                        while(m_bsuspend)
                        {
                            //是否暂停
                            if(gbBreak)
                                goto BREAKEND;
                        }
                        if (direct == 0)
                            ZAux_Direct_Single_Move(g_handle, 0, -90);
                        else
                        {
                            ZAux_Direct_Single_Move(g_handle, 0, 90);
                        }

                        while(true)
                        {
                            ZAux_Direct_GetIfIdle(g_handle, 0, &status);
                            if(status != 0)
                                break;
                        }

                        while(true)
                        {
                            ZAux_Direct_GetIfIdle(g_handle, 1, &status);
                            if(status != 0)
                                break;
                        }
                        jDex++;
                        gbArrival = true;
                        while(!gbDrops)
                        {
                            // wait for drops done!
                            if(gbBreak)
                                goto BREAKEND;
                        }
                        gbDrops = false;
                        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                    jDex = 0;
                    printf("befor y move!\n");
                    if(iDex == 7)
                        break;
                    ZAux_Direct_Single_Move(g_handle, 1, -90);
                    while(true)
                    {
                        ZAux_Direct_GetIfIdle(g_handle, 0, &status);
                        if(status != 0)
                            break;
                    }

                    while(true)
                    {
                        ZAux_Direct_GetIfIdle(g_handle, 1, &status);
                        if(status != 0)
                            break;
                    }
                    printf("after y move!\n");
                    iDex++;

                    gbArrival = true;
                    while(!gbDrops)
                    {
                        // wait for drops done!
                        if(gbBreak)
                            goto BREAKEND;
                    }

                    gbDrops = false;
                    //std::this_thread::sleep_for(std::chrono::milliseconds(1000));

                }

            }

BREAKEND:
            gbBreak = false;
            //走完板子后返回到流水位
            while(true)
            {
                ZAux_Direct_GetIfIdle(g_handle, 0, &status);
                if(status != 0)
                    break;
            }

            while(true)
            {
                ZAux_Direct_GetIfIdle(g_handle, 1, &status);
                if(status != 0)
                    break;
            }
            ZAux_Direct_Single_MoveAbs(
                        g_handle, 0, wait_x);
            ZAux_Direct_Single_MoveAbs(
                        g_handle, 1,  wait_y);
        }
    });
    goTask.detach();
};






void MainWindow::on_btnOrigin_clicked()
{
    ui->xSpeed->setText("800");
    ui->ySpeed->setText("800");
    ui->zSpeed->setText("800");

    on_btnSpeedSet_clicked();

    ZAux_Direct_Single_MoveAbs(
                g_handle, 0, 2680);
    ZAux_Direct_Single_MoveAbs(
                g_handle, 1, 1675);
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

    int fgttype1 = ui->fgtType1->currentIndex();
    int fgttype2 = ui->fgtType2->currentIndex();

    fgt_set_sensorCalibration(sid1, (fgt_calibration_t)fgttype1);
    fgt_set_sensorCalibration(sid2, (fgt_calibration_t)fgttype2);

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
        warnning(3);
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
        //QMessageBox::information(this, "error", "port2 open failed!");
        warnning(4);
        return;
    }
    else
    {
        qDebug() << "open serial port ttys1";
        injector("/1ZIR\r\n", 7);
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

    //QMessageBox::information(this, "OK", "Setting OK!");
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

void MainWindow::warnning(int num)  //报警
{
    switch (num) {
    case 1:
        QMessageBox::information(this, "warn", "相机无法连接");
        break;
    case 2:
        QMessageBox::information(this, "warn", "运动控制无法连接");
        break;
    case 3:
        QMessageBox::information(this, "warn", "切换阀无法连接");
        break;
    case 4:
        QMessageBox::information(this, "warn", "注射泵无法连接");
        break;
    case 5:
        QMessageBox::information(this, "warn", "fgt无法连接");
        break;
    default:
        break;
    }
}

// move to abs point
void MainWindow::goPoint(float x, float y)
{
    int xstate = 0;
    int ystate = 0;

    ZAux_Direct_Single_MoveAbs(g_handle, 0, x);
    ZAux_Direct_Single_MoveAbs(g_handle, 1, y);

    while(xstate == 0){
        ZAux_Direct_GetIfIdle(g_handle, 0, &xstate);
    }
    while(ystate == 0){
        ZAux_Direct_GetIfIdle(g_handle, 1, &ystate);
    }
}

void MainWindow::printBall()
{
    ZAux_Direct_SetOp(g_handle, 8, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(m_printDelay));
    ZAux_Direct_SetOp(g_handle, 8, 0);
}


void MainWindow::on_btnStop_clicked()
{
    if(m_bsuspend){
        ui->btnStop->setText("暂停");
        // start fgt code here...
        fgt_set_sensorRegulation(sid1, pid1, 0);
        fgt_set_sensorRegulation(sid2, pid2, 110);
        m_bsuspend = false;
        bsuspend = false;
    }
    else {
        m_bsuspend = true;
        bsuspend = true;
        // stop fgt code here...
        fgt_set_sensorRegulation(sid1, pid1, 0);
        fgt_set_sensorRegulation(sid2, pid2, 0);
        ui->btnStop->setText("继续");
    }
}

// get new Image for ROI setting and model selected
//volatile bool bgetModel = false;
void MainWindow::on_btnGetImage_clicked()
{
    bgetModel = true;
    QMessageBox::information(this, "OK", "Get model OK!");
}

//volatile bool bsaveModel = false;
//volatile bool bcompareImage = false;
//int icompare=0;
void MainWindow::on_btnSaveModel_clicked()
{
    //bsaveModel = true;
    if(icompare%2==0)
        bcompareImage = true;
    else
        bcompareImage = false;

    icompare++;
    QMessageBox::information(this, "OK", "compare OK!");
}

extern volatile bool bResetCamera;
void MainWindow::on_btnRestCamera_clicked()
{
    bResetCamera = true;
}


void MainWindow::on_btnFgtRun_clicked()
{
    auto coms =  ui->fgtCommand->text();
    char *ch;
    QByteArray ba = coms.toLatin1();
    ch = ba.data();
    auto length = strlen(ch);
    char* sendmsg = (char* )malloc((length+2)*sizeof(char));
    memcpy(sendmsg,ch, length);
    sendmsg[length] = '\r';
    sendmsg[length+1] = '\n';

    injector(sendmsg,length+2);
}




void MainWindow::on_btnFgtTest_clicked()
{


    bool bfgt_finished = false;//fgt速度变化动作
    int fgttype1 = ui->fgtType1->currentIndex();
    int fgttype2 = ui->fgtType2->currentIndex();

//    fgt_set_sensorCalibration(sid1, (fgt_calibration_t)fgttype1);
//    fgt_set_sensorCalibration(sid2, (fgt_calibration_t)fgttype2);

    fgt_set_sensorCalibration(sid1, fgt_SENSOR_CALIBRATION::HFE);
    fgt_set_sensorCalibration(sid2, fgt_SENSOR_CALIBRATION::HFE);

    fgt_set_sensorRegulation(sid1, pid1, 0);//0
    fgt_set_sensorRegulation(sid2, pid2, 140);//100

    switchChannal(1);
    injector("/1V50A1100R\r\n", 13);
    printf("befor test chanal 2\n");


    std::thread GodBless([&](){

        std::this_thread::sleep_for(std::chrono::milliseconds(30000));//抽样到暂存管所需时间
        switchChannal(2);
        printf("after test chanal 2\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));//等待切换阀完成时间
        bool bfgt_finished = true;

    });
    GodBless.detach();


    std::thread GodBless1([&](){

        while(!bfgt_finished)
        {
            //等待GodBless线程完成
        }
        printf("test change 1\n");
        fgt_set_sensorRegulation(sid1, pid1, 140);
        fgt_set_sensorRegulation(sid2, pid2, 140);



        std::this_thread::sleep_for(std::chrono::milliseconds(17000));//推样到剪切点所需时间，时间结束切换阀切换到推样模式

        printf("test change 2\n");
        fgt_set_sensorRegulation(sid1, pid1, 17);
        fgt_set_sensorRegulation(sid2, pid2, 140);



    });
    GodBless1.detach();


}

void MainWindow::on_btnFgtWash_clicked()
{
    switchChannal(2);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    injector("/1gV1600OA1500IA0G2R\r\n", 22);

}

//extern volatile bool bsetRoi;
void MainWindow::on_btnSetRoi_clicked()
{
    bsetRoi = true;
}

void MainWindow::on_btnInjectEMg_clicked()
{
    // injector("/1gOA6000IA0G2R\r\n", 17);

}

//volatile int gMinThresold = 35;
//volatile int gMinArea = 200;
void MainWindow::on_btnSetThresold_clicked()
{
    gMinThresold = ui->lineThresold->text().toInt();
    gMinArea = ui->lineDiffArea->text().toInt();
}


void MainWindow::on_btn1P_clicked()
{
    injector("/1P100R\r\n", 9);
}

void MainWindow::on_btn1D_clicked()
{
    injector("/1D100R\r\n", 9);
}

void MainWindow::on_btnBreak_clicked()
{
    injector("/1T\r\n", 5);
    gbBreak = true;
    //injector("/1I2V1500A0R\r\n", 14);
    fgt_set_sensorRegulation(sid1, pid1, 2);
    fgt_set_sensorRegulation(sid2, pid2, 2);

    gActived = true;
    QMessageBox::information(this, "OK", "Break OK!");
}


void MainWindow::on_btnwash_clicked()
{
    switchChannal(2);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    injector("/1gV1600OA1500IA0G4R\r\n", 22);//循环冲洗四次
    std::this_thread::sleep_for(std::chrono::milliseconds(15000));
    QMessageBox::information(this, "OK", "冲洗抽样管道完成!");
}
extern volatile bool bshowImage;
void MainWindow::on_cbShowImage_stateChanged(int stateInt)
{
    if(stateInt){
        bshowImage = true;
    }else{
        bshowImage = false;
    }
}

void MainWindow::on_btnwash_2_clicked()
{
    on_btnFgtInit_clicked();
    switchChannal(2);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    fgt_set_sensorCalibration(sid1, fgt_SENSOR_CALIBRATION::HFE);
    fgt_set_sensorCalibration(sid2, fgt_SENSOR_CALIBRATION::HFE);
    fgt_set_sensorRegulation(sid1, pid1, 200);
    fgt_set_sensorRegulation(sid2, pid2, 200);
    std::this_thread::sleep_for(std::chrono::milliseconds(25000));
    fgt_set_sensorRegulation(sid1, pid1, 0);
    fgt_set_sensorRegulation(sid2, pid2, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    QMessageBox::information(this, "OK", "冲洗管道完成!");
}

void MainWindow::on_btnLogin_clicked()
{
    ui->btnRun->setDisabled(true);

    //FGT Init
    // Variables declaration

    std::string unit = "";
    float range_min = 0;
    float range_max = 0;
    float sensor_value = 0;
    fgt_ERROR_CODE errCode = fgt_ERROR_CODE::OK;
    fgt_CHANNEL_INFO pinfo[256];
    // Initialize session with all detected Fluigent instrument(s)
    // This step is optional, if not called session will be automatically created
    if(fgtInitTimes == 0){
        errCode = Fgt_init();
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));

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
        fgtInitTimes++;
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
    fgt_set_sensorCalibration(sid1, fgt_SENSOR_CALIBRATION::HFE);
    fgt_set_sensorCalibration(sid2, fgt_SENSOR_CALIBRATION::HFE);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // bind the sensors and pressures
    fgt_set_sensorRegulation(sid1, pid1, 0);
    fgt_set_sensorRegulation(sid2, pid2, 110);

    fgt_set_sensorRegulation(sid1, pid1, 0);
    fgt_set_sensorRegulation(sid2, pid2, 110);
    bfgt = true; // has been started...


    //制备微球
    volatile bool bfgt_finished = false;//fgt速度变化动作
    bool bcarve_finished = false;//到达切割点动作/1T
    bool btestgo_finished = false;//三轴走完板子的动作
    volatile bool pleaseGo = false;
    //    fgt_set_sensorCalibration(sid1, fgt_SENSOR_CALIBRATION::HFE);//设置fgt的液体类型为HFE
    //    fgt_set_sensorCalibration(sid2, fgt_SENSOR_CALIBRATION::HFE);

    //    //设置fgt的推样泵速度为0，推油速度为110
    //    fgt_set_sensorRegulation(sid1, pid1, 0);//0
    //    fgt_set_sensorRegulation(sid2, pid2, 110);//110


//    //切换阀切换到抽液模式，注射泵抽100微升的量，使100微升的样本被抽到暂存管里
//    switchChannal(1);
//    std::this_thread::sleep_for(std::chrono::milliseconds(2000));//等待切换阀完成时间
//    if(injectCounter !=0){
//        //injector("/1ZI1V50A1100R\r\n", 16);
//        //std::this_thread::sleep_for(std::chrono::milliseconds(8000));//等待切换阀完成时间
////        injector("/1OV1000A0R\r\n", 13);
////        std::this_thread::sleep_for(std::chrono::milliseconds(8000));//等待切换阀完成时间
////        injector("/1IV50A1100R\r\n", 14);
////        std::this_thread::sleep_for(std::chrono::milliseconds(10000));//等待切换阀完成时间
//        printf("before zhushebeng1\n");
//        injector("/1OV1000A0IV50A1100R\r\n", 22);
//         printf("after zhushebeng1\n");
//        std::this_thread::sleep_for(std::chrono::milliseconds(40000));//等待切换阀完成时间
//    }
//    else {
//        injector("/1IV50A1100R\r\n", 14);
//    }

//    injectCounter++;



    std::thread GodBless([&](){

//        std::this_thread::sleep_for(std::chrono::milliseconds(30000));//抽样到暂存管所需时间
//        switchChannal(2);
//        printf("after test chanal 2\n");
//        std::this_thread::sleep_for(std::chrono::milliseconds(1000));//等待切换阀完成时间

//        printf("test change 1\n");
//        fgt_set_sensorRegulation(sid1, pid1, 140);
//        fgt_set_sensorRegulation(sid2, pid2, 140);

//        std::this_thread::sleep_for(std::chrono::milliseconds(17000));//推样到剪切点所需时间，时间结束切换阀切换到推样模式
//        printf("test change 2\n");
//        fgt_set_sensorRegulation(sid1, pid1, 17);
//        fgt_set_sensorRegulation(sid2, pid2, 140);

        fgt_set_sensorRegulation(sid1, pid1, 20);
        fgt_set_sensorRegulation(sid2, pid2, 170);

        volatile int direct = 0;
        volatile int iDex = 0;
        volatile int jDex = 0;
        int status;

        float xps[4];
        float yps[4];

        float firstx = 2680;
        float firsty = 1675;

        xps[0] = firstx;
        xps[1] = firstx;
        xps[2] = firstx-1380;
        xps[3] = firstx-1380;

        yps[0] = firsty;
        yps[1] = firsty-955;
        yps[2] = firsty;
        yps[3] = firsty-955;

        printf("prepare to go...\n");

        printf("take photo\n");
        //设置识别区域
        bsetRoi = true;
        //开始识别
        bcompareImage = true;
        gMinArea = 200;

        int boxNumber = 1;
        int i = 0;

        //在某地等到花儿谢为止
        double wait_x, wait_y; // 此处花姑娘大大地有，所以在这等吧。。。
        wait_x = 2770;
        wait_y = 1800;
        do{
            while(true)
            {
                ZAux_Direct_GetIfIdle(g_handle, 0, &status);
                if(status != 0)
                    break;
            }

            while(true)
            {
                ZAux_Direct_GetIfIdle(g_handle, 1, &status);
                if(status != 0)
                    break;
            }
            //运动到流水位
            ZAux_Direct_Single_MoveAbs(
                        g_handle, 0, wait_x);
            ZAux_Direct_Single_MoveAbs(
                        g_handle, 1,  wait_y);
            while(true)
            {
                ZAux_Direct_GetIfIdle(g_handle, 0, &status);
                if(status != 0)
                    break;
            }

            while(true)
            {
                ZAux_Direct_GetIfIdle(g_handle, 1, &status);
                if(status != 0)
                    break;
            }
            gbArrival = true;//运动到了坐标点
            while(!gbDrops)
            {
                // wait for drops done!
                if(gbBreak)
                    goto BREAKEND;
            }
            gbDrops = false;

        }while (0);



        for(i=0; i<=boxNumber-1; i++)
        {
            direct = 0;
            iDex = 0;
            jDex = 0;
            while(true)
            {
                ZAux_Direct_GetIfIdle(g_handle, 0, &status);
                if(status != 0)
                    break;
            }

            while(true)
            {
                ZAux_Direct_GetIfIdle(g_handle, 1, &status);
                if(status != 0)
                    break;
            }
            ////运动到第一孔位
            ZAux_Direct_Single_MoveAbs(
                        g_handle, 0, xps[i]);
            ZAux_Direct_Single_MoveAbs(
                        g_handle, 1, yps[i]);
            while(true)
            {
                ZAux_Direct_GetIfIdle(g_handle, 0, &status);
                if(status != 0)
                    break;
            }

            while(true)
            {
                ZAux_Direct_GetIfIdle(g_handle, 1, &status);
                if(status != 0)
                    break;
            }
            gbArrival = true;
            while(!gbDrops)
            {
                // wait for drops done!
                if(gbBreak)
                    goto BREAKEND;
            }
            gbDrops = false;
            while (iDex < 8)
            {
                direct = iDex%2;
                jDex = 0;
                printf("befor x move!\n");
                while (jDex < 11)
                {
                    printf("enter x move!\n");

                    while(m_bsuspend)
                    {
                        //是否暂停
                        if(gbBreak)
                            goto BREAKEND;
                    }
                    if (direct == 0)
                        ZAux_Direct_Single_Move(g_handle, 0, -90);
                    else
                    {
                        ZAux_Direct_Single_Move(g_handle, 0, 90);
                    }

                    while(true)
                    {
                        ZAux_Direct_GetIfIdle(g_handle, 0, &status);
                        if(status != 0)
                            break;
                    }

                    while(true)
                    {
                        ZAux_Direct_GetIfIdle(g_handle, 1, &status);
                        if(status != 0)
                            break;
                    }
                    jDex++;
                    gbArrival = true;
                    while(!gbDrops)
                    {
                        // wait for drops done!
                        if(gbBreak)
                            goto BREAKEND;
                    }
                    gbDrops = false;
                    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                jDex = 0;
                printf("befor y move!\n");
                if(iDex == 7)
                    break;
                ZAux_Direct_Single_Move(g_handle, 1, -90);
                while(true)
                {
                    ZAux_Direct_GetIfIdle(g_handle, 0, &status);
                    if(status != 0)
                        break;
                }

                while(true)
                {
                    ZAux_Direct_GetIfIdle(g_handle, 1, &status);
                    if(status != 0)
                        break;
                }
                printf("after y move!\n");
                iDex++;

                gbArrival = true;
                while(!gbDrops)
                {
                    // wait for drops done!
                    if(gbBreak)
                        goto BREAKEND;
                }

                gbDrops = false;
                //std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            }

        }

BREAKEND:
        gbBreak = false;
        //走完板子后返回到流水位
        while(true)
        {
            ZAux_Direct_GetIfIdle(g_handle, 0, &status);
            if(status != 0)
                break;
        }

        while(true)
        {
            ZAux_Direct_GetIfIdle(g_handle, 1, &status);
            if(status != 0)
                break;
        }
//        ZAux_Direct_Single_MoveAbs(
//                    g_handle, 0, wait_x);
//        ZAux_Direct_Single_MoveAbs(
//                    g_handle, 1,  wait_y);
        backZero();
        printf("stop!\n");
        bcompareImage = false;//停止识别
        btestgo_finished = true;

        printf("backThrea\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));


        //injector("/1I2V1500A0R\r\n", 14);
        //injector("/1ZR\r\n", 6);
        fgt_set_sensorRegulation(sid1, pid1, 2);
        fgt_set_sensorRegulation(sid2, pid2, 2);
        gActived = true;

    });
     GodBless.detach();
}
