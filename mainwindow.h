#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "zmotion.h"
#include "zmcaux.h"
#include <QComboBox>
#include <QObject>
#include <QButtonGroup>
#include <QTimerEvent>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>

namespace Ui {
class MainWindow;
}

// ROI position and size.
extern int roix;
extern int roiy;
extern int roiw;
extern int roih;

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void setLimitedPos();
    void backZero();    //Zero
    void xMoveTO(float distance);   //Move to distance...
    void yMoveTo(float distance);
    void zMoveTo(float distance);
    void Update_Para();
    void linkZmotion();

    //单双切换阀

    void switchChannal(int number);
    void injector(char* command, int length);
    void initFgtUi();
    void initPorts();   //open com port to commuinicate with switch and injector
    void loadSpeed();
    void warnning(int num);
    void goPoint(float x, float y);
    void printBall();

protected:
    //这是一个虚函数，从QEvent继承而来.
    void timerEvent(QTimerEvent*event);

private slots:

    void on_Stop_clicked();


    void on_btnClear_clicked(); //clear all alarm.

    void on_btnOriginX_clicked();
    void on_btnOriginY_clicked();
    void on_btnOriginZ_clicked();

    void on_btnIO1_clicked();
    void on_btnIO2_clicked();
    void on_btnIO3_clicked();


    //functions
    void on_btnReset_clicked();
    void on_btnRun_clicked();
    void on_btnChannal_clicked();

    void on_btnXGo_clicked();

    void on_btnYGo_clicked();

    void on_btnZGo_clicked();

    void on_btnTestGo_clicked();

    void on_btnOrigin_clicked();

    void on_btnFgtInit_clicked();

    void on_btnFgtSet_clicked();

    void on_btnSw1_clicked();

    void on_btnSw2_clicked();

    void on_btnA6000_clicked();

    void on_btnA0_clicked();

    void on_btn1IR_clicked();

    void on_btn1OR_clicked();

    void on_btnExit_clicked();

    void on_btnStop_clicked();

    void on_btnSpeedSet_clicked();

    void on_btnGetImage_clicked();

    void on_btnSaveModel_clicked();

    void on_btnRestCamera_clicked();

    void on_btnFgtRun_clicked();

    //void on_btnDt_clicked();

    //void on_btnFgtDd_clicked();

    void on_btnFgtTest_clicked();

    void on_btnFgtWash_clicked();


    void on_btnSetRoi_clicked();

    void on_btnInjectEMg_clicked();

    void on_btnSetThresold_clicked();




    void on_btn1P_clicked();

    void on_btn1D_clicked();


    void on_btnBreak_clicked();



    void on_btnwash_clicked();


    void on_cbShowImage_stateChanged(int arg1);

    void on_btnwash_2_clicked();

    void on_btnLogin_clicked();

private:
    Ui::MainWindow *ui;
    QButtonGroup *bt;


    float m_acc = 0.0f;
    float m_creep = 0.0f;
    int m_datumin = -1;
    float m_dec = 0.0f;
    float m_lspeed = 0.0f;
    int m_nAxis = 0;
    float m_speed = 0.0f;
    float m_units = 0.0f;
    int m_datummode = 0;
    ZMC_HANDLE g_handle;
    int id1=0;

    bool bfgt = false;

    bool io1 = true;
    bool io2 = true;
    bool io3 = true;

    float m_xLimit = 1000;
    float m_yLimit = 1000;
    float m_zLimit = 1000;
    volatile bool m_bsuspend = false;

    int m_boardNumber = 1;

    QSerialPort*    m_port1;    //切换阀
    QSerialPort*    m_port2;    //注射泵

    int m_printTime = 100;
    int m_printDelay = 300;
};

#endif // MAINWINDOW_H
