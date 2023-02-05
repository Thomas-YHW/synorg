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

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);

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
    ~MainWindow();
    void setLimitedPos();
    void backZero();    //Zero
    void xMoveTO(float distance);   //Move to distance...
    void yMoveTo(float distance);
    void zMoveTo(float distance);


    //单双切换阀

    void switchChannal(int number);
    void injector(char* command, int length);
    void initFgtUi();
    void initPorts();   //open com port to commuinicate with switch and injector
    void loadSpeed();

protected:
    //这是一个虚函数，从QEvent继承而来.
    void timerEvent(QTimerEvent*event);
private slots:
    void on_open_clicked();
    void on_close_clicked();

    void on_Start_clicked();

    void on_Stop_clicked();

    void on_Clear_clicked();
    void on_btnClear_clicked(); //clear all alarm.

    void on_radio_X_clicked();

    void on_radio_Y_clicked();

    void on_radi_Z_clicked();

    void on_radio_R_clicked();

    void on_btnOriginX_clicked();
    void on_btnOriginY_clicked();
    void on_btnOriginZ_clicked();

    void on_btnXAdd_clicked();
    void on_btnYAdd_clicked();
    void on_btnZAdd_clicked();

    void on_btnXSub_clicked();
    void on_btnYSub_clicked();
    void on_btnZSub_clicked();

    void on_btnIO1_clicked();
    void on_btnIO2_clicked();
    void on_btnIO3_clicked();


    //functions
    void on_btnReset_clicked();
    void on_btnRun_clicked();

    void on_btnCamera_clicked();

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

private:
    Ui::MainWindow *ui;
    QButtonGroup *bt;

    bool bfgt = false;

    bool io1 = true;
    bool io2 = true;
    bool io3 = true;

    float m_xLimit = 1000;
    float m_yLimit = 1000;
    float m_zLimit = 1000;

    QSerialPort*    m_port1;    //切换阀
    QSerialPort*    m_port2;    //注射泵

    void Update_Para();

};

#endif // MAINWINDOW_H
