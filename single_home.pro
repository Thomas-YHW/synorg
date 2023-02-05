#-------------------------------------------------
#
# Project created by QtCreator 2019-04-21T19:42:54
#
#-------------------------------------------------

QT       += core gui serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = single_home
TEMPLATE = app

QMAKE_CXXFLAGS += -D ENABLE_PRECOMPILED_HEADERS=OFF

SOURCES += main.cpp\
    camera.cpp \
    dvlabel.cpp \
    fgt_SDK_Cpp.cpp \
        mainwindow.cpp \
    zmcaux.cpp

HEADERS  += mainwindow.h \
    dvlabel.h \
    fgt_SDK.h \
    fgt_SDK_Cpp.h \
    lib/include/CameraParams.h \
    lib/include/MvCameraControl.h \
    lib/include/MvErrorDefine.h \
    lib/include/MvISPErrorDefine.h \
    lib/include/PixelType.h \
    zmotion.h \
    zmcaux.h

FORMS    += mainwindow_touch.ui \




RESOURCES += \
    res.qrc

unix:!macx: LIBS += -L$$PWD/./ -lzmotion

INCLUDEPATH += $$PWD/.
DEPENDPATH += $$PWD/.


unix:!macx: LIBS += -L$$PWD/lib/ -lMvCameraControl

INCLUDEPATH += $$PWD/.
INCLUDEPATH += $$PWD/include
DEPENDPATH += $$PWD/.

unix:!macx: LIBS += -L$$PWD/../../../usr/lib/ -lfgt_SDK

INCLUDEPATH += $$PWD/''
DEPENDPATH += $$PWD/''

unix:!macx: LIBS += -L$$PWD/../../../usr/lib/ -lopencv_core
unix:!macx: LIBS += -L$$PWD/../../../usr/lib/ -lopencv_imgproc
unix:!macx: LIBS += -L$$PWD/../../../usr/lib/ -lopencv_highgui
unix:!macx: LIBS += -L$$PWD/../../../usr/lib/ -lopencv_imgcodecs



