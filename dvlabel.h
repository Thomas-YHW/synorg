#ifndef DVLABEL_H
#define DVLABEL_H

#include <QLabel>
#include <QMouseEvent>
#include <QPainter>
#include <QWidget>


// ROI position and size.
extern int roix;
extern int roiy;
extern int roiw;
extern int roih;
class DvLabel : public QLabel
{
    Q_OBJECT
public:
    explicit DvLabel(QWidget *parent = nullptr);

protected slots:
    void paintEvent(QPaintEvent *event) override{
        Q_UNUSED(event);
        QLabel::paintEvent(event);
        QPainter painter(this);
        painter.setPen(QPen(Qt::red, 2));
        painter.drawRect(x, y, 20, 120);
//        painter.drawRect(50,50,x,y);
//        painter.end();
    }

   void mousePressEvent(QMouseEvent *e)override
   {
       QLabel::mousePressEvent(e);
        if( e->button() == Qt::LeftButton)
        {
            x = e->pos().x();
            y = e->pos().y();

            roix = x*5;
            roiy = y*5;
            roiw = 20*5;
            roih = 120*5;
//            painter.begin(this);
//            painter.drawRect(150,150,300,300);
//            painter.end();
            update();
        }
   }

private:

   int x = 10;
   int y = 10;
};

#endif // DVLABEL_H
