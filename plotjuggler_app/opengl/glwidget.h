#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QVector2D>
#include <QReadWriteLock>

#include "car_model.h"

class GLWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    GLWidget();

    std::map<std::string, std::vector<std::pair<double, double> > > SetFilePath(const QString &filePath);
    void SetScalef(float scale);
    void SetStartIndex(int index);

protected:
    void initializeGL() override;

    void resizeGL(int w, int h) override;

    void paintGL() override;

    void wheelEvent(QWheelEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;

private:
    void coordinate_convert(float &x, float &y, float last_x, float last_y, double distance, double angle);
    void writeToCSV(const QString &fileName, const QList<QStringList> &data);

    void from_pbox(const QStringList &values);
    void from_ACUYawRate(const QStringList &values, int index, qint64 time, double &angle);
    void from_EPS_StrgAng(const QStringList &values, VehicleState &state, qint64 time);
    void from_pbox_rotate(const QStringList &values, VehicleState &start_point, int index, bool &start_init, bool &index_init);

    void draw_pbox();
    void draw_ACUYawRate();
    void draw_EPS_StrgAng();
    void draw_pbox_rotate();


private:
    QVector<QVector2D> points;  //pbox
    QVector<QVector2D> points2; //横摆角急计算
    QVector<QVector2D> points3; //pbox旋转后
    QVector<QVector2D> points4; //方向盘转角
    QReadWriteLock readWriteLock;

    GLfloat m_scale = 0.005f;

    int m_dragging = 0; // 1:左键 2:右键
    int m_lastX = 0;
    int m_lastY = 0;
    GLfloat m_translateX = 0;
    GLfloat m_translateY = 0;
    GLfloat m_rotate = 0; //角度

    int m_startIndex = 30;
};

#endif // GLWIDGET_H
