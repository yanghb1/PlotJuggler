#include "glwidget.h"

#include <QFile>
#include <QDir>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QDateTime>
#include <QtMath>
#include <QDebug>
#include <utility>

#include "kalmanfilter.h"

int index_x;
int index_y;
int index_VehSpd;  //速度
int index_ACUYawRate;  //横摆角速度
int index_EPS_StrgAng;  //方向盘转角

GLWidget::GLWidget() {}

std::map<std::string, std::vector<std::pair<double, double>>> GLWidget::SetFilePath(const QString &filePath)
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "Cannot open file:" << file.errorString();
        return std::map<std::string, std::vector<std::pair<double, double>>>();
    }

    QTextStream in(&file);
    QString sixthLine;

    // 读取文件的前六行
    for (int i = 0; i < 3 && !in.atEnd(); ++i) {
        sixthLine = in.readLine();
    }
    if (sixthLine.isEmpty()) {
        qDebug() << "The sixth line is empty or does not exist.";
        return std::map<std::string, std::vector<std::pair<double, double>>>();
    }

    // 按逗号分割第六行内容
    QStringList elements = sixthLine.split(',');
    index_x = elements.indexOf("MSG_OdomXPos");
    index_y = elements.indexOf("MSG_OdomYPos");
    index_VehSpd = elements.indexOf("VehSpd");
    index_ACUYawRate = elements.indexOf("ACUYawRate");
    index_EPS_StrgAng = elements.indexOf("EPS_StrgAng");

    int index_timestamp = elements.indexOf("timestamp");

    qint64 lastTimestamp = -1;
    int index = 0;

    //处理空行
    in.readLine();
    if(in.atEnd())
        return std::map<std::string, std::vector<std::pair<double, double>>>();

    QWriteLocker locker(&readWriteLock);

    //pbox
    points.clear();

    //横摆角速度
    points2.clear();
    points2.append(QVector2D(0,0));
    double angle = 0;  //横摆角

    //方向盘转角
    points4.clear();
    points4.append(QVector2D(0,0));
    VehicleState state = {0.0, 0.0, 0.0}; // 初始化车辆状态

    //pbox旋转
    points3.clear();
    points3.append(QVector2D(0,0));
    bool start_init = false;
    bool index_init = false;
    VehicleState start_point = {0.0, 0.0, 0.0};

    //pbox旋转后 2
    points5.clear();
    points5.append(QVector2D(0,0));
    VehicleState point_a = {0.0, 0.0, 0.0};
    VehicleState point_b = {0.0, 0.0, 0.0};
    bool init_a = false;
    bool init_b = false;


    QString content = in.readLine();
    while (!in.atEnd()) {
        QStringList values = content.split(',');

//        QString dateTimeStr = values.at(index_timestamp).split(" ").at(0)
//                              + " "
//                              + values.at(index_timestamp).split(" ").at(1);
//        QDateTime dateTime = QDateTime::fromString(dateTimeStr, "yyyy-MM-dd HH:mm:ss.zzz");
        // 转换为 Unix 时间戳（毫秒）
//        qint64 timestamp = dateTime.toMSecsSinceEpoch();
        qint64 timestamp = values.at(index_timestamp).split(" ").at(2).toLongLong();
        qint64 time = timestamp - lastTimestamp;

        from_pbox(values);

        if(lastTimestamp != -1){
            from_ACUYawRate(values, index, time, angle);
        }
        if(lastTimestamp != -1){
            from_EPS_StrgAng(values, state, time);
        }

        from_pbox_rotate(values, start_point, index, start_init, index_init);

        from_pbox_rotate_2(values, point_a, point_b, init_a, init_b, index);

        lastTimestamp = timestamp;
        ++index;
        content = in.readLine();
    }

//    ----------------------------------------------------------------------------
    std::map<std::string, std::vector<std::pair<double, double>>> data;

    std::vector<std::pair<double, double>> vec_dis_x;
    std::vector<std::pair<double, double>> vec_dis;

    for(int i = 0; i < points2.size(); ++i){
        QVector2D point_1 = points4.at(i);
        QVector2D point_2 = points2.at(i);

        double distance_x = std::abs(point_1.x() - point_2.x());
        double distance = std::sqrt(std::pow(point_1.x() - point_2.x(), 2) + std::pow(point_1.y() - point_2.y(), 2));

        vec_dis_x.push_back(std::pair<double, double>(i, distance_x));
        vec_dis.push_back(std::pair<double, double>(i, distance));
    }
    data.insert({"distance_x", vec_dis_x});
    data.insert({"distance", vec_dis});


  //    ----------------------------------------------------------------------------
//    QList<QStringList> data;
//    data << (QStringList() << "distance_x" << "distance");
//    for(int i = 50; i < points2.size(); ++i){
//        QVector2D point_1 = points4.at(i);
//        QVector2D point_2 = points2.at(i);

//        double distance_x = std::abs(point_1.x() - point_2.x());
//        double distance = std::sqrt(std::pow(point_1.x() - point_2.x(), 2) + std::pow(point_1.y() - point_2.y(), 2));

//        data << (QStringList()
//                 << QString::number(distance_x)
//                 << QString::number(distance));
//    }
//    writeToCSV(filePath + ".csv", data);

    update();

    return data;
}

void GLWidget::SetScalef(float scale)
{
    m_scale = scale;

    update();
}

void GLWidget::SetStartIndex(int index)
{
    m_startIndex = index;
}

void GLWidget::initializeGL()
{
    initializeOpenGLFunctions();
    glClearColor(0.2f, 0.2f, 0.2f, 0.0f); // 背景色
}

void GLWidget::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
}

void GLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity(); // 重置模型视图矩阵

    QReadLocker locker(&readWriteLock);
    if(points.empty())
        return;

    glScalef(m_scale, m_scale, m_scale);
    glTranslatef( m_translateX, - m_translateY, 0);

    draw_pbox();
    draw_ACUYawRate();
//    draw_EPS_StrgAng();
//    draw_pbox_rotate();
    draw_pbox_rotate_2();
}

void GLWidget::wheelEvent(QWheelEvent *event) {
    int numDegrees = event->angleDelta().y() / 8;
    int numSteps = numDegrees / 15;
    m_scale += numSteps * 0.0005f;

    // Ensure scale is positive
    if (m_scale < 0.0005f) {
        m_scale = 0.0005f;
    }
    update();
}

void GLWidget::mousePressEvent(QMouseEvent *event)
{
    m_lastX = event->x();
    m_lastY = event->y();

    m_dragging = event->button() == Qt::LeftButton ? 1 : 2;
}

void GLWidget::mouseReleaseEvent(QMouseEvent *event)
{
    m_dragging = 0;
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (! m_dragging)
        return;

    int dx = event->x() - m_lastX;
    int dy = event->y() - m_lastY;

    if(m_dragging == 1)
    {
        m_translateX += dx * 0.2f;
        m_translateY += dy * 0.2f;
    }else if(m_dragging == 2){
        m_rotate += dx * 0.1;
    }

    m_lastX = event->x();
    m_lastY = event->y();


    update();
}

void GLWidget::coordinate_convert(float &x, float &y, float last_x, float last_y, double distance, double angle){
    x = last_x + distance * qSin(qDegreesToRadians(angle));
    y = last_y + distance * qCos(qDegreesToRadians(angle));
}

void GLWidget::writeToCSV(const QString &fileName, const QList<QStringList> &data)
{
    QFile file(fileName);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            qWarning() << "Cannot open file for writing:" << file.errorString();
            return;
        }

        QTextStream out(&file);

        // 写入数据
        for (const QStringList &row : data) {
            QString line = row.join(",");  // 使用逗号作为分隔符
            out << line << "\n";
        }

        file.close();
}

void GLWidget::from_pbox(const QStringList &values)
{
    float point_x = values.at(index_x).toFloat();
    float point_y = values.at(index_y).toFloat();

    QVector2D point;
    point.setX(-point_x);
    point.setY(point_y);
    points.append(point);
}

void GLWidget::from_ACUYawRate(const QStringList &values, int index, qint64 time, double &angle)
{
    QVector2D point;

    double distance = values.at(index_VehSpd).toFloat() * 5.0 / 18.0 * time / 1000.0;
    angle += values.at(index_ACUYawRate).toFloat() * time / 1000.0;

    float x, y;
    coordinate_convert(x, y, points2.at(index-1).x(), points2.at(index-1).y(), distance, angle);
    point.setX(x);
    point.setY(y);
    points2.append(point);

}

void GLWidget::from_EPS_StrgAng(const QStringList &values, VehicleState &state, qint64 time)
{
    double speed = 0.0;     // 速度，单位：米/秒
    double deltaT = 0;    // 时间步长，单位：秒
    double epsStrgAng = 0; //方向盘转角

    QVector2D point;

    deltaT = time / 1000.0;
    speed = values.at(index_VehSpd).toDouble() * 5.0 / 18.0;

    epsStrgAng = values.at(index_EPS_StrgAng).toDouble();
    state = updateVehicleState(state, speed, deltaT, epsStrgAng);
    point.setX(state.x);
    point.setY(state.y);

    points4.append(point);
}

void GLWidget::from_pbox_rotate(const QStringList &values, VehicleState &start_point, int index, bool &start_init, bool &index_init)
{
    float point_x = values.at(index_x).toFloat();
    float point_y = values.at(index_y).toFloat();

    if(start_init && index_init){
        double distance = std::sqrt(std::pow(point_x - start_point.x, 2) + std::pow(point_y - start_point.y, 2));
        float angle = start_point.theta - qAtan((point_y - start_point.y) / (point_x - start_point.x));
        QVector2D point;
        point.setX(qSin(angle) * distance);
        point.setY(qCos(angle) * distance);
        points3.append(point);
    }

    if(! start_init){
        start_point.x = point_x;
        start_point.y = point_y;
        start_init = true;
    }
    if(index == m_startIndex){
        start_point.theta = qAtan((point_y - start_point.y) / (point_x-start_point.x));
        index_init = true;
    }

}

void GLWidget::from_pbox_rotate_2(const QStringList &values, VehicleState &point_a, VehicleState &point_b, bool &init_a, bool &init_b, int index)
{
    float point_x = values.at(index_x).toFloat();
    float point_y = values.at(index_y).toFloat();

    if(init_a && init_b){
        VehicleState point_c;
        point_c.x = point_x;
        point_c.y = point_y;

        QVector2D point;
        auto side = cal_point_side(point_a, point_b, point_c);
        if (side == PointSide::ON_LINE) {
            point.setX(0);
            double distance = cal_point_distance(point_a, point_c);
            if (is_same_direction(point_a, point_b, point_c)) {
                point.setY(distance);
            } else {
                point.setY(distance * -1);
            }
        } else {
            double a = cal_point_distance(point_b, point_c);
            double b = cal_point_distance(point_a, point_c);
            double c = cal_point_distance(point_a, point_b);
            double A = cal_point_angle(a, b, c);
            if (side == PointSide::LEFT) {
                // 二象限 左正 右负
                if (A < 90) {
                    double radians = A * (M_PI / 180.0);
                    point.setX(b * std::sin(radians));
                    point.setY(b * std::cos(radians));
                // 三象限
                } else {
                    double radians = (180 - A) * (M_PI / 180.0);
                    point.setX(b * std::sin(radians));
                    point.setY(b * std::cos(radians)* -1);
                }
            } else if (side == PointSide::RTGHT) {
                // 一象限 左正 右负
                if (A < 90) {
                    double radians = A * (M_PI / 180.0);
                    point.setX(b * std::sin(radians) * -1);
                    point.setY(b * std::cos(radians));
                // 四象限
                } else {
                    double radians = (180 - A) * (M_PI / 180.0);
                    point.setX(b * std::sin(radians) * -1);
                    point.setY(b * std::cos(radians) * -1);
                }
            }
        }

        points5.append(point);
    }

    if(!init_a){
        point_a.x = point_x;
        point_a.y = point_y;

        init_a = true;
    }
    if(!init_b && index == m_startIndex){
        point_b.x = point_x;
        point_b.y = point_y;

        init_b = true;
    }

}

void GLWidget::draw_pbox()
{
    glPushMatrix();
    glTranslatef(-points.at(0).x(), -points.at(0).y(), 0);
    glBegin(GL_POINTS);

    glColor3f(1.0f, 0.0f, 0.0f); // 红色
    glLineWidth(2.0f); // 设置线宽为 5 像素
    glPointSize(1.0f);

    for(int i = 0; i < points.size(); ++i){
        glVertex2f(points.at(i).x(), points.at(i).y());
    }

    glEnd();
    glPopMatrix();
}

void GLWidget::draw_ACUYawRate()
{
    glPushMatrix();
    glTranslatef(-points2.at(0).x(), -points2.at(0).y(), 0);
//    glRotatef(-m_rotate, 0, 0, 1);
    glBegin(GL_POINTS);

    glColor3f(0.0f, 1.0f, 0.0f); // 绿色
    glLineWidth(2.0f); // 设置线宽为 5 像素
    glPointSize(1.0f);

    for(int i = 0; i < points2.size(); ++i){
        glVertex2f(points2.at(i).x(), points2.at(i).y());
    }

    glEnd();
    glPopMatrix();
}

void GLWidget::draw_EPS_StrgAng()
{
    glPushMatrix();
    glTranslatef(-points4.at(0).x(), -points4.at(0).y(), 0);
//    glRotatef(-m_rotate, 0, 0, 1);
    glBegin(GL_POINTS);

    glColor3f(1.0f, 1.0f, 0.0f); // 黄色
    glLineWidth(2.0f); // 设置线宽为 5 像素
    glPointSize(1.0f);

    for(int i = 0; i < points4.size(); ++i){

        glVertex2f(points4.at(i).x(), points4.at(i).y());
    }

    glEnd();
    glPopMatrix();
}

void GLWidget::draw_pbox_rotate()
{
    glPushMatrix();
    glRotatef(-m_rotate, 0, 0, 1);
    glBegin(GL_POINTS);

    glColor3f(0.0f, 0.0f, 1.0f); // 蓝色
    glLineWidth(2.0f); // 设置线宽为 5 像素
    glPointSize(1.0f);

    for(int i = 0; i < points3.size(); ++i){
        glVertex2f(points3.at(i).x(), points3.at(i).y());
    }

    glEnd();
    glPopMatrix();
}

void GLWidget::draw_pbox_rotate_2()
{
    glPushMatrix();
    glRotatef(-m_rotate, 0, 0, 1);
    glBegin(GL_POINTS);

    glColor3f(0.0f, 0.0f, 1.0f); // 蓝色
    glLineWidth(2.0f); // 设置线宽为 5 像素
    glPointSize(1.0f);

    for(int i = 0; i < points5.size(); ++i){
        glVertex2f(points5.at(i).x(), points5.at(i).y());
    }

    glEnd();
    glPopMatrix();
}
