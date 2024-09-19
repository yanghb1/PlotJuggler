#ifndef HTMLTABLEPARSER_H
#define HTMLTABLEPARSER_H

#include <QString>
#include <QRegularExpression>
#include <QFile>

#include <QDebug>

class HtmlTableParser
{
public:
    HtmlTableParser();

    static QList<QPair<double, double>> parse(const QString& filename) {
        QList<QPair<double, double>> map;

        QFile file(filename);
            if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
                qDebug() << "Cannot open file for reading:" << file.errorString();
                return QList<QPair<double, double>>();
            }

        QTextStream in(&file);
        QString htmlContent = in.readAll();
        file.close();

        // Regular expressions to capture table data
        QRegularExpression rowRegex("<tr>(.*?)</tr>");
        QRegularExpression cellRegex("<td[^>]*>(.*?)</td>");

        QRegularExpressionMatchIterator rowIterator = rowRegex.globalMatch(htmlContent);
        while (rowIterator.hasNext()) {
            QRegularExpressionMatch rowMatch = rowIterator.next();
            QString rowContent = rowMatch.captured(1);

            QRegularExpressionMatchIterator cellIterator = cellRegex.globalMatch(rowContent);
            QStringList rowData;
            while (cellIterator.hasNext()) {
                QRegularExpressionMatch cellMatch = cellIterator.next();
                rowData << cellMatch.captured(1).trimmed();
            }

            if (!rowData.isEmpty()) {
                double key = rowData.at(3).toDouble();
                double value = (rowData.at(1).toDouble() + rowData.at(2).toDouble()) / 2.0;
//                double value = rowData.at(2).toDouble();
                map.append(QPair<double, double>(key, value));
            }
        }

        std::sort(map.begin(), map.end(), [](const QPair<double, double>& a, const QPair<double, double>& b){
            return a.first < b.first; // 按照第一个元素排序
        });

        return map;
    }

    static void parse(const QString& filename, std::vector<double>& x_data, std::vector<double>& y_data) {

        QFile file(filename);
            if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
                qDebug() << "Cannot open file for reading:" << file.errorString();
                return;
            }

        QTextStream in(&file);
        QString htmlContent = in.readAll();
        file.close();

        // Regular expressions to capture table data
        QRegularExpression rowRegex("<tr>(.*?)</tr>");
        QRegularExpression cellRegex("<td[^>]*>(.*?)</td>");

        QRegularExpressionMatchIterator rowIterator = rowRegex.globalMatch(htmlContent);
        while (rowIterator.hasNext()) {
            QRegularExpressionMatch rowMatch = rowIterator.next();
            QString rowContent = rowMatch.captured(1);

            QRegularExpressionMatchIterator cellIterator = cellRegex.globalMatch(rowContent);
            QStringList rowData;
            while (cellIterator.hasNext()) {
                QRegularExpressionMatch cellMatch = cellIterator.next();
                rowData << cellMatch.captured(1).trimmed();
            }

            if (!rowData.isEmpty()) {
                x_data.insert(x_data.end(), rowData.at(2).toDouble());
                y_data.insert(y_data.end(), rowData.at(1).toDouble());
            }
        }
    }


};

#endif // HTMLTABLEPARSER_H
