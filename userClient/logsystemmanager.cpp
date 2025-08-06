#include "LogSystemManager.h"
#include <QFile>
#include <QTextStream>
#include <QStandardPaths>
#include <QDir>

LogSystemManager::LogSystemManager()
{
    QString dataDir = QStandardPaths::writableLocation(
        QStandardPaths::AppDataLocation);
    QDir().mkpath(dataDir);
    m_logFilePath = dataDir + "/logs.txt";
    loadLogData();
}

void LogSystemManager::makeAndSaveLogData(const QString& eventName,
                                          const QString& eventDetail)
{
    LogEntry e{ QDateTime::currentDateTime(),
               eventName, eventDetail };
    m_logs.append(e);
    saveLogData();
    emit logAppended(e);
}

void LogSystemManager::saveLogData()
{
    QFile f(m_logFilePath);
    if (!f.open(QIODevice::WriteOnly|QIODevice::Truncate|QIODevice::Text))
        return;
    QTextStream out(&f);
    for (auto& e : m_logs) {
        out << e.timestamp.toString(Qt::ISODate) << ","
            << e.eventName << ","
            << e.eventDetail << "\n";
    }
}

void LogSystemManager::loadLogData()
{
    m_logs.clear();
    QFile f(m_logFilePath);
    if (!f.open(QIODevice::ReadOnly|QIODevice::Text)) {
        emit logsLoaded();
        return;
    }
    QTextStream in(&f);
    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();
        if (line.isEmpty()) continue;
        QStringList parts = line.split(',', Qt::KeepEmptyParts);
        if (parts.size() < 3) continue;
        QDateTime ts = QDateTime::fromString(parts[0], Qt::ISODate);
        QString    name   = parts[1];
        QString    detail = parts.mid(2).join(',');
        m_logs.append({ ts, name, detail });
    }
    emit logsLoaded();
}

void LogSystemManager::populateModel(QStandardItemModel* model) {
    model->clear();
    model->setColumnCount(3);
    model->setHorizontalHeaderLabels(
        QStringList{"Date", "Event", "Detail"});
    for (const auto& e : m_logs) {
        QList<QStandardItem*> row;
        row << new QStandardItem(e.timestamp.toString(Qt::ISODate))
            << new QStandardItem(e.eventName)
            << new QStandardItem(e.eventDetail);
        for (auto* it : row){
            it->setTextAlignment(Qt::AlignCenter);
            it->setForeground(QColor("#ffffff"));
        }
        model->appendRow(row);
    }
}
