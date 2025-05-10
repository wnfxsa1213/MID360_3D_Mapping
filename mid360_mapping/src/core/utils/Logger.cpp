#include "Logger.h"
#include <QDebug>
#include <QMetaObject>

Logger* Logger::instance = nullptr;

Logger& Logger::getInstance()
{
    if (!instance) {
        instance = new Logger();
    }
    return *instance;
}

Logger::Logger(QObject* parent)
    : QObject(parent), currentLevel(Info)
{
}

Logger::~Logger()
{
    if (logFile.isOpen()) {
        logStream.flush();
        logFile.close();
    }
}

void Logger::initialize(const QString& logDir)
{
    QMutexLocker locker(&mutex);
    
    logDirectory = logDir;
    
    // 创建日志目录
    QDir dir(logDirectory);
    if (!dir.exists()) {
        dir.mkpath(".");
    }
    
    // 打开日志文件
    QString logFileName = getLogFileName();
    logFile.setFileName(logFileName);
    
    if (!logFile.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
        qWarning() << "无法打开日志文件:" << logFileName;
        return;
    }
    
    logStream.setDevice(&logFile);
    
    // 写入日志头
    logStream << "\n=== 日志开始于 " << QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss") << " ===\n";
    logStream.flush();
}

void Logger::setLogLevel(LogLevel level)
{
    QMutexLocker locker(&mutex);
    currentLevel = level;
}

void Logger::log(LogLevel level, const QString& message)
{
    if (level < currentLevel) {
        return;
    }
    
    QMutexLocker locker(&mutex);
    
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz");
    QString levelStr = levelToString(level);
    QString logMessage = QString("[%1] [%2] %3").arg(timestamp, levelStr, message);
    
    // 写入文件
    if (logFile.isOpen()) {
        logStream << logMessage << "\n";
        logStream.flush();
    }
    
    // 输出到控制台
    switch (level) {
        case Debug:
            qDebug() << logMessage;
            break;
        case Info:
            qInfo() << logMessage;
            break;
        case Warning:
            qWarning() << logMessage;
            break;
        case Error:
        case Fatal:
            qCritical() << logMessage;
            break;
    }
    
    // 发射信号 - 使用Q_EMIT并确保参数类型正确
    Q_EMIT this->logMessage(static_cast<int>(level), message);
}

QString Logger::levelToString(LogLevel level)
{
    switch (level) {
        case Debug:   return "DEBUG";
        case Info:    return "INFO";
        case Warning: return "WARNING";
        case Error:   return "ERROR";
        case Fatal:   return "FATAL";
        default:      return "UNKNOWN";
    }
}

QString Logger::getLogFileName()
{
    QString date = QDateTime::currentDateTime().toString("yyyy-MM-dd");
    return QString("%1/mid360_mapping_%2.log").arg(logDirectory, date);
} 