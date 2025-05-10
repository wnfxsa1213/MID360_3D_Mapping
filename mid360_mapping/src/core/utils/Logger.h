#ifndef LOGGER_H
#define LOGGER_H

#include <QObject>
#include <QFile>
#include <QTextStream>
#include <QDateTime>
#include <QMutex>
#include <QDir>

class Logger : public QObject
{
    Q_OBJECT
    
public:
    enum LogLevel {
        Debug,
        Info,
        Warning,
        Error,
        Fatal
    };
    
    static Logger& getInstance();
    
    void initialize(const QString& logDir = "logs");
    void setLogLevel(LogLevel level);
    void log(LogLevel level, const QString& message);
    
    // 便捷日志函数
    void debug(const QString& message) { log(Debug, message); }
    void info(const QString& message) { log(Info, message); }
    void warning(const QString& message) { log(Warning, message); }
    void error(const QString& message) { log(Error, message); }
    void fatal(const QString& message) { log(Fatal, message); }
    
signals:
    void logMessage(int level, const QString& message);
    
private:
    Logger(QObject* parent = nullptr);
    ~Logger();
    
    QString levelToString(LogLevel level);
    QString getLogFileName();
    
    static Logger* instance;
    QFile logFile;
    QTextStream logStream;
    LogLevel currentLevel;
    QMutex mutex;
    QString logDirectory;
};

// 便捷宏
#define LOG_DEBUG(msg) Logger::getInstance().debug(msg)
#define LOG_INFO(msg) Logger::getInstance().info(msg)
#define LOG_WARNING(msg) Logger::getInstance().warning(msg)
#define LOG_ERROR(msg) Logger::getInstance().error(msg)
#define LOG_FATAL(msg) Logger::getInstance().fatal(msg)

#endif // LOGGER_H 