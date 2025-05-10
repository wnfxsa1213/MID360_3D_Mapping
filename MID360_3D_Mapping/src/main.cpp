#include <QApplication>
#include <QSurfaceFormat>
#include <QVTKOpenGLNativeWidget.h>
#include <QDir>
#include <QMessageBox>
#include <QDebug>
#include <QFileInfo>
#include <QFile>
#include <QStatusBar>

#include "MainWindow.h"
#include "Logger.h"

// 检查Qt环境信息
void checkQtEnvironment(QApplication &app) 
{
    // 显示当前目录和Qt插件路径
    QString currentDir = QDir::currentPath();
    QString appDir = QCoreApplication::applicationDirPath();
    QString pluginsDir = app.libraryPaths().join("; ");
    QString qtConfPath;
    
    // 检查qt.conf文件是否存在
    QFile qtConf(appDir + "/qt.conf");
    if (qtConf.exists()) {
        qtConfPath = qtConf.fileName();
        
        // 读取qt.conf内容
        if (qtConf.open(QIODevice::ReadOnly | QIODevice::Text)) {
            QTextStream in(&qtConf);
            qtConfPath += "\nContent: " + in.readAll();
            qtConf.close();
        }
    } else {
        qtConfPath = "qt.conf not found";
        
        // 尝试创建qt.conf文件
        if (qtConf.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QTextStream out(&qtConf);
            out << "[Paths]\nPlugins = ./plugins\n";
            qtConf.close();
            qtConfPath = qtConf.fileName() + " (just created)";
            
            // 重新加载库路径
            app.removeLibraryPath(appDir);
            app.addLibraryPath(appDir);
        }
    }
    
    // 检查platforms目录是否存在
    QDir platformsDir(appDir + "/plugins/platforms");
    bool platformsDirExists = platformsDir.exists();
    
    // 如果platforms目录不存在，尝试创建
    if (!platformsDirExists) {
        QDir().mkpath(appDir + "/plugins/platforms");
        platformsDirExists = platformsDir.exists();
    }
    
    // 检查qwindows.dll是否存在
    QFile qwindowsDll(appDir + "/plugins/platforms/qwindows.dll");
    bool qwindowsExists = qwindowsDll.exists();
    
    // 获取环境变量
    QString qtdir = qgetenv("QTDIR");
    
    // 准备环境信息
    QString info = QString(
        "Application Directory: %1\n\n"
        "Current Directory: %2\n\n"
        "Qt Library Paths:\n%3\n\n"
        "QTDIR Environment: %4\n\n"
        "Qt Conf: %5\n\n"
        "Platforms directory exists: %6\n\n"
        "qwindows.dll exists: %7\n\n"
    ).arg(appDir)
     .arg(currentDir)
     .arg(pluginsDir)
     .arg(qtdir)
     .arg(qtConfPath)
     .arg(platformsDirExists ? "Yes" : "No")
     .arg(qwindowsExists ? "Yes" : "No");
    
    // 打印调试信息
    qDebug() << "Qt Environment Information:";
    qDebug() << info;
    
    // 显示环境信息对话框
    QMessageBox::information(nullptr, "Qt Environment", info);
}

int main(int argc, char *argv[])
{
    try {
        // 设置应用程序属性，必须在创建QApplication之前设置
        QCoreApplication::setAttribute(Qt::AA_UseDesktopOpenGL);
        QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts);
        
        // 创建应用程序
        QApplication app(argc, argv);
        
        // 设置应用程序信息
        QApplication::setApplicationName("MID360 3D Mapping");
        QApplication::setApplicationVersion("1.0.0");
        QApplication::setOrganizationName("Your Organization");
        
        // 初始化日志系统
        QString logDir = QApplication::applicationDirPath() + "/logs";
        Logger::getInstance().initialize(logDir);
        LOG_INFO("应用程序启动");
        
        // 检查配置文件
        QString configPath = QApplication::applicationDirPath() + "/config/mid360_config.json";
        if (!QFile::exists(configPath)) {
            LOG_ERROR("配置文件不存在: " + configPath);
            QMessageBox::critical(nullptr, "错误", "配置文件不存在: " + configPath);
            return 1;
        }
        
        // 创建主窗口
        MainWindow mainWindow;
        mainWindow.show();
        
        // 连接日志信号到状态栏
        QObject::connect(&Logger::getInstance(), &Logger::logMessage,
            [&mainWindow](int level, const QString& message) {
                if (level >= Logger::Warning) {
                    mainWindow.statusBar()->showMessage(message, 5000);
                }
            });
        
        LOG_INFO("主窗口已显示");
        
        // 运行应用程序
        int result = app.exec();
        
        LOG_INFO("应用程序退出，返回值: " + QString::number(result));
        return result;
        
    } catch (const std::exception& e) {
        LOG_FATAL(QString("未捕获的异常: %1").arg(e.what()));
        QMessageBox::critical(nullptr, "致命错误", 
            QString("发生未捕获的异常:\n%1").arg(e.what()));
        return 1;
    } catch (...) {
        LOG_FATAL("发生未知异常");
        QMessageBox::critical(nullptr, "致命错误", "发生未知异常");
        return 1;
    }
} 