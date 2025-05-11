#include "log.h"
#include <filesystem>

// 避免C++17 filesystem不可用的错误
#ifdef _MSC_VER
#if _MSC_VER < 1914  // Visual Studio 2017 version 15.7
namespace fs = std::experimental::filesystem;
#else
namespace fs = std::filesystem;
#endif
#else
namespace fs = std::filesystem;
#endif

namespace common {

void InitG3Logging(bool log_to_file, const std::string &prefix,
                   const std::string &path) {
  // 初始化Google日志库
  google::InitGoogleLogging("oh_my_loam");
  
  // 设置日志输出级别
  FLAGS_minloglevel = google::GLOG_INFO;
  
  // 将日志输出到stderr，与g3log保持一致
  FLAGS_logtostderr = true;
  
  // 如果需要同时输出到文件
  if (log_to_file) {
    // 创建目录（如果不存在）
    fs::create_directories(path);
    
    std::string log_dir = path;
    if (!log_dir.empty() && log_dir.back() != '/' && log_dir.back() != '\\') {
      log_dir += '/';
    }
    
    FLAGS_logtostderr = false;
    FLAGS_alsologtostderr = true;
    FLAGS_log_dir = log_dir;
    
    if (!prefix.empty()) {
      std::string prefix_str = prefix;
      if (prefix_str.back() != '_') {
        prefix_str += "_";
      }
      FLAGS_log_prefix = true;
      google::SetLogDestination(google::GLOG_INFO, (log_dir + prefix_str + "info_").c_str());
      google::SetLogDestination(google::GLOG_WARNING, (log_dir + prefix_str + "warning_").c_str());
      google::SetLogDestination(google::GLOG_ERROR, (log_dir + prefix_str + "error_").c_str());
      google::SetLogDestination(google::GLOG_FATAL, (log_dir + prefix_str + "fatal_").c_str());
    }
  }
  
  // 打印彩色日志
  FLAGS_colorlogtostderr = true;
  
  LOG(INFO) << "Google日志系统初始化完成";
}

}  // namespace common
