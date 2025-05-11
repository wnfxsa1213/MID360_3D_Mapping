#pragma once

#include <chrono>
#include <filesystem>
#include <fstream>
// 确保glog正确包含
#define GLOG_NO_ABBREVIATED_SEVERITIES  // 避免与Windows.h中的宏冲突
#include <glog/logging.h>
#include <iostream>

// 定义日志宏，使原g3log宏适配glog
#define ADEBUG VLOG(1)
#define AINFO LOG(INFO)
#define AWARN LOG(WARNING)
#define AERROR LOG(ERROR)
#define AUSER LOG(INFO)
#define AFATAL LOG(FATAL)

// LOG_IF
#define ADEBUG_IF(cond) VLOG_IF(1, cond)
#define AINFO_IF(cond) LOG_IF(INFO, cond)
#define AWARN_IF(cond) LOG_IF(WARNING, cond)
#define AERROR_IF(cond) LOG_IF(ERROR, cond)
#define AUSER_IF(cond) LOG_IF(INFO, cond)
#define AFATAL_IF(cond) LOG_IF(FATAL, cond)
#define ACHECK(cond) CHECK(cond)

// LOG_EVERY_N
#define ADEBUG_EVERY(n) VLOG_EVERY_N(1, n)
#define AINFO_EVERY(n) LOG_EVERY_N(INFO, n)
#define AWARN_EVERY(n) LOG_EVERY_N(WARNING, n)
#define AERROR_EVERY(n) LOG_EVERY_N(ERROR, n)
#define AUSER_EVERY(n) LOG_EVERY_N(INFO, n)

namespace common {
// 初始化日志
void InitG3Logging(bool log_to_file = false, const std::string &prefix = "",
                   const std::string &path = "./");
}  // namespace common