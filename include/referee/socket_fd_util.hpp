#ifndef RADAR_INCLUDE_REFEREE_SOCKET_FD_UTIL_HPP
#define RADAR_INCLUDE_REFEREE_SOCKET_FD_UTIL_HPP

/**
 * @file  include/referee/socket_fd_util.hpp
 * @brief socket 文件描述符的通用小工具
 */

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>

#include <fcntl.h>

namespace radar::referee {

/**
 * @brief 将文件描述符设置为非阻塞模式
 * @param fd 目标文件描述符
 * @param name 出错时用于日志提示的名字
 */
inline void SetFdNonBlocking(int fd, const std::string &name) {
  const int flags = ::fcntl(fd, F_GETFL, 0);
  if (flags < 0) {
    throw std::runtime_error("fcntl(F_GETFL) failed for " + name + ": " + std::strerror(errno));
  }
  if (::fcntl(fd, F_SETFL, flags | O_NONBLOCK) != 0) {
    throw std::runtime_error("fcntl(F_SETFL) failed for " + name + ": " + std::strerror(errno));
  }
}

}  // namespace radar::referee

#endif  // RADAR_INCLUDE_REFEREE_SOCKET_FD_UTIL_HPP
