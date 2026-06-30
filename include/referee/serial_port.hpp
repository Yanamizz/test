#ifndef RADAR_INCLUDE_REFEREE_SERIAL_PORT_HPP
#define RADAR_INCLUDE_REFEREE_SERIAL_PORT_HPP

/**
 * @file  include/referee/serial_port.hpp
 * @brief 裁判系统串口读写与自动重连辅助封装
 */

#include <cerrno>
#include <cstddef>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include "include/config/config.hpp"
#include "librm/core/typedefs.hpp"

namespace radar::referee {

/// 串口发送等待可写的超时时间。
constexpr int kSerialWritePollTimeoutMs = 100;

/**
 * @brief 将整数波特率转换为 termios 常量
 * @param baud 整数波特率
 * @return 对应的 termios 速率枚举
 */
inline speed_t ToTermiosBaud(int baud) {
  switch (baud) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    default:
      throw std::runtime_error("unsupported baud rate: " + std::to_string(baud));
  }
}

/**
 * @brief 选择当前可用的裁判系统串口设备
 * @return 若默认设备不存在则尝试回退设备名
 */
inline std::string SelectRefereeDevice() {
  if (::access(config::kDefaultRefereeDevice, F_OK) == 0) {
    return config::kDefaultRefereeDevice;
  }
  if (::access(config::kFallbackRefereeDevice, F_OK) == 0) {
    return config::kFallbackRefereeDevice;
  }
  return config::kDefaultRefereeDevice;
}

/**
 * @brief 非阻塞串口封装
 * @note  同时提供抛异常接口和自动重连场景使用的 `Try*` 接口。
 */
class SerialPort {
 public:
  SerialPort() = default;

  SerialPort(std::string device, int baud) { Open(std::move(device), baud); }

  SerialPort(const SerialPort &) = delete;
  SerialPort &operator=(const SerialPort &) = delete;

  SerialPort(SerialPort &&other) noexcept { MoveFrom(std::move(other)); }

  SerialPort &operator=(SerialPort &&other) noexcept {
    if (this != &other) {
      Close();
      MoveFrom(std::move(other));
    }
    return *this;
  }

  ~SerialPort() { Close(); }

  /**
   * @brief 使用默认设备名创建串口实例
   * @param baud 目标波特率
   * @return 已打开的串口对象
   */
  static SerialPort OpenDefault(int baud = config::kDefaultRefereeBaud) {
    return SerialPort(SelectRefereeDevice(), baud);
  }

  /**
   * @brief 使用默认设备名尝试打开串口
   * @param serial 待写入的串口对象
   * @param baud 目标波特率
   * @param error 失败时返回错误信息
   * @return 是否成功打开
   */
  static bool TryOpenDefault(SerialPort *serial, int baud = config::kDefaultRefereeBaud,
                             std::string *error = nullptr) {
    if (serial == nullptr) {
      if (error != nullptr) {
        *error = "serial pointer is null";
      }
      return false;
    }
    return serial->TryOpen(SelectRefereeDevice(), baud, error);
  }

  /**
   * @brief 打开指定串口设备
   * @param device 串口设备节点
   * @param baud 目标波特率
   */
  void Open(std::string device, int baud) {
    Close();

    const int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      throw std::runtime_error("failed to open serial device " + device + ": " + std::strerror(errno));
    }

    termios tio{};
    if (tcgetattr(fd, &tio) != 0) {
      const auto message = std::string("tcgetattr failed for ") + device + ": " + std::strerror(errno);
      ::close(fd);
      throw std::runtime_error(message);
    }

    cfmakeraw(&tio);
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 1;

    const speed_t speed = ToTermiosBaud(baud);
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);

    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
      const auto message = std::string("tcsetattr failed for ") + device + ": " + std::strerror(errno);
      ::close(fd);
      throw std::runtime_error(message);
    }

    tcflush(fd, TCIOFLUSH);
    fd_ = fd;
    device_ = std::move(device);
    baud_ = baud;
  }

  /**
   * @brief 尝试打开指定串口设备
   * @param device 串口设备节点
   * @param baud 目标波特率
   * @param error 失败时返回错误信息
   * @return 是否成功打开
   */
  bool TryOpen(std::string device, int baud, std::string *error = nullptr) {
    const std::string requested_device = device;
    try {
      Open(std::move(device), baud);
      return true;
    } catch (const std::exception &exception) {
      device_ = requested_device;
      baud_ = baud;
      if (error != nullptr) {
        *error = exception.what();
      }
      return false;
    }
  }

  /**
   * @brief 以默认设备名尝试打开串口
   * @param baud 目标波特率
   * @param error 失败时返回错误信息
   * @return 是否成功打开
   */
  bool TryOpenDefault(int baud = config::kDefaultRefereeBaud, std::string *error = nullptr) {
    return TryOpen(SelectRefereeDevice(), baud, error);
  }

  /**
   * @brief 从串口读取数据，失败时抛异常
   * @param data 读缓冲区
   * @param size 缓冲区大小
   * @return 实际读取字节数
   */
  std::size_t Read(rm::u8 *data, std::size_t size) const {
    std::size_t bytes_read = 0;
    std::string error;
    if (!TryRead(data, size, &bytes_read, &error)) {
      throw std::runtime_error(error);
    }
    return bytes_read;
  }

  /**
   * @brief 从串口尝试读取数据
   * @param data 读缓冲区
   * @param size 缓冲区大小
   * @param bytes_read 实际读取字节数
   * @param error 失败时返回错误信息
   * @return 是否成功完成一次非阻塞读取
   */
  bool TryRead(rm::u8 *data, std::size_t size, std::size_t *bytes_read, std::string *error = nullptr) const {
    if (bytes_read != nullptr) {
      *bytes_read = 0;
    }
    if (!is_open()) {
      if (error != nullptr) {
        *error = "serial read attempted while port is closed";
      }
      return false;
    }
    while (true) {
      const ssize_t result = ::read(fd_, data, size);
      if (result >= 0) {
        if (bytes_read != nullptr) {
          *bytes_read = static_cast<std::size_t>(result);
        }
        return true;
      }
      if (errno == EINTR) {
        continue;
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return true;
      }
      if (error != nullptr) {
        *error = "serial read failed on " + device_ + ": " + std::strerror(errno);
      }
      return false;
    }
  }

  /**
   * @brief 向串口完整写出一帧数据，失败时抛异常
   * @param data 待发送数据
   * @param size 发送字节数
   */
  void WriteAll(const rm::u8 *data, std::size_t size) const {
    std::string error;
    if (!TryWriteAll(data, size, &error)) {
      throw std::runtime_error(error);
    }
  }

  /**
   * @brief 向串口尝试完整写出一帧数据
   * @param data 待发送数据
   * @param size 发送字节数
   * @param error 失败时返回错误信息
   * @return 是否成功写完
   */
  bool TryWriteAll(const rm::u8 *data, std::size_t size, std::string *error = nullptr) const {
    if (!is_open()) {
      if (error != nullptr) {
        *error = "serial write attempted while port is closed";
      }
      return false;
    }
    std::size_t written = 0;
    while (written < size) {
      const ssize_t result = ::write(fd_, data + written, size - written);
      if (result < 0) {
        if (errno == EINTR) {
          continue;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          std::string wait_error;
          if (!WaitWritable(&wait_error)) {
            if (error != nullptr) {
              *error = wait_error;
            }
            return false;
          }
          continue;
        }
        if (error != nullptr) {
          *error = "serial write failed on " + device_ + ": " + std::strerror(errno);
        }
        return false;
      }
      if (result == 0) {
        if (error != nullptr) {
          *error = "serial write returned 0 on " + device_;
        }
        return false;
      }
      written += static_cast<std::size_t>(result);
    }
    return true;
  }

  const std::string &device() const { return device_; }
  int baud() const { return baud_; }
  int fd() const { return fd_; }
  bool is_open() const { return fd_ >= 0; }

 private:
  /**
   * @brief 在串口暂不可写时等待其恢复可写
   * @param error 失败时返回错误信息
   * @return 是否等待成功
   */
  bool WaitWritable(std::string *error = nullptr) const {
    pollfd fd{fd_, POLLOUT, 0};
    while (true) {
      const int result = ::poll(&fd, 1, kSerialWritePollTimeoutMs);
      if (result > 0) {
        if ((fd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
          if (error != nullptr) {
            *error = "serial write poll failed on " + device_;
          }
          return false;
        }
        if ((fd.revents & POLLOUT) != 0) {
          return true;
        }
        continue;
      }
      if (result == 0) {
        if (error != nullptr) {
          *error = "serial write timeout on " + device_;
        }
        return false;
      }
      if (errno == EINTR) {
        continue;
      }
      if (error != nullptr) {
        *error = "serial write poll failed on " + device_ + ": " + std::strerror(errno);
      }
      return false;
    }
  }

 public:
  /**
   * @brief 关闭当前串口
   */
  void Close() {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

 private:
  void MoveFrom(SerialPort &&other) {
    fd_ = other.fd_;
    device_ = std::move(other.device_);
    baud_ = other.baud_;
    other.fd_ = -1;
  }

  int fd_ = -1;             ///< 当前串口文件描述符
  std::string device_;      ///< 当前串口设备名
  int baud_ = 0;            ///< 当前串口波特率
};

}  // namespace radar::referee

#endif  // RADAR_INCLUDE_REFEREE_SERIAL_PORT_HPP
