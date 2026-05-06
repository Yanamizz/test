#include <atomic>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")
using socket_t = SOCKET;
const socket_t INVALID_SOCKET_FD = INVALID_SOCKET;
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/socket.h>
#include <unistd.h>
using socket_t = int;
const socket_t INVALID_SOCKET_FD = -1;
#endif

static void close_socket(socket_t fd) {
#ifdef _WIN32
  closesocket(fd);
#else
  close(fd);
#endif
}

static void shutdown_socket(socket_t fd) {
#ifdef _WIN32
  shutdown(fd, SD_BOTH);
#else
  shutdown(fd, SHUT_RDWR);
#endif
}

static bool send_all(socket_t fd, const std::string &data) {
  size_t total = 0;

  while (total < data.size()) {
#ifdef _WIN32
    int n =
        send(fd, data.data() + total, static_cast<int>(data.size() - total), 0);
#else
    ssize_t n = send(fd, data.data() + total, data.size() - total, 0);
#endif
    if (n <= 0) {
      return false;
    }
    total += static_cast<size_t>(n);
  }

  return true;
}

static void recv_loop(socket_t fd, std::atomic<bool> &running) {
  char buffer[1024];

  while (running) {
#ifdef _WIN32
    int n = recv(fd, buffer, sizeof(buffer) - 1, 0);
#else
    ssize_t n = recv(fd, buffer, sizeof(buffer) - 1, 0);
#endif
    if (n > 0) {
      buffer[n] = '\0';
      std::cout << "\n[收到] " << buffer << std::flush;
    } else if (n == 0) {
      std::cout << "\n[提示] 对端已关闭连接\n";
      running = false;
      break;
    } else {
      std::cout << "\n[错误] 接收失败\n";
      running = false;
      break;
    }
  }
}

int main(int argc, char *argv[]) {
#ifdef _WIN32
  WSADATA wsaData;
  if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
    std::cerr << "WSAStartup 失败\n";
    return 1;
  }
#else
  signal(SIGPIPE, SIG_IGN);
#endif

  std::string server_ip = "192.168.10.1";
  int port = 5000;

  if (argc >= 2) {
    server_ip = argv[1];
  }

  if (argc >= 3) {
    port = std::atoi(argv[2]);
  }

  socket_t fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd == INVALID_SOCKET_FD) {
    std::cerr << "创建 socket 失败\n";
#ifdef _WIN32
    WSACleanup();
#endif
    return 1;
  }

  sockaddr_in server_addr;
  std::memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(static_cast<uint16_t>(port));

  if (inet_pton(AF_INET, server_ip.c_str(), &server_addr.sin_addr) <= 0) {
    std::cerr << "服务端 IP 地址格式错误：" << server_ip << std::endl;
    close_socket(fd);
#ifdef _WIN32
    WSACleanup();
#endif
    return 1;
  }

  std::cout << "正在连接设备 A：" << server_ip << ":" << port << " ..."
            << std::endl;

  if (connect(fd, reinterpret_cast<sockaddr *>(&server_addr),
              sizeof(server_addr)) < 0) {
    std::cerr << "连接失败，请检查 IP、网线、防火墙和服务端程序是否已启动\n";
    close_socket(fd);
#ifdef _WIN32
    WSACleanup();
#endif
    return 1;
  }

  std::cout << "已连接设备 A\n";
  std::cout << "输入消息后回车发送；输入 quit 退出。\n";

  std::atomic<bool> running(true);
  std::thread recv_thread(recv_loop, fd, std::ref(running));

  std::string line;
  while (running && std::getline(std::cin, line)) {
    if (line == "quit") {
      running = false;
      break;
    }

    line += "\n";
    if (!send_all(fd, line)) {
      std::cout << "[错误] 发送失败\n";
      running = false;
      break;
    }
  }

  running = false;
  shutdown_socket(fd);
  close_socket(fd);

  if (recv_thread.joinable()) {
    recv_thread.join();
  }

#ifdef _WIN32
  WSACleanup();
#endif

  std::cout << "设备 B 程序退出\n";
  return 0;
}