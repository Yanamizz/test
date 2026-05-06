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

  int port = 5000;
  if (argc >= 2) {
    port = std::atoi(argv[1]);
  }

  socket_t listen_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (listen_fd == INVALID_SOCKET_FD) {
    std::cerr << "创建 socket 失败\n";
#ifdef _WIN32
    WSACleanup();
#endif
    return 1;
  }

  int opt = 1;
#ifdef _WIN32
  setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR,
             reinterpret_cast<const char *>(&opt), sizeof(opt));
#else
  setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#endif

  sockaddr_in server_addr;
  std::memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = INADDR_ANY;
  server_addr.sin_port = htons(static_cast<uint16_t>(port));

  if (bind(listen_fd, reinterpret_cast<sockaddr *>(&server_addr),
           sizeof(server_addr)) < 0) {
    std::cerr << "bind 失败，端口可能被占用或权限不足\n";
    close_socket(listen_fd);
#ifdef _WIN32
    WSACleanup();
#endif
    return 1;
  }

  if (listen(listen_fd, 1) < 0) {
    std::cerr << "listen 失败\n";
    close_socket(listen_fd);
#ifdef _WIN32
    WSACleanup();
#endif
    return 1;
  }

  std::cout << "设备 A 服务端已启动，监听端口：" << port << std::endl;
  std::cout << "等待设备 B 连接..." << std::endl;

  sockaddr_in client_addr;
  std::memset(&client_addr, 0, sizeof(client_addr));

#ifdef _WIN32
  int client_len = sizeof(client_addr);
#else
  socklen_t client_len = sizeof(client_addr);
#endif

  socket_t client_fd = accept(
      listen_fd, reinterpret_cast<sockaddr *>(&client_addr), &client_len);

  if (client_fd == INVALID_SOCKET_FD) {
    std::cerr << "accept 失败\n";
    close_socket(listen_fd);
#ifdef _WIN32
    WSACleanup();
#endif
    return 1;
  }

  char client_ip[64] = {0};
  inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, sizeof(client_ip));

  std::cout << "设备 B 已连接，IP：" << client_ip << std::endl;
  std::cout << "输入消息后回车发送；输入 quit 退出。\n";

  std::atomic<bool> running(true);
  std::thread recv_thread(recv_loop, client_fd, std::ref(running));

  std::string line;
  while (running && std::getline(std::cin, line)) {
    if (line == "quit") {
      running = false;
      break;
    }

    line += "\n";
    if (!send_all(client_fd, line)) {
      std::cout << "[错误] 发送失败\n";
      running = false;
      break;
    }
  }

  running = false;
  shutdown_socket(client_fd);
  close_socket(client_fd);
  close_socket(listen_fd);

  if (recv_thread.joinable()) {
    recv_thread.join();
  }

#ifdef _WIN32
  WSACleanup();
#endif

  std::cout << "设备 A 程序退出\n";
  return 0;
}