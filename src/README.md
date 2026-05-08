一、测试前需要提前做的硬件设置

IP 地址规划

两台设备必须配置在同一个网段，例如：

| 设备   |     IP 地址   | 子网掩码       |  网关  | 
| 设备 A | 192.168.10.1 | 255.255.255.0 | 可不填 |  （发）
| 设备 B | 192.168.10.2 | 255.255.255.0 | 可不填 |  （收）
TCP 端口使用 5000。

然后在设备 A 上测试：
```bash
ping 192.168.10.2
```

在设备 B 上测试：
```bash
ping 192.168.10.1
```


二、头文件调用方式

如果修改默认 IP，可以在头文件中修改默认值，也可以调用 `ConnectToServer` 时直接传入目标 IP。

1. B 收方：监听端口并接收 `0x00 / 0x01`

B 收方使用 `DeviceAServer.hpp` 创建监听 socket。`ReceiveText` 是阻塞接收接口；如果主程序还要继续执行其他任务，建议接收逻辑放到独立线程中，并在调用 `AcceptClient` / `ReceiveText` 前用 `WaitForReadable` 做短超时等待。

```cpp
#include "NetworkTask/DeviceAServer.hpp"

NetworkTask::socket_t listen_fd = NetworkTask::kInvalidSocketFd;
if (!NetworkTask::CreateListeningSocket(listen_fd, 5000)) {
  // 处理错误
}

NetworkTask::socket_t client_fd = NetworkTask::kInvalidSocketFd;
std::string client_ip;
while (running) {
  if (!NetworkTask::WaitForReadable(listen_fd, 100)) {
    continue;
  }
  if (NetworkTask::AcceptClient(listen_fd, client_fd, &client_ip)) {
    break;
  }
}

uint8_t aimbot_target = 0x00;
while (running) {
  if (!NetworkTask::WaitForReadable(client_fd, 100)) {
    continue;
  }

  std::string received_content;
  if (!NetworkTask::ReceiveText(client_fd, received_content)) {
    break;  // 对端关闭连接或接收失败
  }

  for (unsigned char data : received_content) {
    if (data == 0x00 || data == 0x01) {
      aimbot_target = data;
    }
  }
}

NetworkTask::CloseSocket(client_fd);
NetworkTask::CloseSocket(listen_fd);
```

如果发送方发的是文本形式，也可以按 `"0"`、`"1"`、`"0x00"`、`"0x01"` 解析；如果发的是原始字节，直接判断 `0x00 / 0x01` 即可。


2. 启动顺序不确定时：发方循环重连

不需要改成双向互连。保持 B 收方一直监听，A 发方连接失败后等待一小段时间再重试即可。这样 B 收方先启动或 A 发方先启动都可以正常建立连接。

```cpp
#include <chrono>
#include <thread>

#include "NetworkTask/DeviceBClient.hpp"

NetworkTask::socket_t fd = NetworkTask::kInvalidSocketFd;
while (running) {
  if (NetworkTask::ConnectToServer(fd, "192.168.10.2", 5000)) {
    break;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

if (fd == NetworkTask::kInvalidSocketFd) {
  // 处理未连接情况
}

uint8_t target = 0x01;  // 0x00 关激光，0x01 开目标
std::string payload(1, static_cast<char>(target)); //构造要发送的数据内容
if (!NetworkTask::SendText(fd, payload)) {
  // 处理发送失败
}//发送

NetworkTask::ShutdownSocket(fd);
NetworkTask::CloseSocket(fd);
```

如果要持续发送，只需要在判断条件变化时更新 `target`，再调用 `SendText`。如果使用 `ConnectToServer(fd)` 的默认参数，默认连接 IP 为 `192.168.10.2:5000`。



三、测试程序的运行方式

当前测试程序文件名沿用原名称，但实际角色如下：

A 发方：

```bash
./device_a_server
```

B 收方：

```bash
./device_b_client
```

如果 B 收方 IP 不是默认的 `192.168.10.2`，A 发方可以在命令行传入：

```bash
./device_a_server 192.168.10.2
```

四、测试流程建议

1. 先完成硬件连接和 IP 配置。
2. 启动 B 收方 `device_b_client`，监听 TCP 端口 `5000`。
3. 启动 A 发方 `device_a_server`，连接 B 收方。
4. 在 A 发方输入 `0` 发送 `0x00`，输入 `1` 发送 `0x01`。
5. A 发方输入 `quit` 退出；B 收方按 `Ctrl+C` 退出。
