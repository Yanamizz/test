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

如果修改IP需要在头文件中修改默认值

1. 收方：把收到的数据写入变量

```cpp
#include "NetworkTask/DeviceAServer.hpp"

NetworkTask::socket_t listen_fd = NetworkTask::kInvalidSocketFd;
if (!NetworkTask::CreateListeningSocket(listen_fd)) {
  // 处理错误
}

NetworkTask::socket_t client_fd = NetworkTask::kInvalidSocketFd;
std::string client_ip;
if (!NetworkTask::AcceptClient(listen_fd, client_fd, &client_ip)) {
  // 处理错误
}

std::string received_content;
if (NetworkTask::ReceiveText(client_fd, received_content)) {
  // received_content 就是收方收到的数据
}
```

连续循环接收时，可以一直复用同一个变量：

```cpp
std::string received_content;
while (running) {
  if (!NetworkTask::ReceiveText(client_fd, received_content)) {
    break;
  }

  // received_content 每次都会被更新
}
```

2. 发方：把内容传入发送接口

```cpp
#include "NetworkTask/DeviceBClient.hpp"

NetworkTask::socket_t fd = NetworkTask::kInvalidSocketFd;
if (!NetworkTask::ConnectToServer(fd)) {
  // 处理错误
}

std::string input_content = "hello from device B";
NetworkTask::SendText(fd, input_content);
```

如果要持续发送，只需要不断更新 input_content 再调用 SendText 即可。

三、测试程序的运行方式

如果只是直接跑测试程序，也可以继续使用现成可执行文件：

收方：

```bash
./device_a_server
```

发方：

```bash
./device_b_client
```

四、测试流程建议

1. 先完成硬件连接和 IP 配置。
2. 先启动收方 device_a_server。
3. 再启动发方 device_b_client。
4. 收方用 ReceiveText 把数据传出到变量，发方用 SendText 把内容传入发送接口。
5. 任意一端输入 quit，双方退出。
