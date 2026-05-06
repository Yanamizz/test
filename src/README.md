一、测试前网络连接方式

可以使用下面任意一种方式：

方式 1：设备 A 网口  <----网线---->  设备 B 网口

方式 2：设备 A 网口  <----网线---->  交换机  <----网线---->  设备 B 网口

现代网卡一般支持 Auto MDI-X，普通网线直连通常可以工作。若是很老的设备，直连失败时可能需要交叉网线，或者中间接交换机。

二、IP 地址规划

两台设备必须配置在同一个网段，例如：

设备	IP 地址	子网掩码	网关
设备 A	192.168.10.1	255.255.255.0	可不填
设备 B	192.168.10.2	255.255.255.0	可不填

端口使用：

TCP 端口：5000
三、Linux 下网络设置

先查看网卡名：

ip link

假设设备 A 的网卡名是 eth0，配置如下：

sudo ip addr flush dev eth0
sudo ip addr add 192.168.10.1/24 dev eth0
sudo ip link set eth0 up

设备 B 配置如下：

sudo ip addr flush dev eth0
sudo ip addr add 192.168.10.2/24 dev eth0
sudo ip link set eth0 up

如果你的网卡名不是 eth0，例如是 enp3s0、ens33，把命令里的 eth0 替换掉即可。

检查 IP：

ip addr show eth0

在设备 A 上测试能否 ping 通设备 B：

ping 192.168.10.2

在设备 B 上测试能否 ping 通设备 A：

ping 192.168.10.1

如果系统开启了防火墙，需要放行端口 5000。

Ubuntu / Debian 常见命令：

sudo ufw allow 5000/tcp

CentOS / Rocky / Fedora 常见命令：

sudo firewall-cmd --add-port=5000/tcp --permanent
sudo firewall-cmd --reload
四、Windows 下网络设置

进入：

控制面板
→ 网络和 Internet
→ 网络连接
→ 以太网
→ 属性
→ Internet 协议版本 4 TCP/IPv4

设备 A 设置：

IP 地址：192.168.10.1
子网掩码：255.255.255.0
默认网关：不填
DNS：不填

设备 B 设置：

IP 地址：192.168.10.2
子网掩码：255.255.255.0
默认网关：不填
DNS：不填

然后在设备 A 上测试：

ping 192.168.10.2

在设备 B 上测试：

ping 192.168.10.1

如果 Windows 防火墙拦截，需要放行 TCP 5000 端口，或者临时关闭防火墙进行测试。




九、运行测试步骤

先在设备 A 上运行服务端：

./device_a_server 5000

输出类似：

设备 A 服务端已启动，监听端口：5000
等待设备 B 连接...

然后在设备 B 上运行客户端：

./device_b_client 192.168.10.1 5000

连接成功后，设备 A 会显示：

设备 B 已连接，IP：192.168.10.2

此时两边都可以输入消息。

例如设备 B 输入：

hello from device B

设备 A 会收到：

[收到] hello from device B

设备 A 输入：

hello from device A

设备 B 会收到：

[收到] hello from device A

任意一端输入：

quit

即可退出。