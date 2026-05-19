# Build And Run

用途：统一构建与运行入口，避免脚本/命令混淆。  
更新时间：2026-05-19  
适用场景：环境初始化、编译、运行、回归验证。

## 构建命令

```bash
cmake -S . -B build
cmake --build build -j
```

主程序二进制：

```text
build/bin/ImagePredict
```

## 运行命令

```bash
./run
./test
```

`run`：低干扰运行（默认关闭显示与部分调试项）  
`test`：调试运行（默认开启显示窗口）

## 常见环境依赖

- C++17 编译器
- OpenCV 4.x
- Eigen3
- `serial` 串口库
- 大恒相机 SDK (`gxiapi`)
- OpenVINO（可选，未找到时部分目标运行会报错）
