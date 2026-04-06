# radar_track-2026
雷达站目标识别追踪

## ------------主要功能实现及目标------------

1.目标识别模型优化

2.滤波函数优化

3.实际模型建模

4.多线程间通信及优化(kalman滤波、模型推理)


## -------------环境配置---------------

**ubuntu 24.04LTS**

**opencv 4.x:**<br>
<https://docs.opencv.ac.cn/4.12.0/d7/d9f/tutorial_linux_install.html>

**onnxruntime 1.17.3  ------>   3080(CUDA11  cuDNN8.6.0)**<br>
> git clone --recursive https://github.com/Microsoft/onnxruntime.git
> cd onnxruntime
>./build.sh --config RelWithDebInfo --build_shared_lib --parallel --compile_no_warning_as_error --skip_submodule_sync

* 可能出现的问题：<br>
hash值校验无法对应:修改本地hash校验值<br>

**使用到的第三方库:**<br>
kalman filter:<https://github.com/mherb/kalman.git>
Galaxy SDK:<https://www.daheng-imaging.com/downloads/>
Serial:<https://github.com/wjwwood/serial.git>
OpenVINO:<https://github.com/openvinotoolkit/openvino.git>
<https://github.com/openvinotoolkit/openvino/blob/2026.0.0/docs/dev/build_linux.md>







