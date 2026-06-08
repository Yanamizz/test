# 距离标定完整流程

本文档说明如何标定当前目标距离计算，以及如果要增加更多标定距离点，应该怎样采集数据、拟合和改代码。

## 当前距离链路

运行时距离链路如下：

1. `src/ImagePredict.cc` 选择用于距离计算的目标框。
2. `Tools::DistanceCalculator::CalculateDistanceWithDebug()` 接收目标框坐标。
3. `include/Tools/LaserAngleCalculate.hpp` 计算并输出：
   - `Wpx/Hpx`：目标框宽/高像素。
   - `Dw`：由宽度估计的距离。
   - `Dh`：由高度估计的距离。
   - `Src`：最终距离来源，可能是 `FUSED`、`WIDTH`、`HEIGHT`、`NONE`。
   - `Distance`：滤波后的 `used_distance`。
4. `LaserPc` 使用 `used_distance` 计算激光 pitch 补偿角。

`stage1/2` 和 `stage3` 现在使用同一条距离与激光补偿逻辑链路，只有参数不同。

调参集中区在 `include/Tools/LaserAngleCalculate.hpp` 文件末尾，当前按“功能 -> 阶段”分组：

```text
distance_calibration.stage12
distance_calibration.stage3
laser_pitch_compensation.stage12
laser_pitch_compensation.stage3
distance_filter
```

当前代码内置两点标定距离：

```text
kNearCalibrationHorizontalDistanceM = 10.0m
kFarCalibrationHorizontalDistanceM  = 24.0m
```

每个阶段保存四个实机像素锚点：

```text
near_height_pixel
far_height_pixel
near_width_pixel
far_width_pixel
```

代码会用这些像素锚点和相机焦距自动反推等效目标宽高：

```text
target_size_m = distance_m * pixel_size / focal_px
```

所以在当前两点标定方案下，你主要改的是 `*_pixel`，不是手算 `*_calibration_target_*`。

## 标定前固定条件

一次标定过程中尽量固定：

- 相机分辨率和 ROI 行为。
- 镜头焦距/对焦、光圈、曝光、增益。
- `stage1/2` 与 `stage3` 使用的模型文件。
- 目标类型和朝向。
- 目标姿态，尽量不要明显俯仰/偏航。
- 光照条件。

`stage1/2` 和 `stage3` 必须分开标定。`stage3` 可能使用不同模型和相机 ROI，它的 `Wpx/Hpx` 不能直接复用 `stage1/2` 的值。

## 实机采集流程

1. 编译并开启显示。

   ```bash
   cmake --build build -j --target ImagePredict
   ./build/bin/ImagePredict --enable-display --calibration-sliders
   ```

2. 把目标放到已知水平距离。

   距离应从相机光心附近到目标平面水平测量。若目标明显高于或低于相机，不要用斜边距离。

3. 等目标框稳定后记录 overlay 信息。

   重点看：

   ```text
   Wpx Hpx
   Dw Dh Src
   Distance
   LaserPc
   ```

4. 每个距离点采 100-300 帧稳定数据。

   程序退出时会打印像素统计：

   ```text
   [像素尺寸] 样本数=... 宽P25=... 宽中位=... 高P25=... 高中位=...
   ```

   标定优先使用“中位数”，不要用单帧值。

5. 分阶段重复采集。

   当前两点标定至少采：

   ```text
   10m, 24m
   ```

   建议额外验证：

   ```text
   12m, 16m, 20m
   ```

   如果准备做多点拟合，建议采：

   ```text
   8m, 10m, 12m, 16m, 20m, 24m, 28m
   ```

## 记录表模板

每个阶段单独一张表。

```text
stage: stage12 或 stage3

distance_m | width_median_px | height_median_px | width_p25_px | height_p25_px | note
-----------|-----------------|------------------|--------------|---------------|-----
10         |                 |                  |              |               |
12         |                 |                  |              |               |
16         |                 |                  |              |               |
20         |                 |                  |              |               |
24         |                 |                  |              |               |
```

推荐用 `median` 做标定。若目标框偶尔因拖影、眩光、误框膨胀而变大，可以参考 `P25` 判断保守值。

## 当前两点标定怎么改

如果只是更新现有两点标定，在 `include/Tools/LaserAngleCalculate.hpp` 的 `distance_calibration` 调参区修改像素锚点。

`stage12` 当前对应：

```cpp
111.982f, // stage12 near_height_pixel
47.952f,  // stage12 far_height_pixel
105.029f, // stage12 near_width_pixel
43.274f,  // stage12 far_width_pixel
```

`stage3` 当前对应：

```cpp
102.006f, // stage3 near_height_pixel
56.941f,  // stage3 far_height_pixel
85.880f,  // stage3 near_width_pixel
49.605f,  // stage3 far_width_pixel
```

替换规则：

```text
near_*_pixel = 10m 时的中位像素
far_*_pixel  = 24m 时的中位像素
```

默认等效目标宽高由 `CalibrationTargetMetersFromPixel()` 自动反推，不需要手动计算并填 `near_calibration_target_width` 或 `far_calibration_target_height`。

## 两点标定后的验证

1. 重新编译。

   ```bash
   cmake --build build -j --target ImagePredict
   ```

2. 在非锚点距离验证。

   推荐：

   ```text
   12m, 16m, 20m
   ```

3. 看 overlay。

   ```text
   Distance 应接近真实距离。
   Dw 和 Dh 不应严重分裂。
   Dw/Dh 一致时 Src 应显示 FUSED。
   目标静止时 LaserPc 应平滑，不应明显跳变。
   ```

如果 `Dw` 稳而 `Dh` 飘，宽度距离仍可用。  
如果 `Wpx` 本身跳，`Dw` 跟着跳，那优先查检测框抖动，不是先改距离公式。

## 如果要增加更多标定距离点

当前代码还没有保存任意多点标定表。现在是用 10m 和 24m 两个像素锚点，在中间插值等效目标尺寸。

如果你要加入更多距离点，建议这样扩展：

1. 在 `DistanceCalculator` 中新增标定样本结构。

   建议：

   ```cpp
   struct PixelDistanceCalibrationSample {
     float distance_m;
     float width_pixel;
     float height_pixel;
   };
   ```

2. 把 `DistanceCalibrationParams` 从两点字段扩展为每阶段样本表。

   固定点数版本：

   ```cpp
   struct DistanceCalibrationParams {
     std::array<PixelDistanceCalibrationSample, N> samples;
   };
   ```

   如果点数经常变化，可以用 `std::vector`，但固定数组更适合当前热路径，开销和行为更可控。

3. 用像素反比域插值或拟合。

   最低风险方案：

   ```text
   x = 1 / pixel
   在 x 上对 distance 做分段线性插值
   ```

   运行时：

   ```text
   - 按 pixel 或 1/pixel 排序样本。
   - 找到当前像素落在哪两个样本之间。
   - 线性插值距离。
   - 超出样本范围时 clamp 到最近端点或进入外推保护。
   ```

   这通常比继续插值“等效目标宽高”更直接，因为针孔几何近似满足：

   ```text
   distance ~= k / pixel
   ```

4. 宽高仍然分开估计。

   建议保留类似 helper：

   ```cpp
   EstimateDistanceFromWidthSamples(box_width_pixel, params)
   EstimateDistanceFromHeightSamples(box_height_pixel, params)
   ```

   然后继续复用现在的融合门控：

   ```text
   Dw/Dh 一致 -> FUSED
   否则宽度优先 -> WIDTH
   宽度无效再用高度 -> HEIGHT
   ```

5. 保持 overlay 不变。

   当前显示已经足够验证多点标定：

   ```text
   Wpx Hpx Dw Dh Src Distance LaserPc
   ```

## 推荐多点拟合模型

优先从简单模型开始：

```text
distance_m = a / pixel
```

如果残差明显随距离变化，再试：

```text
distance_m = a / (pixel + b) + c
```

每个阶段、每个维度独立拟合：

```text
stage12 width:  D = f12w(Wpx)
stage12 height: D = f12h(Hpx)
stage3 width:   D = f3w(Wpx)
stage3 height:  D = f3h(Hpx)
```

不要把 `stage12` 和 `stage3` 样本混在一起拟合。

建议验收标准：

```text
10-24m 内：目标框稳定时绝对误差 <= 0.5m
检查点：相邻距离的误差不要正负剧烈翻转
LaserPc：目标静止时没有明显阶跃跳变
```

## 激光 pitch 标定必须放在距离之后

只有距离稳定后，才调：

```text
laser_z_offset_m
laser_converge_x_m
laser_target_vertical_trim_m
```

不要一开始就调这些。距离没准时，调 LaserPc 会把距离误差掩盖掉，后面很容易变成 pitch 间歇震荡。

推荐顺序：

1. 先标定 `Wpx/Hpx -> Distance`。
2. 验证 `Distance`、`Dw`、`Dh`、`Src`。
3. 验证 `LaserPc` 是否平滑。
4. 再看激光落点是否整体偏高/偏低，并调激光物理参数。

## 常见问题判断

`Distance` 在目标静止时跳：

```text
优先看 Wpx/Hpx 是否跳。若像素跳，是检测框抖动。
```

`Dw` 和 `Dh` 差很多：

```text
可能是目标姿态变化、框不完整、阶段标定混用，或宽/高像素锚点不匹配当前模型。
```

`Src` 很少显示 `FUSED`：

```text
宽度和高度标定不一致。重新检查 10m/24m 的 Wpx/Hpx 中位数。
```

`Distance` 准但激光点整体偏上/偏下：

```text
这时才调 laser_pitch_compensation。
```

`stage12` 准但 `stage3` 不准：

```text
单独重标 stage3。不要复用 stage12 的像素锚点。
```
