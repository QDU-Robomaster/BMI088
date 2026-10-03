# BMI088

博世 BMI088 6 轴 IMU 的 SPI 驱动模块：陀螺仪与加速度计采样、Topic 发布、加热恒温与陀螺仪零偏校准 / SPI driver Module for the Bosch BMI088 6-axis IMU with gyroscope and accelerometer sampling, Topic publishing, heater temperature control and gyroscope offset calibration

## 1. 模块作用 / Purpose

构造时，BMI088 注册陀螺仪 INT 引脚的下降沿中断和 RamFS 命令 `bmi088`，随后对加速度计和陀螺仪软复位、校验芯片 ID（加速度计 `0x1E`，陀螺仪 `0x0F`），按 `Param` 写入量程与输出频率，并把陀螺仪 data-ready 映射到 INT3。初始化失败时打印错误并每 100 ms 重试，成功后构造继续。

随后创建线程 `bmi088_thread`（优先级 `REALTIME`，栈深 `param.task_stack_depth`）。线程以 30 kHz 使能加热 PWM，之后等待陀螺仪中断，等待超时 50 ms 时打印 `BMI088 wait timeout.`。每次中断后依次读取陀螺仪和加速度计（含芯片温度），将原始值换算为物理量并乘以 `rotation`（传感器坐标系到应用坐标系的旋转），再依次发布加速度和角速度。陀螺仪单位为 rad/s，先减去零偏再旋转；加速度计单位为 g。原始读数全为 0 时丢弃该次数据，保留上一次的值。

加热由周期 50 ms 的 LibXR 定时器任务完成：以 `target_temperature`（°C）为目标、加速度计芯片温度为反馈，用 `pid_param` 计算 PWM 占空比。默认 PID 中仅 `k` 为 1，`p`、`i`、`d` 均为 0，占空比输出为 0。

陀螺仪零偏保存在 Database 键 `bmi088_gyro_data` 中。`OnMonitor()` 在加速度或角速度出现 NaN / Inf 时打印告警，并在陀螺仪中断间隔与 `gyro_freq` 对应的理想周期相差超过 0.3 ms 时打印 `BMI088 Frequency Error`。

Upon construction, BMI088 registers a falling-edge interrupt on the gyroscope INT pin and the RamFS command `bmi088`, then soft-resets the accelerometer and the gyroscope, checks the chip IDs (`0x1E` for the accelerometer, `0x0F` for the gyroscope), writes the range and output frequency from `Param`, and maps the gyroscope data-ready signal to INT3. When the initialization fails, an error is printed and the initialization is retried every 100 ms; the construction continues once it succeeds.

The Module then creates the thread `bmi088_thread` (priority `REALTIME`, stack depth `param.task_stack_depth`). The thread enables the heater PWM at 30 kHz and waits for the gyroscope interrupt, printing `BMI088 wait timeout.` after a 50 ms timeout. After each interrupt it reads the gyroscope and the accelerometer (including the chip temperature), converts the raw values to physical units, multiplies them by `rotation` (the rotation from the sensor frame to the application frame) and publishes the acceleration and the angular velocity in turn. The gyroscope unit is rad/s, with the offset subtracted before the rotation; the accelerometer unit is g. A reading whose raw values are all 0 is discarded and the previous value is kept.

Heating is performed by a LibXR timer task with a 50 ms period: with `target_temperature` (°C) as the target and the accelerometer chip temperature as the feedback, `pid_param` computes the PWM duty cycle. In the default PID only `k` is 1 while `p`, `i` and `d` are 0, so the duty cycle output is 0.

The gyroscope offset is stored in the Database key `bmi088_gyro_data`. `OnMonitor()` prints a warning when the acceleration or the angular velocity contains NaN / Inf, and prints `BMI088 Frequency Error` when the gyroscope interrupt interval differs from the ideal period of `gyro_freq` by more than 0.3 ms.

## 2. 时间戳约定 / Timestamp Convention

`gyro_topic_name` 与 `accl_topic_name` 两个 Topic 使用同一次陀螺仪 data-ready 中断采集到的时间戳（µs）发布。payload 中不包含采样时间，采样时间由 Topic 的 envelope timestamp 给出。

The Topics `gyro_topic_name` and `accl_topic_name` are published with the timestamp (µs) captured at the same gyroscope data-ready interrupt. The payload carries no sampling time; the sampling time is given by the Topic envelope timestamp.

## 3. RamFS 命令 / RamFS Command

模块在 RamFS 中注册命令 `bmi088`：

- `bmi088`：打印用法。
- `bmi088 show <time_ms> <interval_ms>`：在 `time_ms` 内每 `interval_ms`（限制在 2 - 1000 ms）打印一次加速度、角速度和温度。
- `bmi088 list_offset`：打印当前陀螺仪零偏。
- `bmi088 cali`：陀螺仪零偏校准，在设备静止时进行。零偏清零后等待 3 s，采集 120 s 并求平均零偏；随后再采集 60 s，打印平均值与零偏的差；最后把零偏写入 Database。

The Module registers the command `bmi088` in RamFS:

- `bmi088`: prints the usage.
- `bmi088 show <time_ms> <interval_ms>`: prints the acceleration, the angular velocity and the temperature every `interval_ms` (limited to 2 - 1000 ms) for `time_ms`.
- `bmi088 list_offset`: prints the current gyroscope offset.
- `bmi088 cali`: gyroscope offset calibration, performed while the device is stationary. After the offset is cleared it waits 3 s and collects data for 120 s to compute the mean offset; it then collects for another 60 s and prints the difference between the mean and the offset; finally it writes the offset to the Database.

## 4. 构造接口 / Constructor

```cpp
BMI088(LibXR::GPIO& accl_cs,
       LibXR::GPIO& gyro_cs,
       LibXR::GPIO& gyro_int,
       LibXR::SPI& spi,
       LibXR::PWM& heater_pwm,
       LibXR::Database& database,
       LibXR::RamFS& ramfs,
       const Param& param = {...});  // 节选 / excerpt
```

依赖：

- `accl_cs`：加速度计片选 GPIO，低电平选中。
- `gyro_cs`：陀螺仪片选 GPIO，低电平选中。
- `gyro_int`：陀螺仪 INT3 数据就绪中断 GPIO，配置为下降沿中断。
- `spi`：连接 BMI088 的 `LibXR::SPI`，片选由上述两个 GPIO 控制。
- `heater_pwm`：加热电阻的 `LibXR::PWM`。
- `database`：保存陀螺仪零偏的 `LibXR::Database`。
- `ramfs`：注册 `bmi088` 命令的 `LibXR::RamFS`。

配置参数（`Param`，括号内为默认值）：

- `gyro_freq`：陀螺仪输出频率与带宽（`GYRO_2000HZ_BW532HZ`），可选 `GYRO_2000HZ_BW532HZ`、`GYRO_2000HZ_BW230HZ`、`GYRO_1000HZ_BW116HZ`、`GYRO_400HZ_BW46HZ`、`GYRO_200HZ_BW23HZ`、`GYRO_100HZ_BW12HZ`、`GYRO_200HZ_BW64HZ`、`GYRO_100HZ_BW32HZ`。
- `accl_freq`：加速度计输出频率（`ACCL_1600HZ`），可选 `ACCL_1600HZ`、`ACCL_800HZ`、`ACCL_400HZ`、`ACCL_200HZ`、`ACCL_100HZ`、`ACCL_50HZ`、`ACCL_25HZ`、`ACCL_12_5HZ`。
- `gyro_range`：陀螺仪量程（`DEG_2000DPS`），可选 `DEG_2000DPS`、`DEG_1000DPS`、`DEG_500DPS`、`DEG_250DPS`、`DEG_125DPS`。
- `accl_range`：加速度计量程（`ACCL_24G`），可选 `ACCL_3G`、`ACCL_6G`、`ACCL_12G`、`ACCL_24G`。
- `rotation`：传感器坐标系到应用坐标系的单位四元数，顺序为 w、x、y、z（`{1.0f, 0.0f, 0.0f, 0.0f}`）。
- `pid_param`：温控 PID，类型 `LibXR::PID<float>::Param`，字段为 `k, p, i, d, i_limit, out_limit, cycle`（`k = 1.0`，其余为 0，`cycle = false`），输出作为 PWM 占空比。
- `gyro_topic_name`：角速度 Topic 名称（`"bmi088_gyro"`）。
- `accl_topic_name`：加速度 Topic 名称（`"bmi088_accl"`）。
- `target_temperature`：目标温度，单位 °C（45）。
- `task_stack_depth`：线程栈深（2048）。

Dependencies:

- `accl_cs`: chip-select GPIO of the accelerometer, selected when low.
- `gyro_cs`: chip-select GPIO of the gyroscope, selected when low.
- `gyro_int`: GPIO of the gyroscope INT3 data-ready interrupt, configured as a falling-edge interrupt.
- `spi`: the `LibXR::SPI` connected to the BMI088; the chip selects are driven through the two GPIOs above.
- `heater_pwm`: the `LibXR::PWM` of the heating resistor.
- `database`: the `LibXR::Database` that stores the gyroscope offset.
- `ramfs`: the `LibXR::RamFS` in which the `bmi088` command is registered.

Configuration parameters (`Param`, defaults in parentheses):

- `gyro_freq`: gyroscope output frequency and bandwidth (`GYRO_2000HZ_BW532HZ`), one of `GYRO_2000HZ_BW532HZ`, `GYRO_2000HZ_BW230HZ`, `GYRO_1000HZ_BW116HZ`, `GYRO_400HZ_BW46HZ`, `GYRO_200HZ_BW23HZ`, `GYRO_100HZ_BW12HZ`, `GYRO_200HZ_BW64HZ`, `GYRO_100HZ_BW32HZ`.
- `accl_freq`: accelerometer output frequency (`ACCL_1600HZ`), one of `ACCL_1600HZ`, `ACCL_800HZ`, `ACCL_400HZ`, `ACCL_200HZ`, `ACCL_100HZ`, `ACCL_50HZ`, `ACCL_25HZ`, `ACCL_12_5HZ`.
- `gyro_range`: gyroscope range (`DEG_2000DPS`), one of `DEG_2000DPS`, `DEG_1000DPS`, `DEG_500DPS`, `DEG_250DPS`, `DEG_125DPS`.
- `accl_range`: accelerometer range (`ACCL_24G`), one of `ACCL_3G`, `ACCL_6G`, `ACCL_12G`, `ACCL_24G`.
- `rotation`: unit quaternion from the sensor frame to the application frame in the order w, x, y, z (`{1.0f, 0.0f, 0.0f, 0.0f}`).
- `pid_param`: temperature-control PID of type `LibXR::PID<float>::Param` with the fields `k, p, i, d, i_limit, out_limit, cycle` (`k = 1.0`, the others 0, `cycle = false`); the output is used as the PWM duty cycle.
- `gyro_topic_name`: name of the angular-velocity Topic (`"bmi088_gyro"`).
- `accl_topic_name`: name of the acceleration Topic (`"bmi088_accl"`).
- `target_temperature`: target temperature in °C (45).
- `task_stack_depth`: thread stack depth (2048).

## 5. Topic

| Topic | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `param.gyro_topic_name`（默认 `bmi088_gyro`） | 发布 | `Eigen::Matrix<float, 3, 1>` | 角速度，单位 rad/s，已减去零偏并旋转 |
| `param.accl_topic_name`（默认 `bmi088_accl`） | 发布 | `Eigen::Matrix<float, 3, 1>` | 加速度，单位 g，已旋转 |

| Topic | Direction | Type | Meaning |
| --- | --- | --- | --- |
| `param.gyro_topic_name` (default `bmi088_gyro`) | Publish | `Eigen::Matrix<float, 3, 1>` | Angular velocity in rad/s, offset subtracted and rotated |
| `param.accl_topic_name` (default `bmi088_accl`) | Publish | `Eigen::Matrix<float, 3, 1>` | Acceleration in g, rotated |

## 6. 配置示例 / Configuration Example

`xrobot instance add QDU-Robomaster/BMI088` 写入的实例，依赖填写为 BSP 硬件注册（`XR_REGISTER`）中的名称，`param` 按安装方向和加热电阻调整：

An instance written by `xrobot instance add QDU-Robomaster/BMI088`, with the dependencies set to names from the BSP's Registration (`XR_REGISTER`), and `param` adjusted to the mounting orientation and the heating resistor:

```yaml
modules:
  - module: QDU-Robomaster/BMI088
    id: bmi088_0
    args:
      - accl_cs: ACCL_CS
      - gyro_cs: GYRO_CS
      - gyro_int: GYRO_INT
      - spi: spi1
      - heater_pwm: pwm_tim10_ch1
      - database: database
      - ramfs: ramfs
      - param:
          gyro_freq: BMI088::GyroFreq::GYRO_1000HZ_BW116HZ
          accl_freq: BMI088::AcclFreq::ACCL_800HZ
          gyro_range: BMI088::GyroRange::DEG_2000DPS
          accl_range: BMI088::AcclRange::ACCL_24G
          rotation: '{0.707f, 0.0f, 0.0f, 0.707f}'
          pid_param:
            k: 0.15f
            p: 1.0f
            i: 0.1f
            d: 0.0f
            i_limit: 0.3f
            out_limit: 1.0f
            cycle: false
          gyro_topic_name: "bmi088_gyro"
          accl_topic_name: "bmi088_accl"
          target_temperature: 45
          task_stack_depth: 1536
```

## 7. 依赖与硬件 / Dependencies and Hardware

依赖：LibXR。

硬件：通过 SPI 连接的 BMI088，加速度计与陀螺仪各有一个片选 GPIO，陀螺仪 INT3 连接到一个可配置为下降沿中断的 GPIO，加热电阻由一路 PWM 驱动；SPI、GPIO、PWM、Database 与 RamFS 均在 BSP 中通过 `XR_REGISTER` 注册。

Dependencies: LibXR.

Hardware: a BMI088 connected over SPI, with one chip-select GPIO each for the accelerometer and the gyroscope, the gyroscope INT3 wired to a GPIO that can be configured as a falling-edge interrupt, and the heating resistor driven by one PWM channel; the SPI, GPIO, PWM, Database and RamFS objects are registered in the BSP with `XR_REGISTER`.
