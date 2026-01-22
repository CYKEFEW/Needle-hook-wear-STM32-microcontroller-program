# ADCForceSensor

基于 STM32F103RCTx 的双通道 ADC 力传感采集与电机控制示例工程。项目包含：
- ADC1 + DMA 采集 2 路模拟量
- USART1 串口命令交互与数据打印
- USART2 Modbus RTU 接收解析框架
- TIM1/TIM4 PWM 脉冲控制步进电机（含方向/使能）
- 简单 PID 控制闭环（基于采样数据调节转速）

> 备注：仓库中未找到 `AGENTS.md`，因此仅按项目现状撰写说明。

## 硬件与资源
- MCU：STM32F103RCTx（CubeMX 工程：`ADCForceSensor.ioc`）
- 系统时钟：HSE 外部晶振，PLL 72MHz
- ADC：ADC1 通道 0/1（PA0/PA1），DMA 采集 2 路
- 串口
  - USART1：115200 8N1（PA9/PA10）用于终端命令与 `printf`
  - USART2：38400 8N1（PA2/PA3）用于 Modbus RTU 接收
- 电机控制
  - TIM1_CH1（PA8, PUL）与 DIR/ENA（PA12/PA11）
  - TIM4_CH1（PB6, PUL2）与 DIR2/ENA2（PB4/PB5）
- 按键/指示灯
  - ZeroKey：PC9（外部中断下降沿）
  - sampleKey：PC8（外部中断下降沿）
  - LED：PC7
  - PID_LED：PC6

## 主要功能说明

### 1) ADC 采集与数据发送
文件：`Core/Src/Senddata.c`、`Core/Src/adc.c`
- 通过 DMA 采集 2 路 ADC 原始值。
- 电压换算：`V = adc / 4095 * 3.3`
- 输出格式：`{F/g:CH1,CH2}`
  - CH1 使用 `(V1 - zero1) * 872.5 / 2.52` 进行比例换算
  - CH2 使用 `V2 - zero2` 作为偏移量结果

### 2) 零点校准与 Flash 存储
文件：`Core/Src/Zero.c`、`Core/Src/flash.c`
- 触发方式：按下 ZeroKey 或串口命令 `Zero`
- 写入地址：`0x08060000`
- 写入内容：2 个 `float`（两路零点电压）

注意：`0x08060000` 需确保在芯片 Flash 有效范围内，若芯片 Flash 较小需调整地址。

### 3) 步进电机控制
文件：`Core/Src/control.c`
- `control()/silentcontrol()`：根据目标 RPM 修改 PWM 周期以改变脉冲频率
- `forward()/backward()`：方向控制
- `enable()/disable()`：使能控制
- `control2()/forward2()/backward2()/enable2()/disable2()`：第二路电机（TIM4/PB4/PB5）

### 4) PID 闭环控制
文件：`Core/Src/pid.c`
- 输入：`Data CH1 CH2` 命令提供两路测量值
- 目标：`F x` 设置目标力值
- 输出：将 PID 输出映射为电机 RPM（限制 0~400）
- 指示：PID_LED 翻转以指示控制活动

### 5) 串口命令（USART1）
文件：`Core/Src/usart.c`

| 命令 | 说明 |
| --- | --- |
| `ADC` | 立即发送一次 ADC 数据 |
| `Auto` / `Stop` | 开启/停止自动发送（100ms 周期） |
| `Zero` | 执行零点校准并写入 Flash |
| `Con x` | 设置电机转速（RPM） |
| `Forward` / `Backward` | 设置电机方向 |
| `Enable` / `Disable` | 使能/禁用电机 |
| `Con2 x` | 第二路电机转速（当前代码调用 `control()`） |
| `Forward2` / `Backward2` | 第二路电机方向 |
| `Enable2` / `Disable2` | 第二路电机使能 |
| `Data a b` | 输入两路传感数据，触发 PID 计算 |
| `F x` | 设置 PID 目标值 |

### 6) Modbus RTU（USART2）
文件：`Core/Src/usart.c`
- 解析帧格式（例）：
  - 请求：`[Addr][03][00][00][00][02][CRCLo][CRCHi]`
  - 响应：`[Addr][03][06][PY1Hi][PY1Lo][PY2Hi][PY2Lo][DP1][DP2][CRCLo][CRCHi]`
- 解析结果写入：`g_modbus2_data`

注意：当前项目暂不启用 Modbus 接收逻辑（为后续扩展预留）。若要启用解析，需要补齐 USART2 字节接收缓存逻辑并周期调用 `USART2_ModbusPoll()`。

## 运行逻辑概览
文件：`Core/Src/main.c`
- 上电初始化 ADC DMA、USART、定时器、PWM
- 读取 Flash 零点电压
- 当 `ADCAutoSend=1` 时每 100ms 发送一次 ADC 数据

## 目录结构（核心）
- `Core/Inc`：头文件与引脚定义
- `Core/Src`：业务逻辑、外设回调与控制算法
- `Drivers`：HAL/CMSIS 驱动
- `MDK-ARM`：Keil 工程文件
- `ADCForceSensor.ioc`：CubeMX 配置文件

## 构建与烧录
1) 使用 STM32CubeMX 打开 `ADCForceSensor.ioc` 检查引脚与时钟配置  
2) 使用 Keil MDK 或 STM32CubeIDE 编译下载  

如需调整串口波特率、引脚或计时参数，请优先在 `.ioc` 中修改并重新生成代码。

## 可能需要注意的点
- Flash 写入地址是否与实际芯片容量匹配（`0x08060000`）
- `Con2` 命令当前调用的是 `control()`（若需第二路请改为 `control2()`）
- Modbus 接收逻辑目前仅为框架，需补全接收与轮询调用
