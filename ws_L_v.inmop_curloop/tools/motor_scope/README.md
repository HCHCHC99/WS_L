# MotorScope —— J-Link RTT 电机实时动画（FOC，分支 inmop_cur_loop_rtt）

目标固件（`ws/foc.c`）在 FOC ISR（20kHz）内以 **1kHz** 向 RTT 通道 0 发送
MOTF 帧，本工具用 pylink 读取、解析后，在浏览器 Canvas 里**根据真实数据**
实时绘制电机动画：

- 转子：**20 块磁钢（极对数 10）** 按编码器实测电角度 `g_foc_if_rotor_rad` 转动
- **控制角 θ**（I-F 合成角 / RUN 控制角 `g_foc_theta_rad`）白色虚线指针
- **id**（d 轴，青色）/ **iq**（q 轴，橙色）/ **is = id + j·iq**（黄色）矢量
- **电压矢量 v**（vd/vq，品红虚线）
- I-F 状态机：hold → ramp/sync → run（handover），同步灯 + diff 波形
- 转速表、iq/id 波形、转子角 vs 控制角波形、角度偏差波形

![FOC 仿真运行界面](screenshots/sim_foc_run.png)

## 使用

```powershell
# 仿真（无需硬件，演示 I-F 启动 -> 同步 -> 运行全流程）
python motor_scope.py --mode sim-foc

# J-Link 实机
pip install pylink-square
python motor_scope.py --mode jlink --device HC32F460 --speed-khz 4000
```

启动后自动打开 `http://127.0.0.1:8080/`。

## 数据协议（固件端 `ws/foc.c` 的 Foc_RttIsrSend）

文本帧（`FOC_RTT_RATE_HZ <= 2000`，默认 1000）：

```
MOTF,<mode>,<phase>,<rotor_mrad>,<theta_mrad>,<iq_ma>,<id_ma>,
     <vq_mv>,<vd_mv>,<spd_rpm>,<sync>,<diff_mrad>,<freq_cHz>,<ms>,<rotor_mech_mrad>
```

| 字段 | 来源 | 说明 |
|------|------|------|
| mode | g_foc_mode | 0=停止 1=开环 2=电流环 3=对齐 |
| phase | g_foc_phase | 0=idle 1=hold 2=ramp/sync 3=run 4=align |
| rotor_mrad | g_foc_if_rotor_rad×1000 | 编码器实测转子电角度（折返 [0,2π)） |
| theta_mrad | g_foc_theta_rad×1000 | 控制角（I-F 合成角/RUN 控制角） |
| iq_ma / id_ma | g_foc_iq_ma / g_foc_id_ma | dq 电流反馈 |
| vq_mv / vd_mv | g_foc_vq / g_foc_vd ×1000 | dq 电压 PI 输出 |
| spd_rpm | g_enc_speed_rpm | 转速 |
| sync | g_foc_if_sync | 1 = 已同步（handover） |
| diff_mrad | g_foc_if_diff_rad×1000 | 控制角-转子角偏差 |
| freq_cHz | g_foc_if_freq_hz×100 | I-F 电频率 |
| rotor_mech_mrad | g_enc_count 连续累计×2π/CPR×1000 | **连续机械角**（跨圈不折返，动画用，Z 不复位） |

`FOC_RTT_RATE_HZ > 2000` 时固件自动切换为 **48 字节小端二进制帧**
（magic "MOTF"，字段同上），本工具自动识别两种格式。

## 固件端改动（本分支已包含）

1. `ws/foc.c`：新增 `Foc_RttIsrSend()`，在 `Foc_Isr` 里每 `FOC_ISR_HZ/FOC_RTT_RATE_HZ`
   个周期调用一次；`SEGGER_RTT_Write` 非阻塞，缓冲满丢帧不影响控制环。
2. `RTT/SEGGER_RTT_Conf.h`：`SEGGER_RTT_MAX_NUM_UP_BUFFERS` 3 → 7（启用更多通道；MOTF 帧走通道 0）。
3. 配置宏在 `foc.c` 顶部（`FOC_RTT_ENABLE / FOC_RTT_CH / FOC_RTT_RATE_HZ`，`FOC_RTT_CH` 默认 0，与 `MAIN_D/E` 日志共用通道 0；若用 `MAIN_E()` 打印 MOTF 行，上位机解析器同样兼容），
   可用编译器 `-D` 覆盖；如需统一收口可移入 `motor_config.h`。

## 注意

- 采样率 vs 电频率：fe = rpm/60 × 10。1kHz 帧率在约 3000rpm 以下平滑；
  更高转速建议 `FOC_RTT_RATE_HZ` 提到 2000~4000（自动切二进制帧）。
- I-F 启动阶段 `g_foc_theta_rad` 是合成角，动画会显示"控制角指针"与"转子
  磁钢"分离直到同步——这正是调试 I-F 启动要看的现象。
- `g_foc_if_rotor_rad` 依赖编码器方向/零位（FOC_ENC_DIR / 对齐校准）。




