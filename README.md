# Pid — 直流电机速度闭环

STM32F103C8T6 + TB6612 + JGB37-520 编码器电机。5 ms 控制周期、20 kHz PWM。
目标：阶跃响应 **< 50 ms**、静差 **< 5%**、扰动后能回目标。

**当前状态（2026-10-20）**：框架完成，**开环版已可烧**（串口给目标 → 前馈给占空比 → OLED 显示）。
编码器电机到货后把 `SPEED_LOOP_FEEDBACK` 0 → 1 就是闭环。

> 逐脚接线：`../docs/wiring-plan.md`　｜　配置/接口/踩坑全表：`../AGENTS.md`　｜　考核动作：`../docs/acceptance.md`

---

## 1. 现在怎么用

烧录后串口（115200）敲：

| 命令 | 作用 |
|---|---|
| `S200` | 目标 200 rpm（负号=反转，限幅 ±333）|
| `X` | 急停 |
| `A` | 开/关自动阶跃（打开后：2 s → 150 rpm 保持 3 s → 回 0，10 s 循环。**上电默认关**）|
| `V` | 开/关 **VOFA 波形流**（打开后串口是二进制帧，文本终端看起来就是乱码）|
| `?` | 回一行 `tgt/act/out/kp/ki/tick`，外加 `rx=字节数 err=串口错误数`（排障用）|

命令带不带换行都行：带 `\n`/`\r` 立即执行；**不带给也行 —— 停顿 500 ms 自动结算**
（`CMD_FLUSH_TIMEOUT_MS`，在 `App/cmd/cmd.h` 可改）。

⚠️ 串口工具的"行尾"设置有个坑：VS Code 串口监视器的 `vscode-serial-monitor.lineEnding`
枚举值是 **转义序列本身**（`"\n"` / `"\r"` / `"\r\n"`），写成描述文字 `"LF"` 会把
**字母 L 和 F 当成正文发出去**，于是 `S50` 变成 `S50LF` → 被解析器判为非法行 → 目标不动。

OLED 四行：目标 / 实际 / 输出‰ / 参数（有故障时显示 `FAULT xxx`）。
VOFA+ 用 JustFloat，三条线 = 目标、实际、输出。**波形流默认关闭**（敲 `V` 打开），
所以平时串口是纯文本；看到满屏乱码先确认是不是自己开了 `V`，再怀疑波特率。

**没有电机也能验**：PA8 接 `1 kΩ + LED → GND`，`S100`/`S300` 看亮度变化。

---

## 2. 引脚（唯一真源）

| 引脚 | 用途 | 关键参数 |
|---|---|---|
| PA8 | 电机 PWM | TIM1_CH1，PSC 0 / ARR 3599 → **20 kHz**，Label `MOTOR_PWM` |
| PA4 / PA5 | 方向 AIN1/AIN2 | 推挽输出，初始 Low，Label `AIN1/AIN2` |
| PA6 / PA7 | 编码器 A/B | TIM3 **TI1 and TI2**（4 倍频），Filter 6，Pull-up，Label `ENC_A/B` |
| PA9 / PA10 | USART1 TX/RX | 115200 8N1，**要勾 USART1 全局中断**（抢占 2）|
| PB8 / PB9 | OLED SCL/SDA | 软件 I²C，开漏 + 初始 High，Label `OLED_SCL/SDA` |
| PC13 | 板载 LED | **低电平点亮**，Label `LED` |
| PA13 / PA14 | SWD | SYS → Debug = **Serial Wire**（选 Disable 会锁芯片）|
| PD0 / PD1 | 8 MHz 晶振 | RCC → HSE = Crystal |

---

## 3. 供电（三条就够）

```
3S 电池 ──(开关)── TB6612 VM          ← 只有电机功率；11.1 V 不上面包板
开发板 3.3V ──┬── TB6612 VCC          ← 逻辑
              ├── 编码器 蓝线（霍尔）
              └── OLED
```

- `VM` 只能 2.5~13.5 V（满电 12.6 V 可以，**4S 绝对不行**）；`VCC` 2.7~5.5 V。
- ⛔ **`VM` 绝不能从 ST-Link 3V3 / 开发板 3.3V / 板子 USB 取电**：电机启动要 1~3 A，
  而 ST-Link 只能给约 100 mA → 电压瞬间塌陷 → USB 口过流保护 → **整个 USB hub 断电、串口消失**
  （实测日志：`usb 3-1-port3: attempt power cycle` + `ch341-uart ... disconnected`）。
  `VM` 只接电池（经开关），`VCC` 才接 3.3 V。
- **板子 USB 和 ST-Link 3V3 二选一**（3.3 V 排针直连 MCU，两个源会倒灌）。
- 面包板：功率地（电池−/TB6612 GND）与逻辑地分段，**只在一点相连**；830 孔板两条电源轨中间通常是断的。

编码器 6 根线：红/白=电机（→`AO1/AO2`），蓝=**3.3 V**（接 11.1 V 烧霍尔），黑=GND，黄/绿=A/B（→PA6/PA7）。

---

## 4. 目录

```
Pid/
├── Core/          CubeMX 生成（只改 USER CODE 区）
├── Components/    drv 装的外设驱动：motor-tb6612 / oled-ssd1306
├── App/           ★ 自己写的应用层（drv 不碰）
│   ├── pid/       PID 纯数学（无 HAL）
│   ├── cmd/       串口命令 + uart-link（中断收/阻塞发）
│   ├── speed-loop/5 ms 调度：前馈 + PID + 限幅 + 保护 + 虚拟被控对象
│   ├── vofa/      JustFloat 组帧
│   └── ui/        OLED 四行
└── tests/         host 测试（pid / cmd / vofa）
```

**规则**：新增 `.c` 登记根 `CMakeLists.txt`（`cmake/stm32cubemx/` 下的会被 CubeMX 重写）；
`pid.c`/`vofa.c`/`cmd.c` 不许 include HAL，保证能在 PC 上测。

---

## 5. 编译 / 烧录

```bash
bash ~/stm32/scripts/build-flash.sh ~/stm32/PID/Pid            # 编译 + 烧录 + 运行
FLASH=0 bash ~/stm32/scripts/build-flash.sh ~/stm32/PID/Pid    # 只编译
```

VS Code：`Ctrl+Alt+F` 编译并烧录 · **`F5` 调试**（会占 ST-Link，停在 `main`）。
判据：出现 `** Verified OK **`；`st-info --probe` 有 `chipid 0x410`。

host 测试（不用硬件）：

```bash
cd ~/stm32/PID/Pid
gcc -std=c11 -Wall -Wextra -I App/pid  tests/test_pid.c  App/pid/pid.c  -lm -o /tmp/t && /tmp/t   # 20/20
gcc -std=c11 -Wall -Wextra -I App/cmd  tests/test_cmd.c  App/cmd/cmd.c      -o /tmp/t && /tmp/t   # 21/21
gcc -std=c11 -Wall -Wextra -I App/vofa tests/test_vofa.c App/vofa/vofa.c    -o /tmp/t && /tmp/t   # 6/6
```

---

## 6. 要调的东西只有一处

`App/speed-loop/speed-loop.h` 顶部：

```c
#define SPEED_LOOP_FEEDBACK   0     /* 0 = 开环；编码器到了改 1 */
#define SPEED_LOOP_SIM_PLANT  0     /* 1 = 虚拟被控对象（无硬件调 PID）*/
#define SPEED_LOOP_KV       2.70f   /* 前馈 ‰/rpm，实测后修 */
#define SPEED_LOOP_KP        3.0f
#define SPEED_LOOP_KI       30.0f
```

改完 `Ctrl+Alt+F` 重烧，一轮 ≈ 10 秒。调参记录写 `../notes/tuning-log.md`。

---

## 7. 几个关键数字

| 量 | 值 | 由来 |
|---|---|---|
| 系统时钟 | 72 MHz | HSE 8 M × 9 |
| PWM | 20 kHz | 72 M / (3599+1) |
| 控制周期 | 5 ms | 72 M / 7200 / 50 |
| 编码器 | 1320 计数/圈 | 11 PPR × 4 × 减速比 30 |
| 转速换算 | rpm = Δcounts × 9.09 | 60 / (0.005 × 1320) |
| 占空比 | ‰（-1000~1000）| `MOTOR_SetDuty(540)` = 54% |

---

## 8. 别踩这五条

1. **CubeMX 勾 `USART1 global interrupt`（抢占 2）** —— 不勾，串口一收数据整机卡死（弱符号死循环）。
2. `GENERATE CODE` 之前改动不落盘（标题栏有 `*`）；AF 脚的 Pull/速度在 TIM 的 `GPIO Settings` 里，不在 `gpio.c`。
3. F1 的 GPIO 只有 `Low`/`Medium`/`High`，没有 `Very High`。
4. TIM2 `Counter Period` 默认 65535（=6.5 s），必须是 49；TIM3 必须 `Encoder Mode TI1 and TI2`（选 `TI1` 就少一半）。
5. 烧录报端口占用：先 `pgrep -a -x openocd` 确认是残留再杀（无差别杀会让 ST-Link 卡死，得拔插 USB）。
