# Microduck-FabricTouch

织物触觉 → 语义识别 → 舵机动作的完整链路。当前标准织物皮肤输出 32×32 压力阵列，
贴合在机器鸭头部时由 Windows 端识别触觉语义，经 TCP 发给树莓派上的 brain，
由它驱动舵机做出对应动作。

```
触觉皮肤 ──USB(9600)──> Windows ──TCP:9000──> 树莓派 brain ──UART(1Mbps)──> 舵机
                         │                      │
                static_touch / dynamic_touch   head_roll / open_mouth
```

## 目录结构

| 目录 | 内容 | 运行位置 |
| --- | --- | --- |
| `skin/` | 触觉采集、手势识别、TCP 客户端 | Windows |
| `brain/` | TCP 服务、动作库、动作调度、舵机驱动 | 树莓派 |
| `tools/` | 标定与验证工具（`servo_goto` / `servo_watch` / `play_action`） | Windows（经 SSH 驱动树莓派） |
| `diagnostics/` | 触觉帧解析、串口诊断、实测结论 | Windows |
| `docs/` | 项目背景与交接说明 | — |

## 快速开始

完整步骤见 **[skin/docs/tactile-to-action.md](skin/docs/tactile-to-action.md)**。

最短路径（两个终端）：

```powershell
# 终端 1：启动 brain（树莓派，带硬件驱动）
ssh duck "cd /home/pi4/microduck_brain && : > /home/pi4/brain-hw.log && nohup /home/pi4/microduck_venv/bin/python -u -m microduck.src.command_server --host 0.0.0.0 --port 9000 --enable-hardware --port-name /dev/ttyACM0 --baud 1000000 >> /home/pi4/brain-hw.log 2>&1 & sleep 3; cat /home/pi4/brain-hw.log"

# 终端 2：运行触觉识别（Windows）
cd skin
$env:PYTHONIOENCODING="utf-8"
E:\MiniConda\python.exe run.py --robot --brain-host 192.168.1.57 --brain-port 9000 `
  --port COM7 --baud 9600 --threshold 20 --duration 90
```

成功标志：brain 侧打印 `runtime=dynamixel`，Windows 侧触摸后出现 `[手势]` 与 `[brain] ACCEPTED`。

## 手势 → 动作

| 语义 | 判定依据 | 动作 | 驱动舵机 |
| --- | --- | --- | --- |
| `static_touch` | 接触轨迹不足以构成有效滑移，且接触时长在 80–1200 ms | `head_roll` 左右歪头（约 1.4 s） | 仅 ID14 横滚 |
| `dynamic_touch` | 接触形成足够长的有效滑移轨迹；包含单程、往返和部分滑移 | `open_mouth` 张嘴再闭合（约 0.65 s） | 仅 ID15 嘴部 |

当前机器人接口只输出这两种语义。多点事件还会附带接触模式、移动接触数和静止接触数；
这些字段用于诊断，当前 Brain 动作映射仍按 `static_touch` / `dynamic_touch` 处理。

## 实测标定

| 关节 | 中位 | 一端 | 另一端 |
| --- | --- | --- | --- |
| ID14 头部横滚 | 1881 | 左 1712 | 右 2050 |
| ID13 头部偏航（备用） | **1854** | 左 1685 | 右 2023 |
| ID15 嘴部 | — | 张开 1760 | 闭合 2174 |

注意 ID13 的中位**不是** 2048（实测偏差 −194 tick）。旧实现按 2048 镜像推算右端，
会多走 337 tick（29.6°）撞到线缆限位。

## 已验证 / 未解决

**已验证**

- 32×32 触觉阵列读取，当前设备约 40 Hz
- `static_touch` / `dynamic_touch` 两种机器人触觉语义
- 单点、多点以及部分滑移状态字段可通过事件日志观察
- 触觉语义 → brain → 舵机完整链路，`event_id` 可全链路对账

**未解决**

| 问题 | 影响 |
| --- | --- |
| 颈部（ID11/12/13）零力矩 | 用户摸头时头部会被手推动 |
| 新曲面皮肤尚未适配 | 需要重新确认阵列尺寸、通道顺序、有效区域和压力基线 |
| 曲面安装后的识别阈值尚未重标定 | 旧皮肤参数可能不适用于新皮肤的灵敏度与预载 |

## 依赖

Laptop 端依赖见 [skin/requirements.txt](skin/requirements.txt)，用于 USB/BLE 触觉采集和可选鼠标后端。
Pi 端真机依赖见 [brain/requirements.txt](brain/requirements.txt)，应安装到
`/home/pi4/microduck_venv`；Brain 的 dry-run 测试不需要 `dynamixel-sdk`。

## 测试

```powershell
# skin
cd skin; E:\MiniConda\python.exe -m unittest discover -s tests

# brain
cd brain; E:\MiniConda\python.exe -m unittest discover -s tests

# tools
cd tools; $env:PYTHONPATH="src"; E:\MiniConda\python.exe -m unittest discover -s tests

# diagnostics
cd diagnostics; $env:PYTHONPATH="src"; E:\MiniConda\python.exe -m unittest discover -s tests
```

## 说明

本项目为内部开发版本，不含开源许可证。硬件标定数据（舵机 tick 值）来自当前这台
机器鸭的实测，换机后需重新标定。
