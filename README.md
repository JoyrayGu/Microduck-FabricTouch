# Microduck-FabricTouch

织物触觉 → 手势识别 → 舵机动作的完整链路。把电子触觉皮肤（64×64 压力阵列）贴合在
机器鸭头部，用户抚摸时由 Windows 端识别手势，经 TCP 发给树莓派上的 brain，
由它驱动舵机做出对应动作。

```
触觉皮肤 ──USB(9600)──> Windows ──TCP:9000──> 树莓派 brain ──UART(1Mbps)──> 舵机
                         │                      │
                    手势识别                动作调度
                  tap / stroke        head_roll / open_mouth
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

| 手势 | 判定依据 | 动作 | 驱动舵机 |
| --- | --- | --- | --- |
| `tap` | 轨迹 < 3.0 格，按住 80–1200 ms | `head_roll` 左右歪头（1.4 s） | 仅 ID14 横滚 |
| `single_stroke` | 轨迹 ≥ 3.0 格，直线度 > 0.7 | `open_mouth` 张嘴再闭合（0.65 s） | 仅 ID15 嘴部 |
| `back_forth_stroke` | 轨迹 ≥ 3.0 格，直线度 < 0.7 | `open_mouth` 同一个张嘴动作 | 仅 ID15 嘴部 |

直线度 = 净位移 / 累计路程。实测单程 0.986–0.998、来回 0.085–0.094，分离度很大。

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

- 64×64 触觉阵列读取，40 Hz，零丢帧
- 三种手势识别（单指单点场景）
- 手势 → brain → 舵机完整链路，`event_id` 可全链路对账

**未解决**

| 问题 | 影响 |
| --- | --- |
| 颈部（ID11/12/13）零力矩 | 用户摸头时头部会被手推动 |
| 多点接触只取第一个接触点 | 多指或手指+鱼际同时接触时质心失真 |
| 部分滑移 / 部分静止无法区分 | 一指滑动、另一指静止时全局质心被稀释 |

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
