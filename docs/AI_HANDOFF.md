# 机器鸭触觉联动当前任务交接

最后更新：2026-10-15

## 当前任务

当前不实施 ROS 2，也不搭建 3D 仿真。当前任务是在 Raspberry Pi 真机上安全验证同事交接包，确认以下链路能够工作：

```text
触觉织物 -> Windows 串口 -> TCP 9000 -> Raspberry Pi -> OpenRB-150 -> 机器鸭动作
```

验证完成后，再以实测硬件参数为依据规划 RK3566 + Ubuntu 22.04 + ROS 2 的迁移。

## 已完成工作

- 已静态检查同事交接包中的 Windows 触觉桥、控制 GUI、Pi 动作服务、诊断脚本和 systemd 配置。
- 已确认触觉织物串口波特率在当前硬件上实测为 9600。
- 已确认运行时链路使用直接 TCP，不通过 SSH 传输触觉命令。
- 已确认交接包本身不配置 Wi-Fi 或 SSH。
- 已确认服务端动作和命令已实现，但本次会话没有在当前机器鸭上执行动作。
- 已区分当前 Pi 真机、无真机阶段搭建的 brain 原型以及未来 RK3566 目标。
- 已创建项目上下文和本交接文档。

本次工作没有修改任何控制脚本、姿态数据或服务配置。

## 最近一次验证结果

### 已确认

- 触觉织物：9600 baud，已由用户在当前硬件上验证。
- Pi 服务源码：TCP `0.0.0.0:9000`、Dynamixel Protocol 2.0、57600 baud。
- Windows 程序：默认直接连接 `172.20.10.2:9000`。
- systemd：安装脚本会执行 `enable --now`，模板设置 `Restart=always`。
- 自动动作：模板默认 `AUTO_TOUCH_INTERVAL_SECONDS=5`。
- 工作区：`microduck_brain` 和 `microduck_skin` 有既存未提交内容；本次没有改动这些内容。

### 尚未验证

- Pi 是否自动接入公司 Wi-Fi。
- Pi 当前 IP 是否为 `172.20.10.2`。
- Windows 到 Pi 的 TCP 9000 是否可达。
- 当前 OpenRB 串口设备名。
- 当前机器鸭是否确实使用 57600 baud 和源码中的舵机 ID 表。
- 每个舵机的实际关节含义、方向、机械零位和安全范围。
- `TILT`、`MOUTH`、`TOUCH` 及所有腿部动作的实物表现。

## 上机前准备

### 人员和机械安全

1. 两人配合：一人执行命令，一人观察机器人并负责立即断电。
2. 将机器鸭放在稳定支架上，腿部悬空或由支架承重。
3. 清空关节运动范围内的线缆、工具和手指。
4. 确认电源开关或拔电位置能在一秒内触及。
5. 首轮验证不要安装可能碰撞的外壳或负载。
6. 记录机器人上电前姿态，并拍摄各关节朝向，便于判断动作方向是否相反。

### 软件安全

首次启动前，将服务环境中的自动动作关闭：

```ini
Environment=AUTO_TOUCH_INTERVAL_SECONDS=0
```

然后重新加载 systemd，但先不要立即启动服务：

```bash
sudo systemctl daemon-reload
sudo systemctl disable --now pi-duck-control.service
```

检查是否仍有旧服务或脚本占用串口：

```bash
systemctl status pi-duck-control.service --no-pager
ps -ef | grep -E 'pi_duck_control_server|diagnose_dynamixel' | grep -v grep
ls -l /dev/serial/by-id/ /dev/ttyACM* 2>/dev/null
```

通过标准：服务停止，没有控制脚本在运行，OpenRB 串口路径可识别。

停止条件：如果服务无法停止、仍有未知进程持续启动或串口设备反复消失，先排查该问题，不进行舵机测试。

## 分阶段验证步骤

每个阶段都要记录：时间、命令、服务日志、观察到的动作、异常声音、供电情况和结论。只有前一阶段通过后才进入下一阶段。

### 阶段 1：确认 Pi 网络与 SSH

此阶段不连接或不启用舵机扭矩。

1. 在 Pi 本地检查网络：

   ```bash
   hostname
   hostname -I
   ip -brief address
   ip route
   ```

2. 检查已激活连接。若系统使用 NetworkManager：

   ```bash
   nmcli connection show --active
   nmcli device status
   ```

3. 在 Windows PowerShell 检查 Pi 地址和 SSH：

   ```powershell
   ping <PI_IP>
   Test-NetConnection <PI_IP> -Port 22
   ```

通过标准：Pi 和 Windows 位于可互访网络，已记录实际 Pi IP，SSH 端口可按项目配置访问。

说明：同事所说的“自动连接同一 Wi-Fi”目前只是项目说明。若 Pi 没有连接，不要从代码中寻找自动联网逻辑，应检查 Pi 系统镜像中的网络配置；不得把 Wi-Fi 密码写入本仓库。

### 阶段 2：确认触觉串口，不连接动作服务

1. 在 Windows 设备管理器确认触觉织物 COM 口。
2. 确认串口参数为 9600 baud。
3. 只运行一个串口程序，观察是否连续收到完整帧。
4. 分别进行无触碰、轻触、按住和释放，观察活跃单元数或 GUI 图像变化。
5. 关闭厂商 `SerialDataCollection.exe` 后再运行 Python 触觉桥，避免 COM 口冲突。

通过标准：无触碰时数据稳定，触碰位置和强度变化合理，释放后恢复，程序没有持续帧错位或断连。

本阶段不要让触觉桥连接 Pi；必要时使用不可达测试地址或只观察本地解析日志。

### 阶段 3：只读诊断 OpenRB 和舵机

保持动作服务停止，执行同事提供的 `diagnose_dynamixel.py`。先阅读脚本，确认它只做 ping 和状态读取，不启用扭矩或写入模式、位置。

```bash
cd <PI_SCRIPT_DIRECTORY>
python3 diagnose_dynamixel.py
```

应记录：

- 实际串口路径。
- 成功响应的舵机 ID。
- 使用 57600 baud 是否能通信。
- ID 5 和 9 的 operating mode、torque enable、当前位置、电压、温度和硬件错误。
- 预期 ID 中缺失的设备和意外出现的 ID。

通过标准：总线稳定、响应 ID 可重复、无硬件错误、供电电压和温度合理。

停止条件：大量超时、硬件错误、电压异常、温度异常、异味、发热或舵机在只读诊断时运动。此时不要启动动作服务。

如果 57600 下无响应，不要立即运行旧 brain 的 1 Mbps 控制脚本。应先确认 OpenRB 固件、接线和同事实际配置，再用只读扫描方案确认总线参数。

### 阶段 4：不接舵机验证 TCP 服务协议

优先在没有舵机供电、使用测试替身或经确认不会写总线的条件下验证网络协议。启动服务后，在 Windows PowerShell 执行：

```powershell
$client = [Net.Sockets.TcpClient]::new('<PI_IP>', 9000)
$stream = $client.GetStream()
$writer = [IO.StreamWriter]::new($stream)
$reader = [IO.StreamReader]::new($stream)
$writer.AutoFlush = $true
$writer.WriteLine('PING')
$reader.ReadLine()
$client.Dispose()
```

同时在 Pi 查看日志：

```bash
journalctl -u pi-duck-control.service -f
```

通过标准：端口可达，`PING` 返回预期响应，日志中没有周期 `TOUCH`，没有重复启动或异常退出。

注意：当前服务的 `open()` 会准备舵机，不能在舵机已供电时把“只发 PING”视为绝对无动作测试。必须先根据现场接线决定是否断开舵机总线或提供软件 dry-run。

### 阶段 5：验证头部和嘴部单关节动作

这一阶段首次允许舵机动作。机器人必须上支架，观察者手持断电控制。

建议顺序：

1. `TILT`
2. `MOUTH`
3. 再次执行 `TILT`，检查重复性
4. 再次执行 `MOUTH`，检查重复性

每个命令只执行一次，等待完整返回和机械停止后再继续。建议使用 `duck_action_control_gui.py`，因为它会等待 `OK` 和 `DONE`；不要用只读取初始回复的触觉桥判断动作完成。

逐项检查：

- 动作关节是否与预期一致。
- 方向是否正确。
- 是否触碰机械限位。
- 是否能回到初始位置。
- 是否出现抖动、堵转、碰撞、异响、过热或明显压降。
- 服务日志是否出现通信错误。

停止条件：任何非目标关节运动、动作方向明显错误、突然全身使能、冲击限位、持续堵转或无法回位。立即断电并记录最后命令。

### 阶段 6：验证站立和腿部组合动作

只有头部和嘴部验证通过、舵机 ID 与关节映射已经记录后才进行。

推荐顺序：

1. `STAND`
2. `KNEEL`
3. `SQUAT`
4. `STEP`
5. `LEAN_BACK`

要求：

- 首次始终由支架承重。
- 每个动作之间断开自动触发并检查温度。
- 对比动作前后姿态，不连续点击按钮。
- 确认动作函数返回 `DONE` 后，仍需以实物稳定停止为准。
- 每个动作单独标记为“通过”“需调参”或“禁止使用”。

通过标准：关节映射正确，无碰撞或限位，动作完成后机器人稳定，重复执行结果一致。

在支架验证完成之前，不进行自由站立或地面迈步测试。

### 阶段 7：验证组合 `TOUCH`

确认 `TILT` 和 `MOUTH` 分别通过后，手动发送一次 `TOUCH`。

通过标准：动作顺序、幅度和恢复位置正确，服务返回 `DONE`，没有带动非目标关节。

如果 `TOUCH` 失败，应分别复测 `TILT` 和 `MOUTH`，不要直接调整触觉阈值掩盖动作层问题。

### 阶段 8：接入真实触觉织物

1. 保持 `AUTO_TOUCH_INTERVAL_SECONDS=0`。
2. 启动 Pi 服务并持续观察日志。
3. 启动 Windows 触觉桥，确认它连接的是刚刚实测的 Pi IP。
4. 第一次只触碰一次，然后至少等待 3 秒。
5. 确认 Windows 端检测到一次触发、Pi 收到一次 `TOUCH`、机器人只执行一次组合动作。
6. 重复轻触、长按和快速重复触碰，确认消抖和最短命令间隔符合预期。

通过标准：一次有效触碰只产生一次动作；无触碰时不误触发；网络断开和恢复不会造成积压动作或突然补执行。

### 阶段 9：恢复开机服务

只有前述阶段全部通过后才重新启用服务：

```bash
sudo systemctl enable --now pi-duck-control.service
systemctl status pi-duck-control.service --no-pager
journalctl -u pi-duck-control.service -n 100 --no-pager
```

建议长期保持：

```ini
Environment=AUTO_TOUCH_INTERVAL_SECONDS=0
```

除非项目明确需要周期演示且已经完成风险评估，否则不要恢复 5 秒自动动作。

## 动作验证记录模板

每次验证追加一条记录，不要只写“正常”：

```markdown
### YYYY-MM-DD HH:MM - COMMAND

- 硬件：Pi / OpenRB / 电源 / 串口路径
- 总线：协议 / 波特率 / 在线 ID
- 初始姿态：
- 执行命令：
- 服务响应：
- 实物动作：
- 非目标关节：无 / 有，详情
- 异响、发热、堵转：
- 是否正确回位：
- 结论：通过 / 需调参 / 禁止使用
- 后续处理：
```

## 当前未解决问题

1. 当前机器鸭的完整舵机 ID、波特率、串口设备和机械零位仍未知。
2. 同事所述自动连接公司 Wi-Fi 尚未实测，实际 Pi IP 未确认。
3. 服务启动时可能初始化并启用全部候选舵机，首次测试前需要特别防范意外全身动作。
4. 所有动作目前只是“源码已实现”，不是“当前机器鸭已验证”。
5. ROS 2 发行版、消息类型、包名、工作区和 launch 结构尚未设计。
6. RK3566 迁移时需要重新确认串口、权限、Dynamixel 参数和系统服务方案。

## 完成当前任务的判定条件

- 已记录实际 Pi 网络地址和联网方式。
- 已记录 OpenRB 串口、总线波特率和全部在线舵机 ID。
- 已建立舵机 ID 到实际关节的映射。
- `TILT`、`MOUTH` 和 `TOUCH` 已逐项验证并记录。
- 每个腿部动作都有明确的通过、需调参或禁止使用结论。
- 触觉织物能稳定做到一次有效触碰对应一次动作。
- 开机服务不再默认周期触发动作。
- 所有结论都有日志和现场观察记录，不只依赖源码存在或 `OK` 响应。

完成这些条件后，下一阶段才是基于实测接口搭建 Ubuntu 22.04 / RK3566 的 ROS 2 工作区和节点。