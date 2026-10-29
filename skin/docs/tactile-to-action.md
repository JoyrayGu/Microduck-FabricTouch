# Microduck 触觉 → 动作链路

把电子触觉皮肤（当前标准阵列为 32×32）贴在机器鸭头部，用户触摸时由 Windows 端识别触觉语义，
经 TCP 发给树莓派上的 brain，由它驱动舵机做出对应动作。

```
触觉皮肤 ──USB(9600)──> Windows ──TCP:9000──> 树莓派 brain ──UART(1Mbps)──> 舵机
                        │                       │
                   手势识别                 动作调度
          static_touch/dynamic_touch    head_roll/open_mouth
```

## 当前手势 → 动作映射

| 手势 | 判定依据 | 动作 | 驱动舵机 | 时长 |
| --- | --- | --- | --- | --- |
| `static_touch` | 接触轨迹不足以构成有效滑移，且接触时长在 80–1200 ms | `head_roll` 左右歪头 | 仅 ID14 横滚 | 约 1.4 s |
| `dynamic_touch` | 接触形成足够长的有效滑移轨迹；包含单程、往返和部分滑移 | `open_mouth` 张嘴再闭合 | 仅 ID15 嘴部 | 约 0.65 s |

当前机器人接口只输出 `static_touch` / `dynamic_touch` 两种语义。多点状态通过事件字段记录，
当前动作映射仍由 Brain 按这两种语义处理。

`head_wag`（ID13 转头）已完成标定，但当前没有手势映射，作为备用动作保留。

## 前置条件

### 硬件

| 项目 | 值 |
| --- | --- |
| 触觉皮肤 | USB `0483:5740`，COM 口，9600 baud |
| 阵列 | **32×32**，16-bit 大端，约 2050 字节载荷，约 40 Hz |
| 舵机总线 | `/dev/ttyACM0`，1 Mbps，Protocol 2.0 |
| 树莓派 | `pi4@192.168.1.57`（主机名 `duck`） |

### 软件

```bash
# Windows
pip install pyserial

# 树莓派（已有 venv）
/home/pi4/microduck_venv/bin/python -c "import dynamixel_sdk; print('ok')"
```

### SSH 免密

`~/.ssh/config` 里配置主机别名，之后可用 `ssh duck`：

```sshconfig
Host duck
    HostName 192.168.1.57
    User pi4
    IdentityFile ~/.ssh/microduck_ed25519
    IdentitiesOnly yes
    ServerAliveInterval 15
```

验证：

```powershell
ssh -o BatchMode=yes duck "echo OK"
```

---

## 使用

分两个终端：一个启动 brain（树莓派），一个运行触觉识别（Windows）。

### 1. 启动 brain（带硬件驱动）

```powershell
# 1a. 停旧进程，并等到端口真正释放（最多 5 秒）
ssh duck "pkill -f 'command_server.*enable-hardware'; for i in 1 2 3 4 5; do ss -tln 2>/dev/null | grep -q ':9000 ' || break; sleep 1; done; ss -tln 2>/dev/null | grep -q ':9000 ' && echo 'STILL_BUSY' || echo PORT_FREE"

# 1b. 清空日志后启动（清空是为了不把上次失败的 traceback 误读成这次的错误）
ssh duck "cd /home/pi4/microduck_brain && : > /home/pi4/brain-hw.log && nohup /home/pi4/microduck_venv/bin/python -u -m microduck.src.command_server --host 0.0.0.0 --port 9000 --enable-hardware --port-name /dev/ttyACM0 --baud 1000000 >> /home/pi4/brain-hw.log 2>&1 & sleep 3; cat /home/pi4/brain-hw.log"
```

**成功标志**：

```
COMMAND_SERVER listening on 0.0.0.0:9000 runtime=dynamixel
```

`runtime=dynamixel` 说明真的会驱动舵机；若显示 `runtime=dry_run`，说明 `--enable-hardware` 没生效。

> **为什么用 `: > 日志文件` 清空**：日志是 `>>` 追加写入的，上次启动失败留下的
> `OSError: [Errno 98] Address already in use` 会一直留在文件里。清空后
> `tail` 只会看到本次内容，避免把历史错误误读成当前故障。
>
> 判断运行是否正常，**以进程与端口为准**，而不是看日志尾部：
>
> ```powershell
> ssh duck "pgrep -af command_server; ss -tlnp 2>/dev/null | grep 9000"
> ```

### 2. 运行触觉识别

```powershell
cd E:\OneDrive\MatNgine\microduck_skin
$env:PYTHONIOENCODING="utf-8"
E:\MiniConda\python.exe run.py --robot --brain-host 192.168.1.57 --brain-port 9000 `
  --port COM7 --baud 9600 --threshold 20 --duration 90
```

**注意**：不要加 `--brain-dry-run`，加了就只识别不发送。

### 3. 看日志

```powershell
# brain 侧
ssh duck "tail -20 /home/pi4/brain-hw.log"

# 实时
ssh duck "tail -f /home/pi4/brain-hw.log"
```

### 4. 停止 brain

```powershell
ssh duck "pkill -f 'command_server.*enable-hardware'"
```

---

## 期望输出

**Windows 侧**（每个手势产生 3 行）：

```
[设备] USB 通道：二进制协议帧（连续/单次采集）
[手势] tap conf=1.00
[系统] -> brain event_id=a4f06d93... gesture=tap
[brain] ACCEPTED action=head_roll reason=ok event_id=a4f06d93...
```

**树莓派侧**：

```
ACTION_START head_roll ids=[14] gesture=tap event_id=a4f06d93...
ACTION_END head_roll outcome=done
```

两端用同一个 `event_id`，出问题时能直接判断是识别错了、传输丢了、还是硬件没动。

> 应答可能滞后一个事件：brain 是同步执行动作的，`head_roll` 要 1.4 秒才返回应答。
> 这不是故障，`event_id` 仍能对上。

---

## 故障排查

### `Address already in use`

旧 brain 进程没清理干净。

```powershell
ssh duck "pgrep -af command_server"
ssh duck "kill <PID>"
```

### `[brain] REJECTED reason=busy`

上一个动作还没跑完（`head_roll` 1.4 s、`open_mouth` 0.65 s），新事件被互斥拒绝。
这是设计行为，不是故障。

### `[brain] REJECTED reason=expired`

事件时间戳与 brain 时钟差超过 2000 ms。
skin 侧已用 `time.time()`（墙钟）生成时间戳，若出现这个错误说明两端时钟漂移严重。

### `STEP error: IDxx write4@116: no status packet!`

舵机通信失败。两个已知原因：

1. **单步位移过大**：已修（`MAX_STEP_TICKS = 12`，超过则拆成多次小写）
2. **串口被两个进程占用**：确认只有一个 `command_server` 在跑

### 完全没有 `[手势]` 输出

按顺序检查：

1. 是否出现 `[设备] USB 通道：二进制协议帧（连续/单次采集）`？
   - 没有 → 串口没通。检查 COM 口、`--baud 9600`、设备是否被其他程序占用
2. `--threshold` 是否太高？
   - 实测轻点峰值 66–248，噪声地板 0–8。`--threshold 20` 对应软件阈值 12，通常够用
3. 手势动作是否符合判定条件？
   - `static_touch` 需要有效接触持续 80–1200 ms，且轨迹不足以构成滑移
   - `dynamic_touch` 需要形成累计轨迹 ≥ 3.0 格

### 舵机动但动作不对

用 `play_action` 单独验证动作序列（绕过触觉与网络）：

```powershell
cd E:\OneDrive\MatNgine\Microduck-FabricTouch\tools
$env:PYTHONPATH="src"
E:\MiniConda\python.exe -m microduck_behavior_control.play_action --action tilt_head_smooth --execute
E:\MiniConda\python.exe -m microduck_behavior_control.play_action --action mouth_open_close --execute
```

若 `play_action` 正常但触觉链路异常，问题在识别或传输；反之在动作序列。

---

## 实测标定值

| 关节 | 中位 | 一端 | 另一端 | 说明 |
| --- | --- | --- | --- | --- |
| ID14 头部横滚 | 1881 | 左 1712 | 右 2050 | ±169 tick（±14.9°） |
| ID13 头部偏航 | **1854** | 左 1685 | 右 2023 | 备用，无手势映射 |
| ID15 嘴部 | — | 张开 1760 | 闭合 2174 | 硬限位 1711 / 2188，各留余量 |

几个关键点：

- **ID13 中位不是 2048**，实测偏差 −194 tick。旧实现用 `2048 - (1736−2048) = 2360` 镜像推算右端，
  比对称值多 337 tick（29.6°），会撞线缆限位。该假设已证伪。
- **嘴部闭合目标用 2174 而非硬限位 2188**：主动驱动到 2188 时实测只能到 2174（ERR=−14），
  弹簧挡住了。若目标写硬限位，每次闭合舵机都会顶到最大扭矩去够一个到不了的位置。
- **每个动作只写单一个 ID**：ID13 与 ID14 机械耦合（摆一个会带动另一个 100+ tick），
  同时给两者固定目标会让舵机互相顶住。

---

## 已验证与未解决

### 已验证

- 32×32 触觉阵列读取，当前设备约 40 Hz
- `static_touch` / `dynamic_touch` 两种机器人触觉语义
- 手势 → brain → 舵机的完整链路
- `event_id` 全链路对账

### 未解决（后续开发）

| 问题 | 影响 |
| --- | --- |
| **颈部（ID11/12/13）零力矩** | 用户摸头时头部会被手推动 |
| **多点接触只取 `contacts[0]`** | 多指或手指+鱼际同时接触时质心失真 |
| **部分滑移/部分静止无法区分** | 单指滑动、另一指静止时，全局质心被平均稀释 |

这三项需要先标定颈部行程、采集真实多点触觉数据，再重做识别层的数据模型。
