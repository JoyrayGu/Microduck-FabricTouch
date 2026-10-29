# Microduck 鸭子动作部署包（SSH 控制蹲起 + 坐立系列）

本包用于通过 SSH 远程控制小黄鸭机器人执行**蹲下-站起**和**坐立系列**（坐立 / 抬左脚 / 抬右脚 / 歪头）动作。
所有文件都是从已经跑通的实机上原样取出的，姿态数据经过实机验证。

> **Milestone 1（触觉皮肤 → 大脑）**：本包还包含一个常驻的 TCP 命令服务器
> （`microduck/src/command_server.py`），它接收笔记本端
> [`microduck_skin`](../microduck_skin) 通过局域网发来的**手势事件**，驱动头部
> 做拟静力的"卖萌"动作。**默认 dry-run，只有显式 `--enable-hardware` 才碰舵机总线。**
> 详见下方 [§1.5 触觉命令服务器](#15-触觉命令服务器milestone-1)。

---

## 1. 环境信息

| 项目 | 值 |
|---|---|
| 树莓派地址 | `192.168.1.57` |
| SSH 用户 / 密码 | `pi4` / `123456` |
| 部署目录 | `/home/pi4/microduck-replica-next/tools/deployment_validation` |
| Python 虚拟环境 | `/home/pi4/microduck_venv/bin/python` |
| PYTHONPATH | `/home/pi4/microduck-replica-next/microduck/src` |
| 舵机总线 | `/dev/ttyACM0`，**1 Mbps**，Dynamixel **Protocol 2.0** |
| 舵机型号 | ROBOTIS XL330，共 14 个（ID 1–14） |
| 目标平台 | **RK3566 / Ubuntu**（笔记：`pi_ssh.py` 里的"树莓派"是历史命名） |

### 舵机 ID 映射

| ID | 关节 | ID | 关节 |
|---|---|---|---|
| 1 / 6 | 右/左 踝 | 2 / 7 | 右/左 膝 |
| 3 / 8 | 右/左 髋俯仰 | 4 / 9 | 右/左 髋横滚 |
| 5 / 10 | 右/左 髋偏航 | 11 / 12 | 头颈俯仰（两轴） |
| 13 / 14 | 头偏航 / 头横滚 | | |

### 包内目录

```
microduck-deploy/
├── pi_ssh.py                       # SSH 执行/上传工具（本地运行）
├── 使用说明.md                      # 本文件
├── microduck/src/
│   ├── constants.py                # imu_reader 依赖
│   └── imu_reader.py               # BNO08x IMU 读取（squat_stand 依赖）
└── tools/deployment_validation/    # 整个目录原样传到树莓派同名路径
    ├── squat_stand.py              # 蹲下-站起
    ├── move_to_pose.py             # 姿态回放（任意两个已存姿态间平滑切换）
    ├── pose_loop.py                # 三个动作循环播放
    ├── lock_current.py             # 采集当前姿态并锁定
    ├── relax_legs.py               # 放松指定舵机（便于手工摆姿势）
    ├── servo_status.py             # 查看模式/力矩/位置
    ├── return_to_stand.py          # 全部回到标定站姿
    ├── imu_static_check.py         # IMU 静态可靠性检查
    ├── imu_balance_step.py         # 单脚抬起/踏步（需要 leg_geometry.json）
    ├── leg_geometry_calibration.py # 腿长标定 + 逆解（生成 leg_geometry.json）
    ├── microduck-pose-loop.service # 开机自启服务
    └── *.json                      # 姿态与标定数据
```

---

## 1.5 触觉命令服务器（Milestone 1）

Pi 真机运行时需要在 `/home/pi4/microduck_venv` 中安装 [brain/requirements.txt](requirements.txt)；
默认 dry-run 模式不导入舵机 SDK。

笔记本（`microduck_skin`）识别出触觉手势后，通过局域网把**手势名**发给大脑；
大脑做**最终安全判定**并驱动头部动作。这是"常驻实时"链路，与前面用 SSH 启脚本
（部署/运维）不是一回事——命令服务器长期运行、不每次重启进程。

```bash
# 默认在 0.0.0.0:9000 监听，但永远 dry-run（不碰舵机总线）
python -m microduck.src.command_server

# 真机：显式打开硬件
python -m microduck.src.command_server --enable-hardware --port-name /dev/ttyACM0 --baud 1000000
```

**线上协议**：TCP 长连接，每行一个 JSON（newline-delimited JSON，UTF-8）。
单客户端（Milestone 1，多余连接被拒）。skin 发手势事件，brain 回一行应答：

```jsonc
// skin -> brain（手势事件，不含任何舵机参数）
{"v":1,"type":"gesture","event_id":"…","gesture":"static_touch","timestamp_ms":…}
// brain -> skin（一行应答）
{"v":1,"ok":true,"event_id":"…","action":"head_roll","reason":"ok"}
```

**手势 → 动作映射**（不改变重心，每个动作只驱动一个舵机）：

| 手势 | 动作 | 实际驱动 | 说明 |
| --- | --- | --- | --- |
| `static_touch` | `head_roll` | **仅 ID14** | 接触轨迹不足以构成有效滑移 → 左右歪头（约 1.4s） |
| `dynamic_touch` | `open_mouth` | **仅 ID15** | 形成有效滑移轨迹（单程、往返或部分滑移）→ 张嘴再闭合（约 0.65s） |

> 设计取舍（2026-10-22 确认）：最终形态是把织物装到鸭头上。用户持续触摸时，
> 头部不停做动作会明显影响体验，因此「持续滑动」只给一次短促的张嘴，
> 而不是持续摆动。`head_wag`（ID13 转头）已完成标定但**当前无手势映射**，
> 作为备用动作保留。

**实测标定汇总**

| 关节 | 中位 | 一端 | 另一端 | 说明 |
| --- | --- | --- | --- | --- |
| ID14 头部横滚 | 1881 | 左 1712 | 右 2050 | ±169 tick（±14.9°），误差 ≤3 tick |
| ID13 头部偏航 | **1854** | 左 1685 | 右 2023 | ±169 tick（±14.9°），备用 |
| ID15 嘴部 | — | 张开 1760 | 闭合 2174 | 硬限位 1711 / 2188，各留余量 |

> ID13 的中位**不是** 2048，实测偏差 -194 tick。旧实现用 `2048 - (1736-2048) = 2360`
> 镜像推算右端，比对称值多 337 tick（29.6°），会撞线缆限位。该假设已证伪。
>
> 每个动作都**只写单一个 ID**：ID13 与 ID14 机械耦合（摆一个会带动另一个 100+ tick），
> 同时给两者固定目标会让舵机互相顶住。
>
> 嘴部闭合目标用 **2174** 而非硬限位 2188：主动驱动到 2188 时实测只能到
> 2174（ERR=-14），弹簧挡住了。若把目标写成硬限位，每次闭合舵机都会顶到
> 最大扭矩去够一个到不了的位置，长期发热而无视觉收益。


**安全边界（都在大脑端，skin 越想权越被拒）**：

- 只接受白名单手势；**不接受**舵机 ID / 数量 / 速度 / 角度；
- 事件过期（超时）、重复（event_id 去重）、动作执行期间的新事件一律拒绝；
- 单动作互斥——同一时刻只跑一个动作；
- 默认 dry-run（`DryRunRuntime`），只有 `--enable-hardware` 才切到 `DynamixelRuntime`。

常用参数：

| 参数 | 作用 |
| --- | --- |
| `--host 0.0.0.0` / `--port 9000` | 监听地址 / 端口（默认 9000） |
| `--enable-hardware` | 允许访问真实舵机总线；**不加则永远 dry-run** |
| `--port-name /dev/ttyACM0` | 舵机串口 |
| `--baud 1000000` | 舵机波特率 |
| `--verbose` | 打印每一帧目标位置 |
| `--duration 5` | 跑 5 秒后退出（0 = 一直运行） |

相关模块：`command_schema.py`（协议校验）、`action_library.py`（头部动作库）、
`action_dispatcher.py`（时效/去重/互斥调度）、`servo_runtime.py`（dry-run / 真机运行时）。

---

## 2. 部署

```bash
# 1) 上传整个 tools/deployment_validation 目录到树莓派同名路径
#    （保持相对结构不变，脚本靠 __file__ 找同目录的 json）
#    （保持相对结构不变，脚本靠 __file__ 找同目录的 json）
scp -r tools/deployment_validation pi4@192.168.1.57:/home/pi4/microduck-replica-next/

# 2) 上传 microduck/src 下的两个模块
scp microduck/src/constants.py microduck/src/imu_reader.py \
    pi4@192.168.1.57:/home/pi4/microduck-replica-next/microduck/src/
```

`pi_ssh.py` 是更省事的替代（自带重试；密码从环境变量读）：

```powershell
# Windows PowerShell
$env:PI_PASSWORD='123456'
python pi_ssh.py run "cd /home/pi4/microduck-replica-next/tools/deployment_validation && PYTHONPATH=/home/pi4/microduck-replica-next/microduck/src /home/pi4/microduck_venv/bin/python servo_status.py"
python pi_ssh.py upload <本地文件> <树莓派路径>
```

> **注意**：远程命令里不要用 `$` 变量（会被本地 shell 吞掉），直接用完整路径。

---

## 3. 蹲下-站起

### 3.1 先看状态

```bash
cd /home/pi4/microduck-replica-next/tools/deployment_validation
export PYTHONPATH=/home/pi4/microduck-replica-next/microduck/src

/home/pi4/microduck_venv/bin/python servo_status.py
```

期望输出：14 路全部 `mode=3`、`t=1`，末尾 `MODES_ALL_POSITION` / `ALL_TORQUE_ON`。

### 3.2 跑一次蹲起（一键预设）

```bash
/home/pi4/microduck_venv/bin/python squat_stand.py --mode run --preset squat-stand
```

`--preset squat-stand` 等价于下面这串（预设只提供默认值，显式传参仍可覆盖任意一项）：

```
--squat-time 1.0 --hold-time 1.0 --stand-time 1.0 --squat-depth 1.0
--pitch-kp 0 --pitch-kd 0 --roll-kp 0 --ankle-gain 0
--pitch-target 0 --pitch-target-early 0 --cog-ff 0 --attitude-tol 180
```

含义：**满深度蹲下 1 秒 → 底部保持 1 秒 → 站起 1 秒，纯开环、平衡环全部关闭**。
实机验证：这套配置在 2026-09-30 连续 5 次 `SQUAT_COMPLETE` 通过。

成功标志：日志末尾出现 `SQUAT_COMPLETE`，并打印 `FINAL_STAND {...}`。

### 3.3 常用可调项

| 参数 | 说明 |
|---|---|
| `--squat-depth 0~1` | 蹲的深度比例（1.0 = 示教到底）。调试时建议从 0.3 起，稳定后再加深 |
| `--squat-time` / `--stand-time` | 蹲下 / 站起耗时（秒）。1.0 是实测上限，再快会超出舵机带载能力 |
| `--squat-gate-pitch` | 底部稳定性判据的俯仰上限（默认 6°，偏严；摆放稍歪就会误判而中止，可放宽到 9） |
| `--goal-current 500` | 切成电流位置模式（mode 5）限流，动作更软；**退出时自动恢复 mode 3** |
| `--trace` | 打印每一帧下发的目标位置（约 40 行/秒），用于观察数据流 |

### 3.4 重新示教蹲姿（一般不需要）

```bash
/home/pi4/microduck_venv/bin/python squat_stand.py --mode teach
```
会放松 1/2/3/6/7/8，人工摆好蹲姿底部后按回车采集，自动存 `squat_pose.json` 并回到站姿。

---

## 4. 坐立系列（坐立 / 抬左脚 / 抬右脚 / 歪头）

这一系列用**姿态回放**实现：每个姿态是一份 JSON（14 路舵机的目标计数），用 `move_to_pose.py` 在两者之间平滑过渡。

| 姿态 | 文件 |
|---|---|
| 坐立 | `sit_pose.json` |
| 抬左脚 | `lift_left_pose.json` |
| 抬右脚 | `lift_right_pose.json` |
| 歪头 | `tilt_head_pose.json` |

### 4.1 回放单个姿态

```bash
# 平滑回放（余弦缓动 1.5 秒，两端速度为零）
/home/pi4/microduck_venv/bin/python move_to_pose.py --pose sit_pose.json --duration 1.5

# 满速回放（直接把目标交给舵机，位置模式全速过去，实测 0.08~0.22 秒到位）
/home/pi4/microduck_venv/bin/python move_to_pose.py --pose lift_left_pose.json --duration 0
```

成功标志：打印 `MOVE_OK`。

### 4.2 三个动作依次播放

```bash
# 坐立 -> 抬左脚 -> 坐立 -> 抬右脚 -> 坐立 -> 歪头 -> 坐立
/home/pi4/microduck_venv/bin/python move_to_pose.py --pose lift_left_pose.json --duration 1.5 && sleep 1.2 && \
/home/pi4/microduck_venv/bin/python move_to_pose.py --pose sit_pose.json      --duration 1.5 && sleep 1.2 && \
/home/pi4/microduck_venv/bin/python move_to_pose.py --pose lift_right_pose.json --duration 1.5 && sleep 1.2 && \
/home/pi4/microduck_venv/bin/python move_to_pose.py --pose sit_pose.json      --duration 1.5 && sleep 1.2 && \
/home/pi4/microduck_venv/bin/python move_to_pose.py --pose tilt_head_pose.json --duration 1.5 && sleep 1.2 && \
/home/pi4/microduck_venv/bin/python move_to_pose.py --pose sit_pose.json      --duration 1.5
```

> **重要**：两个单脚姿态之间**必须插入「坐立」中转**。直接从「抬左脚」切到「抬右脚」会让两条腿同时改变姿态，机器人必然倒。

### 4.3 循环播放（一圈 10 秒）

```bash
/home/pi4/microduck_venv/bin/python pose_loop.py --cycles 0        # 0 = 无限循环
/home/pi4/microduck_venv/bin/python pose_loop.py --cycles 1        # 只跑一圈，用于验证
```

每圈：三个动作各 0.9 秒缓动 + 保持，剩余时间静置在坐立，**整圈精确 10 秒**。
日志每圈打印 `CYCLE n motions=7.82s rest=2.18s`。

### 4.4 开机自启

`microduck-pose-loop.service` 已配置好开机自动运行循环播放：

```bash
sudo cp microduck-pose-loop.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now microduck-pose-loop

# 查看
systemctl status microduck-pose-loop --no-pager
sudo journalctl -u microduck-pose-loop -n 20

# 停止 / 取消自启
sudo systemctl stop microduck-pose-loop
sudo systemctl disable microduck-pose-loop
```

服务里 `Restart=always` + `RestartSec=5`：即使开机时串口设备还没枚举出来（会报错退出），也会每 5 秒重试直到控制板就绪。

---

## 5. 采集新姿态

```bash
# 1) 放松要手工摆的关节（例：右腿的膝+髋俯仰）
/home/pi4/microduck_venv/bin/python relax_legs.py --ids 2,3

# 2) 手工把机器人摆成目标姿态，扶稳

# 3) 采集 + 重新上电 + 校验 + 存文件
/home/pi4/microduck_venv/bin/python lock_current.py --ids 1,2,3,4,5,6,7,8,9,10,11,12,13,14 \
    --save new_pose.json
```

`lock_current.py` 的流程是"读当前位置 → 写成目标 → 使能力矩 → 校验漂移"，所以**采集完姿态就被锁住**，不会自己掉下来。
输出的 `drift` 应为 0；超过 15 会打 `WARNING: pose did not hold`。

回放新姿态同样用 `move_to_pose.py --pose new_pose.json`。

---

## 6. 必须遵守的约定（踩过的坑）

1. **一律用位置模式（XL330 mode 3）跑这些动作**，不要切到电流位置模式（mode 5）。
   当前代码只在 `squat_stand.py --goal-current` / `imu_balance_step.py --goal-current` 会临时切 mode 5，且退出时恢复。
2. **`squat_pose.json` 用包内这一份**（joints `2508/3669/1672/776/604/2229`）。
   历史上存在一份旧的 `1149/1669/1179/2235/2849/2908`，没有在实机上验证过，不要替换。
3. **两个单脚姿态之间必须经「坐立」中转**，否则两腿同时动、必倒。
4. **掉电后 14 路舵机力矩全部为 0**（重启默认不使能）。跑动作前先 `servo_status.py` 确认 `ALL_TORQUE_ON`；`squat_stand.py` / `imu_balance_step.py` 会在检测到未使能时直接拒绝启动。
5. **树莓派重启后 OpenRB-150 可能不枚举**（`/dev/ttyACM0` 不存在、`lsusb` 只有 root hub）。
   按一下板子上的物理 RESET 键即可恢复。
6. **树莓派容易因供电跌落掉线**（表现为 ping 不可达 / SSH 超时）。重启后注意重连。
7. **平衡环不要开**。这是最重要的教训：`squat_stand.py` 的 pitch/roll PD 环（`--pitch-kp/--pitch-kd/--roll-kp`）会产生 **2Hz 发散振荡**，幅度越滚越大，是之前所有摔倒的元凶。开环（增益全 0）反而能跑通。`--pitch-target 3.0` 这类姿态整形目标单独启用也会导致不稳定。
8. **`move_to_pose.py` / `pose_loop.py` 只写目标位置寄存器、从不碰模式寄存器**，因此天然符合第 1 条。

---

## 7. 已知限制

- **蹲起是纯开环**：到位准确，但**抗扰能力为零**——外力推、地面倾斜、摆放不水平都会直接导致底部前后倾倒。实测它对摆放姿态敏感（底部残余重心偏差会让它单向前后倒）。若要恢复抗扰，需重新设计一个"很轻"的闭环（kp ≤ 0.5），而不是复用现在这个会泵发散的环。
- **抬脚/坐立系列同样是开环姿态回放**，只有起止两个姿态点、**没有中间轨迹规划**，也不做 IMU 平衡。动作到位准确，但过程中不保证动态优美。
- **`imu_balance_step.py`（单脚抬起/踏步）当前的开环高度上限约 5mm**：超过这个高度躯干前倾会发散。它的抬脚轨迹已经改成笛卡尔空间（任务空间）垂直抬升 + 踝关节水平补偿，**构造上不会蹭地**，抬脚高度由 `--clearance`（mm）直接给定。使用前需要先生成 `leg_geometry.json`：

```bash
# 左右腿各跑一次（尺寸是实测值，单位 mm）
/home/pi4/microduck_venv/bin/python leg_geometry_calibration.py --mode measure --side left \
    --thigh 41.7 --shin 39.7 --ankle-height 21.7 --hip-height 98.7 --hip-to-ankle-x 22.0
/home/pi4/microduck_venv/bin/python leg_geometry_calibration.py --mode measure --side right \
    --thigh 41.7 --shin 39.7 --ankle-height 21.7 --hip-height 98.7 --hip-to-ankle-x 22.0
/home/pi4/microduck_venv/bin/python leg_geometry_calibration.py --mode show --side left
```

- **站立基准**：`standing_zero_calibration.json` 是所有姿态的零位来源，不要手工改动。

---

## 8. 快速排错

| 现象 | 原因 / 处理 |
|---|---|
| `could not open port /dev/ttyACM0` | 控制板没枚举，按 OpenRB 的 RESET 键 |
| `not all servos enabled` | 掉电后力矩为 0，先跑一次 `lock_current.py` 或让脚本重新使能 |
| `SQUAT_GATE not stable` | 底部俯仰超过判据（默认 6°）。多为摆放偏斜导致，可 `--squat-gate-pitch 9` 放宽，或重新摆正机器人 |
| `SQUAT_RECOVERY attitude recovery` | 真的不稳（俯仰 >16° 或角速度超阈值），属实际倾倒，检查摆放 |
| `MOVE_INCOMPLETE` | 姿态没到位（某路偏差 >15 counts），检查是否被外力挡住或该关节限位 |
| SSH 超时 / ping 不可达 | 树莓派掉线，检查供电和 WiFi |