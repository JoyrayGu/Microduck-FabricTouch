"""@file config.py
@brief 配置对象定义与 JSON 持久化。
@details
    该模块维护滤波、手势、映射、USB 与绘图相关配置，并提供
    AppConfig.to_dict() / from_dict() / save() / load() 用于运行时保存和恢复。

配置：滤波 / 手势 / 映射参数，支持 JSON 存取，便于现场调参后保存。

所有阈值都与硬件强相关，第一次上手最省事的办法是：
    1. 先跑 ``python run.py --mock --gui``，在界面上观察「总压力」数值范围；
    2. 再跑 ``python run.py --gui`` 连真机，看实际轻触/重压分别是多少；
    3. 把 ``--save-config`` 存下来，之后 ``--config`` 直接复用。
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field, fields
from typing import Any, Dict, Optional

from . import protocol as P


@dataclass
class FilterConfig:
    """原始帧 -> 清洁后的 32x32 压力场。"""

    rows: int = P.DEFAULT_ROWS
    cols: int = P.DEFAULT_COLS
    #: 3x3 空间平滑次数（0 = 关闭）。阵列小、噪声大时开 1 次很有效。
    spatial_smooth_passes: int = 0
    #: 时域指数平滑系数，越大越跟手、越小越稳。0.5 是较均衡的起点。
    ema_alpha: float = 0.5
    #: 开机静置校准的帧数（用于扣除各通道零点漂移），0 = 不校准
    baseline_frames: int = 40
    use_baseline: bool = True
    #: 绝对激活阈值：单格压力低于此值视为未接触（应与设备的 0x0C 阈值配合）
    #:
    #: 默认 300 曾基于「单位是 kPa」的假设，而实测设备输出是 16-bit 原始值
    #: （轻点峰值仅 14~40）。机器人模式在 RobotSession 里会覆盖成 12。
    #: 鼠标模式的整定与测试都基于 300，这里保持不动以免连带破坏。
    activate_abs: float = 300.0
    #: 相对激活阈值：低于「当前最大格 x 该比例」的格子也视为噪声
    activate_rel: float = 0.12
    #: 判定为「有效接触」所需的最少激活格数（滤掉单点毛刺）
    #:
    #: 保持 2 作为鼠标模式的默认值：鼠标手势的既有测试与整定都基于它。
    #: 机器人模式在 RobotSession 里会覆盖为 1（见 robot_session 的说明），
    #: 因为手指快速滑动时压力向前集中，实测会出现 peak=14~17 却只有 1 个格
    #: 超过阈值的情况；若要求 2 格就会把 active 误判为 False，把一次连续
    #: 滑动切成碎片。
    min_active_cells: int = 2
    #: 连通域少于该格数时忽略（去毛刺）
    min_contact_cells: int = 2
    #: 局部峰值相距小于该值（格）时视为同一根手指的两个峰，合并
    merge_radius_cells: float = 1.6
    #: 最多同时跟踪的接触点数量（多指）
    max_contacts: int = 3


@dataclass
class GestureConfig:
    """时序手势参数（单位：毫秒 / 格）。"""

    tap_max_ms: float = 260.0  # 按住时间短于此值 + 几乎没移动 => 单击
    double_tap_ms: float = 420.0  # 两次点按间隔小于此值 => 双击
    long_press_ms: float = 650.0  # 按住超过此值 => 长按（右键）
    tap_move_tol: float = 0.85  # 点按允许的手指移动量（格）
    #: 重压判定用的绝对下限（原始压力值）。设备量纲不同，务必按实测调整：
    #: 界面上「总压力」显示轻触大约是多少，把这个值设成它的 1.2~1.5 倍。
    heavy_total: float = 7000.0
    #: 稳定期：落指后这么久之内不判重压，避免落指瞬间的冲击被误判成拖拽
    heavy_settle_ms: float = 200.0
    #: 相对判据：本次接触压力 >= heavy_auto_ratio x 历史「轻触」压力 也算重压。
    #: 这样即使不知道设备的压力量纲也能用，是默认启用的主判据。
    heavy_auto: bool = True
    heavy_auto_ratio: float = 1.7
    #: 双指滚动：横向/纵向位移比小于该值才认为是纵向滚动（|dx| <= k*|dy|）
    scroll_axis_ratio: float = 0.8
    scroll_deadzone: float = 0.35  # 累计位移超过该值才滚一格
    #: True 时由程序显式发出双击（迟滞 double_tap_ms 才出单击）；False 时
    #: 连发两次单击，交给操作系统的双击间隔判定，点击延迟最低
    double_click_as_event: bool = False


@dataclass
class MappingConfig:
    """手指位置 -> 光标动作。"""

    #: joystick（中心摇杆，推荐） | absolute（绝对定位） | ninegrid（九宫格按钮）
    mode: str = "joystick"
    #: 摇杆的「零点」来源：
    #:   touch = 每次落指的初始位置作为零点（类似笔记本触控板，推荐，点按不会漂）
    #:   array = 阵列几何中心作为零点（手指停在中心不动）
    origin: str = "touch"
    #: origin="touch" 时，手指相对零点移动多少格达到满速
    touch_range_cells: float = 2.2
    #: 手指顶到阵列边缘后，是否让零点缓慢跟随手指，以便持续长距离移动
    edge_recenter: bool = False
    #: 边缘跟随速度（格/秒）
    edge_recenter_cells_s: float = 1.6
    #: 摇杆最大速度（像素/秒）。屏幕大的话调大，精细操作用 400~600。
    max_speed_px_s: float = 780.0
    #: 死区（归一化 0~1）：手指离中心多近算「不动」
    deadzone: float = 0.13
    #: 响应曲线指数：>1 中心附近更细腻，边缘更快
    gamma: float = 1.7
    #: 速度低通系数 0~1，越大越跟手但越抖
    speed_smoothing: float = 0.45
    #: 按压力度参与调速：压力越大光标越快（0 = 关闭，1 = 完全由压力决定）
    pressure_speed_mix: float = 0.35
    #: 参考压力（用于 pressure_speed_mix 的归一化），0 = 自动
    pressure_ref: float = 0.0
    invert_x: bool = False
    invert_y: bool = False
    #: 绝对定位模式的边距（像素），避免手指贴边时光标顶到屏幕角落
    abs_margin_px: int = 6
    #: 单帧最大位移（像素），防止异常帧把光标甩飞
    max_step_px: float = 220.0
    #: 滚轮：每格位移对应的滚动格数
    scroll_gain: float = 1.0
    #: 滚轮方向是否反转
    scroll_invert: bool = False


@dataclass
class RobotGestureConfig:
    """机器人手势识别参数（与鼠标手势**分开**，避免互相干扰）。

    这里只负责把压力场变成 tap / single_stroke / back_forth_stroke 三种
    与动作无关的手势事件。动作语义由 brain 端决定。
    """

    #: 接触期间质心位移小于此值（格）才可能判为 tap。
    #:
    #: 注意：这个值**不再**作为 tap 的必要条件。真实手指按压时质心会漂移
    #: 1~2 格（手指有肉、按压力心随之移动），用 0.9 格当硬门槛会让大部分
    #: 真实轻点被静默丢弃。现在 tap 的判定改为「轨迹不足以构成 stroke」，
    #: 本字段仅保留用于诊断输出。
    tap_move_tol: float = 0.9
    #: tap 的最大按住时间（毫秒）。仅作参考，判定已改用 press_max_ms。
    tap_max_ms: float = 320.0
    #: 「轨迹不足以构成 stroke」时，按住超过此毫秒数就不算 tap。
    #:
    #: 用于排除手掌一直搭在织物上的情况：那种接触没有位移，但持续时间很长。
    #: 取 1200ms 是因为实测最长的一次有意轻点为 750ms，留出余量。
    press_max_ms: float = 1200.0
    #: 消抖恢复时允许的最大质心跳变（格）。**已废弃**，仅保留兼容旧配置。
    #:
    #: 实测证明该特征无法区分「两次独立轻点」与「一次连续滑动」：连续滑动
    #: 内部的帧间位移（最大 10.6 格）反而大于独立轻点之间的间距（3.4 格）。
    debounce_resume_max_jump: float = 1.5

    #: 释放消抖（毫秒）：压力归零后的确认时长。
    #:
    #: 判据已改为「压力是否归零」为主、「时长」为辅（见 robot_gestures.update）。
    #: 碎片空档（织物仍有接触）即使超过此时长也不判抬手，只有压力真正归零
    #: 才用它做最终确认，防止单帧掉零造成误判。
    release_debounce_ms: float = 120.0
    #: 空档内峰值低于此值即认为「压力已归零」，可判定抬手。
    #:
    #: 实测真实抬手的空档内最低峰为 0.0~0.5，内部碎片最低峰为 1.7~157.8。
    #: 取 1.0 落在两组之间。
    release_zero_peak: float = 1.0
    #: 压力未归零时的兜底超时（毫秒）：超过它再久也判抬手。
    #:
    #: 防止「手一直搭着但压力始终略高于零」的情况永远不出结果。
    release_debounce_hard_ms: float = 900.0
    #: 最小接触时长（毫秒）：短于此值的接触直接丢弃，不产生任何事件。
    #:
    #: 实测输出里大量 `held_ms=50 drift=0.0 path_len=0.0` 的幽灵事件，那是
    #: 单帧噪声激活（EMG 抖动、织物回弹）。这类接触没有任何物理意义，
    #: 却会触发真实动作，必须在判定前就滤掉。
    min_contact_ms: float = 80.0
    #: 判定 stroke 所需的最小累计轨迹长度（格）。
    stroke_min_path: float = 3.0
    #: 判定 stroke 所需的最小起止净位移（格）。
    stroke_min_net: float = 2.0
    #: 单帧质心位移超过此值视为跳变，不计入轨迹（抗噪）。
    stroke_max_step: float = 6.0
    #: 累计停顿（帧间位移 < stroke_pause_tol）超过此毫秒数则打断连续性。
    stroke_pause_tol: float = 0.15
    stroke_pause_break_ms: float = 400.0
    #: 接触起始稳定期（毫秒）：这段时间内只跟踪质心位置，不计入轨迹。
    #:
    #: 手指刚压上织物时压力分布尚未稳定，实测质心在头两帧内跳 1.99 格，
    #: 随后才稳定。这些帧会给静止按压凭空堆积 path_len（实测 3.52），
    #: 使一次轻点被误判成 back_forth_stroke。取 120ms 覆盖典型稳定过程。
    settle_ms: float = 120.0
    #: 「净位移 / 累计路程」低于此值判为来回抚摸（back_forth_stroke）。

    #:
    #: 实测：单程滑动 ≈ 0.90~1.00；来回抚摸 ≈ 0.01~0.25。取 0.55 留足余量。
    #: 这个比值替代了早期的「方向反转计数」：后者在单程滑动上会因横向抖动
    #: 误报 6~10 次反转（run5 实测反向累计 3.1 格，全是左右抖动）。
    back_forth_max_straightness: float = 0.7
    #: 方向反转的最小位移（格）。**已不用于手势分类**，仅保留以兼容旧配置。
    reversal_min_disp: float = 1.2
    #: 反向累计位移占累计行程的最小比例。**已不用于手势分类**，仅兼容旧配置。
    reversal_min_ratio: float = 0.25
    #: 顺向移动时反向累计的衰减系数。**已不用于手势分类**，仅兼容旧配置。
    reversal_decay: float = 0.7
    #: 主方向基准的跟随系数。**已不用于手势分类**，仅兼容旧配置。
    reversal_dir_follow: float = 0.08
    #: 判定「主方向」时，单一轴向占比超过此比例才认为方向明确。
    #:
    #: 实测对角滑动的 |Δx|/(|Δx|+|Δy|) 约为 0.47~0.67，用 0.6 会漏判，
    #: 因此默认降到 0.4。
    direction_dominance: float = 0.4
    #: 连续接触长度超过该值（毫秒）后，抬手时若满足条件则输出 stroke。
    stroke_min_ms: float = 120.0


@dataclass
class RobotLinkConfig:
    """skin -> brain 的网络连接参数。"""

    #: brain（RK3566）地址与端口。
    host: str = "192.168.1.50"
    port: int = 9000
    #: 建立连接的超时（秒）。
    connect_timeout: float = 3.0
    #: 发送/接收超时（秒）。
    timeout: float = 2.0
    #: 断线后自动重连。
    reconnect: bool = True
    #: 重连间隔（秒）。
    reconnect_interval: float = 1.0
    #: 发送失败时的本地队列上限（超过丢弃最旧事件）。
    queue_size: int = 64


@dataclass
class UsbConfig:
    """USB（默认通道）参数。

    协议文档没有给出 USB 虚拟串口的波特率，所以默认 `baudrate=0` 表示
    连接时自动试探；已知具体参数时填上可省掉 1~5 秒的试探时间。
    """

    #: 串口名，如 "COM5"；None 表示自动选择（优先识别到的 USB 转串口）
    port: Optional[str] = None
    #: 自动选口时的名称/描述关键字（如 "CH340"、"Silicon"、"USB Serial"）
    port_hint: Optional[str] = None
    #: 0 表示自动试探（见 protocol.USB_BAUD_CANDIDATES）
    baudrate: int = 0
    #: 数据形态：auto（自动识别） | binary（只认二进制帧） | ascii（只认明文块）
    format: str = "auto"
    #: ASCII 明文块的单位换算：文档 3.2 注 2 说明 USB 单次采集返回的是
    #: **kPa**，而内置阈值以原始 ADC 计数为量纲。默认 x100，使
    #: activate_abs=300 对应 3 kPa、heavy_total=7000 对应 70 kPa。
    ascii_scale: float = 100.0
    #: 明文块空闲多久算「收完了」（秒）。32x32 明文在 115200 波特率下
    #: 要传 0.6 秒，所以不要设得太小。
    ascii_timeout: float = 0.5
    #: 明文只收到部分行时：False=丢弃（默认，宁缺毋滥）；True=缺行按 0 补齐
    allow_partial: bool = False
    read_timeout: float = 0.05
    reconnect: bool = True
    #: 自动试探波特率
    autobaud: bool = True
    #: DTR 置位。设备一连上就反复重启 / 收不到数据时设为 False
    dtr: bool = True
    #: 独占打开串口（Windows 下拒绝其它程序同时占用）
    exclusive: bool = True


@dataclass
class AppConfig:
    filter: FilterConfig = field(default_factory=FilterConfig)
    gesture: GestureConfig = field(default_factory=GestureConfig)
    mapping: MappingConfig = field(default_factory=MappingConfig)
    usb: UsbConfig = field(default_factory=UsbConfig)
    robot_gesture: RobotGestureConfig = field(default_factory=RobotGestureConfig)
    robot_link: RobotLinkConfig = field(default_factory=RobotLinkConfig)
    #: 默认通道：usb（本程序默认） | ble | mock
    transport_kind: str = "usb"
    #: GUI/会话启动后是否自动进入零点校准流程；默认关闭，避免启动即长时间等待静置。
    auto_calibrate_on_start: bool = False
    #: 连接后自动下发的设备参数
    device_threshold: int = P.DEFAULT_THRESHOLD
    device_array: tuple = (P.DEFAULT_ROWS, P.DEFAULT_COLS)
    #: 设备上行通道（0x0D 指令参数）：默认 USB，与 transport_kind 保持一致。
    #: ⚠ 设备出厂默认蓝牙且掉电后自动恢复蓝牙，所以每次连接都会重发该指令。
    device_mode: int = P.DEFAULT_TRANSPORT_MODE
    #: 采集方式：continuous（0x01，设备持续推送） | single（0x02，上位机轮询）
    acquire: str = "continuous"
    single_poll_hz: float = 30.0  # single 模式的轮询频率
    invert_grid_x: bool = False  # 阵列行列方向与实际不符时翻转
    invert_grid_y: bool = False

    # -- 序列化 ------------------------------------------------------------ #
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "AppConfig":
        cfg = cls()
        sections = ("filter", "gesture", "mapping", "usb", "robot_gesture", "robot_link")
        for section_name in sections:
            section = data.get(section_name) or {}
            target = getattr(cfg, section_name)
            valid = {f.name for f in fields(target)}
            for key, value in section.items():
                if key in valid:
                    setattr(target, key, value)
        top_valid = {f.name for f in fields(cls)}
        for key, value in data.items():
            if key in sections:
                continue
            if key in top_valid:
                setattr(cfg, key, value)
        cfg.device_array = tuple(cfg.device_array)
        return cfg

    def save(self, path: str) -> None:
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(self.to_dict(), fh, ensure_ascii=False, indent=2)

    @classmethod
    def load(cls, path: str) -> "AppConfig":
        with open(path, "r", encoding="utf-8") as fh:
            return cls.from_dict(json.load(fh))

    @classmethod
    def load_optional(cls, path: Optional[str]) -> "AppConfig":
        if not path:
            return cls()
        try:
            return cls.load(path)
        except FileNotFoundError:
            return cls()
