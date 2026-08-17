# -*- coding: utf-8 -*-
"""MotorScope 原生版：数据缓冲（环形 + 最新帧）+ 暂停/回看导航 + 日志。
纯 Python，不依赖 Qt，便于单测。
导航模型（与 web 版一致）：
  follow_live=True  实时跟随：push 写入缓冲并裁剪；
  follow_live=False 暂停/回看：缓冲写入与裁剪全部冻结，view_frame 由 view_end 时间插值。
view_frame：当前"查看时刻"帧（实时=latest；回看=按时间插值，字段与 FocFrame 对齐）。
日志：logs 为 (seq, text) 的 deque（上限 2000），由 GUI 定时器批量刷新面板。"""
from collections import deque
from math import pi

RAD2DEG = 180.0 / pi
MRAD2DEG = RAD2DEG / 1000.0
DEG2MRAD = 1000.0 * pi / 180.0     # deg -> mrad


class ViewFrame:
    """回看历史时按时间插值出的帧；字段与 motor_scope.FocFrame 对齐（协议单位）。
    mode 取最近采样，phase/sync 无历史列，置 0（状态栏仍用 latest 显示）。"""
    __slots__ = ("mode", "phase", "sync", "rotor_mrad", "theta_mrad",
                 "iq_ma", "id_ma", "vq_mv", "vd_mv", "spd_rpm", "freq_cHz",
                 "diff_mrad", "ms", "mech_mrad", "is_ma", "is_angle_mrad",
                 "v_mv", "v_angle_mrad", "theta_mech_mrad", "cnt")

    def __init__(self, **kw):
        for k in self.__slots__:
            setattr(self, k, kw.get(k, 0))


class Hub:
    """按时间裁剪的环形缓冲：每帧把显示所需字段按"度/rad/mA/mV"换算后入队。
    与 web 版 scopeAppend 的换算一致（mrad->deg、mrad->rad、mA/mV 原值）。
    latest 为原始单位 FocFrame（mrad/mA/mV）；各 deque 为显示换算值（deg/rad/mA/mV）。
    cur_max 为电流显示量程（单调历史最大值，初始 500mA，push 时随 iq/id/is 更新）。
    线程契约：push/log 由主线程（GUI 槽）调用；显示侧读取可容忍撕裂（不加锁）。"""

    KEEP_SEC = 20.0          # 保留时长（s）
    MAXLEN = 30000           # 最大点数（与 web 版 SCOPE_CAP 一致）
    LOG_MAX = 2000           # 日志面板最大条数（与 web 版一致）

    def __init__(self):
        self.t = deque(maxlen=self.MAXLEN)
        self.mode = deque(maxlen=self.MAXLEN)
        self.iq = deque(maxlen=self.MAXLEN)
        self.id = deque(maxlen=self.MAXLEN)
        self.vd = deque(maxlen=self.MAXLEN)
        self.vq = deque(maxlen=self.MAXLEN)
        self.spd = deque(maxlen=self.MAXLEN)
        self.freq = deque(maxlen=self.MAXLEN)
        self.diff_rad = deque(maxlen=self.MAXLEN)
        self.rotor_deg = deque(maxlen=self.MAXLEN)
        self.theta_deg = deque(maxlen=self.MAXLEN)
        self.mech_deg = deque(maxlen=self.MAXLEN)
        self.theta_mech_deg = deque(maxlen=self.MAXLEN)
        self.is_ma = deque(maxlen=self.MAXLEN)
        self.is_ang_deg = deque(maxlen=self.MAXLEN)
        self.v_mv = deque(maxlen=self.MAXLEN)
        self.v_ang_deg = deque(maxlen=self.MAXLEN)
        self.cnt = deque(maxlen=self.MAXLEN)
        self.latest = None
        self.frames_total = 0
        self.cur_max = 500.0            # 电流显示量程单调最大值（mA）
        self.last_frame_wall = 0.0
        # ---- 暂停 / 回看导航 ----
        self.follow_live = True         # False = 暂停/回看：缓冲写入与裁剪冻结
        self.window_sec = 1.0           # 示波器窗口宽度（0.1 ~ 20s）
        self.view_end = 0.0             # 回看时间戳（墙钟秒）
        self.view_frame = None          # 当前"查看时刻"帧（实时=latest，回看=插值）
        # ---- 日志 ----
        self.logs = deque(maxlen=self.LOG_MAX)   # (seq, text)
        self.log_seq = 0
        self.logs_dirty = False

    # ---------------- 缓冲写入 ----------------
    def _deques(self):
        return (self.t, self.mode, self.iq, self.id, self.vd, self.vq, self.spd,
                self.freq, self.diff_rad, self.rotor_deg, self.theta_deg,
                self.mech_deg, self.theta_mech_deg, self.is_ma, self.is_ang_deg,
                self.v_mv, self.v_ang_deg, self.cnt)

    def _popleft_all(self):
        for dq in self._deques():
            if dq:
                dq.popleft()

    def push(self, fr, wall_t):
        """fr: motor_scope.FocFrame；wall_t: 接收时间（秒，须单调——内部会钳制倒流）。
        暂停/回看（follow_live=False）时缓冲写入与裁剪全部冻结：
        仅更新帧龄与计数（避免误报数据中断），latest/曲线不更新。"""
        if wall_t < self.last_frame_wall:
            wall_t = self.last_frame_wall
        self.last_frame_wall = wall_t
        self.frames_total += 1
        if not self.follow_live:
            return
        while self.t and self.t[0] < wall_t - self.KEEP_SEC:
            self._popleft_all()
        self.t.append(wall_t)
        self.mode.append(fr.mode)
        self.iq.append(fr.iq_ma)
        self.id.append(fr.id_ma)
        self.vd.append(fr.vd_mv)
        self.vq.append(fr.vq_mv)
        self.spd.append(fr.spd_rpm)
        self.freq.append(fr.freq_cHz)
        self.diff_rad.append(fr.diff_mrad / 1000.0)
        self.rotor_deg.append(fr.rotor_mrad * MRAD2DEG)
        self.theta_deg.append(fr.theta_mrad * MRAD2DEG)
        self.mech_deg.append(fr.mech_mrad * MRAD2DEG)
        self.theta_mech_deg.append(fr.theta_mech_mrad * MRAD2DEG)
        self.is_ma.append(fr.is_ma)
        self.is_ang_deg.append(fr.is_angle_mrad * MRAD2DEG)
        self.v_mv.append(fr.v_mv)
        self.v_ang_deg.append(fr.v_angle_mrad * MRAD2DEG)
        self.cnt.append(fr.cnt)
        self.latest = fr
        self.cur_max = max(self.cur_max, abs(fr.iq_ma), abs(fr.id_ma), fr.is_ma)

    def last_t(self):
        return self.t[-1] if self.t else 0.0

    def first_t(self):
        return self.t[0] if self.t else 0.0

    def window_bounds(self):
        """返回 (minT, maxT) 或 None（无数据）。"""
        if not self.t:
            return None
        return self.t[0], self.t[-1]

    # ---------------- 暂停 / 回看 ----------------
    def pause(self):
        """暂停冻结：停在当前最新时刻（缓冲/裁剪/最新帧全停）。"""
        self.follow_live = False
        if self.t:
            self.view_end = self.t[-1]

    def live(self):
        """回到实时。"""
        self.follow_live = True

    def refresh_view(self):
        """计算当前"查看时刻"帧：实时=latest；回看=view_end 时间插值。"""
        if self.follow_live:
            self.view_frame = self.latest
            return
        if not self.t:
            self.view_frame = self.latest
            return
        tv = min(max(self.view_end, self.t[0]), self.t[-1])
        self.view_frame = self._interp(tv)

    # ---------------- 日志 ----------------
    def log(self, text):
        """追加一行日志（MOTF 节流行或固件日志行），标记面板需刷新。"""
        self.log_seq += 1
        self.logs.append((self.log_seq, text))
        self.logs_dirty = True

    def clear_logs(self):
        """清空日志面板缓冲（不删 history 文件）。"""
        self.logs.clear()
        self.log_seq = 0
        self.logs_dirty = True

    # ---------------- 时间插值（回看） ----------------
    def _lower_bound(self, t):
        tq = self.t
        lo, hi = 0, len(tq)
        while lo < hi:
            mid = (lo + hi) // 2
            if tq[mid] < t:
                lo = mid + 1
            else:
                hi = mid
        return lo

    def _interp(self, t):
        n = len(self.t)
        if n == 0:
            return None
        lo = self._lower_bound(t)
        if lo == 0:
            lo = 1
        a, b = lo - 1, min(lo, n - 1)
        ta, tb = self.t[a], self.t[b]
        f = (t - ta) / ((tb - ta) or 1.0)

        def L(dq):
            va, vb = dq[a], dq[b]
            return va + (vb - va) * f

        mode_val = int(round(self.mode[lo if lo < n else n - 1]))
        return ViewFrame(
            mode=mode_val, phase=0, sync=0,
            rotor_mrad=L(self.rotor_deg) * DEG2MRAD,
            theta_mrad=L(self.theta_deg) * DEG2MRAD,
            iq_ma=L(self.iq), id_ma=L(self.id),
            vq_mv=L(self.vq), vd_mv=L(self.vd),
            spd_rpm=L(self.spd), freq_cHz=L(self.freq),
            diff_mrad=L(self.diff_rad) * 1000.0,
            ms=0,
            mech_mrad=L(self.mech_deg) * DEG2MRAD,
            is_ma=L(self.is_ma),
            is_angle_mrad=L(self.is_ang_deg) * DEG2MRAD,
            v_mv=L(self.v_mv),
            v_angle_mrad=L(self.v_ang_deg) * DEG2MRAD,
            theta_mech_mrad=L(self.theta_mech_deg) * DEG2MRAD,
            cnt=int(round(L(self.cnt))),
        )