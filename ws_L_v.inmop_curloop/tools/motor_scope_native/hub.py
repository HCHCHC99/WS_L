# -*- coding: utf-8 -*-
"""MotorScope 原生版：数据缓冲（环形 + 最新帧）。纯 Python，不依赖 Qt，便于单测。"""
from collections import deque
from math import pi

RAD2DEG = 180.0 / pi
MRAD2DEG = RAD2DEG / 1000.0


class Hub:
    """按时间裁剪的环形缓冲：每帧把显示所需字段按"度/rad/mA/mV"换算后入队。
    与 web 版 scopeAppend 的换算一致（mrad->deg、mrad->rad、mA/mV 原值）。
    latest 为原始单位 FocFrame（mrad/mA/mV）；各 deque 为显示换算值（deg/rad/mA/mV）。
    cur_max 为电流显示量程（单调历史最大值，初始 500mA，push 时随 iq/id/is 更新）。
    线程契约：push 由主线程（GUI 槽 _on_frame）调用；显示侧读取可容忍撕裂（Phase 1 不加锁）。"""

    KEEP_SEC = 20.0          # 保留时长（s）
    MAXLEN = 30000           # 最大点数（与 web 版 SCOPE_CAP 一致）

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
        """fr: motor_scope.FocFrame；wall_t: 接收时间（秒，须单调——内部会钳制倒流）。"""
        if wall_t < self.last_frame_wall:
            wall_t = self.last_frame_wall
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
        self.frames_total += 1
        self.last_frame_wall = wall_t
        self.cur_max = max(self.cur_max, abs(fr.iq_ma), abs(fr.id_ma), fr.is_ma)

    def last_t(self):
        return self.t[-1] if self.t else 0.0

    def first_t(self):
        return self.t[0] if self.t else 0.0