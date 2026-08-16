# -*- coding: utf-8 -*-
import struct, sys, io, math
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
import motor_scope as ms

# 1) 文本帧（21 字段：...theta_mech, cnt）
line = ("MOTF,2,3,3141,6283,-2000,-25,5000,400,3000,1,250,2000,"
        "123456,78540,2061,3217,5024,-1234,628,12345\r\n")
fr = ms.parse_text_line(line)
assert fr is not None
assert (fr.is_ma, fr.is_angle_mrad, fr.v_mv, fr.v_angle_mrad,
        fr.theta_mech_mrad, fr.cnt) == (2061.0, 3217.0, 5024.0, -1234.0, 628.0, 12345), fr
print("text 21-field OK:", fr)

# 2) 旧 15 字段文本帧兼容
old = ms.parse_text_line(
    "MOTF,2,3,3141,6283,-2000,-25,5000,400,3000,1,250,2000,123456,78540\r\n")
assert old is not None and old.is_ma == 0.0 and old.v_mv == 0.0
print("old 15-field compat OK")

# 3) 二进制帧 76B round-trip
assert ms._BIN_SIZE == 76, ms._BIN_SIZE
packed = struct.pack(ms._BIN_FMT, b"MOTF", 123456, 3141, 6283, -2000, -25,
                     5000, 400, 3000, 250, 2000, 2, 3, 1, 0,
                     78540, 2061, 3217, 5024, -1234, 628, 12345)
assert len(packed) == 76
fb = ms.parse_binary(packed)
assert (fb.is_ma, fb.is_angle_mrad, fb.v_mv, fb.v_angle_mrad,
        fb.theta_mech_mrad, fb.cnt) == (2061.0, 3217.0, 5024.0, -1234.0, 628.0, 12345)
print("binary 76B OK:", fb)

# 4) to_list 长度与前端索引一致（latest[14..19]）
lst = fr.to_list()
assert len(lst) == 20 and lst[13] == 78540 and lst[14] == 2061 and lst[18] == 628 and lst[19] == 12345, lst
print("to_list 20 fields OK")

# 5) 仿真：确定性时间驱动 0..10s，覆盖 phase 0..3（theta_mech 连续无跳变）
sim = ms.SimFoc(sample_hz=200)
prev = None
phases = set()
for i in range(2001):
    f = sim.next_frame(t=i / 200.0)
    phases.add(f.phase)
    assert f.is_ma >= 0.0 and f.v_mv >= 0.0
    if f.phase in (2, 3):
        assert f.cnt >= 0
    if prev and f.phase in (2, 3) and prev.phase in (2, 3):
        assert abs(f.theta_mech_mrad - prev.theta_mech_mrad) < 500.0
        assert f.theta_mech_mrad > 0.0
    prev = f
assert phases >= {0, 1, 2, 3}, phases   # 必须真的跑到 phase 2/3，否则上面断言是空测试
print("sim is/v/theta_mech OK")
print("ALL TESTS PASSED")