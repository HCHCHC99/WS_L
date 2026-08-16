# -*- coding: utf-8 -*-
import struct, sys, io, math
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
import motor_scope as ms

# 1) 文本帧（20 字段：...ms, mech, is_ma, is_angle, v_mv, v_angle, theta_mech）
line = ("MOTF,2,3,3141,6283,-2000,-25,5000,400,3000,1,250,2000,"
        "123456,78540,2061,3217,5024,-1234,628\r\n")
fr = ms.parse_text_line(line)
assert fr is not None
assert (fr.is_ma, fr.is_angle_mrad, fr.v_mv, fr.v_angle_mrad,
        fr.theta_mech_mrad) == (2061.0, 3217.0, 5024.0, -1234.0, 628.0), fr
print("text 20-field OK:", fr)

# 2) 旧 15 字段文本帧兼容
old = ms.parse_text_line(
    "MOTF,2,3,3141,6283,-2000,-25,5000,400,3000,1,250,2000,123456,78540\r\n")
assert old is not None and old.is_ma == 0.0 and old.v_mv == 0.0
print("old 15-field compat OK")

# 3) 二进制帧 72B round-trip
assert ms._BIN_SIZE == 72, ms._BIN_SIZE
packed = struct.pack(ms._BIN_FMT, b"MOTF", 123456, 3141, 6283, -2000, -25,
                     5000, 400, 3000, 250, 2000, 2, 3, 1, 0,
                     78540, 2061, 3217, 5024, -1234, 628)
assert len(packed) == 72
fb = ms.parse_binary(packed)
assert (fb.is_ma, fb.is_angle_mrad, fb.v_mv, fb.v_angle_mrad,
        fb.theta_mech_mrad) == (2061.0, 3217.0, 5024.0, -1234.0, 628.0)
print("binary 72B OK:", fb)

# 4) to_list 长度与前端索引一致（latest[14..18]）
lst = fr.to_list()
assert len(lst) == 19 and lst[13] == 78540 and lst[14] == 2061 and lst[18] == 628, lst
print("to_list 19 fields OK")

# 5) 仿真：is/v 幅值非负、theta_mech 连续无跳变
sim = ms.SimFoc(sample_hz=200)
prev = None
for i in range(2000):
    f = sim.next_frame()
    assert f.is_ma >= 0.0 and f.v_mv >= 0.0
    if prev and f.phase in (2, 3) and prev.phase in (2, 3):
        assert abs(f.theta_mech_mrad - prev.theta_mech_mrad) < 500.0
        assert f.theta_mech_mrad > 0.0
    prev = f
print("sim is/v/theta_mech OK")
print("ALL TESTS PASSED")