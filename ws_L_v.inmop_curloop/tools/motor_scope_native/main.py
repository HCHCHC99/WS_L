# -*- coding: utf-8 -*-
"""MotorScope 原生版入口：python main.py [--mock] [--device ...]"""
import argparse
import os
import sys

from PySide6.QtWidgets import QApplication

from app import MainWindow
from data import DataThread
from hub import Hub


def main():
    ap = argparse.ArgumentParser(description="MotorScope 原生版（PySide6）")
    ap.add_argument("--mock", action="store_true",
                    help="开发自测：用本地生成的 MOTF 帧喂界面（非仿真模式）")
    ap.add_argument("--mock-rate", type=int, default=200)
    ap.add_argument("--device", default="HC32F460")
    ap.add_argument("--speed-khz", type=int, default=1000)
    ap.add_argument("--channel", type=int, default=0)
    ap.add_argument("--rtt-addr", type=lambda x: int(x, 0), default=0)
    ap.add_argument("--rtt-ram-base", type=lambda x: int(x, 0), default=0x1FFF8000)
    ap.add_argument("--rtt-ram-size", type=lambda x: int(x, 0), default=0x2F000)
    args = ap.parse_args()

    app = QApplication(sys.argv)
    hub = Hub()
    thread = DataThread(mock=args.mock, mock_rate=args.mock_rate,
                        device=args.device, speed_khz=args.speed_khz,
                        channel=args.channel, rtt_addr=args.rtt_addr,
                        ram_base=args.rtt_ram_base, ram_size=args.rtt_ram_size)
    win = MainWindow(hub, thread)
    win.resize(1400, 900)
    win.show()
    thread.start()
    rc = app.exec()
    thread.requestInterruption()
    if not thread.wait(2000):
        os._exit(rc)   # 线程卡在 J-Link 阻塞调用：硬退出，跳过运行中 QThread 的析构
    sys.exit(rc)


if __name__ == "__main__":
    main()