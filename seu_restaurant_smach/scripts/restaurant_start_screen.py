#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""餐厅赛题启动前的终端等待界面。"""

import sys


GREEN = "\033[92m"
BOLD = "\033[1m"
RESET = "\033[0m"


def _color(text, *styles):
    """只在交互式终端里输出颜色，避免重定向日志出现 ANSI 字符。"""
    if not getattr(sys.stdout, "isatty", lambda: False)():
        return text
    return "{}{}{}".format("".join(styles), text, RESET)


def _print_centered(text, width):
    print(text.center(width))


def wait_for_restaurant_start():
    """展示启动界面，并等待按下 ENTER 后继续执行赛题。"""
    width = 72
    divider = "=" * width
    title = "SEU Robocup@home Restaurant 赛题"
    subtitle = "press ENTER to start"

    print()
    print(_color(divider, GREEN))
    print()
    print(_color(title.center(width), GREEN, BOLD))
    _print_centered(subtitle, width)
    print()
    print(_color(divider, GREEN))
    print()

    try:
        input(">>> ")
    except EOFError:
        # 非交互式运行时没有 stdin 可等，直接继续，避免阻塞自动化启动。
        print()
