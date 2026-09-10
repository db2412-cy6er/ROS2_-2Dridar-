#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P5 interactive terminal: type Chinese, the robot answers / navigates.

Usage::

    ros2 run xuegecar_llm_navigation llm_console

Type a question, press Enter.  ``exit`` / ``quit`` / Ctrl+C leave.
The terminal only publishes on /llm/query and displays /llm/response and
/llm/task_status, so it can run anywhere on the ROS graph.
"""

import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

BANNER = (
    '\nP5 语义机器人终端已启动。\n'
    '可以问：小车前方有几个瓶子？/ 瓶子都在哪里？/ 第三远的瓶子在哪里？\n'
    '也可以说：请去最近的瓶子旁边 / 请去第三远的瓶子的右边 / 停下\n'
    '输入 exit 退出。\n'
)


class LLMConsole(Node):
    """Thin console around /llm/query, /llm/response, /llm/task_status."""

    def __init__(self):
        super().__init__('llm_console')
        self.query_pub = self.create_publisher(String, '/llm/query', 10)
        self.response_sub = self.create_subscription(
            String, '/llm/response', self.response_callback, 10)
        self.status_sub = self.create_subscription(
            String, '/llm/task_status', self.status_callback, 10)
        self._stop = False

    # ------------------------------------------------------------- printing
    @staticmethod
    def response_callback(msg):
        print('\n机器人 > %s\n' % msg.data)

    def status_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except ValueError:
            print('[状态] %s' % msg.data)
            return
        state = payload.pop('state', '?')
        payload.pop('stamp', None)
        extras = ' '.join(
            '%s=%s' % (key, value) for key, value in payload.items())
        print('[状态] %s %s' % (state, extras))

    # ---------------------------------------------------------- input loop
    def input_loop(self):
        print(BANNER)
        while rclpy.ok() and not self._stop:
            try:
                text = input('你 > ').strip()
            except (EOFError, KeyboardInterrupt):
                break
            if text.lower() in ('exit', 'quit'):
                break
            if not text:
                continue
            message = String()
            message.data = text
            self.query_pub.publish(message)
        self._stop = True
        print('\n已退出 P5 终端。')


def main(args=None):
    rclpy.init(args=args)
    node = LLMConsole()

    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    try:
        node.input_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spinner.join(timeout=1.0)


if __name__ == '__main__':
    main()
