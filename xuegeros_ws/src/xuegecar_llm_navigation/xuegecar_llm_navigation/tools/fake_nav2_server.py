#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Fake Nav2 stack for offline P5 tests.

Provides, on the real topic/service names:

* ``/navigate_to_pose``      (nav2_msgs/action/NavigateToPose)
* ``/compute_path_to_pose``  (nav2_msgs/action/ComputePathToPose)
* ``/global_costmap/get_costmap`` (nav2_msgs/srv/GetCostmap)

Every event is printed as one JSON line (``NAV_GOAL``, ``NAV_CANCELED``,
``NAV_SUCCEEDED``, ``PATH``, ``COSTMAP``) so a scenario script can assert
on it, for example that P5 really cancelled the goal mid-flight.

``--via x,y`` injects a waypoint into the returned path: that is how the
"short bottle the LiDAR cannot see is straight ahead" case is simulated.
"""

import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import Path
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


def yaw_of(pose_stamped):
    quaternion = pose_stamped.pose.orientation
    return math.atan2(2.0 * (quaternion.w * quaternion.z +
                             quaternion.x * quaternion.y),
                      1.0 - 2.0 * (quaternion.y * quaternion.y +
                                   quaternion.z * quaternion.z))


def densify(points, step=0.05):
    if len(points) < 2:
        return list(points)
    dense = [points[0]]
    for index in range(len(points) - 1):
        ax, ay = points[index]
        bx, by = points[index + 1]
        length = math.hypot(bx - ax, by - ay)
        count = max(1, int(math.ceil(length / step)))
        for k in range(1, count + 1):
            t = float(k) / float(count)
            dense.append((ax + (bx - ax) * t, ay + (by - ay) * t))
    return dense


class FakeNav2(Node):
    """Accept-everything Nav2 stand-in with scriptable results."""

    def __init__(self, args):
        super().__init__('fake_nav2_server')
        self.args = args
        self.group = ReentrantCallbackGroup()

        self.nav_server = ActionServer(
            self, NavigateToPose, '/navigate_to_pose',
            execute_callback=self.execute_navigate,
            goal_callback=self.on_goal,
            cancel_callback=self.on_cancel,
            callback_group=self.group)

        self.path_server = ActionServer(
            self, ComputePathToPose, '/compute_path_to_pose',
            execute_callback=self.execute_compute_path,
            callback_group=self.group)

        self.costmap_server = self.create_service(
            GetCostmap, '/global_costmap/get_costmap', self.on_costmap,
            callback_group=self.group)

        self.get_logger().info(
            'fake Nav2 ready | result=%s delay=%.1fs start=(%.2f, %.2f) '
            'via=%s paths=%s' % (args.result, args.delay, args.start[0],
                                 args.start[1], args.via, not args.no_paths))

    # ------------------------------------------------------------- helpers
    @staticmethod
    def emit(event, **fields):
        payload = {'event': event, 't': round(time.time(), 3)}
        payload.update(fields)
        print(json.dumps(payload, ensure_ascii=False), flush=True)

    # -------------------------------------------------------- navigate
    def on_goal(self, goal_request):
        pose = goal_request.pose
        self.emit('NAV_GOAL_REQUEST',
                  frame=pose.header.frame_id,
                  x=round(pose.pose.position.x, 4),
                  y=round(pose.pose.position.y, 4))
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        self.emit('NAV_CANCEL_REQUEST',
                  x=round(goal_handle.request.pose.pose.position.x, 4),
                  y=round(goal_handle.request.pose.pose.position.y, 4))
        return CancelResponse.ACCEPT

    def execute_navigate(self, goal_handle):
        pose = goal_handle.request.pose
        x = pose.pose.position.x
        y = pose.pose.position.y
        self.emit('NAV_GOAL', frame=pose.header.frame_id,
                  x=round(x, 4), y=round(y, 4),
                  yaw_deg=round(math.degrees(yaw_of(pose)), 2))

        deadline = time.time() + max(0.0, self.args.delay)
        while time.time() < deadline:
            if goal_handle.is_cancel_requested:
                self.emit('NAV_CANCELED', x=round(x, 4), y=round(y, 4))
                goal_handle.canceled()
                return NavigateToPose.Result()
            time.sleep(0.05)

        if goal_handle.is_cancel_requested:
            self.emit('NAV_CANCELED', x=round(x, 4), y=round(y, 4))
            goal_handle.canceled()
            return NavigateToPose.Result()

        if self.args.result == 'abort':
            self.emit('NAV_ABORTED', x=round(x, 4), y=round(y, 4))
            goal_handle.abort()
        else:
            self.emit('NAV_SUCCEEDED', x=round(x, 4), y=round(y, 4))
            goal_handle.succeed()
        return NavigateToPose.Result()

    # ------------------------------------------------------ compute path
    def execute_compute_path(self, goal_handle):
        if self.args.no_paths:
            self.emit('PATH_REFUSED',
                      x=round(goal_handle.request.goal.pose.position.x, 4),
                      y=round(goal_handle.request.goal.pose.position.y, 4))
            goal_handle.abort()
            return ComputePathToPose.Result()

        goal_pose = goal_handle.request.goal
        waypoints = [tuple(self.args.start)]
        if self.args.via:
            waypoints.append(tuple(self.args.via))
        waypoints.append((goal_pose.pose.position.x,
                          goal_pose.pose.position.y))

        path = Path()
        path.header.frame_id = goal_pose.header.frame_id or 'map'
        for x, y in densify(waypoints, 0.05):
            pose = PoseStamped()
            pose.header.frame_id = path.header.frame_id
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.emit('PATH', waypoints=[[round(x, 3), round(y, 3)]
                                     for x, y in waypoints],
                  poses=len(path.poses))
        goal_handle.succeed()
        result = ComputePathToPose.Result()
        result.path = path
        return result

    # ---------------------------------------------------------- costmap
    def on_costmap(self, request, response):
        resolution = self.args.costmap_resolution
        size = self.args.costmap_size
        origin = self.args.costmap_origin
        data = [0] * (size * size)
        for x, y in (self.args.lethal or []):
            mx = int((x - origin[0]) / resolution)
            my = int((y - origin[1]) / resolution)
            if 0 <= mx < size and 0 <= my < size:
                data[my * size + mx] = 254

        response.map.header.frame_id = 'map'
        metadata = response.map.metadata
        metadata.resolution = float(resolution)
        metadata.size_x = int(size)
        metadata.size_y = int(size)
        metadata.origin.position.x = float(origin[0])
        metadata.origin.position.y = float(origin[1])
        metadata.origin.orientation.w = 1.0
        metadata.layer = 'costmap'
        response.map.data = data
        self.emit('COSTMAP', size=size, resolution=resolution,
                  lethal=[[float(a), float(b)]
                          for a, b in (self.args.lethal or [])])
        return response


def parse_pair(text, cast=float):
    parts = str(text).split(',')
    if len(parts) != 2:
        raise ValueError('expected x,y')
    return (cast(parts[0]), cast(parts[1]))


def main():
    parser = argparse.ArgumentParser(description='fake Nav2 server')
    parser.add_argument('--result', choices=['success', 'abort'],
                        default='success')
    parser.add_argument('--delay', type=float, default=2.0,
                        help='seconds before the navigation result')
    parser.add_argument('--start', type=lambda t: parse_pair(t), default=None,
                        help='robot start pose for the fake path, x,y')
    parser.add_argument('--via', type=lambda t: parse_pair(t), default=None,
                        help='extra waypoint injected into the fake path')
    parser.add_argument('--no-paths', action='store_true',
                        help='make ComputePathToPose always fail')
    parser.add_argument('--costmap-size', type=int, default=200)
    parser.add_argument('--costmap-resolution', type=float, default=0.05)
    parser.add_argument('--costmap-origin', type=lambda t: parse_pair(t),
                        default=None)
    parser.add_argument('--lethal', type=lambda t: parse_pair(t),
                        action='append', default=None,
                        help='lethal costmap cell in world coordinates x,y')
    args = parser.parse_args()

    args.start = args.start or (0.0, 0.0)
    args.costmap_origin = args.costmap_origin or (-5.0, -5.0)

    rclpy.init()
    node = FakeNav2(args)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
