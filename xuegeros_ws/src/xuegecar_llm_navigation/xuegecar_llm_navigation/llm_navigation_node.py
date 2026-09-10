#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P5 LLM semantic query + navigation node (xuegecar_llm_navigation).

Chain (see report P5)::

    /llm/query  (natural language)
        -> DeepSeek   : intent only, structured JSON, never coordinates
        -> P5 local   : deterministic count / select / geometry
                        reading /semantic/dynamic_map  (P4)
        -> /llm/response      (deterministic fact, rephrased by DeepSeek)
        -> /llm/task_status   (state machine, structured)
        -> Nav2 ComputePathToPose -> clearance audit -> NavigateToPose

Safety: a planned path is audited against every semantic object and (when
enabled) against the global costmap before the goal is sent, and a 10 Hz
watchdog cancels the task if the real clearance ever drops below the
configured stop threshold.

This node never publishes /cmd_vel and never touches YOLO or P3.
"""

import copy
import json
import math
import os
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.srv import GetCostmap
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from xuegecar_llm_navigation import clearance, intent_schema, query_engine
from xuegecar_llm_navigation.deepseek_client import (DeepSeekClient,
                                                     DeepSeekError)


class LLMNavigationNode(Node):
    """Semantic query + navigation front-end for the xuegecar."""

    def __init__(self):
        super().__init__('llm_navigation_node')

        defaults = [
            ('dynamic_map_topic', '/semantic/dynamic_map'),
            ('query_topic', '/llm/query'),
            ('response_topic', '/llm/response'),
            ('status_topic', '/llm/task_status'),
            ('navigate_action', '/navigate_to_pose'),
            ('compute_path_action', '/compute_path_to_pose'),
            ('global_costmap_service', '/global_costmap/get_costmap'),
            ('map_frame', 'map'),
            ('robot_frame', 'base_footprint'),
            ('api_base_url', 'https://api.deepseek.com'),
            ('api_model', 'deepseek-v4-flash'),
            ('api_models_fallback', 'deepseek-v4-flash,deepseek-v4-pro'),
            ('api_key_env', 'DEEPSEEK_API_KEY'),
            ('api_key_file', ''),
        ]
        for name, value in defaults:
            self.declare_parameter(name, value)

        numeric = [
            ('api_timeout_sec', 15.0),
            ('query_deadline_sec', 40.0),
            ('map_timeout_sec', 5.0),
            # 停车/接近距离：必须与 config/p5_llm_navigation.yaml 一致
            # （0.75 的旧默认值会让小车在 75cm 外回答"已经在旁边"、一步不走）
            ('goal_offset_m', 0.30),
            ('min_goal_offset_m', 0.28),
            ('min_approach_distance_m', 0.30),
            ('min_surface_clearance_m', 0.05),
            ('robot_physical_radius_m', 0.12),
            ('assumed_object_radius_m', 0.04),
            ('clearance_warn_surface_m', 0.05),
            ('safety_stop_surface_m', 0.00),
            ('watchdog_rate', 10.0),
            ('nav_timeout_sec', 120.0),
        ]
        for name, value in numeric:
            self.declare_parameter(name, value)

        integers = [
            ('path_max_cell_cost', 253),
        ]
        for name, value in integers:
            self.declare_parameter(name, value)

        booleans = [
            ('enable_json_mode', True),
            ('disable_thinking', True),
            ('auto_execute_navigation', True),
            ('single_flight', True),
            ('enable_path_precheck', True),
            ('path_check_use_costmap', True),
            ('allow_relation_fallback', True),
        ]
        for name, value in booleans:
            self.declare_parameter(name, value)

        self._read_parameters()

        self.api_key = self._load_api_key()
        if not self.api_key:
            self.get_logger().error(
                'DeepSeek API key not found: set %s (queries will fail)'
                % self.api_key_env)

        self.client = DeepSeekClient(
            api_key=self.api_key,
            base_url=self.api_base_url,
            model=self.api_model,
            timeout_sec=self.api_timeout_sec,
            models_fallback=self.api_models_fallback,
            enable_json_mode=self.enable_json_mode,
            disable_thinking=self.disable_thinking,
            logger=self.get_logger(),
        )

        # ---------------------------------------------------------- state
        self.latest_map = None
        self.latest_map_rx = 0.0
        self.map_lock = threading.Lock()

        self.goal_lock = threading.Lock()
        self.active_target_id = None
        self.active_goal_handle = None
        self.active_seq = 0
        self.nav_start_time = None
        self.cancel_reason = None
        self.min_clearance_observed = None
        self.warn_published = False

        self.busy = False
        self.busy_lock = threading.Lock()

        # ------------------------------------------------------- interfaces
        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.map_sub = self.create_subscription(
            String, self.dynamic_map_topic, self.dynamic_map_callback,
            transient_qos)
        self.query_sub = self.create_subscription(
            String, self.query_topic, self.query_callback, 10)
        self.response_pub = self.create_publisher(
            String, self.response_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.nav_client = ActionClient(self, NavigateToPose,
                                       self.navigate_action)
        self.path_client = ActionClient(self, ComputePathToPose,
                                        self.compute_path_action)
        self.costmap_client = self.create_client(GetCostmap,
                                                 self.global_costmap_service)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.watchdog_timer = self.create_timer(
            1.0 / max(1.0, self.watchdog_rate), self.watchdog)

        threading.Thread(target=self._log_startup, daemon=True).start()

        self.get_logger().info(
            'P5 LLM navigation node started | map=%s query=%s response=%s '
            'status=%s | action=%s configured_model=%s offset=%.2fm '
            'clearance=%.2fm'
            % (self.dynamic_map_topic, self.query_topic,
               self.response_topic, self.status_topic, self.navigate_action,
               self.api_model, self.goal_offset_m,
               self.min_surface_clearance_m))

    # ------------------------------------------------------------ parameters
    def _read_parameters(self):
        def text(name):
            return str(self.get_parameter(name).value)

        def number(name):
            return float(self.get_parameter(name).value)

        def flag(name):
            return bool(self.get_parameter(name).value)

        self.dynamic_map_topic = text('dynamic_map_topic')
        self.query_topic = text('query_topic')
        self.response_topic = text('response_topic')
        self.status_topic = text('status_topic')
        self.navigate_action = text('navigate_action')
        self.compute_path_action = text('compute_path_action')
        self.global_costmap_service = text('global_costmap_service')
        self.map_frame = text('map_frame')
        self.robot_frame = text('robot_frame')
        self.api_base_url = text('api_base_url')
        self.api_model = text('api_model')
        self.api_key_env = text('api_key_env')
        self.api_key_file = text('api_key_file')
        self.api_models_fallback = [
            item.strip()
            for item in text('api_models_fallback').split(',')
            if item.strip()
        ]

        self.api_timeout_sec = number('api_timeout_sec')
        self.query_deadline_sec = number('query_deadline_sec')
        self.map_timeout_sec = number('map_timeout_sec')
        self.goal_offset_m = number('goal_offset_m')
        self.min_goal_offset_m = number('min_goal_offset_m')
        self.min_approach_distance_m = number('min_approach_distance_m')
        self.min_surface_clearance_m = number('min_surface_clearance_m')
        self.robot_physical_radius_m = number('robot_physical_radius_m')
        self.assumed_object_radius_m = number('assumed_object_radius_m')
        self.clearance_warn_surface_m = number('clearance_warn_surface_m')
        self.safety_stop_surface_m = number('safety_stop_surface_m')
        self.path_max_cell_cost = int(self.get_parameter(
            'path_max_cell_cost').value)
        self.watchdog_rate = number('watchdog_rate')
        self.nav_timeout_sec = number('nav_timeout_sec')

        self.enable_json_mode = flag('enable_json_mode')
        self.disable_thinking = flag('disable_thinking')
        self.auto_execute_navigation = flag('auto_execute_navigation')
        self.single_flight = flag('single_flight')
        self.enable_path_precheck = flag('enable_path_precheck')
        self.path_check_use_costmap = flag('path_check_use_costmap')
        self.allow_relation_fallback = flag('allow_relation_fallback')

    def _load_api_key(self):
        """Environment variable first (never store keys in yaml/params)."""
        key = os.getenv(self.api_key_env)
        if key and key.strip():
            return key.strip()
        if self.api_key_file:
            path = os.path.expanduser(self.api_key_file)
            try:
                with open(path, 'r', encoding='utf-8') as handle:
                    return handle.read().strip()
            except OSError as exc:
                self.get_logger().warn('cannot read %s: %s' % (path, exc))

    # --------------------------------------------------------- map snapshot
    def dynamic_map_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError as exc:
            self.get_logger().error('dynamic_map JSON parse error: %s' % exc)
            return
        with self.map_lock:
            self.latest_map = data
            self.latest_map_rx = time.monotonic()
        self._check_active_target(data)

    def get_map_snapshot(self):
        """Return (deep copy of map, age in seconds) or (None, age)."""
        with self.map_lock:
            if self.latest_map is None:
                return None, None
            age = time.monotonic() - self.latest_map_rx
            if age > self.map_timeout_sec:
                return None, age
            return copy.deepcopy(self.latest_map), age

    def robot_pose_from_tf(self):
        """Latest map -> base_footprint pose (used by the 10 Hz watchdog)."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, Time())
        except TransformException:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(2.0 * (rotation.w * rotation.z +
                                rotation.x * rotation.y),
                         1.0 - 2.0 * (rotation.y * rotation.y +
                                      rotation.z * rotation.z))
        return {'x': translation.x, 'y': translation.y, 'yaw': yaw}

    # ----------------------------------------------------- target lifecycle
    def _check_active_target(self, data):
        """Cancel the task when P4 removed the object we are driving to."""
        with self.goal_lock:
            target_id = self.active_target_id
        if target_id is None:
            return
        object_ids = set()
        for obj in (data.get('objects') or []):
            if isinstance(obj, dict):
                object_ids.add(obj.get('id'))
        if target_id in object_ids:
            return
        self.get_logger().warn(
            'active target %s disappeared from the semantic map; cancelling'
            % target_id)
        self.publish_response(
            '目标 %s 已经被移走，当前导航任务已取消。' % target_id)
        self.publish_status('TARGET_REMOVED', {'target_id': target_id})
        self._cancel_active_goal('TARGET_REMOVED')

    def _cancel_active_goal(self, reason):
        with self.goal_lock:
            handle = self.active_goal_handle
            if self.cancel_reason is None:
                self.cancel_reason = reason
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exc:                   # noqa: BLE001
                self.get_logger().warn('cancel_goal failed: %s' % exc)

    def _clear_session(self, seq):
        with self.goal_lock:
            if seq is not None and seq != self.active_seq:
                return False
            self.active_target_id = None
            self.active_goal_handle = None
            self.nav_start_time = None
            self.cancel_reason = None
            self.warn_published = False
            return True

    # -------------------------------------------------------------- watchdog
    def watchdog(self):
        """10 Hz runtime clearance guard (the last line of defence)."""
        with self.goal_lock:
            target_id = self.active_target_id
            started = self.nav_start_time
            seq = self.active_seq
        if target_id is None:
            return

        if (started is not None and self.nav_timeout_sec > 0.0
                and (time.monotonic() - started) > self.nav_timeout_sec):
            self.publish_response('导航到 %s 超时，已取消。' % target_id)
            self.publish_status('NAVIGATION_TIMEOUT', {'target_id': target_id})
            self._cancel_active_goal('NAVIGATION_TIMEOUT')
            return

        pose = self.robot_pose_from_tf()
        with self.map_lock:
            data = self.latest_map
        if pose is None or not data:
            return
        state = query_engine.parse_map_snapshot(data)
        if not state['ok'] or not state['objects']:
            return

        nearest, distance = clearance.nearest_object(
            pose['x'], pose['y'], state['objects'])
        kind, surface = clearance.classify_clearance(
            distance, self.robot_physical_radius_m,
            self.assumed_object_radius_m,
            warn_surface_m=self.clearance_warn_surface_m,
            stop_surface_m=self.safety_stop_surface_m)

        with self.goal_lock:
            if surface is not None:
                if (self.min_clearance_observed is None
                        or surface < self.min_clearance_observed):
                    self.min_clearance_observed = surface

        object_id = nearest.get('id') if nearest else 'unknown'

        if kind == clearance.CLEARANCE_STOP:
            self.get_logger().error(
                'SAFETY STOP: surface clearance to %s is %.3f m'
                % (object_id, surface if surface is not None else 0.0))
            self.publish_response(
                '安全停车：与 %s 的距离过近（表面间隙 %.3f 米），'
                '已取消导航任务。'
                % (object_id, surface if surface is not None else 0.0))
            self.publish_status('SAFETY_STOP', {
                'target_id': target_id,
                'blocking_id': object_id,
                'surface_clearance_m': round(
                    surface if surface is not None else 0.0, 3),
            })
            self._cancel_active_goal('SAFETY_STOP')
            return

        if kind == clearance.CLEARANCE_WARN:
            with self.goal_lock:
                already = self.warn_published
                self.warn_published = True
            if not already:
                self.get_logger().warn(
                    'clearance warning: %.3f m surface gap to %s'
                    % (surface if surface is not None else 0.0, object_id))
                self.publish_status('CLEARANCE_WARNING', {
                    'target_id': target_id,
                    'blocking_id': object_id,
                    'surface_clearance_m': round(
                        surface if surface is not None else 0.0, 3),
                    'seq': seq,
                })

    # ------------------------------------------------------------- outputs
    def publish_response(self, text):
        message = String()
        message.data = text
        self.response_pub.publish(message)
        self.get_logger().info('Response: %s' % text)

    def publish_status(self, state, extra=None):
        message = String()
        payload = {
            'state': state,
            'stamp': round(self.get_clock().now().nanoseconds / 1e9, 3),
        }
        if extra:
            payload.update(extra)
        message.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(message)

    # -------------------------------------------------------------- queries
    def query_callback(self, msg):
        question = (msg.data or '').strip()
        if not question:
            return
        self.get_logger().info('User query: %s' % question)

        with self.busy_lock:
            if self.busy and self.single_flight:
                self.publish_status('BUSY', {'question': question})
                self.publish_response(
                    '上一条指令还在处理中，请稍等一下再说。')
                return
            self.busy = True

        worker = threading.Thread(target=self.process_query,
                                  args=(question,), daemon=True)
        worker.start()

        return ''

    def _log_startup(self):
        """Verify the configured model against GET /models (non blocking)."""
        if not self.api_key:
            return
        try:
            self.client.resolve_model()
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().warn('model discovery failed: %s' % exc)
        if self.client.model != self.api_model:
            self.api_model = self.client.model
        self.get_logger().info('DeepSeek model in use: %s' % self.client.model)

    # ------------------------------------------------------- query pipeline
    def process_query(self, question):
        deadline = time.monotonic() + self.query_deadline_sec
        try:
            self._process_query_inner(question, deadline)
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().error('query handling failed: %s' % exc)
            self.publish_response('处理这句话时出现异常，当前没有执行机器人任务。')
            self.publish_status('QUERY_ERROR', {'error': str(exc)})
        finally:
            with self.busy_lock:
                self.busy = False

    def _process_query_inner(self, question, deadline):
        snapshot, age = self.get_map_snapshot()
        if snapshot is None:
            if age is None:
                text = '当前没有收到有效的实时语义地图，请先确认 P4 正常运行。'
            else:
                text = ('实时语义地图已经 %.1f 秒没有更新，'
                        '请检查 P4 是否正常运行。' % age)
            self.publish_response(text)
            self.publish_status('MAP_NOT_READY', {'age_sec': age})
            return

        state = query_engine.parse_map_snapshot(snapshot)
        if not state['ok']:
            self.publish_response(state['error_cn'])
            self.publish_status('MAP_NOT_READY', {'error': state['error']})
            return

        if state['robot'] is None:
            self.get_logger().warn(
                'dynamic map has no robot pose (TF map->%s missing)'
                % self.robot_frame)
            self.publish_response(
                '当前定位未就绪（没有收到 map 到 %s 的坐标变换），'
                '无法判断目标方位，因此没有执行任何操作。' % self.robot_frame)
            self.publish_status('MAP_NOT_LOCALIZED', {})
            return

        self.publish_status('UNDERSTANDING', {'question': question})

        world = intent_schema.world_summary(state)
        try:
            if time.monotonic() > deadline:
                raise DeepSeekError('query deadline exceeded')
            plan, warnings = self.client.parse_intent(question, world)
        except DeepSeekError as exc:
            self.get_logger().error('DeepSeek intent failed: %s' % exc)
            self.publish_response(
                '大模型接口调用失败，当前没有执行机器人任务。')
            self.publish_status('LLM_ERROR', {'error': str(exc)})
            return

        for warning in warnings:
            self.get_logger().warn('intent warning: %s' % warning)
        self.get_logger().info(
            'LLM plan: %s' % json.dumps(plan, ensure_ascii=False))
        self.publish_status('INTENT', {'plan': plan, 'warnings': warnings})

        intent = plan['intent']
        if intent == 'cancel':
            self._handle_cancel()
        elif intent == 'query_count':
            self._handle_query_count(question, state, plan, deadline)
        elif intent == 'query_objects':
            self._handle_query_objects(question, state, plan, deadline)
        elif intent == 'navigate':
            self._handle_navigate(question, state, plan, deadline)
        else:
            self.publish_response(
                '我理解了这句话，但当前 P5 还没有定义对应的机器人操作，'
                '请换一种说法。')
            self.publish_status('NOT_SUPPORTED', {'plan': plan})

    # ------------------------------------------------------------ answering
    def _answer(self, question, fact, status_state, deadline, extra=None):
        if time.monotonic() <= deadline:
            answer = self.client.phrase(question, fact)
        else:
            self.get_logger().warn('phrasing skipped: query deadline reached')
            answer = fact
        self.publish_response(answer)
        payload = {'fact': fact}
        if extra:
            payload.update(extra)
        self.publish_status(status_state, payload)

    def _handle_cancel(self):
        with self.goal_lock:
            target_id = self.active_target_id
        if target_id is None:
            self.publish_response('当前没有正在执行的导航任务。')
            self.publish_status('NOTHING_TO_CANCEL', {})
            return
        self.publish_response('好的，已经取消前往 %s 的导航任务。' % target_id)
        self.publish_status('CANCELED_BY_USER', {'target_id': target_id})
        self._cancel_active_goal('CANCELED_BY_USER')

    def _handle_query_count(self, question, state, plan, deadline):
        objects = query_engine.filter_objects(
            state['objects'], plan['object_class'], plan['spatial_filter'])
        missing = (state['num_missing_distance']
                   if plan['spatial_filter'] == 'all' else 0)
        fact = query_engine.fact_count(plan['object_class'],
                                       plan['spatial_filter'], len(objects),
                                       missing)
        self._answer(question, fact, 'ANSWERED', deadline,
                     {'count': len(objects)})

    def _handle_query_objects(self, question, state, plan, deadline):
        objects = query_engine.filter_objects(
            state['objects'], plan['object_class'], plan['spatial_filter'])
        if plan['selection'] == 'all':
            missing = (state['num_missing_distance']
                       if plan['spatial_filter'] == 'all' else 0)
            fact = query_engine.fact_objects_all(plan['object_class'],
                                                 objects, missing)
            self._answer(question, fact, 'ANSWERED', deadline,
                         {'count': len(objects)})
            return

        target, reason = query_engine.select_object(
            objects, plan['selection'], plan['rank'], plan['object_id'])
        note = query_engine.rank_cross_check(
            objects, plan['selection'], plan['rank'], plan['object_id'])
        if note:
            self.get_logger().warn(note)
            self.publish_status('RANK_MISMATCH', {'note': note})

        if target is None:
            fact = query_engine.fact_target(
                None, self._reason_cn(reason, plan))
            self._answer(question, fact, 'ANSWERED', deadline,
                         {'select_reason': reason})
            return

        fact = query_engine.fact_target(target)
        self._answer(question, fact, 'ANSWERED', deadline,
                     {'target_id': target['id'], 'select_reason': reason})

    @staticmethod
    def _reason_cn(reason, plan):
        mapping = {
            'NO_OBJECTS': '当前语义地图里没有这个类别的目标。',
            'NO_DISTANCE': '目标存在，但当前无法测距（定位未就绪）。',
            'ID_NOT_FOUND': '没有找到 ID 为 %s 的目标。'
                            % plan.get('object_id', ''),
            'RANK_OUT_OF_RANGE': '当前目标数量不足，没有第 %d 远/近的目标。'
                                 % plan.get('rank', 1),
            'UNKNOWN_SELECTION': '无法理解要选择哪一个目标。',
        }
        return mapping.get(reason, reason)

    # --------------------------------------------------------- navigation
    def _make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        quaternion_z, quaternion_w = query_engine.yaw_to_quaternion(yaw)
        pose.pose.orientation.z = quaternion_z
        pose.pose.orientation.w = quaternion_w
        return pose

    @staticmethod
    def _wait_future(future, timeout_sec):
        """Block a worker thread until an rclpy future completes."""
        event = threading.Event()
        future.add_done_callback(lambda _future: event.set())
        if not event.wait(timeout_sec):
            return None
        try:
            return future.result()
        except Exception:                              # noqa: BLE001
            return None

    def _compute_path(self, x, y, yaw):
        """Ask the Nav2 planner for the path (no motion involved)."""
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = self._make_pose(x, y, yaw)
        goal_msg.use_start = False
        send_future = self.path_client.send_goal_async(goal_msg)
        handle = self._wait_future(send_future, 5.0)
        if handle is None or not handle.accepted:
            return {'ok': False, 'reason': 'PLANNER_REJECTED', 'path': None}
        result = self._wait_future(handle.get_result_async(), 10.0)
        if result is None:
            return {'ok': False, 'reason': 'PLANNER_TIMEOUT', 'path': None}
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            return {'ok': False, 'reason': 'PLANNER_FAILED', 'path': None}
        poses = getattr(result.result, 'path', None)
        if poses is None:
            return {'ok': False, 'reason': 'EMPTY_PATH', 'path': None}
        points = [(pose.pose.position.x, pose.pose.position.y)
                  for pose in poses.poses]
        if len(points) < 2:
            return {'ok': False, 'reason': 'EMPTY_PATH', 'path': None}
        return {'ok': True, 'reason': 'OK', 'path': points}

    def _get_costmap(self):
        """Fetch the global costmap for the lethal-cell part of the audit."""
        if not self.costmap_client.service_is_ready():
            if not self.costmap_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(
                    '%s unavailable; costmap audit skipped'
                    % self.global_costmap_service)
                return None
        request = GetCostmap.Request()
        request.specs.layer = 'costmap'
        response = self._wait_future(
            self.costmap_client.call_async(request), 3.0)
        if response is None:
            return None
        metadata = response.map.metadata
        return {
            'resolution': float(metadata.resolution),
            'origin_x': float(metadata.origin.position.x),
            'origin_y': float(metadata.origin.position.y),
            'width': int(metadata.size_x),
            'height': int(metadata.size_y),
            'data': list(response.map.data),
        }

    def _precheck_goal(self, goal, objects):
        """Plan first, audit the path, only then allow motion."""
        if not self.enable_path_precheck:
            return {'ok': True, 'reason': 'SKIPPED', 'reason_cn': '',
                    'detail': '', 'report': None}

        if not self.path_client.wait_for_server(timeout_sec=2.0):
            return {
                'ok': False,
                'reason': 'PLANNER_UNAVAILABLE',
                'reason_cn': ('导航规划器（%s）当前不可用，'
                              '为安全起见没有发送导航目标。'
                              % self.compute_path_action),
                'detail': 'compute_path_to_pose action server not found',
                'report': None,
            }

        planned = self._compute_path(goal['x'], goal['y'], goal['yaw'])
        if not planned['ok']:
            return {
                'ok': False,
                'reason': planned['reason'],
                'reason_cn': ('规划器无法规划到该目标点（可能太靠近墙、'
                              '障碍物或未知区域）。'),
                'detail': planned['reason'],
                'report': None,
            }

        costmap = None
        if self.path_check_use_costmap:
            costmap = self._get_costmap()

        report = clearance.audit_path(
            planned['path'], objects, self.robot_physical_radius_m,
            self.assumed_object_radius_m, self.min_surface_clearance_m,
            costmap=costmap, max_cell_cost=self.path_max_cell_cost)

        if report['ok']:
            return {
                'ok': True,
                'reason': 'OK',
                'reason_cn': '',
                'detail': '',
                'report': report,
                'path_points': len(planned['path']),
            }

        if report['reason'] == 'SEMANTIC_CLEARANCE':
            reason_cn = (
                '路径会经过 %s 附近，最小表面间隙只有 %.2f 米，'
                '低于安全阈值 %.2f 米。'
                % (report['blocking_id'],
                   report['min_surface_clearance_m'],
                   self.min_surface_clearance_m))
        elif report['reason'] == 'COSTMAP_LETHAL':
            reason_cn = ('路径会穿过代价地图的致命/内切区域'
                         '（最大代价 %d）。' % (report['max_cost'] or 0))
        else:
            reason_cn = report['reason_cn']

        return {'ok': False, 'reason': report['reason'],
                'reason_cn': reason_cn, 'detail': report['reason_cn'],
                'report': report}

    def _handle_navigate(self, question, state, plan, deadline):
        objects = query_engine.filter_objects(
            state['objects'], plan['object_class'], plan['spatial_filter'])

        selection = plan['selection']
        selection_note = ''
        if selection == 'all':
            selection = 'nearest'
            selection_note = '（未指定具体目标，已按最近的目标处理）'

        target, reason = query_engine.select_object(
            objects, selection, plan['rank'], plan['object_id'])
        if target is None:
            fact = '没有执行导航：' + self._reason_cn(reason, plan)
            self._answer(question, fact, 'NO_TARGET', deadline,
                         {'select_reason': reason})
            return

        note = query_engine.rank_cross_check(
            objects, selection, plan['rank'], plan['object_id'])
        if note:
            self.get_logger().warn(note)
            self.publish_status('RANK_MISMATCH', {'note': note})

        if not self.auto_execute_navigation:
            fact = ('已经找到 %s，但 auto_execute_navigation=false，'
                    '因此没有真正控制小车。' % target['id'])
            self._answer(question, fact, 'DRY_RUN', deadline,
                         {'target_id': target['id']})
            return

        offset = plan['goal_offset_m'] or self.goal_offset_m
        relations = [plan['goal_relation']]
        if self.allow_relation_fallback:
            relations.extend(
                query_engine.goal_relation_fallback(plan['goal_relation']))

        blocked_notes = []
        for index, relation in enumerate(relations):
            goal = query_engine.compute_goal(
                state['robot'], target, relation, offset,
                min_approach_m=self.min_approach_distance_m,
                min_offset_m=self.min_goal_offset_m)

            if not goal['ok']:
                if goal['reason'] == 'ALREADY_AT_TARGET':
                    fact = query_engine.fact_already_there(
                        target, goal['centre_distance'])
                    self._answer(question, fact, 'ALREADY_AT_TARGET',
                                 deadline, {'target_id': target['id']})
                    return
                self.publish_response('没有执行导航：' + goal['reason_cn'])
                self.publish_status('GOAL_INVALID', {
                    'target_id': target['id'], 'reason': goal['reason']})
                return

            check = self._precheck_goal(goal, state['objects'])
            if check['ok']:
                fallback_note = ''
                if index > 0:
                    fallback_note = (
                        '（原目标点不可达或间隙不足，已改为%s）'
                        % query_engine.GOAL_RELATION_CN.get(relation, '旁边'))
                    self.publish_status('FALLBACK_RELATION_USED', {
                        'target_id': target['id'], 'relation': relation})
                fallback_note += selection_note
                report = check['report']
                self.publish_status('PATH_CHECK_OK', {
                    'target_id': target['id'],
                    'relation': relation,
                    'min_surface_clearance_m': (
                        None if report is None
                        else report['min_surface_clearance_m']),
                    'max_cell_cost': (None if report is None
                                      else report['max_cost']),
                })
                self.start_navigation(target, relation, goal, question,
                                      fallback_note)
                return

            self.get_logger().warn(
                'path check failed for %s (%s): %s'
                % (target['id'], relation, check['reason']))
            self.publish_status('PATH_CHECK_FAILED', {
                'target_id': target['id'],
                'relation': relation,
                'reason': check['reason'],
                'detail': check['detail'],
            })
            blocked_notes.append('%s 方向：%s' % (
                query_engine.GOAL_RELATION_CN.get(relation, '旁边'),
                check['reason_cn']))

        fact = '没有执行导航：' + '；'.join(blocked_notes)
        self._answer(question, fact, 'PATH_BLOCKED', deadline,
                     {'target_id': target['id']})

    # ------------------------------------------------------ goal lifecycle
    def start_navigation(self, target, relation, goal, question,
                         fallback_note=''):
        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            self.publish_response(
                'Nav2 的 %s 当前不可用，请先确认导航系统已经启动。'
                % self.navigate_action)
            self.publish_status('NAVIGATION_UNAVAILABLE', {})
            return

        with self.goal_lock:
            previous = self.active_goal_handle
            had_previous = self.active_target_id is not None
        if had_previous:
            self.get_logger().warn('cancelling the previous navigation task')
            self.publish_status('PREVIOUS_GOAL_CANCELED',
                                {'reason': 'new goal requested'})
            if previous is not None:
                try:
                    previous.cancel_goal_async()
                except Exception as exc:               # noqa: BLE001
                    self.get_logger().warn('previous cancel failed: %s' % exc)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._make_pose(goal['x'], goal['y'], goal['yaw'])

        with self.goal_lock:
            self.active_seq += 1
            seq = self.active_seq
            self.active_target_id = target['id']
            self.active_goal_handle = None
            self.nav_start_time = time.monotonic()
            self.cancel_reason = None
            self.min_clearance_observed = None
            self.warn_published = False

        fact = query_engine.fact_navigation(target, relation, goal,
                                            fallback_note)
        answer = self.client.phrase(question, fact)
        self.publish_response(answer)
        self.publish_status('NAVIGATION_REQUESTED', {
            'target_id': target['id'],
            'goal_relation': relation,
            'goal_x': round(goal['x'], 4),
            'goal_y': round(goal['y'], 4),
            'goal_yaw_deg': round(math.degrees(goal['yaw']), 2),
            'offset_m': round(goal['eff_offset'], 3),
            'seq': seq,
        })

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(
            lambda done: self.goal_response_callback(done, target['id'], seq))

    def goal_response_callback(self, future, target_id, seq):
        try:
            goal_handle = future.result()
        except Exception as exc:                       # noqa: BLE001
            self.publish_response('导航目标发送失败。')
            self.publish_status('NAVIGATION_SEND_ERROR',
                                {'error': str(exc)})
            self._clear_session(seq)
            return

        if goal_handle is None or not goal_handle.accepted:
            self.publish_response('Nav2 拒绝了当前导航目标。')
            self.publish_status('NAVIGATION_REJECTED',
                                {'target_id': target_id})
            self._clear_session(seq)
            return

        with self.goal_lock:
            if seq != self.active_seq:
                self.get_logger().warn(
                    'stale goal callback (seq %d) -> cancelling' % seq)
                goal_handle.cancel_goal_async()
                return
            self.active_goal_handle = goal_handle
            reason = self.cancel_reason

        if reason is not None:
            goal_handle.cancel_goal_async()

        self.publish_status('NAVIGATING',
                            {'target_id': target_id, 'seq': seq})
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done: self.navigation_result_callback(done, target_id, seq))

    def navigation_result_callback(self, future, target_id, seq):
        try:
            wrapped = future.result()
            status = wrapped.status
        except Exception as exc:                       # noqa: BLE001
            self.publish_status('NAVIGATION_RESULT_ERROR',
                                {'error': str(exc)})
            return

        with self.goal_lock:
            cancel_reason = self.cancel_reason
            min_clearance = self.min_clearance_observed

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.publish_response(
                '导航完成，已经到达 %s 对应的目标区域。' % target_id)
            state = 'NAVIGATION_SUCCEEDED'
        elif status == GoalStatus.STATUS_CANCELED:
            state = 'NAVIGATION_CANCELED'
        else:
            self.publish_response('导航到 %s 没有成功完成。' % target_id)
            state = 'NAVIGATION_FAILED'

        self.publish_status(state, {
            'target_id': target_id,
            'nav2_status': int(status),
            'seq': seq,
            'cancel_reason': cancel_reason,
            'min_surface_clearance_m': (
                None if min_clearance is None
                else round(min_clearance, 3)),
        })
        self._clear_session(seq)


def main(args=None):
    rclpy.init(args=args)
    node = LLMNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
