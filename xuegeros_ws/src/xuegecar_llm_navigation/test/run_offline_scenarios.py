#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P5 offline end-to-end scenario runner.

Runs the whole P5 chain with no robot, no camera, no Nav2 and no network:

    fake P4 map  ->  P5 node  ->  mock DeepSeek (127.0.0.1:8899)
                          |---->  fake Nav2 (NavigateToPose,
                                          ComputePathToPose,
                                          global costmap)

Every scenario asserts on the real topics (/llm/response, /llm/task_status)
and on the fake Nav2 event log, so behaviour such as "no goal was sent" or
"the goal was cancelled" is actually verified rather than assumed.

Run it with both workspaces sourced::

    source /opt/ros/humble/setup.bash
    source ~/xuegeros_ws/install/setup.bash
    python3 ~/xuegeros_ws/src/xuegecar_llm_navigation/test/run_offline_scenarios.py
"""

import json
import atexit
import os
import signal
import subprocess
import sys
import threading
import time
from collections import deque

import yaml

os.environ.setdefault('ROS_DOMAIN_ID', '42')
os.environ['PYTHONUNBUFFERED'] = '1'

import rclpy                                          # noqa: E402
from geometry_msgs.msg import TransformStamped        # noqa: E402
from rclpy.node import Node                           # noqa: E402
from std_msgs.msg import String                       # noqa: E402
from tf2_ros import TransformBroadcaster              # noqa: E402

from urllib import request as urlrequest              # noqa: E402

from ament_index_python.packages import (             # noqa: E402
    get_package_share_directory,
)

from xuegecar_llm_navigation import intent_schema     # noqa: E402

#: the real YAML config is loaded on purpose: launching P5 with the shipped
#: parameter file is what the user actually does, and it is where parameter
#: type mismatches (int vs double) show up.
CONFIG_FILE = os.path.join(
    get_package_share_directory('xuegecar_llm_navigation'),
    'config', 'p5_llm_navigation.yaml')

LOG_DIR = '/tmp/p5_offline_logs'
MAP_CONTROL = '/tmp/p5_offline_map.json'
LLM_CONTROL = '/tmp/p5_offline_llm.json'
LLM_PORT = 8899
LLM_URL = 'http://127.0.0.1:%d' % LLM_PORT
#: second mock used by the model-fallback regression (S19): it advertises
#: only the models a real account actually offers.
LLM_PORT_REAL = 8901
LLM_URL_REAL = 'http://127.0.0.1:%d' % LLM_PORT_REAL

DEFAULT_MAP = {
    'robot': {'x_map': 0.0, 'y_map': 0.0, 'yaw_deg': 0.0},
    'objects': [
        {'id': 'bottle_001', 'x': 0.8, 'y': 0.0},
        {'id': 'bottle_002', 'x': 1.4, 'y': 0.0},
        {'id': 'bottle_003', 'x': -1.0, 'y': 0.0},
        {'id': 'bottle_004', 'x': 0.0, 'y': 1.2},
    ],
}

RESULTS = []


def _cleanup_all():
    for proc in list(PROCS):
        try:
            proc.stop()
        except Exception:                              # noqa: BLE001
            pass


PROCS = []
atexit.register(_cleanup_all)


# --------------------------------------------------------------------------
# process helpers
# --------------------------------------------------------------------------

def log_path(name):
    os.makedirs(LOG_DIR, exist_ok=True)
    return os.path.join(LOG_DIR, name + '.log')


class Proc:
    """A background node process with its stdout redirected to a log file."""

    def __init__(self, name, module, extra_args=(), env_extra=None):
        self.name = name
        self.path = log_path(name)
        self.handle = open(self.path, 'w', encoding='utf-8')
        environment = dict(os.environ)
        environment['DEEPSEEK_API_KEY'] = 'offline-test-key'
        environment['ROS_DOMAIN_ID'] = os.environ.get('ROS_DOMAIN_ID', '42')
        if env_extra:
            environment.update(env_extra)
        command = [sys.executable, '-m', module] + list(extra_args)
        self.process = subprocess.Popen(command, stdout=self.handle,
                                        stderr=subprocess.STDOUT,
                                        env=environment,
                                        preexec_fn=os.setsid)
        PROCS.append(self)
        print('  started %-22s pid=%d log=%s'
              % (name, self.process.pid, self.path))

    def stop(self):
        if self.process.poll() is None:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
                self.process.wait(timeout=5)
            except Exception:                          # noqa: BLE001
                try:
                    os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                except Exception:                      # noqa: BLE001
                    pass
        self.handle.close()

    def log(self):
        try:
            with open(self.path, 'r', encoding='utf-8') as handle:
                return handle.read()
        except OSError:
            return ''

    def log_events(self, event_name):
        return [line for line in self.log().splitlines()
                if line.startswith('{') and '"%s"' % event_name in line]


def write_json(path, payload):
    with open(path, 'w', encoding='utf-8') as handle:
        json.dump(payload, handle, ensure_ascii=False)


def set_map(spec):
    write_json(MAP_CONTROL, spec)


# --------------------------------------------------------------------------
# test harness node (listens, drives TF, publishes queries)
# --------------------------------------------------------------------------

class Harness(Node):

    def __init__(self):
        super().__init__('p5_offline_harness')
        self.lock = threading.Lock()
        self.responses = deque()
        self.states = deque()
        self.event = threading.Event()
        self.robot = (0.0, 0.0, 0.0)

        self.create_subscription(String, '/llm/response', self._on_response,
                                 10)
        self.create_subscription(String, '/llm/task_status', self._on_status,
                                 10)
        self.query_pub = self.create_publisher(String, '/llm/query', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(0.1, self._broadcast_tf)

    def _on_response(self, msg):
        with self.lock:
            self.responses.append(msg.data)
            self.event.set()

    def _on_status(self, msg):
        try:
            payload = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.states.append(payload)

    def _broadcast_tf(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = float(self.robot[0])
        transform.transform.translation.y = float(self.robot[1])
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)

    def set_robot(self, x, y):
        self.robot = (float(x), float(y), 0.0)

    def ask(self, question, timeout=30.0):
        with self.lock:
            self.responses.clear()
            self.states.clear()
            self.event.clear()
        message = String()
        message.data = question
        self.query_pub.publish(message)
        self.event.wait(timeout)
        time.sleep(0.5)                # let trailing status messages arrive
        with self.lock:
            return list(self.responses), list(self.states)

    @staticmethod
    def answer_contains(responses, text):
        return any(text in answer for answer in responses)

    @staticmethod
    def state_seen(states, name):
        return any(state.get('state') == name for state in states)


def check(name, ok, detail=''):
    RESULTS.append((name, bool(ok), detail))
    print('  [%s] %-46s %s' % ('PASS' if ok else 'FAIL', name, detail),
          flush=True)
    return bool(ok)


def start_fake_nav(extra_args=()):
    return Proc('fake_nav2%s' % ('_' + '_'.join(extra_args).replace(
        '--', '').replace(',', '-').replace(' ', '') if extra_args else ''),
        'xuegecar_llm_navigation.tools.fake_nav2_server', extra_args)


def set_llm(mode='normal'):
    write_json(LLM_CONTROL, {'mode': mode})


def probe_mock(question):
    """Ask the mock server for an intent plan using the real prompt builder.

    This is the protocol self-check: it catches the failure mode where the
    mock silently stops recognising the intent call and starts answering
    with the phrasing branch (which produces a plan with intent=unknown).
    """
    world = {'robot': None, 'num_objects': 0, 'objects': []}
    body = {
        'model': 'deepseek-v4-flash',
        'messages': [
            {'role': 'system', 'content': intent_schema.SYSTEM_PROMPT},
            {'role': 'user',
             'content': intent_schema.build_user_prompt(question, world)},
        ],
    }
    payload = json.dumps(body).encode('utf-8')
    request = urlrequest.Request(
        LLM_URL + '/chat/completions', data=payload,
        headers={'Content-Type': 'application/json'})
    with urlrequest.urlopen(request, timeout=10) as response:
        content = json.loads(response.read().decode('utf-8'))
    text = content['choices'][0]['message']['content']
    return json.loads(text)


def parse_events(nav, event_name):
    """Structured events emitted by the fake Nav2 server."""
    events = []
    for line in nav.log().splitlines():
        if not line.startswith('{'):
            continue
        try:
            payload = json.loads(line)
        except ValueError:
            continue
        if payload.get('event') == event_name:
            events.append(payload)
    return events


def map_with_objects(objects, robot=(0.0, 0.0, 0.0)):
    return {
        'robot': {'x_map': robot[0], 'y_map': robot[1], 'yaw_deg': robot[2]},
        'objects': objects,
    }


def _load_yaml(path):
    with open(path, 'r', encoding='utf-8') as handle:
        return yaml.safe_load(handle) or {}


def check_config_contracts():
    """Check the static contracts that keep the field fixes from regressing.

    These read the *installed* yaml files (the ones the robot actually loads)
    and encode the numbers from the 2026-09 field session: a 4-5 m room, a
    5 cm/s crawl, two bottles that must stay two objects, and a map that must
    not look like it is overlapping itself.
    """
    print('== 5. configuration contracts (S20-S21) ==', flush=True)
    nav2 = _load_yaml(os.path.join(
        get_package_share_directory('xuegecar_navigation2'),
        'param', 'xuegebot_p5.yaml'))
    p4 = _load_yaml(os.path.join(
        get_package_share_directory('xuegecar_dynamic_semantic_mapper'),
        'config', 'p4_dynamic_mapper.yaml'))
    p5 = _load_yaml(CONFIG_FILE)

    n_amcl = nav2['amcl']['ros__parameters']
    n_ctrl = nav2['controller_server']['ros__parameters']
    n_follow = n_ctrl['FollowPath']
    n_prog = n_ctrl['progress_checker']
    n_goal = n_ctrl['general_goal_checker']
    n_local = nav2['local_costmap']['local_costmap']['ros__parameters']
    n_global = nav2['global_costmap']['global_costmap']['ros__parameters']
    n_smooth = nav2['velocity_smoother']['ros__parameters']
    m4 = p4['dynamic_semantic_mapper']['ros__parameters']
    m5 = p5['llm_navigation_node']['ros__parameters']

    check('S20 two bottles in one frame stay two objects (P4 gates)',
          m4['same_frame_dedupe_distance'] <= 0.06
          and m4['same_frame_dedupe_distance'] < m4['association_distance']
          and m4['association_distance'] <= 0.15,
          'same_frame=%.2f assoc=%.2f' % (m4['same_frame_dedupe_distance'],
                                          m4['association_distance']))

    check('S20b approach thresholds let the robot drive up to a bottle',
          m5['min_approach_distance_m'] <= 0.35
          and m5['goal_offset_m'] <= 0.35
          and m5['min_approach_distance_m'] >= m5['goal_offset_m'],
          'approach=%.2f offset=%.2f min_offset=%.2f'
          % (m5['min_approach_distance_m'], m5['goal_offset_m'],
             m5['min_goal_offset_m']))

    worst = (m5['min_goal_offset_m'] - n_goal['xy_goal_tolerance']
             - m5['robot_physical_radius_m'] - m5['assumed_object_radius_m'])
    check('S20c worst-case stop keeps >= min_surface_clearance_m',
          worst >= m5['min_surface_clearance_m'],
          'worst surface clearance=%.3f m (required %.2f)'
          % (worst, m5['min_surface_clearance_m']))

    check('S21 5 cm/s crawl is consistent end to end',
          n_follow['max_vel_x'] <= 0.06
          and n_follow['max_speed_xy'] <= 0.06
          and n_follow['trans_stopped_velocity'] < n_follow['max_vel_x']
          and n_smooth['max_velocity'][0] <= 0.06,
          'max_vel_x=%.2f trans_stopped=%.2f smoother=%.2f'
          % (n_follow['max_vel_x'], n_follow['trans_stopped_velocity'],
             n_smooth['max_velocity'][0]))

    check('S21b a crawling robot is not mistaken for a stuck one',
          n_prog['required_movement_radius'] <= 0.06
          and n_prog['movement_time_allowance'] >= 20.0,
          'progress radius=%.2f allowance=%.0fs'
          % (n_prog['required_movement_radius'],
             n_prog['movement_time_allowance']))

    check('S21c AMCL corrects the pose often enough while crawling',
          n_amcl['update_min_d'] <= 0.05 and n_amcl['update_min_a'] <= 0.05
          and n_amcl['do_beamskip'] is True,
          'update_min_d=%.2f update_min_a=%.2f do_beamskip=%s'
          % (n_amcl['update_min_d'], n_amcl['update_min_a'],
             n_amcl['do_beamskip']))

    check('S21d no sliding map copy and no ghost walls',
          'static_layer' not in n_local['plugins']
          and 'voxel_layer' not in n_local['plugins']
          and n_global['obstacle_layer']['scan']['raytrace_max_range']
          <= n_global['obstacle_layer']['scan']['obstacle_max_range'],
          'local=%s raytrace=%.1f obstacle=%.1f'
          % (n_local['plugins'],
             n_global['obstacle_layer']['scan']['raytrace_max_range'],
             n_global['obstacle_layer']['scan']['obstacle_max_range']))


def main():
    os.makedirs(LOG_DIR, exist_ok=True)
    set_map(DEFAULT_MAP)
    set_llm('normal')

    print('== 1. starting offline stack (ROS_DOMAIN_ID=%s) =='
          % os.environ['ROS_DOMAIN_ID'], flush=True)
    llm = Proc('mock_deepseek',
               'xuegecar_llm_navigation.tools.mock_deepseek',
               ['--control', LLM_CONTROL])
    fake_map = Proc('fake_dynamic_map',
                    'xuegecar_llm_navigation.tools.fake_dynamic_map',
                    ['--control', MAP_CONTROL])
    nav = start_fake_nav(['--delay', '1.5'])
    time.sleep(2.0)

    rclpy.init()
    harness = Harness()
    spin_thread = threading.Thread(target=rclpy.spin, args=(harness,),
                                   daemon=True)
    spin_thread.start()

    p5 = Proc('p5_node',
              'xuegecar_llm_navigation.llm_navigation_node',
              ['--ros-args',
               '--params-file', CONFIG_FILE,
               '-p', 'api_base_url:=' + LLM_URL,
               '-p', 'api_timeout_sec:=5.0',
               '-p', 'map_timeout_sec:=2.0',
               '-p', 'nav_timeout_sec:=25.0'])
    time.sleep(3.5)
    p5_log = p5.log()
    if 'P5 LLM navigation node started' not in p5_log:
        check('S0 P5 node starts with the shipped YAML config', False,
              p5_log[-400:])
        return 1

    print('== 2. query scenarios (S0-S7) ==', flush=True)
    node_log = p5.log()
    check('S0 P5 node started + model discovered',
          'P5 LLM navigation node started' in node_log
          and 'DeepSeek models available' in node_log
          and 'deepseek-v4-flash' in node_log,
          'api_model verified against GET /models')

    plan = probe_mock('小车前方有几个瓶子？')
    check('S0b mock intent protocol self-check',
          plan.get('intent') == 'query_count'
          and plan.get('spatial_filter') == 'front', str(plan))

    answers, states = harness.ask('小车前方有几个瓶子？')
    answered = [s for s in states if s.get('state') == 'ANSWERED']
    check('S1 front count == 2 (computed locally)',
          harness.answer_contains(answers, '2')
          and answered and answered[0].get('count') == 2,
          '%s | count=%s' % ((answers[:1] or [''])[0][:60],
                             answered[0].get('count') if answered else None))

    answers, states = harness.ask('瓶子都在哪里？')
    check('S2 all objects listed',
          bool(answers) and all(('bottle_00%d' % index) in answers[0]
                                for index in (1, 2, 3, 4)),
          (answers[:1] or [''])[0][:110])

    answers, states = harness.ask('最近的瓶子在哪里？')
    targets = [s.get('target_id') for s in states
               if s.get('state') == 'ANSWERED']
    check('S3 nearest == bottle_001',
          bool(targets) and targets[-1] == 'bottle_001', str(targets))

    answers, states = harness.ask('第三远的瓶子在哪里？')
    targets = [s.get('target_id') for s in states
               if s.get('state') == 'ANSWERED']
    check('S4 third farthest == bottle_003 (local sort)',
          bool(targets) and targets[-1] == 'bottle_003', str(targets))

    set_map(map_with_objects(DEFAULT_MAP['objects'][:2]))
    time.sleep(0.8)
    goals_before = len(parse_events(nav, 'NAV_GOAL'))
    answers, states = harness.ask('第三远的瓶子在哪里？')
    check('S5 out-of-range rank is refused, not invented',
          harness.answer_contains(answers, '没有满足')
          and harness.state_seen(states, 'ANSWERED')
          and len(parse_events(nav, 'NAV_GOAL')) == goals_before,
          (answers[:1] or [''])[0][:90])

    set_map(DEFAULT_MAP)
    time.sleep(0.8)

    answers, states = harness.ask('请去最近的瓶子旁边')
    goals = parse_events(nav, 'NAV_GOAL')
    goal = goals[-1] if goals else {}
    # goal_offset_m = 0.30 (config) -> 0.8 - 0.30 = 0.50
    check('S6 navigate near -> goal (0.5, 0.0) yaw 0',
          abs(goal.get('x', 99) - 0.5) < 0.02
          and abs(goal.get('y', 99)) < 0.02
          and abs(goal.get('yaw_deg', 99)) < 1.0,
          'goal=%s' % goal)

    answers, states = harness.ask('请去第三远的瓶子的右边')
    goals = parse_events(nav, 'NAV_GOAL')
    goal = goals[-1] if goals else {}
    # bottle_003 at (-1, 0), offset 0.30 to its right -> (-1.0, 0.30)
    check('S7 third-farthest right -> goal (-1.0, 0.3) yaw -90',
          abs(goal.get('x', 99) + 1.0) < 0.02
          and abs(goal.get('y', 99) - 0.3) < 0.02
          and abs(goal.get('yaw_deg', 99) + 90.0) < 1.0,
          'goal=%s' % goal)

    # --------------------------------------------------------------- S13b
    # Field regression: min_approach_distance_m used to be 0.75, so a bottle
    # 0.6 m ahead was answered with "already next to it" and the robot never
    # moved.  It must now drive to it (goal = 0.6 - 0.30 = 0.30).
    set_map(map_with_objects([{'id': 'bottle_001', 'x': 0.6, 'y': 0.0}]))
    time.sleep(0.8)
    goals_before = len(parse_events(nav, 'NAV_GOAL'))
    answers, states = harness.ask('请去最近的瓶子旁边')
    goals = parse_events(nav, 'NAV_GOAL')
    goal = goals[-1] if goals else {}
    check('S13b target 0.6 m away -> DOES move (old 0.75 rule refused)',
          len(goals) > goals_before
          and not harness.state_seen(states, 'ALREADY_AT_TARGET')
          and abs(goal.get('x', 99) - 0.3) < 0.02,
          'goal=%s' % goal)
    time.sleep(4.0)          # let the 1.5 s fake navigation finish first
    set_map(DEFAULT_MAP)

    # ---------------------------------------------------------------- S8
    print('== 3. dynamic interaction scenarios (S8-S12) ==', flush=True)
    nav.stop()
    nav = start_fake_nav(['--delay', '12.0'])
    time.sleep(2.0)
    harness.set_robot(0.0, 0.0)
    answers, states = harness.ask('请去最近的瓶子旁边')
    time.sleep(1.0)
    set_map(map_with_objects([obj for obj in DEFAULT_MAP['objects']
                              if obj['id'] != 'bottle_001']))
    time.sleep(2.0)
    with harness.lock:
        answers = list(harness.responses)
        states = list(harness.states)
    cancels = (parse_events(nav, 'NAV_CANCEL_REQUEST')
               + parse_events(nav, 'NAV_CANCELED'))
    check('S8 target removed mid-navigation -> cancelled',
          bool(cancels) and harness.state_seen(states, 'TARGET_REMOVED')
          and harness.answer_contains(answers, '已经被移走'),
          'cancels=%d states=%s' % (len(cancels),
                                    [s.get('state') for s in states]))

    # ---------------------------------------------------------------- S9
    set_map({'no_robot': True, 'objects': DEFAULT_MAP['objects']})
    time.sleep(0.8)
    answers, states = harness.ask('小车前方有几个瓶子？')
    check('S9 no robot pose -> refusal, never a wrong count',
          harness.answer_contains(answers, '定位未就绪')
          and harness.state_seen(states, 'MAP_NOT_LOCALIZED')
          and not harness.answer_contains(answers, '0 个'),
          (answers[:1] or [''])[0][:80])

    # --------------------------------------------------------------- S10
    set_map(DEFAULT_MAP)
    time.sleep(0.8)
    fake_map.stop()
    time.sleep(3.0)
    answers, states = harness.ask('最近的瓶子在哪里？')
    check('S10 stale map -> MAP_NOT_READY',
          harness.state_seen(states, 'MAP_NOT_READY')
          and harness.answer_contains(answers, '没有更新'),
          (answers[:1] or [''])[0][:80])
    fake_map = Proc('fake_dynamic_map',
                    'xuegecar_llm_navigation.tools.fake_dynamic_map',
                    ['--control', MAP_CONTROL])
    time.sleep(1.5)

    # --------------------------------------------------------------- S11
    goals_before = len(parse_events(nav, 'NAV_GOAL'))
    set_llm('http500')
    answers, states = harness.ask('请去最近的瓶子旁边')
    check('S11 LLM 500 -> no motion, clear error',
          harness.answer_contains(answers, '大模型接口调用失败')
          and harness.state_seen(states, 'LLM_ERROR')
          and len(parse_events(nav, 'NAV_GOAL')) == goals_before,
          (answers[:1] or [''])[0][:80])

    # --------------------------------------------------------------- S12
    set_llm('fenced')
    answers, states = harness.ask('最近的瓶子在哪里？')
    check('S12 fenced ```json output is repaired',
          harness.answer_contains(answers, 'bottle_001'),
          (answers[:1] or [''])[0][:80])

    # --------------------------------------------------------------- S13
    set_llm('normal')
    # 0.25 m < min_approach_distance_m (0.30) -> refuse to move, answer "already there"
    set_map(map_with_objects([{'id': 'bottle_001', 'x': 0.25, 'y': 0.0}]))
    time.sleep(0.8)
    goals_before = len(parse_events(nav, 'NAV_GOAL'))
    answers, states = harness.ask('请去最近的瓶子旁边')
    check('S13 target already 0.25 m away -> no motion',
          harness.state_seen(states, 'ALREADY_AT_TARGET')
          and len(parse_events(nav, 'NAV_GOAL')) == goals_before,
          (answers[:1] or [''])[0][:80])

    # --------------------------------------------------------------- S14
    set_map(DEFAULT_MAP)
    time.sleep(0.8)
    set_llm('slow')
    with harness.lock:
        harness.responses.clear()
        harness.states.clear()
        harness.event.clear()
    harness.query_pub.publish(String(data='最近的瓶子在哪里？'))
    time.sleep(0.2)
    harness.query_pub.publish(String(data='瓶子都在哪里？'))
    time.sleep(4.5)
    with harness.lock:
        states = list(harness.states)
    check('S14 two queries at once -> second one is rejected',
          harness.state_seen(states, 'BUSY'),
          str([s.get('state') for s in states]))
    set_llm('normal')

    # --------------------------------------------------------------- S15
    print('== 4. obstacle safety scenarios (S15-S18) ==', flush=True)
    nav.stop()
    nav = start_fake_nav(['--delay', '1.5', '--via', '0.8,0.0'])
    time.sleep(2.0)
    goals_before = len(parse_events(nav, 'NAV_GOAL'))
    answers, states = harness.ask('请去最远的瓶子旁边')
    failed = [s for s in states if s.get('state') == 'PATH_CHECK_FAILED']
    check('S15 path through a bottle -> refused, no goal sent',
          bool(failed) and failed[0].get('reason') == 'SEMANTIC_CLEARANCE'
          and len(parse_events(nav, 'NAV_GOAL')) == goals_before
          and harness.state_seen(states, 'PATH_BLOCKED'),
          'reason=%s answer=%s' % (failed[0].get('reason') if failed else None,
                                   (answers[:1] or [''])[0][:70]))

    # --------------------------------------------------------------- S16
    nav.stop()
    nav = start_fake_nav(['--delay', '1.5', '--no-paths'])
    time.sleep(2.0)
    goals_before = len(parse_events(nav, 'NAV_GOAL'))
    answers, states = harness.ask('请去最远的瓶子旁边')
    failed = [s for s in states if s.get('state') == 'PATH_CHECK_FAILED']
    check('S16 planner cannot reach the goal -> refused',
          bool(failed) and failed[0].get('reason') == 'PLANNER_FAILED'
          and len(parse_events(nav, 'NAV_GOAL')) == goals_before,
          'reason=%s' % (failed[0].get('reason') if failed else None))

    # --------------------------------------------------------------- S17
    nav.stop()
    nav = start_fake_nav(['--delay', '12.0'])
    time.sleep(2.0)
    harness.set_robot(0.0, 0.0)
    # 2nd farthest bottle is bottle_004 at (0, 1.2): the straight path to its
    # near goal (0, 0.7) is clear of every other bottle.
    answers, states = harness.ask('请去第二远的瓶子旁边')
    time.sleep(1.0)
    harness.set_robot(0.0, 1.05)       # 0.15 m from bottle_004 -> contact
    time.sleep(2.0)
    with harness.lock:
        states = list(harness.states)
        answers = list(harness.responses)
    cancels = (parse_events(nav, 'NAV_CANCEL_REQUEST')
               + parse_events(nav, 'NAV_CANCELED'))
    check('S17 runtime clearance < 0 -> safety stop + cancel',
          harness.state_seen(states, 'SAFETY_STOP') and bool(cancels)
          and harness.answer_contains(answers, '安全停车'),
          'cancels=%d states=%s' % (len(cancels),
                                    [s.get('state') for s in states]))
    harness.set_robot(0.0, 0.0)

    # --------------------------------------------------------------- S18
    nav.stop()
    # the lateral goal for bottle_004 (0, 1.2) with offset 0.30 is (0.30, 1.2):
    # that is where the lethal cell is placed so only the right-goal is blocked
    nav = start_fake_nav(['--delay', '1.5', '--lethal', '0.3,1.2'])
    time.sleep(2.0)
    answers, states = harness.ask('请去第二远的瓶子的右边')
    goals = parse_events(nav, 'NAV_GOAL')
    goal = goals[-1] if goals else {}
    failed = [s for s in states if s.get('state') == 'PATH_CHECK_FAILED']
    check('S18 lateral goal blocked -> fallback to near',
          harness.state_seen(states, 'FALLBACK_RELATION_USED')
          and bool(failed) and failed[0].get('reason') == 'COSTMAP_LETHAL'
          and abs(goal.get('x', 99)) < 0.02
          and abs(goal.get('y', 99) - 0.9) < 0.02,
          'goal=%s states=%s' % (goal,
                                 [s.get('state') for s in states]))

    # --------------------------------------------------------------- S19
    # Regression for the real account: GET /models answers only
    # deepseek-flash + deepseek-v4-pro, so the configured
    # deepseek-v4-flash does not exist and the node must fall back
    # *and* actually use the fallback model (a logging error used to
    # abort the switch, leaving a non-existent model in place).
    p5.stop()
    llm_real = Proc('mock_deepseek_realmodels',
                    'xuegecar_llm_navigation.tools.mock_deepseek',
                    ['--port', str(LLM_PORT_REAL), '--control', LLM_CONTROL,
                     '--models', 'deepseek-flash,deepseek-v4-pro'])
    time.sleep(1.5)
    p5b = Proc('p5_node_model_fallback',
               'xuegecar_llm_navigation.llm_navigation_node',
               ['--ros-args',
                '--params-file', CONFIG_FILE,
                '-p', 'api_base_url:=' + LLM_URL_REAL,
                '-p', 'api_model:=deepseek-v4-flash',
                '-p', 'api_models_fallback:=deepseek-v4-flash,'
                      'deepseek-v4-pro',
                '-p', 'api_timeout_sec:=5.0',
                '-p', 'map_timeout_sec:=2.0'])
    time.sleep(4.0)
    log_b = p5b.log()
    check('S19 model not offered -> fallback is applied, not aborted',
          'falling back to deepseek-v4-pro' in log_b
          and 'model discovery failed' not in log_b
          and 'DeepSeek model in use: deepseek-v4-pro' in log_b,
          'warn=%s | in-use line=%s | discovery-error=%s'
          % ('falling back to deepseek-v4-pro' in log_b,
             'DeepSeek model in use: deepseek-v4-pro' in log_b,
             'model discovery failed' in log_b))
    answers, states = harness.ask('最近的瓶子在哪里？')
    check('S19b query works with the fallback model',
          harness.answer_contains(answers, 'bottle_001')
          and not harness.state_seen(states, 'LLM_ERROR'),
          (answers[:1] or [''])[0][:80])
    p5b.stop()
    llm_real.stop()

    # ------------------------------------------- 5. configuration contracts
    check_config_contracts()

    # ------------------------------------------------------------ summary
    print('\n== 5. summary ==', flush=True)
    failed_items = [item for item in RESULTS if not item[1]]
    for name, ok, detail in RESULTS:
        print('  %s  %s' % ('PASS' if ok else 'FAIL', name), flush=True)
    print('\n%d/%d scenarios passed' % (len(RESULTS) - len(failed_items),
                                        len(RESULTS)), flush=True)

    nav.stop()
    p5.stop()
    fake_map.stop()
    llm.stop()
    if rclpy.ok():
        rclpy.shutdown()
    spin_thread.join(timeout=2.0)
    harness.destroy_node()
    return 1 if failed_items else 0


if __name__ == '__main__':
    sys.exit(main())
