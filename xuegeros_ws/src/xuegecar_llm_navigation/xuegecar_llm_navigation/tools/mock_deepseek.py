#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline DeepSeek stand-in (no network, no API key needed).

Implements just enough of the OpenAI-compatible API for P5 tests:

* ``GET  /models``            -> the verified model ids
* ``POST /chat/completions``  -> intent JSON or a rephrased FACT

The intent rules mirror the real contract closely enough to drive the
end-to-end scenarios; the phrasing call deliberately returns the FACT
unchanged so a test can prove the numbers came from the local code.

Usage::

    ros2 run xuegecar_llm_navigation mock_deepseek --port 8899

Failure injection (used by the scenario script) via a control file::

    {"mode": "empty" | "fenced" | "badjson" | "http500" | "normal"}
"""

import argparse
import json
import re
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

MODELS = ['deepseek-v4-flash', 'deepseek-v4-pro',
          'deepseek-v4-flash-vision-exp']

STATE = {
    'mode': 'normal',
    'intent_override': None,
    'control_file': None,
    'requests': 0,
    'models': list(MODELS),
}


def load_control():
    path = STATE['control_file']
    if not path:
        return
    try:
        with open(path, 'r', encoding='utf-8') as handle:
            payload = json.load(handle)
    except (OSError, ValueError):
        return
    if isinstance(payload, dict):
        STATE['mode'] = str(payload.get('mode', STATE['mode']))
        STATE['intent_override'] = payload.get('intent',
                                               STATE['intent_override'])


def spatial_filter_of(question):
    if re.search(r'前方|前面|车前|正前', question):
        return 'front'
    if re.search(r'左侧|左边|左方', question):
        return 'left'
    if re.search(r'右侧|右边|右方', question):
        return 'right'
    if re.search(r'后方|后面|身后', question):
        return 'back'
    return 'all'


def goal_relation_of(question):
    if re.search(r'右边|右侧|右手边', question):
        return 'right'
    if re.search(r'左边|左侧|左手边', question):
        return 'left'
    return 'near'


CHINESE_DIGITS = {'一': 1, '二': 2, '三': 3, '四': 4, '五': 5,
                  '六': 6, '七': 7, '八': 8, '九': 9, '十': 10}


def selection_of(question):
    match = re.search(r'第\s*([一二三四五六七八九十\d]+)\s*(远|近)', question)
    if match:
        rank_text = match.group(1)
        rank = CHINESE_DIGITS.get(rank_text)
        if rank is None:
            rank = int(rank_text)
        if match.group(2) == '远':
            return 'nth_farthest', rank
        return 'nth_nearest', rank
    if re.search(r'最远', question):
        return 'farthest', 1
    if re.search(r'最近', question):
        return 'nearest', 1
    return None, 1


def make_response(content):
    return {
        'id': 'mock-chatcmpl-1',
        'object': 'chat.completion',
        'model': 'mock-deepseek',
        'choices': [{
            'index': 0,
            'message': {'role': 'assistant', 'content': content},
            'finish_reason': 'stop',
        }],
    }


def build_intent(question):
    """Keyword rules that mimic the real intent contract."""
    if STATE['intent_override']:
        return dict(STATE['intent_override'])

    plan = {
        'intent': 'unknown',
        'object_class': 'bottle',
        'spatial_filter': 'all',
        'selection': 'all',
        'rank': 1,
        'object_id': '',
        'goal_relation': 'near',
        'goal_offset_m': None,
    }

    if re.search(r'停下|停止|取消|别去|不要动', question):
        plan['intent'] = 'cancel'
        return plan

    id_match = re.search(r'(bottle_[0-9]{3})', question, re.IGNORECASE)
    selection, rank = selection_of(question)

    if re.search(r'去|导航|前往|过去|走到|开到', question):
        plan['intent'] = 'navigate'
        plan['goal_relation'] = goal_relation_of(question)
        if id_match:
            plan['selection'] = 'id'
            plan['object_id'] = id_match.group(1).lower()
        else:
            plan['selection'] = selection or 'nearest'
            plan['rank'] = rank
        return plan

    if re.search(r'几个|多少|数量', question):
        plan['intent'] = 'query_count'
        plan['spatial_filter'] = spatial_filter_of(question)
        return plan

    if re.search(r'在哪|位置|坐标|都在哪|都有什么', question):
        plan['intent'] = 'query_objects'
        if id_match:
            plan['selection'] = 'id'
            plan['object_id'] = id_match.group(1).lower()
        else:
            plan['selection'] = selection or 'all'
            plan['rank'] = rank
        return plan

    return plan


class Handler(BaseHTTPRequestHandler):

    protocol_version = 'HTTP/1.1'

    def log_message(self, *_args):
        return                      # keep the test output clean

    def _send(self, code, payload):
        body = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):                                   # noqa: N802
        if self.path.rstrip('/').endswith('/models'):
            self._send(200, {
                'object': 'list',
                'data': [{'id': model, 'object': 'model'}
                         for model in STATE['models']],
            })
            return
        self._send(404, {'error': {'message': 'not found'}})

    def do_POST(self):                                  # noqa: N802
        length = int(self.headers.get('Content-Length', '0'))
        raw = self.rfile.read(length) if length else b'{}'
        try:
            body = json.loads(raw.decode('utf-8'))
        except ValueError:
            self._send(400, {'error': {'message': 'bad request json'}})
            return

        load_control()
        STATE['requests'] += 1
        mode = STATE['mode']
        system = ''
        user = ''
        for message in (body.get('messages') or []):
            if message.get('role') == 'system':
                system += str(message.get('content', ''))
            elif message.get('role') == 'user':
                user = str(message.get('content', ''))

        if mode == 'http500':
            self._send(500, {'error': {'message': 'injected server error'}})
            return
        if mode == 'no_choices':
            self._send(200, {'id': 'mock', 'choices': []})
            return
        if mode == 'empty':
            self._send(200, make_response(''))
            return
        if mode == 'slow':
            time.sleep(1.0)

        if 'P5_ROLE=intent_parser' in system:
            question = ''
            if 'P5_QUESTION=' in user:
                tail = user.split('P5_QUESTION=', 1)[1].strip()
                question = tail.split('\n')[0].strip()
            content = json.dumps(build_intent(question), ensure_ascii=False)
            if mode == 'fenced':
                content = '```json\n%s\n```' % content
            elif mode == 'badjson':
                content = '{"intent": "navigate", "rank": '
            self._send(200, make_response(content))
            return

        # phrasing call: echo the deterministic FACT unchanged
        if 'P5_FACT=' in user:
            fact = user.split('P5_FACT=', 1)[1].strip()
        else:
            fact = user
        self._send(200, make_response(fact))


def main():
    parser = argparse.ArgumentParser(description='mock DeepSeek server')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8899)
    parser.add_argument('--control', default='',
                        help='json file used to inject failure modes')
    parser.add_argument('--models', default='',
                        help='comma separated GET /models answer, e.g. '
                             '"deepseek-flash,deepseek-v4-pro"')
    args = parser.parse_args()

    STATE['control_file'] = args.control or None
    if args.models:
        STATE['models'] = [item.strip() for item in args.models.split(',')
                           if item.strip()]
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print('mock DeepSeek listening on http://%s:%d '
          '(set api_base_url to exactly this) | models=%s'
          % (args.host, args.port, ', '.join(STATE['models'])),
          flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
