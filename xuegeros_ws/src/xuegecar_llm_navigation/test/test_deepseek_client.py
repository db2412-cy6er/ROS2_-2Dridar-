#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unit tests for the DeepSeek client helpers (no ROS, no network).

These cover the contract that a broken logger must never be able to change
program behaviour.  The original bug: ``_log`` dispatched with a single
``getattr(self.logger, level)(...)`` call site, and rclpy refuses to reuse
one call site with two different severities ("Logger severity cannot be
changed between calls.").  The exception aborted ``resolve_model`` before
``self.model`` was switched, so the node kept a model the account does not
have and every later query failed.
"""

import pytest

from xuegecar_llm_navigation import deepseek_client as dc


class RecordingLogger:

    def __init__(self):
        self.records = []
        self.raise_on = set()

    def _record(self, level, message):
        if level in self.raise_on:
            raise RuntimeError('logger exploded')
        self.records.append((level, message))

    def info(self, message):
        self._record('info', message)

    def warn(self, message):
        self._record('warn', message)

    def error(self, message):
        self._record('error', message)


class FakeResponse:

    def __init__(self, payload, status_code=200):
        self._payload = payload
        self.status_code = status_code
        self.text = ''

    def json(self):
        return self._payload

    def raise_for_status(self):
        if self.status_code >= 400:
            raise RuntimeError('http %d' % self.status_code)


def make_client(logger, model='deepseek-v4-flash',
                fallback=('deepseek-v4-pro',)):
    return dc.DeepSeekClient(api_key='test-key', base_url='http://example',
                             model=model, models_fallback=list(fallback),
                             logger=logger)


def test_log_dispatches_every_level():
    logger = RecordingLogger()
    client = make_client(logger)
    client._log('info', 'i')
    client._log('warn', 'w')
    client._log('error', 'e')
    assert logger.records == [('info', 'i'), ('warn', 'w'), ('error', 'e')]


def test_log_without_logger_is_silent():
    client = make_client(None)
    client._log('warn', 'nobody is listening')


def test_log_never_raises_when_logger_is_broken():
    logger = RecordingLogger()
    logger.raise_on = {'warn', 'error', 'info'}
    client = make_client(logger)
    client._log('info', 'i')
    client._log('warn', 'w')
    client._log('error', 'e')


def test_resolve_model_switches_before_logging(monkeypatch):
    """The fallback must be applied even if the warning cannot be logged."""
    logger = RecordingLogger()
    logger.raise_on = {'warn'}
    client = make_client(logger)
    monkeypatch.setattr(
        dc.requests, 'get',
        lambda *args, **kwargs: FakeResponse(
            {'data': [{'id': 'deepseek-flash'}, {'id': 'deepseek-v4-pro'}]}))
    assert client.resolve_model() == 'deepseek-v4-pro'
    assert client.model == 'deepseek-v4-pro'


def test_resolve_model_keeps_offered_model(monkeypatch):
    logger = RecordingLogger()
    client = make_client(logger)
    monkeypatch.setattr(
        dc.requests, 'get',
        lambda *args, **kwargs: FakeResponse(
            {'data': [{'id': 'deepseek-v4-flash'}]}))
    assert client.resolve_model() == 'deepseek-v4-flash'
    assert any('in use: deepseek-v4-flash' in message
               for _, message in logger.records)


def test_resolve_model_when_nothing_matches(monkeypatch):
    logger = RecordingLogger()
    client = make_client(logger)
    monkeypatch.setattr(
        dc.requests, 'get',
        lambda *args, **kwargs: FakeResponse({'data': [{'id': 'other'}]}))
    assert client.resolve_model() == 'deepseek-v4-flash'
    assert any(level == 'warn' for level, _ in logger.records)


def test_resolve_model_survives_http_error(monkeypatch):
    logger = RecordingLogger()
    client = make_client(logger)
    monkeypatch.setattr(
        dc.requests, 'get',
        lambda *args, **kwargs: FakeResponse({}, status_code=500))
    assert client.resolve_model() == 'deepseek-v4-flash'


def test_request_variants_order_and_content():
    client = make_client(RecordingLogger())
    variants = client._request_variants()
    assert [name for name, _, _ in variants] == [
        'full', 'no_thinking', 'no_json_mode']
    body = dc.DeepSeekClient._build_body('m', [], 16, 0.1, True, True)
    assert body['thinking'] == {'type': 'disabled'}
    assert body['response_format'] == {'type': 'json_object'}
    body = dc.DeepSeekClient._build_body('m', [], 16, 0.1, False, False)
    assert 'thinking' not in body and 'response_format' not in body


def test_chat_requires_api_key():
    client = dc.DeepSeekClient(api_key='', model='m')
    with pytest.raises(dc.DeepSeekError):
        client.chat([{'role': 'user', 'content': 'x'}])
