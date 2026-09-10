#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P5 DeepSeek client (OpenAI-compatible chat completions).

Only two jobs live here:

1. ``parse_intent``  : Chinese sentence + world snapshot -> validated plan
2. ``phrase``        : deterministic FACT -> short Chinese sentence

Robustness rules baked in (verified against the official DeepSeek docs
before writing this file):

* thinking mode is ON by default with effort ``high``; P5 disables it
  (``{"thinking": {"type": "disabled"}}``) because intent parsing is a
  small, deterministic task and latency matters for a robot.
* JSON output uses ``response_format={"type": "json_object"}`` and the
  prompt must contain the word "json" (it does, see intent_schema).
* the API may occasionally return an *empty* content in JSON mode, so an
  empty answer is retried once and never crashes the node.
* unknown request fields / models are handled by a staged fallback: the
  request is retried without the optional fields before giving up.
"""

import json

import requests

from xuegecar_llm_navigation import intent_schema


class DeepSeekError(RuntimeError):
    """Any non-recoverable API problem (network, auth, 5xx, bad body)."""


class DeepSeekClient:
    """Thin synchronous client; the ROS node calls it from a worker thread."""

    def __init__(self, api_key, base_url='https://api.deepseek.com',
                 model='deepseek-v4-flash', timeout_sec=15.0,
                 models_fallback=(), enable_json_mode=True,
                 disable_thinking=True, logger=None):
        self.api_key = api_key
        self.base_url = str(base_url).rstrip('/')
        self.model = model
        self.timeout_sec = float(timeout_sec)
        self.models_fallback = [m.strip() for m in models_fallback
                                if m.strip()]
        self.enable_json_mode = bool(enable_json_mode)
        self.disable_thinking = bool(disable_thinking)
        self.logger = logger
        self.last_error = ''

    # ------------------------------------------------------------- logging
    def _log(self, level, message):
        """Log a message without ever breaking the caller.

        Two deliberate details:

        * every severity uses its **own call site**.  rclpy caches the
          severity per caller id (function + file + line + bytecode index)
          and raises ``Logger severity cannot be changed between calls.``
          when the same call site is later reused with another severity.
          A single ``getattr(self.logger, level)(...)`` line does exactly
          that, which is why the model fallback used to blow up.
        * logging never raises: a broken logger must not be able to abort
          model discovery or a request.
        """
        if self.logger is None:
            return
        try:
            if level == 'error':
                self.logger.error(message)
            elif level == 'warn':
                self.logger.warn(message)
            else:
                self.logger.info(message)
        except Exception:                              # noqa: BLE001
            pass

    # -------------------------------------------------------------- models
    def list_models(self):
        """GET /models -> list of model ids (used to verify api_model)."""
        response = requests.get(
            self.base_url + '/models',
            headers=self._headers(),
            timeout=(5.0, self.timeout_sec),
        )
        response.raise_for_status()
        payload = response.json()
        return [item.get('id', '') for item in payload.get('data', [])]

    def resolve_model(self):
        """Pick the first model that the endpoint actually offers.

        The switch happens *before* any logging, so a logging hiccup can
        never leave ``self.model`` pointing at a model the account does not
        have (that failure mode produced "all request variants failed" on
        the first real query).
        """
        candidates = [self.model] + [m for m in self.models_fallback
                                     if m != self.model]
        try:
            available = self.list_models()
        except Exception as exc:                       # noqa: BLE001
            self._log('warn', 'GET /models failed (%s); keeping model %s'
                      % (exc, self.model))
            return self.model
        if not available:
            return self.model

        requested = self.model
        for candidate in candidates:
            if candidate in available:
                self.model = candidate                 # switch first
                if candidate != requested:
                    self._log('warn',
                              'api_model %s not offered; falling back to %s'
                              % (requested, candidate))
                self._log('info', 'DeepSeek models available: %s | in use: %s'
                          % (', '.join(available), self.model))
                return self.model
        self._log('warn', 'none of %s is offered; keeping %s'
                  % (candidates, self.model))
        return self.model

    # ---------------------------------------------------------- transport
    def _headers(self):
        return {
            'Authorization': 'Bearer ' + self.api_key,
            'Content-Type': 'application/json',
        }

    def _request_variants(self):
        """Ordered list of optional-field combinations to attempt."""
        if self.enable_json_mode and self.disable_thinking:
            return [('full', True, True), ('no_thinking', False, True),
                    ('no_json_mode', True, False)]
        if self.enable_json_mode:
            return [('full', False, True), ('no_json_mode', False, False)]
        if self.disable_thinking:
            return [('full', True, False)]
        return [('plain', False, False)]

    @staticmethod
    def _build_body(model, messages, max_tokens, temperature,
                    disable_thinking, json_mode):
        body = {
            'model': model,
            'messages': messages,
            'max_tokens': int(max_tokens),
            'temperature': float(temperature),
            'stream': False,
        }
        if disable_thinking:
            body['thinking'] = {'type': 'disabled'}
        if json_mode:
            body['response_format'] = {'type': 'json_object'}
        return body

    @staticmethod
    def _api_error(response):
        try:
            payload = response.json()
        except ValueError:
            return (response.text or '')[:200]
        error = payload.get('error')
        if isinstance(error, dict):
            return str(error.get('message', error))
        return json.dumps(payload, ensure_ascii=False)[:200]

    def chat(self, messages, max_tokens=512, temperature=0.1):
        """POST /chat/completions with staged fallback; returns content."""
        if not self.api_key:
            raise DeepSeekError('DEEPSEEK_API_KEY is not set')

        last_error = None
        for name, disable_thinking, json_mode in self._request_variants():
            body = self._build_body(self.model, messages, max_tokens,
                                    temperature, disable_thinking, json_mode)
            try:
                response = requests.post(
                    self.base_url + '/chat/completions',
                    headers=self._headers(),
                    data=json.dumps(body, ensure_ascii=False).encode('utf-8'),
                    timeout=(5.0, self.timeout_sec),
                )
            except requests.RequestException as exc:
                raise DeepSeekError('network error: %s' % exc)

            if response.status_code in (400, 404, 422):
                last_error = self._api_error(response)
                self._log('warn',
                          'chat variant %s rejected (%d: %s); trying next'
                          % (name, response.status_code, last_error))
                continue
            if response.status_code == 401:
                raise DeepSeekError('authentication failed (401): %s'
                                    % self._api_error(response))
            if response.status_code >= 500:
                raise DeepSeekError('server error %d: %s'
                                    % (response.status_code,
                                       self._api_error(response)))
            if response.status_code >= 400:
                raise DeepSeekError('http %d: %s'
                                    % (response.status_code,
                                       self._api_error(response)))

            try:
                payload = response.json()
            except ValueError:
                raise DeepSeekError('response is not json')

            if isinstance(payload.get('error'), dict):
                raise DeepSeekError(
                    'api error: %s'
                    % payload['error'].get('message', str(payload['error'])))
            choices = payload.get('choices') or []
            if not choices:
                raise DeepSeekError('response has no choices')
            message = choices[0].get('message') or {}
            content = message.get('content')
            return '' if content is None else str(content)

        self.last_error = last_error or 'unknown error'
        raise DeepSeekError('all request variants failed: %s' % last_error)

    # ------------------------------------------------------------ intents
    def parse_intent(self, question, world):
        """Return (plan, warnings).  Retries once on empty/broken output."""
        messages = [
            {'role': 'system', 'content': intent_schema.SYSTEM_PROMPT},
            {'role': 'user',
             'content': intent_schema.build_user_prompt(question, world)},
        ]

        last_error = None
        for attempt in range(2):
            content = self.chat(messages, max_tokens=512, temperature=0.1)
            if not content.strip():
                last_error = 'empty content from model'
                self._log('warn', 'intent parse attempt %d: empty content'
                          % (attempt + 1))
                continue
            try:
                return intent_schema.parse_plan_text(content)
            except intent_schema.IntentSchemaError as exc:
                last_error = str(exc)
                self._log('warn', 'intent parse attempt %d failed: %s'
                          % (attempt + 1, exc))
        raise DeepSeekError('intent parsing failed: %s' % last_error)

    # ------------------------------------------------------------ phrasing
    def phrase(self, question, fact):
        """Rephrase a deterministic fact; returns ``fact`` on any failure."""
        messages = [
            {'role': 'system',
             'content': intent_schema.phrase_system_prompt()},
            {'role': 'user',
             'content': 'P5_QUESTION=%s\nP5_FACT=%s' % (question, fact)},
        ]
        try:
            content = self.chat(messages, max_tokens=200, temperature=0.2)
        except DeepSeekError as exc:
            self._log('warn', 'phrasing failed (%s); using raw fact' % exc)
            return fact
        content = content.strip()
        return content if content else fact
