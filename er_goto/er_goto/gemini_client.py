# SPDX-License-Identifier: GPL-3.0-only
"""Gemini Robotics-ER pointing: request with timeout and retry, response parsing.

google-genai is imported only when a client is created, so the parsing helpers
can be unit-tested without the SDK installed.
"""

import json
import os
import re
import time
from dataclasses import dataclass

DEFAULT_MODEL = 'gemini-robotics-er-2-preview'

PROMPT_TEMPLATE = (
    'Point to the {target}. Answer as JSON: [{{"point": [y, x], "label": "<name>"}}]. '
    'Points are [y, x] normalized to 0-1000. If several match, put the best match first. '
    'If none is visible, answer [].'
)

_FENCE = re.compile(r'```(?:json)?\s*(.*?)\s*```', re.DOTALL)


@dataclass(frozen=True)
class Point:
    """One Gemini point: y and x normalized to 0-1000."""
    y: float
    x: float
    label: str


class GeminiError(Exception):
    """Transient failure (timeout, network, server or rate limit) after all retries."""


class GeminiFatalError(Exception):
    """Request rejected (bad key, unknown model, bad request); retrying will not help."""


class ResponseParseError(ValueError):
    """The model answered, but not with the expected JSON list of points."""


def _is_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def parse_points(text):
    """Parse a pointing answer into a list of Points; [] means nothing was found.

    Accepts bare JSON or JSON inside ``` / ```json fences, with or without text
    around the fence. Raises ResponseParseError on anything else.
    """
    if not text or not text.strip():
        raise ResponseParseError('empty response')
    fenced = _FENCE.search(text)
    body = fenced.group(1) if fenced else text.strip()
    try:
        data = json.loads(body)
    except json.JSONDecodeError as exc:
        raise ResponseParseError(f'invalid JSON: {exc}') from exc
    if isinstance(data, dict):
        data = [data]
    if not isinstance(data, list):
        raise ResponseParseError(f'expected a JSON list, got {type(data).__name__}')

    points = []
    for item in data:
        if not isinstance(item, dict) or 'point' not in item:
            raise ResponseParseError(f'item without "point": {item!r}')
        point = item['point']
        if not (isinstance(point, (list, tuple)) and len(point) == 2
                and all(_is_number(c) for c in point)):
            raise ResponseParseError(f'"point" is not [y, x]: {point!r}')
        y, x = float(point[0]), float(point[1])
        if not (0.0 <= y <= 1000.0 and 0.0 <= x <= 1000.0):
            raise ResponseParseError(f'point outside 0-1000: {point!r}')
        points.append(Point(y=y, x=x, label=str(item.get('label', ''))))
    return points


def point_to_pixel(point, width, height):
    """Gemini [y, x] (0-1000) to integer pixel (u, v), clamped inside the image."""
    u = min(int(point.x / 1000.0 * width), width - 1)
    v = min(int(point.y / 1000.0 * height), height - 1)
    return u, v


class GeminiPointer:
    """Asks Gemini Robotics-ER to point at a target in a JPEG image."""

    def __init__(self, model=DEFAULT_MODEL, thinking_level='medium', timeout=15.0,
                 retries=1, retry_delay=1.0, api_key=None):
        try:
            from google import genai
            from google.genai import errors, types
        except ImportError as exc:
            raise GeminiFatalError(
                'google-genai is not installed: pip3 install google-genai') from exc

        api_key = api_key or os.environ.get('GEMINI_API_KEY')
        if not api_key:
            raise GeminiFatalError('GEMINI_API_KEY is not set (see .env.example)')

        self._types = types
        self._errors = errors
        self._model = model
        self._attempts = 1 + max(0, int(retries))
        self._retry_delay = retry_delay
        # HttpOptions.timeout is in milliseconds.
        self._client = genai.Client(
            api_key=api_key, http_options=types.HttpOptions(timeout=int(timeout * 1000)))
        thinking = (types.ThinkingConfig(thinking_level=thinking_level)
                    if thinking_level else None)
        self._config = types.GenerateContentConfig(thinking_config=thinking)

    def ask(self, jpeg_bytes, prompt):
        """Send one image and prompt; return the response text.

        Raises GeminiFatalError on 4xx errors other than 429, GeminiError when
        every attempt timed out or failed transiently.
        """
        image = self._types.Part.from_bytes(data=jpeg_bytes, mime_type='image/jpeg')
        last_error = None
        for attempt in range(1, self._attempts + 1):
            try:
                response = self._client.models.generate_content(
                    model=self._model, contents=[image, prompt], config=self._config)
                return response.text
            except self._errors.ClientError as exc:
                if exc.code != 429:
                    raise GeminiFatalError(f'Gemini rejected the request: {exc}') from exc
                last_error = exc
            except Exception as exc:  # server errors, timeouts, connection errors
                last_error = exc
            if attempt < self._attempts:
                time.sleep(self._retry_delay)
        raise GeminiError(
            f'Gemini failed after {self._attempts} attempt(s): '
            f'{type(last_error).__name__}: {last_error}') from last_error

    def point(self, jpeg_bytes, target):
        """Return (points, raw_text) for target; points is [] if it is not visible.

        Raises ResponseParseError, GeminiError or GeminiFatalError.
        """
        text = self.ask(jpeg_bytes, PROMPT_TEMPLATE.format(target=target))
        return parse_points(text), text
