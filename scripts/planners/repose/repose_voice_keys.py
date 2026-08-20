#!/usr/bin/env python3
"""
Turn six spoken colour names into keys for the focused Repose console.

This process runs on the offboard X11 desktop.  It does not join ROS: ALSA
feeds a grammar-constrained Vosk recognizer, and XTest types one digit into the
currently focused window.  Thus the existing Repose console remains the only
publisher of /vibe/sonic/goal_color in both sim2sim and hardware runs.
"""

from __future__ import annotations

import argparse
import ctypes
import ctypes.util
from dataclasses import dataclass
import json
import os
import re
import shutil
import subprocess
import sys
import time
from typing import Any


COLORS = ('red', 'orange', 'green', 'yellow', 'blue', 'pink')
COLOR_KEYS = {name: str(index) for index, name in enumerate(COLORS)}
DEFAULT_DEVICE = 'CMTECK'
DEFAULT_RATE = 16000


class VoiceKeysError(RuntimeError):
    """An operator-facing setup or runtime error."""


@dataclass(frozen=True)
class CaptureDevice:
    card: int
    device: int
    card_id: str
    card_name: str
    description: str

    @property
    def alsa_name(self) -> str:
        # Resolve the user's stable name at process start, then use the numeric
        # card.  USB card numbers can move between boots but cannot move while
        # this arecord process is alive.
        return f'plughw:CARD={self.card},DEV={self.device}'


def capture_hardware() -> str:
    if shutil.which('arecord') is None:
        raise VoiceKeysError('arecord is missing; install the alsa-utils package')
    result = subprocess.run(
        ['arecord', '-l'], check=False, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True)
    return result.stdout


def parse_capture_devices(output: str) -> list[CaptureDevice]:
    devices = []
    pattern = re.compile(
        r'^card\s+(\d+):\s+(\S+)\s+\[(.*?)\],\s+'
        r'device\s+(\d+):\s+(.*)$')
    for line in output.splitlines():
        match = pattern.match(line.strip())
        if match:
            devices.append(CaptureDevice(
                card=int(match.group(1)), device=int(match.group(4)),
                card_id=match.group(2), card_name=match.group(3),
                description=line.strip()))
    return devices


def resolve_device(selector: str, output: str) -> str:
    # ALSA device expressions are passed through.  Otherwise accept a friendly
    # case-insensitive substring such as the USB product name 'CMTECK'.
    if selector == 'default' or ':' in selector:
        return selector
    matches = [
        device for device in parse_capture_devices(output)
        if selector.lower() in device.description.lower()
    ]
    if not matches:
        raise VoiceKeysError(
            f'no capture device matches {selector!r}; run --list-devices')
    if len(matches) > 1:
        choices = '\n'.join(f'  {item.description}' for item in matches)
        raise VoiceKeysError(
            f'capture selector {selector!r} is ambiguous:\n{choices}\n'
            'pass an ALSA name such as plughw:CARD=2,DEV=0')
    return matches[0].alsa_name


def recognized_color(result: dict[str, Any], confidence: float) -> tuple[str, float] | None:
    """Return one exact, confident grammar word; reject every other result."""
    text = str(result.get('text', '')).strip().lower()
    words = result.get('result', [])
    if text not in COLOR_KEYS or len(words) != 1:
        return None
    word = words[0]
    if str(word.get('word', '')).lower() != text:
        return None
    score = float(word.get('conf', 0.0))
    return (text, score) if score >= confidence else None


class X11Keyboard:
    """The tiny piece of xdotool we need, directly through X11/XTest."""

    def __init__(self) -> None:
        if os.environ.get('XDG_SESSION_TYPE', 'x11').lower() != 'x11':
            raise VoiceKeysError(
                'keyboard injection requires an X11 session; this desktop is not X11')
        x11_name = ctypes.util.find_library('X11')
        xtst_name = ctypes.util.find_library('Xtst')
        if not x11_name or not xtst_name:
            raise VoiceKeysError('libX11/libXtst is missing')
        self.x11 = ctypes.CDLL(x11_name)
        self.xtst = ctypes.CDLL(xtst_name)
        self.x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        self.x11.XOpenDisplay.restype = ctypes.c_void_p
        self.x11.XStringToKeysym.argtypes = [ctypes.c_char_p]
        self.x11.XStringToKeysym.restype = ctypes.c_ulong
        self.x11.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        self.x11.XKeysymToKeycode.restype = ctypes.c_ubyte
        self.x11.XFlush.argtypes = [ctypes.c_void_p]
        self.x11.XFlush.restype = ctypes.c_int
        self.x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
        self.x11.XCloseDisplay.restype = ctypes.c_int
        self.xtst.XTestFakeKeyEvent.argtypes = [
            ctypes.c_void_p, ctypes.c_uint, ctypes.c_int, ctypes.c_ulong]
        self.xtst.XTestFakeKeyEvent.restype = ctypes.c_int
        self.display = self.x11.XOpenDisplay(None)
        if not self.display:
            display_name = os.environ.get('DISPLAY', '(unset)')
            raise VoiceKeysError(
                f'cannot open X11 display {display_name}')

    def type_digit(self, digit: str) -> None:
        if digit not in '012345':
            raise VoiceKeysError(f'refusing to type invalid colour key {digit!r}')
        keysym = self.x11.XStringToKeysym(digit.encode('ascii'))
        keycode = self.x11.XKeysymToKeycode(self.display, keysym)
        if not keycode:
            raise VoiceKeysError(f'X11 has no keycode for {digit!r}')
        if not self.xtst.XTestFakeKeyEvent(self.display, keycode, True, 0):
            raise VoiceKeysError('XTest key-down injection failed')
        if not self.xtst.XTestFakeKeyEvent(self.display, keycode, False, 0):
            raise VoiceKeysError('XTest key-up injection failed')
        self.x11.XFlush(self.display)

    def close(self) -> None:
        if self.display:
            self.x11.XCloseDisplay(self.display)
            self.display = None


def load_recognizer(rate: int, model_path: str):
    try:
        from vosk import KaldiRecognizer, Model, SetLogLevel
    except ImportError as error:
        raise VoiceKeysError(
            'the offboard Vosk package is missing; install it with:\n'
            '  python3 -m pip install --user vosk') from error

    SetLogLevel(-1)
    if model_path:
        path = os.path.abspath(os.path.expanduser(model_path))
        if not os.path.isdir(path):
            raise VoiceKeysError(f'Vosk model directory does not exist: {path}')
        model = Model(path)
    else:
        print('voice_keys: loading the small English Vosk model '
              '(the first run may download it)', flush=True)
        model = Model(lang='en-us')
    grammar = json.dumps([*COLORS, '[unk]'])
    recognizer = KaldiRecognizer(model, rate, grammar)
    recognizer.SetWords(True)
    return recognizer


def start_capture(device: str, rate: int) -> subprocess.Popen:
    command = [
        'arecord', '-q', '-D', device, '-t', 'raw', '-f', 'S16_LE',
        '-c', '1', '-r', str(rate),
    ]
    try:
        return subprocess.Popen(command, stdout=subprocess.PIPE)
    except OSError as error:
        raise VoiceKeysError(f'could not start arecord: {error}') from error


def stop_capture(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=1.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()


def listen(args: argparse.Namespace) -> None:
    hardware = capture_hardware()
    device = resolve_device(args.device, hardware)
    recognizer = load_recognizer(args.rate, args.model)
    keyboard = None if args.dry_run else X11Keyboard()
    capture = start_capture(device, args.rate)
    if capture.stdout is None:
        raise VoiceKeysError('arecord did not provide an audio stream')

    mode = 'DRY RUN — no keys will be typed' if args.dry_run else 'ARMED'
    print(f'voice_keys: input={device} rate={args.rate} Hz | {mode}')
    print('voice_keys: mute; focus the Repose console; then unmute, say one of:')
    print(f"            {', '.join(COLORS)}")
    last_command = 0.0
    try:
        while True:
            audio = capture.stdout.read(8000)
            if not audio:
                raise VoiceKeysError(
                    f'arecord stopped unexpectedly (exit {capture.poll()})')
            if not recognizer.AcceptWaveform(audio):
                continue
            result = json.loads(recognizer.Result())
            accepted = recognized_color(result, args.confidence)
            text = str(result.get('text', '')).strip()
            if accepted is None:
                if text:
                    print(f'voice_keys: ignored {text!r}', flush=True)
                continue
            color, score = accepted
            now = time.monotonic()
            if now - last_command < args.cooldown:
                print(f'voice_keys: ignored fast duplicate {color!r}', flush=True)
                continue
            key = COLOR_KEYS[color]
            if keyboard is not None:
                keyboard.type_digit(key)
            last_command = now
            action = 'heard' if args.dry_run else 'typed'
            print(f'\a voice_keys: {action} {color.upper()} -> {key} '
                  f'(confidence {score:.2f})', flush=True)
            # Start the next mute/unmute cycle with a clean recognizer state.
            recognizer.Reset()
    finally:
        stop_capture(capture)
        if keyboard is not None:
            keyboard.close()


def self_test() -> None:
    sample = (
        'card 3: Device [MV-SILICON CMTECK], device 0: USB Audio [USB Audio]\n')
    devices = parse_capture_devices(sample)
    assert len(devices) == 1
    assert devices[0].alsa_name == 'plughw:CARD=3,DEV=0'
    assert resolve_device('cmteck', sample) == 'plughw:CARD=3,DEV=0'
    assert resolve_device('default', sample) == 'default'
    good = {'text': 'blue', 'result': [{'word': 'blue', 'conf': 0.91}]}
    assert recognized_color(good, 0.8) == ('blue', 0.91)
    assert recognized_color(good, 0.95) is None
    phrase = {'text': 'blue blue', 'result': [
        {'word': 'blue', 'conf': 0.99}, {'word': 'blue', 'conf': 0.99}]}
    assert recognized_color(phrase, 0.8) is None
    assert list(COLOR_KEYS.values()) == ['0', '1', '2', '3', '4', '5']
    print('repose_voice_keys: self-test passed')


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--device', default=DEFAULT_DEVICE,
        help='ALSA device or substring from arecord -l (default: CMTECK)')
    parser.add_argument(
        '--list-devices', action='store_true',
        help='print ALSA capture hardware and exit')
    parser.add_argument(
        '--model', default=os.environ.get('REPOSE_VOICE_MODEL', ''),
        help='unpacked Vosk model; default lets Vosk find/download small en-us')
    parser.add_argument('--rate', type=int, default=DEFAULT_RATE)
    parser.add_argument(
        '--confidence', type=float, default=0.75,
        help='minimum Vosk word confidence (default: 0.75)')
    parser.add_argument(
        '--cooldown', type=float, default=1.0,
        help='minimum seconds between injected keys (default: 1.0)')
    parser.add_argument(
        '--dry-run', action='store_true',
        help='recognize and report colors without injecting keys')
    parser.add_argument('--self-test', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if not 0.0 <= args.confidence <= 1.0:
        parser.error('--confidence must be in [0, 1]')
    if args.cooldown < 0.0:
        parser.error('--cooldown cannot be negative')
    if args.rate <= 0:
        parser.error('--rate must be positive')
    return args


def main() -> None:
    args = parse_args()
    if args.self_test:
        self_test()
        return
    if args.list_devices:
        print(capture_hardware(), end='')
        return
    listen(args)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nvoice_keys: stopped', file=sys.stderr)
    except (VoiceKeysError, json.JSONDecodeError) as error:
        print(f'voice_keys: {error}', file=sys.stderr)
        raise SystemExit(1)
