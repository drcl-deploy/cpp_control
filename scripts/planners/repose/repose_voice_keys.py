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
import json
import os
import re
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
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


@dataclass(frozen=True)
class PulseSource:
    name: str
    description: str


@dataclass(frozen=True)
class CaptureInput:
    backend: str
    device: str
    description: str


def capture_hardware() -> str:
    if shutil.which('arecord') is None:
        raise VoiceKeysError('arecord is missing; install the alsa-utils package')
    result = subprocess.run(
        ['arecord', '-l'], check=False, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True)
    return result.stdout


def capture_pulse_sources() -> str:
    """Return Pulse source JSON, or an empty string when Pulse is unavailable."""
    if shutil.which('pactl') is None or shutil.which('parec') is None:
        return ''
    result = subprocess.run(
        ['pactl', '--format=json', 'list', 'sources'], check=False,
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    return result.stdout if result.returncode == 0 else ''


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


def parse_pulse_sources(output: str) -> list[PulseSource]:
    if not output:
        return []
    try:
        records = json.loads(output)
    except (json.JSONDecodeError, TypeError):
        return []
    if not isinstance(records, list):
        return []
    sources = []
    for record in records:
        if not isinstance(record, dict):
            continue
        name = str(record.get('name', ''))
        if not name or name.endswith('.monitor'):
            continue
        description = str(record.get('description', name))
        sources.append(PulseSource(name=name, description=description))
    return sources


def resolve_capture(selector: str, alsa_output: str,
                    pulse_output: str) -> CaptureInput:
    """Prefer the shared desktop audio server, then fall back to raw ALSA."""
    pulse_sources = parse_pulse_sources(pulse_output)
    if selector in ('default', 'pulse', '@DEFAULT_SOURCE@'):
        if pulse_sources:
            return CaptureInput(
                backend='pulse', device='@DEFAULT_SOURCE@',
                description='PulseAudio/PipeWire default source')
        if selector == 'default':
            return CaptureInput(
                backend='alsa', device='default',
                description='ALSA default source')
        raise VoiceKeysError(
            'the PulseAudio/PipeWire capture service is unavailable')

    # Exact Pulse source names and raw ALSA device expressions bypass friendly
    # matching. Most ALSA expressions contain a colon.
    pulse_exact = [source for source in pulse_sources
                   if source.name == selector]
    if pulse_exact:
        return CaptureInput(
            backend='pulse', device=selector,
            description=f'{pulse_exact[0].description} via PulseAudio/PipeWire')
    if selector.startswith('alsa_input.'):
        raise VoiceKeysError(f'Pulse source does not exist: {selector}')
    if ':' in selector:
        return CaptureInput(
            backend='alsa', device=selector, description=selector)

    # Friendly selectors (for example CMTECK) use Pulse/PipeWire first. The
    # desktop sound server normally owns the raw USB endpoint, so opening
    # plughw directly would otherwise fail with EBUSY.
    pulse_matches = [
        source for source in pulse_sources
        if selector.lower() in f'{source.name} {source.description}'.lower()
    ]
    if len(pulse_matches) == 1:
        source = pulse_matches[0]
        return CaptureInput(
            backend='pulse', device=source.name,
            description=f'{source.description} via PulseAudio/PipeWire')
    if len(pulse_matches) > 1:
        choices = '\n'.join(
            f'  {item.description} [{item.name}]' for item in pulse_matches)
        raise VoiceKeysError(
            f'capture selector {selector!r} is ambiguous:\n{choices}\n'
            'pass the full Pulse source name')

    matches = [
        device for device in parse_capture_devices(alsa_output)
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
    device = matches[0]
    return CaptureInput(
        backend='alsa', device=device.alsa_name,
        description=f'{device.card_name} via raw ALSA')


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


def capture_command(capture_input: CaptureInput, rate: int) -> list[str]:
    if capture_input.backend == 'pulse':
        return [
            'parec', '--record', '--device', capture_input.device, '--raw',
            '--format=s16le', '--channels=1', f'--rate={rate}',
            '--client-name=repose_voice_keys',
            '--stream-name=Repose voice commands',
        ]
    return [
        'arecord', '-q', '-D', capture_input.device, '-t', 'raw',
        '-f', 'S16_LE', '-c', '1', '-r', str(rate),
    ]


def start_capture(capture_input: CaptureInput,
                  rate: int) -> subprocess.Popen:
    command = capture_command(capture_input, rate)
    try:
        return subprocess.Popen(
            command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except OSError as error:
        raise VoiceKeysError(
            f'could not start {command[0]}: {error}') from error


def capture_failure(process: subprocess.Popen,
                    capture_input: CaptureInput) -> VoiceKeysError:
    try:
        exit_code = process.wait(timeout=0.25)
    except subprocess.TimeoutExpired:
        exit_code = process.poll()
    detail = ''
    if exit_code is not None and process.stderr is not None:
        raw_detail = process.stderr.read()
        if isinstance(raw_detail, bytes):
            detail = raw_detail.decode(errors='replace').strip()
        else:
            detail = str(raw_detail).strip()
    program = 'parec' if capture_input.backend == 'pulse' else 'arecord'
    message = f'{program} stopped unexpectedly (exit {exit_code})'
    if detail:
        message += f': {detail}'
    if capture_input.backend == 'alsa' and 'busy' in detail.lower():
        message += (
            '\nAnother process (often the desktop audio service) owns this '
            'raw ALSA device. Select the microphone in Sound Settings and '
            'retry with --device default.')
    return VoiceKeysError(message)


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
    pulse_output = capture_pulse_sources()
    capture_input = resolve_capture(args.device, hardware, pulse_output)
    recognizer = load_recognizer(args.rate, args.model)
    capture = start_capture(capture_input, args.rate)
    keyboard = None
    if capture.stdout is None:
        raise VoiceKeysError('capture process did not provide an audio stream')

    mode = 'DRY RUN — no keys will be typed' if args.dry_run else 'ARMED'
    try:
        keyboard = None if args.dry_run else X11Keyboard()
        print(f'voice_keys: input={capture_input.description} '
              f'rate={args.rate} Hz | {mode}')
        print('voice_keys: mute; focus the Repose console; then unmute, '
              'say one of:')
        print(f"            {', '.join(COLORS)}")
        last_command = 0.0
        while True:
            audio = capture.stdout.read(8000)
            if not audio:
                raise capture_failure(capture, capture_input)
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


def list_devices() -> None:
    pulse_sources = parse_pulse_sources(capture_pulse_sources())
    if pulse_sources:
        print('PulseAudio/PipeWire sources:')
        for source in pulse_sources:
            print(f'  {source.description} [{source.name}]')
        print()
    print('ALSA capture hardware:')
    print(capture_hardware(), end='')


def self_test() -> None:
    sample = (
        'card 3: Device [MV-SILICON CMTECK], device 0: USB Audio [USB Audio]\n')
    pulse_sample = json.dumps([{
        'index': 7,
        'name': 'alsa_input.usb-MV_SILICON_CMTECK-00.mono-fallback',
        'description': 'MV-SILICON CMTECK Mono',
    }, {
        'index': 8,
        'name': 'alsa_output.pci-0000_00_1f.3.analog-stereo.monitor',
        'description': 'Monitor of Built-in Audio',
    }])
    devices = parse_capture_devices(sample)
    assert len(devices) == 1
    assert devices[0].alsa_name == 'plughw:CARD=3,DEV=0'
    sources = parse_pulse_sources(pulse_sample)
    assert len(sources) == 1
    pulse_capture = resolve_capture('cmteck', sample, pulse_sample)
    assert pulse_capture.backend == 'pulse'
    assert pulse_capture.device == sources[0].name
    assert resolve_capture(sources[0].name, sample, pulse_sample) == pulse_capture
    alsa_capture = resolve_capture('cmteck', sample, '')
    assert alsa_capture == CaptureInput(
        backend='alsa', device='plughw:CARD=3,DEV=0',
        description='MV-SILICON CMTECK via raw ALSA')
    default_capture = resolve_capture('default', sample, pulse_sample)
    assert default_capture.device == '@DEFAULT_SOURCE@'
    assert capture_command(pulse_capture, 16000)[0] == 'parec'
    assert capture_command(alsa_capture, 16000)[0] == 'arecord'
    failed_capture = subprocess.Popen(
        [sys.executable, '-c',
         'import sys; sys.stderr.write("Device or resource busy\\n"); '
         'raise SystemExit(7)'],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    assert failed_capture.stdout is not None
    assert failed_capture.stdout.read() == b''
    failure = str(capture_failure(failed_capture, alsa_capture))
    assert 'exit 7' in failure
    assert '--device default' in failure
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
        help='audio source name or friendly substring (default: CMTECK)')
    parser.add_argument(
        '--list-devices', action='store_true',
        help='print Pulse/PipeWire and ALSA capture devices, then exit')
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
        list_devices()
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
