#!/usr/bin/env python3
"""Video recording for the mj_sim viewer, toggled with F9.

The same idea as crl-humanoid-ros's `FrameRecorder.h`, ported to mj_sim's
Python viewer: screen-recording the window is laggy (the recorder has to
composite and re-encode the whole desktop) and it captures whatever happens to
be on top of it, so instead the scene is rendered a SECOND time into MuJoCo's
offscreen framebuffer and the raw RGB is piped into ffmpeg. Encoding runs on a
writer thread, so the sim loop pays for one extra scene draw plus a readback,
and only at the capture rate (30 Hz by default) rather than every step.

Offscreen rather than the window, for the same three reasons as the monitor's:
the video resolution is independent of the window size, the capture is
unaffected by occlusion or minimisation, and it does not care what the
compositor reports the drawable size to be (this rig runs XWayland, where that
goes wrong).

WHAT IS IN THE VIDEO. The scene as the viewer is showing it -- the recorder is
handed a copy of the viewer's camera and visualisation options every frame, so
following the robot, orbiting with the mouse and toggling frames with `E` all
show up. MuJoCo's own on-screen overlays (the help text, the left/right UI
panels) are drawn by the viewer straight to the window and never reach here.

THE CLOCK. Frames are emitted on a schedule and the last one is duplicated to
fill a gap, so the output is a constant-rate stream at `fps` whatever the loop
does. Which clock drives that schedule is the `clock` setting:

  sim   (default) SIMULATION time, `data.time`. The video always plays back at
        1x real time whatever the simulator was paced at -- so a `--speed 0`
        free-run, which plays 1.4x-1.8x fast in the window, still records a
        walk that walks at the right speed. Pauses are cut out.
  wall  WALL-CLOCK time. The video is what you watched, at the speed you
        watched it, pauses included as frozen frames.

CONFIGURATION. Defaults below, overridden by a `recording:` block in the sim
yaml, overridden in turn by the environment (both spellings are read, so a
shell set up for the crl-humanoid-ros monitor works here unchanged):

  DRCL_RECORD_DIR      / CRL_RECORD_DIR      output directory
  DRCL_RECORD_SIZE     / CRL_RECORD_SIZE     WxH of the video
  DRCL_RECORD_FPS      / CRL_RECORD_FPS      output frame rate
  DRCL_RECORD_ENCODER  / CRL_RECORD_ENCODER  ffmpeg encoder (try h264_nvenc)
  DRCL_RECORD_CRF      / CRL_RECORD_CRF      quality, lower is better
  DRCL_RECORD_FFMPEG   / CRL_RECORD_FFMPEG   ffmpeg binary
  DRCL_RECORD_PREFIX   / CRL_RECORD_PREFIX   file name prefix
  DRCL_RECORD_CLOCK    / CRL_RECORD_CLOCK    sim | wall

720p30 rather than the monitor's 1080p30 because this render happens INSIDE the
physics loop, which is paced to the wall clock. Measured on the G1 scene here:
8.2 ms per captured frame at 720p and 13.7 ms at 1080p, against a 33 ms budget
that also has to fit 16 physics steps (0.124 ms each). Both fit -- a 720p
recording held 1.00x real time over a 40 s run -- so 1080p is a `size` away;
just check the `[rate]` line still says 1.00x afterwards.
"""

import collections
import copy
import datetime
import os
import shutil
import subprocess
import threading
import time
from pathlib import Path

import numpy as np
import mujoco


DEFAULTS = {
    "dir": str(Path(__file__).resolve().parent.parent / "recordings"),
    "size": "1280x720",
    "fps": 30,
    "encoder": "libx264",
    "crf": 18,
    "ffmpeg": "ffmpeg",
    "prefix": "mj_sim",
    "clock": "sim",
}


def _env(key):
    """Environment override for one setting, DRCL_ first then CRL_."""
    for prefix in ("DRCL_RECORD_", "CRL_RECORD_"):
        value = os.environ.get(prefix + key.upper())
        if value:
            return value
    return None


def _parse_size(text, fallback=(1280, 720)):
    try:
        width, height = (int(part) for part in str(text).lower().split("x", 1))
    except (ValueError, AttributeError):
        print(f"[record] cannot parse size '{text}'; using {fallback[0]}x{fallback[1]}")
        return fallback
    if width < 16 or height < 16:
        return fallback
    # yuv420p needs even dimensions.
    return width & ~1, height & ~1


class FrameRecorder:
    """Offscreen scene capture piped into ffmpeg. Not thread-safe: `start`,
    `tick` and `stop` all belong to the thread that owns the sim loop."""

    # A long stall would otherwise dump an unbounded run of duplicate frames.
    # Past this the gap is DROPPED instead (the video is short by the stall)
    # rather than played back as a freeze that lasts as long as the stall did.
    MAX_REPEAT = 10
    MAX_BUFFERS = 8

    def __init__(self, model, cfg=None):
        settings = dict(DEFAULTS)
        settings.update(cfg or {})
        for key in settings:
            override = _env(key)
            if override is not None:
                settings[key] = override

        self._model = model
        self.width, self.height = _parse_size(settings["size"],
                                              _parse_size(DEFAULTS["size"]))
        try:
            self.fps = max(1, int(settings["fps"]))
        except (TypeError, ValueError):
            print(f"[record] cannot parse fps '{settings['fps']}'; using "
                  f"{DEFAULTS['fps']}")
            self.fps = int(DEFAULTS["fps"])
        self._dir = str(settings["dir"])
        self._prefix = str(settings["prefix"])
        self._encoder = str(settings["encoder"])
        self._crf = str(settings["crf"])
        self._ffmpeg = str(settings["ffmpeg"])
        self._clock = str(settings["clock"]).lower()
        if self._clock not in ("sim", "wall"):
            print(f"[record] unknown clock '{self._clock}'; using 'sim'")
            self._clock = "sim"

        self._renderer = None
        self._proc = None
        self._output_path = ""
        self._last_error = ""

        self._queue = collections.deque()
        self._pool = []
        self._allocated = 0
        self._cv = threading.Condition()
        self._writer = None
        self._stop_requested = False
        self._write_failed = False

        self._t0 = 0.0
        self._frames_written = 0

    @property
    def is_recording(self):
        return self._proc is not None

    @property
    def output_path(self):
        return self._output_path

    @property
    def last_error(self):
        return self._last_error

    def toggle(self, data):
        """F9. Returns True if a recording is running afterwards."""
        if self.is_recording:
            self.stop()
        else:
            self.start(data)
        return self.is_recording

    def start(self, data):
        if self.is_recording:
            return True
        self._last_error = ""

        if shutil.which(self._ffmpeg) is None:
            self._last_error = f"'{self._ffmpeg}' is not on PATH"
            print(f"[record] cannot start: {self._last_error}")
            return False

        if self._renderer is None and not self._make_renderer():
            return False

        try:
            os.makedirs(self._dir, exist_ok=True)
        except OSError as exc:
            self._last_error = f"cannot create {self._dir}: {exc}"
            print(f"[record] cannot start: {self._last_error}")
            return False

        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self._output_path = os.path.join(self._dir, f"{self._prefix}_{stamp}.mp4")

        # The Renderer hands back rows top-down already (it flips the GL
        # readback itself), so unlike the C++ recorder there is no -vf vflip.
        cmd = [
            self._ffmpeg, "-hide_banner", "-loglevel", "error", "-y",
            "-f", "rawvideo", "-pixel_format", "rgb24",
            "-video_size", f"{self.width}x{self.height}",
            "-framerate", str(self.fps),
            "-i", "-", "-an", "-c:v", self._encoder,
        ]
        if "nvenc" in self._encoder:
            cmd += ["-preset", "p4", "-tune", "hq", "-rc", "vbr", "-cq", self._crf]
        else:
            cmd += ["-preset", "ultrafast", "-crf", self._crf]
        cmd += ["-pix_fmt", "yuv420p", self._output_path]

        try:
            self._proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
        except OSError as exc:
            self._last_error = f"could not start ffmpeg: {exc}"
            print(f"[record] cannot start: {self._last_error}")
            self._proc = None
            return False

        self._stop_requested = False
        self._write_failed = False
        self._frames_written = 0
        self._t0 = self._now(data)
        self._writer = threading.Thread(target=self._writer_loop, daemon=True)
        self._writer.start()

        print(f"[record] REC -> {self._output_path} "
              f"({self.width}x{self.height} @ {self.fps} fps, {self._clock} clock, "
              f"{self._encoder}) — F9 to stop", flush=True)
        return True

    def tick(self, data, view=None):
        """Call once per sim step with `data` in a consistent state. Cheap
        unless an output frame is due, which is the point of calling it often.

        `view` is a callable returning `(camera, scene_option)`; it is invoked
        only when a frame is actually rendered, so the caller does not copy the
        viewer's camera 500 times a second to use 30 of them."""
        if not self.is_recording:
            return
        if self._write_failed:
            self._last_error = "ffmpeg stopped accepting frames"
            print(f"[record] {self._last_error}; stopping", flush=True)
            self.stop()
            return

        elapsed = self._now(data) - self._t0
        if elapsed < 0.0:
            # The simulation was reset under us; rebase rather than wait for
            # data.time to climb back up to where it was.
            self._t0 = self._now(data)
            elapsed = 0.0
        due = int(elapsed * self.fps) + 1
        need = due - self._frames_written
        if need <= 0:
            return
        if need > self.MAX_REPEAT:
            need = self.MAX_REPEAT
            self._frames_written = due - need

        buffer = self._acquire()
        if buffer is None:
            # The writer is behind and the pool is empty. Skip this capture; the
            # schedule above duplicates the next frame to cover the gap, so the
            # timing stays right.
            return

        camera, scene_option = (-1, None)
        if view is not None:
            camera, scene_option = view()
        self._renderer.update_scene(data, camera=camera, scene_option=scene_option)
        self._renderer.render(out=buffer)

        self._frames_written += need
        with self._cv:
            self._queue.append((buffer, need))
            self._cv.notify()

    def stop(self):
        """Flush the queue, close the pipe, wait for ffmpeg. Safe when idle."""
        if not self.is_recording:
            return
        with self._cv:
            self._stop_requested = True
            self._cv.notify_all()
        if self._writer is not None:
            self._writer.join(timeout=30.0)
            self._writer = None
        proc, self._proc = self._proc, None
        try:
            if proc.stdin is not None:
                proc.stdin.close()
        except OSError:
            pass
        try:
            proc.wait(timeout=30.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
        with self._cv:
            self._queue.clear()
            self._pool.clear()
            self._allocated = 0

        # "Saved" only if ffmpeg actually left a file behind. It refuses on its
        # own terms -- an encoder this build does not have, a full disk -- and
        # prints why on the stderr it shares with us, so the useful thing to add
        # is a line that does not claim otherwise.
        written = os.path.exists(self._output_path) and os.path.getsize(self._output_path) > 0
        if written and not self._write_failed:
            seconds = self._frames_written / float(self.fps)
            print(f"[record] saved {self._output_path} "
                  f"({self._frames_written} frames, {seconds:.1f}s)", flush=True)
        elif not written:
            self._last_error = f"ffmpeg exited {proc.returncode} without a usable file"
            print(f"[record] NOT saved: {self._last_error} "
                  f"(its own message is above)", flush=True)
        else:
            self._last_error = f"ffmpeg exited {proc.returncode} mid-stream"
            print(f"[record] {self._output_path} is TRUNCATED: {self._last_error} "
                  f"(its own message is above)", flush=True)

    def close(self):
        """Shutdown path: finalise the mp4 and abandon the GL context.

        The context is deliberately NOT destroyed. This runs on the way out --
        window closed, Ctrl-C, controller gone -- and MuJoCo's GLContext frees
        its GLFW window with a `glfwMakeContextCurrent(None)`, which during
        interpreter teardown lands in a terminated GLFW and aborts the process:

            python3: .../glfw-3.4/src/posix_thread.c:70: _glfwPlatformSetTls:
            Assertion `tls->posix.allocated == GLFW_TRUE' failed.

        It fires after the video is safely closed, so it costs nothing but a
        crash report where a clean exit should be -- and it fires whether or not
        we call `Renderer.close()` ourselves, because the garbage collector gets
        there anyway. Nulling the handles is what actually silences it (measured
        both ways). The process is exiting; the OS reclaims the context.
        """
        self.stop()
        renderer, self._renderer = self._renderer, None
        if renderer is not None:
            gl_context = renderer._gl_context  # pylint: disable=protected-access
            if gl_context is not None:
                # Drop the window handle so GLContext.free(), which the garbage
                # collector calls on the way out, does not run
                # glfwMakeContextCurrent on it.
                gl_context._context = None  # pylint: disable=protected-access
            renderer._gl_context = None  # pylint: disable=protected-access
            renderer._mjr_context = None  # pylint: disable=protected-access

    # -- internals -----------------------------------------------------------

    def _now(self, data):
        return data.time if self._clock == "sim" else time.perf_counter()

    def _make_renderer(self):
        # mjr_makeContext sizes the offscreen buffer from the MODEL, whose
        # default is 640x480 -- too small for a usable video, and Renderer
        # raises rather than shrinking to fit. Grow it before the context is
        # made; the viewer's own context was built earlier and is unaffected.
        vis = self._model.vis.global_
        vis.offwidth = max(vis.offwidth, self.width)
        vis.offheight = max(vis.offheight, self.height)
        try:
            started = time.perf_counter()
            self._renderer = mujoco.Renderer(self._model, self.height, self.width)
        except Exception as exc:  # GL context creation fails in many ways
            self._renderer = None
            self._last_error = f"offscreen renderer failed: {exc}"
            print(f"[record] cannot start: {self._last_error}")
            print("[record] (a headless shell needs MUJOCO_GL=egl or osmesa)")
            return False
        print(f"[record] offscreen renderer ready in "
              f"{time.perf_counter() - started:.2f}s", flush=True)
        return True

    def _writer_loop(self):
        while True:
            with self._cv:
                while not self._queue and not self._stop_requested:
                    self._cv.wait()
                if not self._queue:
                    return  # stop requested and drained
                buffer, repeat = self._queue.popleft()
            if not self._write_failed:
                try:
                    for _ in range(repeat):
                        self._proc.stdin.write(buffer)
                except (BrokenPipeError, OSError, ValueError, AttributeError):
                    self._write_failed = True
            self._release(buffer)

    def _acquire(self):
        with self._cv:
            if self._pool:
                return self._pool.pop()
            if self._allocated < self.MAX_BUFFERS:
                self._allocated += 1
                return np.empty((self.height, self.width, 3), dtype=np.uint8)
        return None

    def _release(self, buffer):
        with self._cv:
            self._pool.append(buffer)


def camera_snapshot(viewer):
    """A copy of the viewer's camera and visualisation options, taken under the
    handle's lock -- the viewer renders on its own thread and writes both while
    the mouse is moving, and mjv_updateScene reading them mid-write is a torn
    frame. Copying is cheaper than holding the lock across the scene update,
    which would stutter the window."""
    with viewer.lock():
        return copy.copy(viewer.cam), copy.copy(viewer.opt)
