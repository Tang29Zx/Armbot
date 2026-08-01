from __future__ import annotations

import atexit
import logging
import shlex
import signal
import subprocess
import threading
import time
from typing import Any


class VideoPusher:
    def __init__(self, config: dict[str, Any], logger: logging.Logger | None = None) -> None:
        self.config = config
        self.logger = logger or logging.getLogger("rdk_video_push")
        self.process: subprocess.Popen[str] | None = None
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._monitor_thread: threading.Thread | None = None
        self._restart_count = 0
        self._last_exit_code = 0
        self._registered_exit = False

    def start_stream(self) -> None:
        with self._lock:
            if self.is_running():
                self.logger.info("Stream is already running with pid=%s", self.process.pid)
                return

            self._stop_event.clear()
            self._restart_count = 0
            self._last_exit_code = 0
            self._start_process()
            self._ensure_monitor_thread()
            self._register_exit_handlers()

    def stop_stream(self) -> None:
        with self._lock:
            self._stop_event.set()
            self._stop_process()

    def restart_stream(self) -> None:
        self.logger.info("Restarting stream")
        self.stop_stream()
        monitor_thread = self._monitor_thread
        if monitor_thread and monitor_thread is not threading.current_thread():
            monitor_thread.join()
        self.start_stream()

    def is_running(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def wait(self) -> int:
        monitor_thread = self._monitor_thread
        if monitor_thread is None:
            return self._last_exit_code
        if monitor_thread is threading.current_thread():
            raise RuntimeError("VideoPusher.wait() cannot run in the monitor thread")
        monitor_thread.join()
        return self._last_exit_code

    def build_command(self) -> list[str]:
        video = self.config["video"]
        codec = video["codec"].lower()
        transport = video["transport"].lower()
        ffmpeg_path = str(video.get("ffmpeg_path") or "ffmpeg")

        command = [
            ffmpeg_path,
            "-hide_banner",
            "-loglevel",
            "info",
            "-fflags",
            "nobuffer",
            "-flags",
            "low_delay",
        ]
        command.extend(_as_list(video.get("extra_input_args")))
        command.extend([
            "-f",
            "v4l2",
            "-input_format",
            str(video.get("input_format") or "mjpeg"),
            "-video_size",
            f"{int(video['width'])}x{int(video['height'])}",
            "-framerate",
            str(int(video["fps"])),
            "-i",
            str(video["camera_device"]),
            "-an",
            "-vf",
            f"fps={int(video['fps'])}",
        ])
        command.extend(self._encoder_args(codec))
        command.extend(_as_list(video.get("extra_encoder_args")))
        command.extend(["-f", self._output_format(transport)])
        command.extend(_as_list(video.get("extra_output_args")))
        command.append(self._output_url(transport))
        return command

    def _encoder_args(self, codec: str) -> list[str]:
        video = self.config["video"]
        bitrate = str(video["bitrate"])
        encoder_name = str(video.get("encoder_name") or "").strip()

        if encoder_name:
            encoder = encoder_name
        elif codec == "h264":
            encoder = "libx264"
        elif codec in ("h265", "hevc"):
            encoder = "libx265"
        else:
            raise ValueError(f"Unsupported codec: {codec}")

        args = ["-c:v", encoder, "-b:v", bitrate]

        if encoder in ("libx264", "libx265"):
            args.extend([
                "-preset",
                "veryfast",
                "-tune",
                "zerolatency",
                "-g",
                str(int(video["fps"]) * 2),
                "-bf",
                "0",
                "-pix_fmt",
                "yuv420p",
            ])
        else:
            args.extend(["-g", str(int(video["fps"]) * 2), "-bf", "0"])

        return args

    def _output_format(self, transport: str) -> str:
        return str(self.config["video"].get("output_format") or ("rtsp" if transport == "rtsp" else "mpegts"))

    def _output_url(self, transport: str) -> str:
        video = self.config["video"]
        if transport == "srt":
            return str(video["srt_url"])
        rtsp_url = video.get("rtsp_url")
        if not rtsp_url:
            raise ValueError("video.rtsp_url is required when video.transport is 'rtsp'")
        return str(rtsp_url)

    def _start_process(self) -> None:
        command = self.build_command()
        self.logger.info("Starting video stream")
        self.logger.info("Command: %s", shlex.join(command))
        self.process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        self.logger.info("ffmpeg pid=%s", self.process.pid)
        threading.Thread(target=self._pipe_output, args=(self.process,), daemon=True).start()

    def _stop_process(self) -> None:
        process = self.process
        if process is None:
            return

        if process.poll() is not None:
            self.logger.info("Stream process already exited with code=%s", process.returncode)
            self.process = None
            return

        self.logger.info("Stopping stream process pid=%s", process.pid)
        process.terminate()
        try:
            process.wait(timeout=int(self.config["runtime"].get("stop_timeout_sec", 5)))
        except subprocess.TimeoutExpired:
            self.logger.warning("Process did not stop in time; killing pid=%s", process.pid)
            process.kill()
            process.wait()

        self.logger.info("Stream process exited with code=%s", process.returncode)
        self.process = None

    def _ensure_monitor_thread(self) -> None:
        if self._monitor_thread and self._monitor_thread.is_alive():
            return
        self._monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self._monitor_thread.start()

    def _monitor_loop(self) -> None:
        runtime = self.config["runtime"]
        auto_restart = bool(runtime.get("auto_restart", True))
        max_restart_count = int(runtime.get("max_restart_count", 5))
        restart_interval_sec = float(runtime.get("restart_interval_sec", 3))

        while not self._stop_event.is_set():
            with self._lock:
                process = self.process

            if process is None:
                time.sleep(0.2)
                continue

            exit_code = process.wait()
            self._last_exit_code = exit_code
            self.logger.info("Stream process exited with code=%s", exit_code)

            with self._lock:
                if self.process is process:
                    self.process = None

            if self._stop_event.is_set():
                break

            if exit_code == 0:
                self.logger.info("Stream process ended normally; not restarting")
                break

            if not auto_restart:
                self.logger.warning("Stream process exited unexpectedly; auto restart disabled")
                break

            if self._restart_count >= max_restart_count:
                self.logger.error(
                    "Stream process exited unexpectedly; restart limit reached (%s)",
                    max_restart_count,
                )
                break

            self._restart_count += 1
            self.logger.warning(
                "Stream process exited unexpectedly; restarting %s/%s in %.1fs",
                self._restart_count,
                max_restart_count,
                restart_interval_sec,
            )
            if self._stop_event.wait(restart_interval_sec):
                break
            with self._lock:
                self._start_process()

    def _pipe_output(self, process: subprocess.Popen[str]) -> None:
        if process.stdout is None:
            return
        for line in process.stdout:
            self.logger.info("[ffmpeg] %s", line.rstrip())

    def _register_exit_handlers(self) -> None:
        if self._registered_exit:
            return

        def stop_on_signal(signum: int, _frame: object) -> None:
            self.logger.info("Received signal %s", signum)
            self.stop_stream()
            raise SystemExit(128 + signum)

        atexit.register(self.stop_stream)
        signal.signal(signal.SIGINT, stop_on_signal)
        signal.signal(signal.SIGTERM, stop_on_signal)
        self._registered_exit = True


def _as_list(value: Any) -> list[str]:
    if value is None:
        return []
    if isinstance(value, list):
        return [str(item) for item in value]
    if isinstance(value, str):
        return shlex.split(value)
    return [str(value)]
