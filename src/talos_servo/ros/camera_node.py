"""ROS camera node backed by Raspberry Pi's rpicam command-line tools."""

from __future__ import annotations

import subprocess
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import SetBool, Trigger

DEFAULT_CAMERA_INDEX = 0
DEFAULT_SNAPSHOT_WIDTH = 4608
DEFAULT_SNAPSHOT_HEIGHT = 2592
DEFAULT_STREAM_WIDTH = 640
DEFAULT_STREAM_HEIGHT = 360
DEFAULT_STREAM_FPS = 15
SNAPSHOT_TIMEOUT_MS = 1000
SNAPSHOT_PROCESS_TIMEOUT_S = 20.0
CONTINUOUS_STREAM_TIMEOUT_MS = 0
STARTUP_TIMEOUT_S = 5.0
PROCESS_TIMEOUT_S = 2.0
READ_SIZE = 4096
MAX_STREAM_FRAME_BYTES = 16 * 1024 * 1024


class CameraNode(Node):
    def __init__(self) -> None:
        super().__init__("talos_camera")
        self._camera_index = int(
            self.declare_parameter("camera_index", DEFAULT_CAMERA_INDEX).value
        )
        self._snapshot_width = int(
            self.declare_parameter("snapshot_width", DEFAULT_SNAPSHOT_WIDTH).value
        )
        self._snapshot_height = int(
            self.declare_parameter("snapshot_height", DEFAULT_SNAPSHOT_HEIGHT).value
        )
        self._stream_width = int(
            self.declare_parameter("stream_width", DEFAULT_STREAM_WIDTH).value
        )
        self._stream_height = int(
            self.declare_parameter("stream_height", DEFAULT_STREAM_HEIGHT).value
        )
        self._stream_fps = int(
            self.declare_parameter("stream_fps", DEFAULT_STREAM_FPS).value
        )
        self._frame_id = str(
            self.declare_parameter("frame_id", "camera_optical_frame").value
        )
        self._still_command = str(
            self.declare_parameter("still_command", "rpicam-still").value
        )
        self._video_command = str(
            self.declare_parameter("video_command", "rpicam-vid").value
        )
        if self._camera_index < 0:
            raise ValueError("camera_index must be zero or greater")
        if min(
            self._snapshot_width,
            self._snapshot_height,
            self._stream_width,
            self._stream_height,
            self._stream_fps,
        ) <= 0:
            raise ValueError("camera dimensions and stream_fps must be positive")

        self._publisher = self.create_publisher(
            CompressedImage, "/camera/image/compressed", 2
        )
        self._snapshot_publisher = self.create_publisher(
            CompressedImage,
            "/camera/snapshot/compressed",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_service(Trigger, "/camera/take_snapshot", self._take_snapshot)
        self.create_service(SetBool, "/camera/stream", self._set_streaming)
        self._stream_timer = self.create_timer(
            1.0 / self._stream_fps, self._publish_latest_frame
        )

        self._stream_process: subprocess.Popen[bytes] | None = None
        self._reader: threading.Thread | None = None
        self._stop_reader = threading.Event()
        self._first_frame = threading.Event()
        self._frame_lock = threading.Lock()
        self._latest_frame: bytes | None = None
        self._frame_sequence = 0
        self._published_sequence = 0
        self.get_logger().info(
            "Camera ready; camera index 0 is the default, streaming is disabled"
        )

    def _take_snapshot(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        restart_stream = self._stream_process is not None
        if restart_stream:
            self._stop_streaming()
        try:
            result = subprocess.run(
                [
                    self._still_command,
                    "--camera",
                    str(self._camera_index),
                    "--nopreview",
                    "--timeout",
                    str(SNAPSHOT_TIMEOUT_MS),
                    "--width",
                    str(self._snapshot_width),
                    "--height",
                    str(self._snapshot_height),
                    "--encoding",
                    "jpg",
                    "--output",
                    "-",
                ],
                capture_output=True,
                timeout=SNAPSHOT_PROCESS_TIMEOUT_S,
                check=False,
            )
            if result.returncode != 0:
                detail = result.stderr.decode(errors="replace").strip()
                raise RuntimeError(detail or f"rpicam-still exited {result.returncode}")
            if not result.stdout.startswith(b"\xff\xd8"):
                raise RuntimeError("rpicam-still did not return a JPEG image")
            snapshot = CompressedImage()
            snapshot.header.stamp = self.get_clock().now().to_msg()
            snapshot.header.frame_id = self._frame_id
            snapshot.format = "jpeg"
            snapshot.data = result.stdout
            self._snapshot_publisher.publish(snapshot)
            response.success = True
            response.message = "Snapshot published on /camera/snapshot/compressed"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        finally:
            if restart_stream:
                try:
                    self._start_streaming()
                except Exception as exc:
                    response.success = False
                    response.message = f"Snapshot done; could not resume stream: {exc}"
        return response

    def _set_streaming(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        try:
            if request.data:
                self._start_streaming()
                response.message = "Camera stream enabled"
            else:
                self._stop_streaming()
                response.message = "Camera stream disabled"
            response.success = True
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _start_streaming(self) -> None:
        if self._stream_process is not None:
            return
        self._stop_reader.clear()
        self._first_frame.clear()
        self._latest_frame = None
        self._stream_process = subprocess.Popen(
            [
                self._video_command,
                "--camera",
                str(self._camera_index),
                "--nopreview",
                "--timeout",
                str(CONTINUOUS_STREAM_TIMEOUT_MS),
                "--codec",
                "mjpeg",
                "--width",
                str(self._stream_width),
                "--height",
                str(self._stream_height),
                "--framerate",
                str(self._stream_fps),
                "--output",
                "-",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=0,
        )
        self._reader = threading.Thread(
            target=self._read_frames,
            args=(self._stream_process,),
            name="camera-frame-reader",
            daemon=True,
        )
        self._reader.start()
        if not self._first_frame.wait(STARTUP_TIMEOUT_S):
            exit_code = self._stream_process.poll()
            self._stop_streaming()
            raise RuntimeError(
                f"No camera frame received within {STARTUP_TIMEOUT_S:g}s"
                f" (rpicam-vid exit code: {exit_code})"
            )

    def _stop_streaming(self) -> None:
        process, self._stream_process = self._stream_process, None
        reader, self._reader = self._reader, None
        self._stop_reader.set()
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=PROCESS_TIMEOUT_S)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        if reader is not None:
            reader.join(timeout=PROCESS_TIMEOUT_S)
        if process is not None and process.stdout is not None:
            process.stdout.close()
        with self._frame_lock:
            self._latest_frame = None
            self._published_sequence = self._frame_sequence

    def _read_frames(self, process: subprocess.Popen[bytes]) -> None:
        if process.stdout is None:
            return
        buffer = bytearray()
        while not self._stop_reader.is_set():
            chunk = process.stdout.read(READ_SIZE)
            if not chunk:
                return
            buffer.extend(chunk)
            while True:
                start = buffer.find(b"\xff\xd8")
                if start < 0:
                    buffer[:] = buffer[-1:]
                    break
                if start:
                    del buffer[:start]
                end = buffer.find(b"\xff\xd9", 2)
                if end < 0:
                    if len(buffer) > MAX_STREAM_FRAME_BYTES:
                        buffer.clear()
                    break
                frame = bytes(buffer[: end + 2])
                del buffer[: end + 2]
                with self._frame_lock:
                    self._latest_frame = frame
                    self._frame_sequence += 1
                self._first_frame.set()

    def _publish_latest_frame(self) -> None:
        process = self._stream_process
        if process is None:
            return
        if process.poll() is not None:
            self.get_logger().error("rpicam-vid stopped unexpectedly")
            self._stop_streaming()
            return
        with self._frame_lock:
            frame = self._latest_frame
            sequence = self._frame_sequence
        if frame is None or sequence == self._published_sequence:
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.format = "jpeg"
        msg.data = frame
        self._publisher.publish(msg)
        self._published_sequence = sequence

    def destroy_node(self) -> bool:
        self._stop_streaming()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
