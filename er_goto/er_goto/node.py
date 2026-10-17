# SPDX-License-Identifier: GPL-3.0-only
"""ROS 2 node: walk the robot to an object that Gemini Robotics-ER points at.

Stop, look, move: the robot stands still while Gemini answers "where is the
target in this image", then turns or walks for a short, bounded time and looks
again. Velocities go to /cmd_vel, which inference_node holds forever, so every
motion ends with a zero Twist and a watchdog covers a stalled main loop.
"""

import math
import signal
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo, Image

from er_goto import control, geometry
from er_goto.gemini_client import GeminiError, GeminiPointer, ResponseParseError, point_to_pixel

# clip_cmd of the rpo default policy (src/inference/robots/rpo/configs/default.yaml).
POLICY_MAX_LINEAR_X = 0.6
POLICY_MAX_ANGULAR_Z = 0.8

EXIT_OK = 0
EXIT_FAILED = 1
EXIT_STOPPED = 130

PARAMETERS = {
    # Runtime flags
    'target': '',
    'dry_run': False,
    'save_debug_images': False,
    'debug_dir': '~/er_goto_debug',
    # Topics
    'color_topic': '/camera/camera/color/image_raw',
    'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
    'camera_info_topic': '/camera/camera/color/camera_info',
    'cmd_vel_topic': '/cmd_vel',
    'dry_run_topic': '/cmd_vel_test',
    # Gemini
    'model': 'gemini-robotics-er-2-preview',
    'thinking_level': 'low',
    'gemini_timeout': 15.0,
    'gemini_retries': 1,
    'max_api_errors': 3,
    'jpeg_quality': 90,
    # Control
    'stop_dist': 0.8,
    'align_tol': 0.15,
    'turn_in_place_angle': 0.5,
    'turn_speed': 0.5,
    'walk_speed': 0.35,
    'max_step': 2.0,
    'min_step': 0.3,
    'settle_time': 0.3,
    'search_turn_time': 1.5,
    'max_search': 10,
    'max_iterations': 40,
    'camera_yaw_offset': 0.0,
    'depth_patch_radius': 5,
    'depth_scale': 0.001,
    # Safety
    'max_linear_x': 0.5,
    'max_angular_z': 0.6,
    'publish_rate': 20.0,
    'max_motion_time': 8.0,
    'max_run_time': 300.0,
    'max_frame_age': 0.5,
    'watchdog_timeout': 0.5,
    'obstacle_dist': 0.5,
    'obstacle_min_depth': 0.1,
    'startup_timeout': 10.0,
}


class StopRequested(BaseException):
    """Raised from the signal handler; BaseException so library code cannot swallow it."""


@dataclass
class Frame:
    msg: object
    received: float  # time.monotonic()


@dataclass
class LookResult:
    label: str
    u: int
    v: int
    bearing: float
    distance: float  # m, or None when depth is unknown


class ErGotoNode(Node):

    def __init__(self):
        super().__init__('er_goto')
        for name, default in PARAMETERS.items():
            self.declare_parameter(name, default)
        self.p = {name: self.get_parameter(name).value for name in PARAMETERS}
        self._validate_parameters()

        p = self.p
        self.target = control.extract_target(p['target'])
        self.tuning = control.Tuning(
            stop_dist=p['stop_dist'], align_tol=p['align_tol'],
            turn_in_place_angle=p['turn_in_place_angle'], turn_speed=p['turn_speed'],
            walk_speed=p['walk_speed'], max_step=p['max_step'], min_step=p['min_step'],
            search_turn_time=p['search_turn_time'], max_motion_time=p['max_motion_time'])

        self._pub_lock = threading.RLock()  # re-entrant: the signal handler path may nest
        self._state_lock = threading.Lock()
        self._frames = {}  # 'rgb' / 'depth' / 'info' -> Frame
        self._motion_active = False
        self._last_motion_publish = 0.0
        self._watchdog_tripped = False
        self._last_obstacle_check = None  # (depth msg id, result)

        cmd_topic = p['dry_run_topic'] if p['dry_run'] else p['cmd_vel_topic']
        self.cmd_topic = cmd_topic
        # inference_node subscribes with KeepLast(1), best_effort, volatile.
        self._pub = self.create_publisher(
            Twist, cmd_topic, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Image, p['color_topic'],
                                 lambda m: self._store('rgb', m), qos_profile_sensor_data)
        self.create_subscription(Image, p['depth_topic'],
                                 lambda m: self._store('depth', m), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, p['camera_info_topic'],
                                 lambda m: self._store('info', m), qos_profile_sensor_data)

        self.gemini = GeminiPointer(
            model=p['model'], thinking_level=p['thinking_level'],
            timeout=p['gemini_timeout'], retries=p['gemini_retries'])

        self.debug_dir = None
        if p['save_debug_images']:
            stamp = time.strftime('%Y%m%d-%H%M%S')
            self.debug_dir = Path(p['debug_dir']).expanduser() / stamp
            self.debug_dir.mkdir(parents=True, exist_ok=True)

        self._watchdog_stop = threading.Event()
        self._watchdog = threading.Thread(target=self._watchdog_loop, daemon=True)
        self._watchdog.start()

    # ------------------------------------------------------------------ setup

    def _validate_parameters(self):
        p = self.p
        errors = []
        if not control.extract_target(p['target']):
            errors.append('target is empty (e.g. target:=chair)')
        if not 0.0 < p['max_linear_x'] <= POLICY_MAX_LINEAR_X:
            errors.append(f'max_linear_x must be in (0, {POLICY_MAX_LINEAR_X}]')
        if not 0.0 < p['max_angular_z'] <= POLICY_MAX_ANGULAR_Z:
            errors.append(f'max_angular_z must be in (0, {POLICY_MAX_ANGULAR_Z}]')
        if not 0.0 < p['walk_speed'] <= p['max_linear_x']:
            errors.append('walk_speed must be in (0, max_linear_x]')
        if not 0.0 < p['turn_speed'] <= p['max_angular_z']:
            errors.append('turn_speed must be in (0, max_angular_z]')
        for name in ('stop_dist', 'align_tol', 'max_step', 'min_step', 'search_turn_time',
                     'max_motion_time', 'max_run_time', 'max_frame_age', 'watchdog_timeout',
                     'gemini_timeout', 'publish_rate', 'startup_timeout', 'depth_scale'):
            if not p[name] > 0.0:
                errors.append(f'{name} must be > 0')
        if p['turn_in_place_angle'] < p['align_tol']:
            errors.append('turn_in_place_angle must be >= align_tol')
        if p['min_step'] > p['max_step']:
            errors.append('min_step must be <= max_step')
        if p['publish_rate'] > 0.0 and 2.0 / p['publish_rate'] >= p['watchdog_timeout']:
            errors.append('publish_rate too low for watchdog_timeout (need 2 periods < timeout)')
        if p['obstacle_dist'] >= p['stop_dist']:
            errors.append('obstacle_dist must be < stop_dist, or the target blocks the last step')
        for name in ('max_search', 'max_iterations', 'max_api_errors'):
            if p[name] < 1:
                errors.append(f'{name} must be >= 1')
        if errors:
            raise ValueError('invalid parameters: ' + '; '.join(errors))

    def _store(self, key, msg):
        with self._state_lock:
            self._frames[key] = Frame(msg, time.monotonic())

    def _frame(self, key):
        with self._state_lock:
            return self._frames.get(key)

    # ------------------------------------------------------------- publishing

    def _twist(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = float(min(max(linear_x, -self.p['max_linear_x']), self.p['max_linear_x']))
        msg.angular.z = float(min(max(angular_z, -self.p['max_angular_z']),
                                  self.p['max_angular_z']))
        return msg

    def _publish(self, msg):
        with self._pub_lock:
            self._pub.publish(msg)

    def publish_zero(self, count=5, interval=0.02):
        """Publish zero several times: the topic is best effort and the robot keeps
        the last command it received."""
        zero = Twist()
        for i in range(count):
            try:
                self._publish(zero)
            except Exception as exc:  # context already shut down
                self.get_logger().error(f'Failed to publish zero: {exc}')
                return
            if i + 1 < count:
                time.sleep(interval)

    def hold_zero(self, duration):
        """Publish zero at publish_rate for duration seconds."""
        period = 1.0 / self.p['publish_rate']
        end = time.monotonic() + duration
        zero = Twist()
        while True:
            self._publish(zero)
            remaining = end - time.monotonic()
            if remaining <= 0.0:
                return
            time.sleep(min(period, remaining))

    def _watchdog_loop(self):
        while not self._watchdog_stop.wait(0.05):
            with self._state_lock:
                stalled = (self._motion_active and time.monotonic() - self._last_motion_publish
                           > self.p['watchdog_timeout'])
                if stalled:
                    self._motion_active = False
                    self._watchdog_tripped = True
            if stalled:
                self.get_logger().error(
                    f'Watchdog: no command for {self.p["watchdog_timeout"]:.1f} s during a '
                    'motion, publishing zero')
                self.publish_zero()

    def stop_watchdog(self):
        self._watchdog_stop.set()

    def move(self, step, abort_check=None):
        """Publish the step's velocity at publish_rate for its duration, then zero.

        Returns None if the motion completed, otherwise the reason it was aborted.
        """
        twist = self._twist(step.linear_x, step.angular_z)
        duration = min(step.duration, self.p['max_motion_time'])
        period = 1.0 / self.p['publish_rate']
        self.get_logger().info(
            f'{step.action}: linear.x={twist.linear.x:+.2f} angular.z={twist.angular.z:+.2f} '
            f'for {duration:.2f} s ({step.reason})')
        aborted = None
        start = time.monotonic()
        end = start + duration
        with self._state_lock:
            self._motion_active = True
            self._watchdog_tripped = False
            self._last_motion_publish = start
        try:
            while True:
                now = time.monotonic()
                if now >= end:
                    break
                with self._state_lock:
                    tripped = self._watchdog_tripped
                if tripped:
                    aborted = 'watchdog fired'
                    break
                if abort_check is not None:
                    aborted = abort_check()
                    if aborted:
                        break
                self._publish(twist)
                with self._state_lock:
                    self._last_motion_publish = time.monotonic()
                time.sleep(min(period, max(end - time.monotonic(), 0.0)))
        finally:
            with self._state_lock:
                self._motion_active = False
            self.publish_zero()
        elapsed = time.monotonic() - start
        if aborted:
            self.get_logger().warning(
                f'{step.action} aborted after {elapsed:.2f} s: {aborted}; published zero')
        else:
            self.get_logger().info(f'{step.action} done after {elapsed:.2f} s; published zero')
        return aborted

    # -------------------------------------------------------------- perception

    def _walk_abort_reason(self):
        """Abort check while walking: stale frames or an obstacle in the corridor."""
        now = time.monotonic()
        depth, rgb = self._frame('depth'), self._frame('rgb')
        max_age = self.p['max_frame_age']
        if depth is None or now - depth.received > max_age:
            return f'depth frames older than {max_age:.1f} s'
        if rgb is None or now - rgb.received > max_age:
            return f'RGB frames older than {max_age:.1f} s'
        blocked, nearest = self._obstacle(depth.msg)
        if blocked:
            where = f'{nearest:.2f} m' if nearest is not None else 'too close / no depth'
            return f'obstacle in path ({where})'
        return None

    def _obstacle(self, depth_msg):
        """check_obstacle on a depth message, cached per message."""
        cached = self._last_obstacle_check
        if cached is not None and cached[0] is depth_msg:
            return cached[1]
        depth_m = self._depth_meters(depth_msg)
        result = geometry.check_obstacle(depth_m, self.p['obstacle_dist'],
                                         min_depth=self.p['obstacle_min_depth'])
        self._last_obstacle_check = (depth_msg, result)
        return result

    def _depth_meters(self, msg):
        depth = geometry.image_to_array(msg.encoding, msg.height, msg.width, msg.step,
                                        msg.is_bigendian, msg.data)
        return geometry.depth_to_meters(depth, self.p['depth_scale'])

    def wait_for_camera(self):
        """Wait until colour, depth and camera_info have all arrived."""
        deadline = time.monotonic() + self.p['startup_timeout']
        topics = {'rgb': 'color_topic', 'depth': 'depth_topic', 'info': 'camera_info_topic'}
        while time.monotonic() < deadline:
            if all(self._frame(k) is not None for k in topics):
                return
            time.sleep(0.1)
        missing = [self.p[t] for k, t in topics.items() if self._frame(k) is None]
        raise RuntimeError(f'no data on {", ".join(missing)}; is the camera running?')

    def wait_for_robot(self):
        """Check that something (inference_node) subscribes to /cmd_vel."""
        deadline = time.monotonic() + self.p['startup_timeout']
        while time.monotonic() < deadline:
            if self.count_subscribers(self.cmd_topic) > 0:
                return
            time.sleep(0.1)
        raise RuntimeError(f'nothing subscribes to {self.cmd_topic}; '
                           'start tools/start_robot.sh first')

    def _fresh_frames(self, after):
        """Newest RGB and depth frames received after `after`, plus camera_info."""
        deadline = time.monotonic() + max(2.0, 4 * self.p['max_frame_age'])
        while time.monotonic() < deadline:
            rgb, depth, info = self._frame('rgb'), self._frame('depth'), self._frame('info')
            if (rgb and depth and info and rgb.received >= after
                    and depth.received >= after):
                return rgb.msg, depth.msg, info.msg
            time.sleep(0.02)
        raise RuntimeError('camera stopped sending frames')

    def look(self, iteration):
        """Stand still, photograph, ask Gemini. Returns a LookResult, or None if the
        target is not visible or the answer could not be parsed.

        Raises GeminiError (transient) or GeminiFatalError.
        """
        self.hold_zero(self.p['settle_time'])
        rgb_msg, depth_msg, info = self._fresh_frames(after=time.monotonic())

        rgb = geometry.image_to_array(rgb_msg.encoding, rgb_msg.height, rgb_msg.width,
                                      rgb_msg.step, rgb_msg.is_bigendian, rgb_msg.data)
        bgr = geometry.to_bgr(rgb, rgb_msg.encoding)
        ok, jpeg = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, self.p['jpeg_quality']])
        if not ok:
            raise RuntimeError('JPEG encoding failed')

        started = time.monotonic()
        try:
            points, text = self.gemini.point(jpeg.tobytes(), self.target)
        except ResponseParseError as exc:
            latency = time.monotonic() - started
            self.get_logger().warning(
                f'LOOK {iteration}: unusable answer ({exc}), latency {latency:.1f} s')
            self._save_debug(iteration, bgr, None, f'unparseable: {exc}')
            return None
        latency = time.monotonic() - started
        self.get_logger().debug(f'Gemini answer: {text}')

        if not points:
            self.get_logger().info(f'LOOK {iteration}: "{self.target}" not visible, '
                                   f'latency {latency:.1f} s')
            self._save_debug(iteration, bgr, None, 'not visible')
            return None

        point = points[0]
        width, height = rgb_msg.width, rgb_msg.height
        u, v = point_to_pixel(point, width, height)
        fx, cx = info.k[0], info.k[2]
        if info.width and info.width != width:  # intrinsics for another resolution
            scale = width / info.width
            fx, cx = fx * scale, cx * scale
        bearing = geometry.pixel_to_bearing(u, fx, cx, self.p['camera_yaw_offset'])

        depth_m = self._depth_meters(depth_msg)
        du, dv = geometry.scale_pixel(u, v, (width, height), (depth_msg.width, depth_msg.height))
        distance = geometry.depth_at(depth_m, du, dv, self.p['depth_patch_radius'])

        result = LookResult(point.label, u, v, bearing, distance)
        dist_text = f'{distance:.2f} m' if distance is not None else 'unknown'
        self.get_logger().info(
            f'LOOK {iteration}: label="{point.label}" pixel=({u}, {v}) '
            f'bearing={math.degrees(bearing):+.1f} deg distance={dist_text} '
            f'latency={latency:.1f} s ({len(points)} point(s))')
        self._save_debug(iteration, bgr, result, f'{latency:.1f} s')
        return result

    def _save_debug(self, iteration, bgr, result, note):
        if self.debug_dir is None:
            return
        image = bgr.copy()
        if result is not None:
            dist_text = f'{result.distance:.2f}m' if result.distance is not None else '?m'
            cv2.circle(image, (result.u, result.v), 8, (0, 0, 255), 2)
            cv2.drawMarker(image, (result.u, result.v), (0, 0, 255), cv2.MARKER_CROSS, 24, 1)
            note = (f'{result.label} {math.degrees(result.bearing):+.1f}deg {dist_text} '
                    f'| {note}')
        cv2.putText(image, f'#{iteration} {self.target}: {note}', (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)
        path = self.debug_dir / f'look_{iteration:03d}.jpg'
        if not cv2.imwrite(str(path), image):
            self.get_logger().warning(f'Could not write {path}')

    # -------------------------------------------------------------- main loop

    def run(self):
        """Stop-look-move until the target is reached or a limit is hit. Returns an exit code."""
        log = self.get_logger()
        p = self.p
        log.info(f'Target: "{self.target}"; publishing to {self.cmd_topic}'
                 f'{" (dry run)" if p["dry_run"] else ""}')
        self.wait_for_camera()
        if not p['dry_run']:
            self.wait_for_robot()
            log.info('The robot moves only after Y was pressed on the gamepad. '
                     'Keep the gamepad in hand: Y takes control back, B pauses inference.')

        deadline = time.monotonic() + p['max_run_time']
        misses = 0
        api_errors = 0
        for iteration in range(1, p['max_iterations'] + 1):
            if time.monotonic() > deadline:
                log.error(f'Gave up: max_run_time {p["max_run_time"]:.0f} s reached')
                return EXIT_FAILED

            try:
                found = self.look(iteration)
            except GeminiError as exc:
                api_errors += 1
                log.error(f'LOOK {iteration}: {exc} ({api_errors}/{p["max_api_errors"]}); '
                          'standing still')
                if api_errors >= p['max_api_errors']:
                    log.error('Gave up: too many consecutive Gemini errors')
                    return EXIT_FAILED
                continue
            api_errors = 0

            if found is None:
                misses += 1
                if misses >= p['max_search']:
                    log.error(f'Gave up: "{self.target}" not found after {misses} looks')
                    return EXIT_FAILED
                self.move(control.search_step(self.tuning))
                continue
            misses = 0

            step = control.decide(found.bearing, found.distance, self.tuning)
            if step.action == control.DONE:
                log.info(f'Arrived at "{self.target}": {step.reason}')
                return EXIT_OK
            if step.action == control.WALK:
                reason = self._walk_abort_reason()
                if reason:
                    log.error(f'Cannot walk: {reason}')
                    return EXIT_FAILED
                self.move(step, abort_check=self._walk_abort_reason)
            else:
                self.move(step)

        log.error(f'Gave up: max_iterations {p["max_iterations"]} reached')
        return EXIT_FAILED


def _report(node, message, warning=False):
    if node is None:
        print(f'[er_goto] {message}', file=sys.stderr)
    elif warning:
        node.get_logger().warning(message)
    else:
        node.get_logger().error(message)


def main(args=None):
    # rclpy's own SIGINT handler shuts the context down, after which zero can no
    # longer be published; handle the signals here instead.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stopping = threading.Event()

    def on_signal(signum, _frame):
        if stopping.is_set():
            return  # already stopping; let the cleanup finish
        stopping.set()
        raise StopRequested(signal.Signals(signum).name)

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, on_signal)

    node = None
    executor = None
    exit_code = EXIT_FAILED
    try:
        node = ErGotoNode()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        threading.Thread(target=executor.spin, daemon=True).start()
        exit_code = node.run()
    except StopRequested as exc:
        exit_code = EXIT_STOPPED
        _report(node, f'{exc} received, stopping', warning=True)
    except Exception as exc:  # GeminiFatalError, invalid parameters, camera loss, ...
        _report(node, f'{type(exc).__name__}: {exc}')
    finally:
        stopping.set()  # a signal from here on must not interrupt the zero publish
        if node is not None:
            node.stop_watchdog()
            node.publish_zero(count=10)
            node.get_logger().info(f'Published zero; exiting with code {exit_code}')
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
