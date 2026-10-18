# SPDX-License-Identifier: GPL-3.0-only
"""ROS 2 node: walk the robot to an object that Gemini Robotics-ER points at.

Look while walking: Gemini answers "where is the target in this image" on a
background thread while the control loop keeps publishing at publish_rate,
steering toward the newest answer by dead reckoning (control.Navigator). The
robot stops only to confirm arrival, when the newest answer is too old, when the
path is blocked, and to search. Velocities go to /cmd_vel, which inference_node
holds forever, so the node publishes zero on exit and a watchdog covers a
stalled control loop.

With interactive:=true the node reads commands from the terminal instead of
exiting on arrival; a command typed during a walk replaces the current target.
"""

import math
import queue
import re
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
from rclpy.utilities import remove_ros_args
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
    'interactive': False,
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
    'arrive_tol': 0.1,
    'align_tol': 0.15,
    'turn_in_place_angle': 0.5,
    'k_turn': 1.5,
    'turn_speed': 0.5,
    'walk_speed': 0.35,
    'min_walk_speed': 0.15,
    'slow_dist': 0.5,
    'settle_time': 0.3,
    'search_turn_time': 1.5,
    'max_search': 10,
    'min_progress': 0.3,
    'max_stalls': 2,
    'max_iterations': 40,
    'camera_yaw_offset': 0.0,
    'depth_patch_radius': 5,
    'depth_scale': 0.001,
    # Safety
    'max_linear_x': 0.5,
    'max_angular_z': 0.6,
    'publish_rate': 20.0,
    'max_blind_time': 10.0,
    'max_blind_dist': 2.5,
    'max_run_time': 300.0,
    'max_frame_age': 0.5,
    'watchdog_timeout': 0.5,
    'obstacle_dist': 0.5,
    'obstacle_min_depth': 0.1,
    'startup_timeout': 10.0,
}


class StopRequested(BaseException):
    """Raised from the signal handler; BaseException so library code cannot swallow it."""


class NewCommand(Exception):
    """A command was typed while walking; it replaces the current target."""

    def __init__(self, line):
        super().__init__(line)
        self.line = line


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


class Background:
    """fn(*args) on a daemon thread, so that exiting never waits for a Gemini call."""

    def __init__(self, fn, *args):
        self._done = threading.Event()
        self._result = None
        self._error = None
        threading.Thread(target=self._run, args=(fn, args), daemon=True).start()

    def _run(self, fn, args):
        try:
            self._result = fn(*args)
        except Exception as exc:  # re-raised in the control loop by result()
            self._error = exc
        finally:
            self._done.set()

    def done(self):
        return self._done.is_set()

    def result(self):
        if self._error is not None:
            raise self._error
        return self._result


class ErGotoNode(Node):

    def __init__(self):
        super().__init__('er_goto')
        for name, default in PARAMETERS.items():
            self.declare_parameter(name, default)
        self.p = {name: self.get_parameter(name).value for name in PARAMETERS}
        self._validate_parameters()

        p = self.p
        self.commands = queue.Queue()  # prompt lines, interactive mode
        self._told_operator = False
        self.tuning = control.Tuning(
            stop_dist=p['stop_dist'], arrive_tol=p['arrive_tol'], align_tol=p['align_tol'],
            turn_in_place_angle=p['turn_in_place_angle'], k_turn=p['k_turn'],
            turn_speed=p['turn_speed'], walk_speed=p['walk_speed'],
            min_walk_speed=p['min_walk_speed'], slow_dist=p['slow_dist'],
            settle_time=p['settle_time'], search_turn_time=p['search_turn_time'],
            max_search=p['max_search'], max_blind_time=p['max_blind_time'],
            max_blind_dist=p['max_blind_dist'], min_progress=p['min_progress'],
            max_stalls=0 if p['dry_run'] else p['max_stalls'])  # nothing walks in a dry run

        self._pub_lock = threading.RLock()  # re-entrant: the signal handler path may nest
        self._state_lock = threading.Lock()
        self._frames = {}  # 'rgb' / 'depth' / 'info' -> Frame
        self._loop_active = False  # the control loop is publishing; the watchdog checks it
        self._last_loop_publish = 0.0
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

        self._watchdog_stop = threading.Event()
        self._watchdog = threading.Thread(target=self._watchdog_loop, daemon=True)
        self._watchdog.start()

    # ------------------------------------------------------------------ setup

    def _validate_parameters(self):
        p = self.p
        errors = []
        if not p['interactive'] and not control.extract_target(p['target']):
            errors.append('target is empty (e.g. target:=chair)')
        if not 0.0 < p['max_linear_x'] <= POLICY_MAX_LINEAR_X:
            errors.append(f'max_linear_x must be in (0, {POLICY_MAX_LINEAR_X}]')
        if not 0.0 < p['max_angular_z'] <= POLICY_MAX_ANGULAR_Z:
            errors.append(f'max_angular_z must be in (0, {POLICY_MAX_ANGULAR_Z}]')
        if not 0.0 < p['walk_speed'] <= p['max_linear_x']:
            errors.append('walk_speed must be in (0, max_linear_x]')
        if not 0.0 < p['min_walk_speed'] <= p['walk_speed']:
            errors.append('min_walk_speed must be in (0, walk_speed]')
        if not 0.0 < p['turn_speed'] <= p['max_angular_z']:
            errors.append('turn_speed must be in (0, max_angular_z]')
        for name in ('stop_dist', 'align_tol', 'k_turn', 'slow_dist', 'search_turn_time',
                     'max_blind_time', 'max_blind_dist', 'max_run_time', 'max_frame_age',
                     'watchdog_timeout', 'gemini_timeout', 'publish_rate', 'startup_timeout',
                     'depth_scale'):
            if not p[name] > 0.0:
                errors.append(f'{name} must be > 0')
        for name in ('arrive_tol', 'settle_time'):
            if p[name] < 0.0:
                errors.append(f'{name} must be >= 0')
        if not 0.0 <= p['min_progress'] <= 1.0:
            errors.append('min_progress must be in [0, 1]')
        if p['max_stalls'] < 0:
            errors.append('max_stalls must be >= 0')
        if p['turn_in_place_angle'] < p['align_tol']:
            errors.append('turn_in_place_angle must be >= align_tol')
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

    def _send(self, linear_x, angular_z):
        """Publish one control-loop command and feed the watchdog."""
        self._publish(self._twist(linear_x, angular_z))
        with self._state_lock:
            self._last_loop_publish = time.monotonic()

    def _watchdog_loop(self):
        while not self._watchdog_stop.wait(0.05):
            with self._state_lock:
                stalled = (self._loop_active and time.monotonic() - self._last_loop_publish
                           > self.p['watchdog_timeout'])
                if stalled:
                    self._loop_active = False
                    self._watchdog_tripped = True
            if stalled:
                self.get_logger().error(
                    f'Watchdog: no command for {self.p["watchdog_timeout"]:.1f} s, '
                    'publishing zero')
                self.publish_zero()

    def _watchdog_fired(self):
        """True once after the watchdog stopped the robot; re-arms it."""
        with self._state_lock:
            tripped = self._watchdog_tripped
            if tripped:
                self._watchdog_tripped = False
                self._loop_active = True
                self._last_loop_publish = time.monotonic()
        return tripped

    def stop_watchdog(self):
        self._watchdog_stop.set()

    # -------------------------------------------------------------- perception

    def _walk_abort_reason(self):
        """Why the robot must not walk forward now: stale frames or an obstacle in the corridor."""
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

    def wait_for_camera(self, max_age=2.0):
        """Wait until colour, depth and camera_info have all arrived within max_age s."""
        deadline = time.monotonic() + self.p['startup_timeout']
        topics = {'rgb': 'color_topic', 'depth': 'depth_topic', 'info': 'camera_info_topic'}

        def fresh(key):
            frame = self._frame(key)
            return frame is not None and time.monotonic() - frame.received <= max_age

        while time.monotonic() < deadline:
            if all(fresh(k) for k in topics):
                return
            time.sleep(0.1)
        missing = [self.p[t] for k, t in topics.items() if not fresh(k)]
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

    def _frames_for(self, look):
        """RGB, depth and camera_info for a LOOK, or None until suitable frames arrive.

        A standing LOOK needs frames received after it was requested, when the robot
        had settled; a moving LOOK takes the newest frames if they are fresh.
        """
        rgb, depth, info = self._frame('rgb'), self._frame('depth'), self._frame('info')
        if not (rgb and depth and info):
            return None
        after = look.time if look.standing else time.monotonic() - self.p['max_frame_age']
        if rgb.received < after or depth.received < after:
            return None
        return rgb.msg, depth.msg, info.msg

    def _look(self, target, debug_dir, iteration, standing, rgb_msg, depth_msg, info):
        """Ask Gemini where target is in these frames; runs on a Background thread.

        Returns a LookResult, or None if the target is not visible or the answer
        could not be parsed. Raises GeminiError (transient) or GeminiFatalError.
        """
        name = f'LOOK {iteration} ({"standing" if standing else "moving"})'
        rgb = geometry.image_to_array(rgb_msg.encoding, rgb_msg.height, rgb_msg.width,
                                      rgb_msg.step, rgb_msg.is_bigendian, rgb_msg.data)
        bgr = geometry.to_bgr(rgb, rgb_msg.encoding)
        ok, jpeg = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, self.p['jpeg_quality']])
        if not ok:
            raise RuntimeError('JPEG encoding failed')

        started = time.monotonic()
        try:
            points, text = self.gemini.point(jpeg.tobytes(), target)
        except ResponseParseError as exc:
            latency = time.monotonic() - started
            self.get_logger().warning(f'{name}: unusable answer ({exc}), latency {latency:.1f} s')
            self._save_debug(debug_dir, target, iteration, bgr, None, f'unparseable: {exc}')
            return None
        latency = time.monotonic() - started
        self.get_logger().debug(f'Gemini answer: {text}')

        if not points:
            self.get_logger().info(f'{name}: "{target}" not visible, '
                                   f'latency {latency:.1f} s')
            self._save_debug(debug_dir, target, iteration, bgr, None, 'not visible')
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
            f'{name}: label="{point.label}" pixel=({u}, {v}) '
            f'bearing={math.degrees(bearing):+.1f} deg distance={dist_text} '
            f'latency={latency:.1f} s ({len(points)} point(s))')
        self._save_debug(debug_dir, target, iteration, bgr, result, f'{latency:.1f} s')
        return result

    def _save_debug(self, debug_dir, target, iteration, bgr, result, note):
        if debug_dir is None:
            return
        image = bgr.copy()
        if result is not None:
            dist_text = f'{result.distance:.2f}m' if result.distance is not None else '?m'
            cv2.circle(image, (result.u, result.v), 8, (0, 0, 255), 2)
            cv2.drawMarker(image, (result.u, result.v), (0, 0, 255), cv2.MARKER_CROSS, 24, 1)
            note = (f'{result.label} {math.degrees(result.bearing):+.1f}deg {dist_text} '
                    f'| {note}')
        cv2.putText(image, f'#{iteration} {target}: {note}', (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)
        path = debug_dir / f'look_{iteration:03d}.jpg'
        if not cv2.imwrite(str(path), image):
            self.get_logger().warning(f'Could not write {path}')

    # -------------------------------------------------------------- main loop

    def run(self, target, poll_command=lambda: None):
        """Walk to target until it is reached or a limit is hit. Returns an exit code.

        poll_command() is called every tick and returns a newly typed command or
        None; a command ends the walk with NewCommand.
        """
        log = self.get_logger()
        p = self.p
        log.info(f'Target: "{target}"; publishing to {self.cmd_topic}'
                 f'{" (dry run)" if p["dry_run"] else ""}')
        self.wait_for_camera()
        if not p['dry_run']:
            self.wait_for_robot()
            if not self._told_operator:
                self._told_operator = True
                log.info('The robot moves only after Y was pressed on the gamepad. '
                         'Keep the gamepad in hand: Y takes control back, B pauses inference.')

        debug_dir = None
        if p['save_debug_images']:
            name = time.strftime('%Y%m%d-%H%M%S') + '-' + re.sub(r'\W+', '_', target)[:40]
            debug_dir = Path(p['debug_dir']).expanduser() / name
            debug_dir.mkdir(parents=True, exist_ok=True)

        deadline = time.monotonic() + p['max_run_time']
        period = 1.0 / p['publish_rate']
        frame_timeout = max(2.0, 4 * p['max_frame_age'])
        nav = control.Navigator(self.tuning, time.monotonic(), log=log.info)
        wanted = None   # (Look, requested at) waiting for suitable frames
        running = None  # (Look, iteration, Background) waiting for Gemini
        looks = 0
        api_errors = 0
        with self._state_lock:
            self._loop_active = True
            self._last_loop_publish = time.monotonic()
        try:
            while True:
                now = time.monotonic()
                if now > deadline:
                    raise control.GaveUp(f'max_run_time {p["max_run_time"]:.0f} s reached')
                line = poll_command()
                if line is not None:
                    raise NewCommand(line)
                if self._watchdog_fired():
                    nav.stop(now, 'watchdog fired')

                if running is not None and running[2].done():
                    look, iteration, task = running
                    running = None
                    try:
                        found = task.result()
                    except GeminiError as exc:
                        api_errors += 1
                        log.error(f'LOOK {iteration}: {exc} '
                                  f'({api_errors}/{p["max_api_errors"]})')
                        if api_errors >= p['max_api_errors']:
                            raise control.GaveUp('too many consecutive Gemini errors') from exc
                        nav.failed(look)
                    else:
                        api_errors = 0
                        if found is None:
                            nav.missed(look, now)
                        else:
                            nav.seen(look, found.bearing, found.distance, now)

                command = nav.tick(now, self._walk_abort_reason)
                if command.done:
                    log.info(f'Arrived at "{target}"')
                    return EXIT_OK
                if command.look is not None:
                    wanted = (command.look, now)
                if wanted is not None:
                    frames = self._frames_for(wanted[0])
                    if frames is not None:
                        if looks >= p['max_iterations']:
                            raise control.GaveUp(f'max_iterations {p["max_iterations"]} reached')
                        looks += 1
                        look = wanted[0]
                        running = (look, looks, Background(self._look, target, debug_dir, looks,
                                                           look.standing, *frames))
                        wanted = None
                    elif now - wanted[1] > frame_timeout:
                        raise RuntimeError('camera stopped sending frames')

                self._send(command.linear_x, command.angular_z)
                time.sleep(max(period - (time.monotonic() - now), 0.0))
        except control.GaveUp as exc:
            log.error(f'Gave up: {exc}')
            return EXIT_FAILED
        finally:
            with self._state_lock:
                self._loop_active = False


HELP = ('Type where to go, e.g. "go to the black chair". A new command replaces the current '
        'target, "stop" halts the robot, "quit" or Ctrl+D exits, Ctrl+C stops the robot and exits.')


def _read_commands(commands):
    """Put terminal lines on commands; runs on a daemon thread. Ctrl+D quits."""
    while True:
        try:
            line = input('er_goto> ')
        except EOFError:
            commands.put('quit')
            return
        if line.strip():
            commands.put(line)


def repl(node, first_line=''):
    """Walk to each typed target until "quit". Returns an exit code."""
    log = node.get_logger()
    log.info(HELP)
    threading.Thread(target=_read_commands, args=(node.commands,), daemon=True).start()

    def poll_command():
        try:
            return node.commands.get_nowait()
        except queue.Empty:
            return None

    pending = first_line.strip() or None
    while True:
        line, pending = pending, None
        while line is None:  # with a timeout, so that a signal is handled promptly
            try:
                line = node.commands.get(timeout=0.5)
            except queue.Empty:
                pass
        action, target = control.parse_command(line)
        if action == 'quit':
            return EXIT_OK
        if action == 'help':
            log.info(HELP)
        elif action == 'stop':
            node.publish_zero()
            log.info('Stopped. Waiting for the next command')
        elif action == 'go':
            try:
                node.run(target, poll_command)
            except NewCommand as exc:
                log.info(f'Leaving "{target}" for: {exc.line}')
                pending = exc.line
            except Exception as exc:  # GeminiFatalError, camera loss, ...
                log.error(f'{type(exc).__name__}: {exc}')
            finally:
                node.publish_zero(count=10)
            if pending is None:
                log.info('Waiting for the next command')


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
        if node.p['interactive']:
            # A first command may follow the executable: er_goto_node walk to the chair --ros-args
            exit_code = repl(node, ' '.join(remove_ros_args(args)[1:]) or node.p['target'])
        else:
            exit_code = node.run(control.extract_target(node.p['target']))
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
