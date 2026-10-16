# er_goto

Tell the RPO humanoid "walk to the chair": `er_goto` asks Gemini Robotics-ER where the chair is in the RealSense image, then walks the robot there by publishing `/cmd_vel`, stopping about 0.8 m in front of it. Nothing in the existing robot packages changes; the robot runs the `default` policy as usual and takes velocity commands after the operator presses **Y**.

## How it works

The node runs a stop-look-move loop, so Gemini's answer is never stale:

1. **LOOK**: publish zero for `settle_time`, take a fresh RGB frame, ask Gemini to point at the target (`[y, x]`, normalized 0-1000).
2. Convert the point to a pixel, then to a bearing (from the camera intrinsics) and a distance (median of the aligned depth around the pixel).
3. **DONE** if closer than `stop_dist` and within `align_tol`; **TURN** if the bearing is larger than `align_tol`; otherwise **WALK** `min(distance - stop_dist, max_step)`. Each motion is open loop, bounded in time, and ends with a zero `Twist`.
4. **SEARCH** (rotate left for `search_turn_time`) when the target is not visible; give up after `max_search` misses in a row.
5. Back to LOOK, up to `max_iterations` cycles and `max_run_time` seconds.

Gemini never outputs velocities: each call takes seconds, and local code turns its point into motion.

| Path | Purpose |
| --- | --- |
| `er_goto/node.py` | ROS node: camera subscriptions, `/cmd_vel` publisher, state machine, watchdog, signal handling |
| `er_goto/gemini_client.py` | Gemini call with timeout and retry, JSON parsing (code fences), `[y, x]` to pixel |
| `er_goto/geometry.py` | Image decoding, pixel to bearing/distance, obstacle check (pure functions) |
| `er_goto/control.py` | TURN / WALK / DONE / SEARCH decisions, command-to-target parsing (pure functions) |
| `config/realsense_d435i_rgbd.yaml` | RealSense config: colour on, depth aligned to colour, 640x480 at 15 fps |
| `config/params.yaml` | All tunables |
| `launch/er_goto.launch.py` | Starts the node with `params.yaml`; `target`, `dry_run`, `save_debug_images` arguments |
| `scripts/snapshot_test.py` | Offline: Gemini on saved photos, draws the points, prints latency |
| `test/` | Unit tests for geometry, parsing and decisions |
| `../tools/start_er_goto.sh` | Sets the DDS environment, builds, starts the camera in `screen`, runs the node |

## Setup on the RDK X5

These steps assume the robot already walks with the gamepad via `./tools/start_robot.sh` and the RealSense SDK (librealsense2 2.58 or newer) is installed. Plug the D435i into a USB 3 port.

**1. Submodules.** Run `git submodule update --init --recursive` so that `src/camera/thirdparty/realsense-ros` exists; `start_er_goto.sh` builds the RealSense ROS wrapper from it on first use.

**2. Python packages.**

```bash
sudo apt install -y python3-opencv python3-numpy python3-pytest
pip3 install --user google-genai
```

Avoid `pip install opencv-python` on the robot: it pulls numpy 2, which breaks ROS Humble's Python packages built against the system numpy.

**3. API key.** In the repository root, copy `.env.example` to `.env` and set `GEMINI_API_KEY`. `.env` is git-ignored; `start_er_goto.sh` and `snapshot_test.py` read it.

## Running

```bash
./tools/start_robot.sh                     # default policy, as usual
./tools/start_er_goto.sh chair             # or: ./tools/start_er_goto.sh walk to the red chair
```

Then, on the gamepad:

1. Start the policy as usual and let the robot stand.
2. Press **Y** to hand control to `/cmd_vel`. Until then the robot ignores `er_goto`.
3. Keep the gamepad in hand. **Y** takes control back instantly; **B** pauses inference.

The node exits when it arrives or gives up. The camera keeps running in `er_camera_session` for the next command; stop it with `screen -S er_camera_session -X quit`. Do not run `tools/start_camera.sh` at the same time: it opens the same camera with the parkour config (the script refuses to start if it is running).

Options: `--dry-run` publishes to `/cmd_vel_test` (robot not needed), `--debug-images` saves each LOOK frame with the Gemini point to `~/er_goto_debug/<time>/`, `--restart-camera` restarts the camera session.

Without the script, after sourcing ROS, `install/setup.bash` and the DDS variables from `start_robot.sh`:

```bash
ros2 launch realsense2_camera rs_launch.py config_file:=$PWD/er_goto/config/realsense_d435i_rgbd.yaml
ros2 launch er_goto er_goto.launch.py target:=chair dry_run:=true save_debug_images:=true
```

## Safety

`inference_node` keeps the last `/cmd_vel` it received forever; there is no timeout. If `er_goto` stops publishing mid-walk, the robot keeps walking. The node therefore:

- bounds every motion (`max_motion_time`) and ends each with several zero `Twist` messages;
- publishes zero on normal exit, on any exception, on SIGINT/SIGTERM/SIGHUP, and for `settle_time` before every Gemini call;
- runs a watchdog thread that publishes zero if the main loop stops publishing for `watchdog_timeout` during a motion;
- clamps commands to `max_linear_x` / `max_angular_z`, which must stay inside the `default` policy clamps (0.6 m/s, 0.8 rad/s);
- aborts a walk when the obstacle check fires (anything closer than `obstacle_dist` in the centre third / lower two thirds of the depth image, or too few valid depth pixels) or when RGB or depth frames are older than `max_frame_age`;
- stands still on a Gemini timeout or error (`gemini_timeout`, `gemini_retries`) and gives up after `max_api_errors` failures in a row; a rejected request (bad key, unknown model) stops the run;
- caps the run with `max_iterations`, `max_search` and `max_run_time`, and logs every LOOK (label, pixel, bearing, distance, latency) and every command.

A killed process (`kill -9`, power loss of the computer running it) cannot publish zero. The operator must hold the gamepad and press **Y** to take control back.

Camera frames are sent to Google's Gemini API.

## Test plan

Each stage must pass before the next; never skip to the floor test.

| Stage | Setup | Pass criteria |
| --- | --- | --- |
| 1. Unit tests | Laptop, no ROS: `cd er_goto && python3 -m pytest test` | All pass |
| 2. Snapshot | Laptop, saved room photos: `python3 er_goto/scripts/snapshot_test.py --target chair photos/*.jpg` | Chair marked correctly on 5 of 5 photos; median latency recorded |
| 3. Camera dry run | RDK X5 + camera, robot off: `./tools/start_er_goto.sh --dry-run --debug-images chair` | Holding the camera, bearing sign and distance (within about 15 % of a tape measure) are correct; commands appear on `/cmd_vel_test` only (`ros2 topic echo /cmd_vel_test`) |
| 4. Crane | Robot hanging, default policy running, Y pressed | Legs step in the commanded direction; robot stops after every motion; Ctrl+C mid-motion returns the legs to stepping in place; `kill -9` mid-motion keeps the last command until the operator presses Y |
| 5. Floor | Open area, chair 2-3 m away, operator holding the gamepad | Robot reaches 0.8 m (± 0.2 m) in front of the chair within `max_iterations`; also with the chair starting 90 deg to the side |

If the robot turns the wrong way in stage 4, fix the sign in `geometry.pixel_to_bearing` and rerun stage 3. If the camera is not mounted facing straight ahead, set `camera_yaw_offset` (rad, + = camera turned left). The math assumes a roughly level camera: if it is pitched down at the floor, distances read long and the floor itself can trip the obstacle check, so check both in stage 3 and tune `stop_dist` / `obstacle_dist`. If `distance` is often unknown (thin chair legs), a later improvement is to ask Gemini for a bounding box and take depth from its lower centre.

## Parameters

All tunables are in `config/params.yaml` with comments; the defaults follow the plan (`stop_dist` 0.8 m, `align_tol` 0.15 rad, `turn_speed` 0.4 rad/s, `walk_speed` 0.25 m/s, `max_step` 1.0 m, `thinking_level` medium, 15 s Gemini timeout). Two additions: `min_step` (0.15 m) makes the last short walks long enough for the policy to actually step, and `max_linear_x` / `max_angular_z` (0.4 / 0.6) are the node's clamps, below the policy's own limits for first tests. Topic names, `depth_scale`, `jpeg_quality` and `debug_dir` have defaults in `node.py` and can be overridden the same way.
