# er_goto

Tell the RPO humanoid "walk to the chair": `er_goto` asks Gemini Robotics-ER where the chair is in the RealSense image, then walks the robot there by publishing `/cmd_vel`, stopping about 0.8 m in front of it. Nothing in the existing robot packages changes; the robot runs the `default` policy as usual and takes velocity commands after the operator presses **Y**.

## How it works

The node keeps walking while Gemini thinks. An answer takes seconds, so the control loop never waits for one:

1. **LOOK** (background thread): send the newest RGB frame to Gemini (`[y, x]`, normalized 0-1000), convert the point to a bearing (camera intrinsics) and a distance (median of the aligned depth around the pixel). One LOOK is in flight at a time; the next starts as soon as one returns, unless it would answer only after the robot has reached `stop_dist`.
2. **Dead reckoning**: the control loop adds up the velocities it publishes into a pose. Each answer is placed using the pose at the moment its photo was taken, so an answer that is seconds old still gives the bearing and distance from where the robot is now.
3. **PURSUE** at `publish_rate`: `angular.z = k_turn * bearing` (capped at `turn_speed`) while walking at up to `walk_speed`, slower when misaligned and over the last `slow_dist`, never below `min_walk_speed`; turn in place when the bearing exceeds `turn_in_place_angle`.
4. **STAND** (zero, then a standing LOOK after `settle_time`) only when needed: at the start; when dead reckoning reaches `stop_dist`; when the newest answer's photo is older than `max_blind_time` or the robot walked `max_blind_dist` since; when the path is blocked; after the watchdog fired. The standing LOOK decides: **DONE** within `stop_dist + arrive_tol` and `align_tol`, otherwise back to PURSUE, or SEARCH if the target is not visible. A moving LOOK still in flight when the robot stops sends it on if the target is still ahead; it finishes the run only if photographed within `arrive_tol` of where the robot stopped, and after a blocked path only a standing LOOK decides.
5. **SEARCH**: rotate left for `search_turn_time`, then STAND. Give up after `max_search` standing LOOKs in a row without the target, `max_iterations` LOOKs or `max_run_time` seconds.

Gemini never outputs velocities: local code turns its point into motion. A LOOK taken while walking that misses the target is ignored; the robot keeps going on the newest answer within the limits of step 4.

| Path | Purpose |
| --- | --- |
| `er_goto/node.py` | ROS node: camera subscriptions, `/cmd_vel` publisher, state machine, watchdog, signal handling |
| `er_goto/gemini_client.py` | Gemini call with timeout and retry, JSON parsing (code fences), `[y, x]` to pixel |
| `er_goto/geometry.py` | Image decoding, pixel to bearing/distance, obstacle check (pure functions) |
| `er_goto/control.py` | Dead reckoning, pursuit law, STAND / PURSUE / SEARCH state machine (`Navigator`), command-to-target parsing (no ROS) |
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

- publishes from one control loop at `publish_rate`; Gemini runs on a daemon thread and never publishes, and exiting never waits for it;
- runs a watchdog thread that publishes zero if the control loop stops publishing for `watchdog_timeout`; the loop then stands and looks again;
- walks on one answer for at most `max_blind_time` seconds since its photo and `max_blind_dist` metres, then stands and asks again;
- publishes zero on normal exit, on any exception and on SIGINT/SIGTERM/SIGHUP;
- clamps commands to `max_linear_x` / `max_angular_z`, which must stay inside the `default` policy clamps (0.6 m/s, 0.8 rad/s);
- checks on every tick that walks forward for an obstacle (anything closer than `obstacle_dist` in the centre third / lower two thirds of the depth image, or too few valid depth pixels) and for RGB or depth frames older than `max_frame_age`; a blocked path stops the robot for a standing LOOK, and the run gives up if it is still blocked, unless that LOOK shows the robot has arrived;
- on a Gemini timeout or error (`gemini_timeout`, `gemini_retries`) keeps going on the newest answer within those limits, asks again, and gives up after `max_api_errors` failures in a row; a rejected request (bad key, unknown model) stops the run;
- caps the run with `max_iterations`, `max_search` and `max_run_time`, and logs every LOOK (standing or moving, label, pixel, bearing, distance, latency), every stop with its reason, and the command after each answer.

Dead reckoning assumes the robot moves as commanded. A humanoid usually covers less, so the robot stops a little short and the standing LOOK sends it on; the blind limits bound how far an error can grow. It also ignores the gait's response lag, so a robot that tracks commands closely can coast up to about 0.15 m past `stop_dist`; `obstacle_dist` is the backstop. An answer is used until the next one arrives, so both limits must cover two Gemini latencies (`max_blind_time` > 2 x latency, `max_blind_dist` > `walk_speed` x 2 x latency), or the robot stops on every answer.

A killed process (`kill -9`, power loss of the computer running it) cannot publish zero. The operator must hold the gamepad and press **Y** to take control back.

Camera frames are sent to Google's Gemini API.

## Test plan

Each stage must pass before the next; never skip to the floor test.

| Stage | Setup | Pass criteria |
| --- | --- | --- |
| 1. Unit tests | Laptop, no ROS: `cd er_goto && python3 -m pytest test` | All pass |
| 2. Snapshot | Laptop, saved room photos: `python3 er_goto/scripts/snapshot_test.py --target chair photos/*.jpg` | Chair marked correctly on 5 of 5 photos; median latency recorded |
| 3. Camera dry run | RDK X5 + camera, robot off: `./tools/start_er_goto.sh --dry-run --debug-images chair` | Holding the camera, bearing sign and distance (within about 15 % of a tape measure) are correct; commands stream at about `publish_rate` on `/cmd_vel_test` only (`ros2 topic hz /cmd_vel_test`). Nothing moves, so dead reckoning drifts: expect PURSUE / STAND cycles rather than DONE |
| 4. Crane | Robot hanging, default policy running, Y pressed | Legs step in the commanded direction and keep stepping while Gemini thinks; Ctrl+C mid-walk returns the legs to stepping in place; `kill -9` mid-walk keeps the last command until the operator presses Y |
| 5. Floor | Open area, chair 2-3 m away, operator holding the gamepad | Robot walks without stopping between answers and ends 0.8 m (± 0.2 m) in front of the chair; also with the chair starting 90 deg to the side |

In stage 5, `STAND: walked ... since the newest fix` or `STAND: newest fix is ... old` on most answers means Gemini is slower than the blind limits allow: raise them a little or lower `walk_speed`. If the robot often stops well short or overshoots turns, the gait follows commands less closely than dead reckoning assumes; integrating the D435i gyro (`enable_gyro` in `realsense_d435i_rgbd.yaml`) for yaw would be the next step. If the robot turns the wrong way in stage 4, fix the sign in `geometry.pixel_to_bearing` and rerun stage 3. If the camera is not mounted facing straight ahead, set `camera_yaw_offset` (rad, + = camera turned left). The math assumes a roughly level camera: if it is pitched down at the floor, distances read long and the floor itself can trip the obstacle check, so check both in stage 3 and tune `stop_dist` / `obstacle_dist`. If `distance` is often unknown (thin chair legs), a later improvement is to ask Gemini for a bounding box and take depth from its lower centre.

## Parameters

All tunables are in `config/params.yaml` with comments. Main defaults: `stop_dist` 0.8 m, `walk_speed` 0.35 m/s, `turn_speed` 0.5 rad/s, `k_turn` 1.5, `turn_in_place_angle` 0.5 rad, `max_blind_time` 10 s, `max_blind_dist` 2.5 m, `thinking_level` low, 15 s Gemini timeout. `max_linear_x` / `max_angular_z` (0.5 / 0.6) are the node's clamps, below the policy's own limits (0.6 / 0.8). Topic names, `depth_scale`, `jpeg_quality` and `debug_dir` have defaults in `node.py` and can be overridden the same way.
