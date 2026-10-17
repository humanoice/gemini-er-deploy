# roboparty_deploy

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS2-Humble%20%7C%20Jazzy-silver)](https://docs.ros.org/en/humble/index.html)
![C++](https://img.shields.io/badge/C++-17-blue)
![Linux](https://img.shields.io/badge/Linux-x86_64%20%7C%20aarch64-orange)

[English](README.md) | [中文](README_CN.md)

## Repository Overview

`roboparty_deploy` is a ROS 2 deployment framework for RPO and RP1, with motor drivers, IMU interfaces, policy inference, and depth perception.

The repository provides complete deployment code with a modular architecture that supports custom packages.

Repository: [Roboparty/roboparty_deploy](https://github.com/Roboparty/roboparty_deploy). Maintainer: RoboParty. Support: GitHub Issues.

This fork adds `er_goto`, which takes text commands typed at a prompt, such as "walk to the chair", and walks RPO to each object. The object is located with Google Gemini Robotics-ER and an Intel RealSense D435i on the RDK X5 controller. See [Walk to the Chair](#walk-to-the-chair).

## Documentation

| Guide | Contents |
| --- | --- |
| [RPO Deployment](docs/rpo_deploy.md) | RPO controller connection, environment, real-time kernel, AP setup, wiring, zeroing, and policy/camera startup |
| RP1 Deployment | Coming soon |
| [YAML Configuration Reference](docs/configuration.md) | Configuration loading, hardware, inference, camera, and zeroing parameters, ordering, and constraints |
| [Common Operation](docs/operation.md) | Build, automatic ROS detection, logs, gamepad, and services |
| [Python SDK](docs/python_sdk.md) | IMU, motor, and robot interfaces, plus example-script compatibility |
| [Walk to an Object (er_goto)](er_goto/README.md) | Gemini Robotics-ER walk-to-object node: how it works, parameters, safety, and test plan |

## Project Architecture

| Path | Responsibility |
| --- | --- |
| `src/motors` | DM, EVO, LRO, RP, and XYN motor drivers and communication |
| `src/imu` | HIPNUC and MCT7123 IMU drivers |
| `src/inference` | Robot interface, ONNX inference, observation assembly, and motion data loading |
| `src/inference/robots` | Hardware settings, policy parameters, models, and motion data for each robot |
| `src/camera` | RealSense integration, depth processing, and observation publishing |
| `er_goto` | Walk to an object found by Gemini Robotics-ER in the RealSense image; publishes `/cmd_vel` |
| `tools` | Environment detection and robot/camera/`er_goto` startup tools |
| `scripts` | Python SDK examples and motor zeroing tools |
| `docs` | Environment, deployment, configuration, and interface documentation |

Inference reads motor and IMU state through the robot interface, converts policy output into joint targets, and sends them through motor drivers. The camera module processes depth independently and supplies perception observations over ROS topics. `er_goto` runs beside inference and only sends velocity commands on `/cmd_vel`, which the `default` policy follows after the operator presses Y on the gamepad.

## Walk to the Chair

`er_goto` walks while it looks. Gemini Robotics-ER points at the chair in the camera image on a background thread, and local code turns that point into a bearing and a distance from the aligned depth image. Between answers the robot keeps walking toward the newest one, corrected by dead reckoning for how far it moved since that photo was taken. It stops only to confirm arrival, to look again when the answer is too old or the path is blocked, and to search, and ends about 0.8 m in front of the chair.

### One-time Setup on the RDK X5

The RealSense SDK must already be installed. Then:

```bash
git submodule update --init --recursive        # includes the RealSense ROS wrapper
sudo apt install -y python3-opencv python3-numpy python3-pytest
pip3 install --user google-genai
cp .env.example .env                           # then set GEMINI_API_KEY=... in .env
```

Do not `pip install opencv-python`: it pulls numpy 2, which breaks ROS Humble's Python packages. `.env` is git-ignored.

### Steps

1. Place the robot in an open area with the chair 2-3 m away. The chair does not have to be in view; the robot turns to search for it. Plug the D435i into a USB 3 port.
2. Terminal 1: start the robot with the `default` policy, then start the policy on the gamepad as usual (see [Common Operation](docs/operation.md)) and let the robot balance.

   ```bash
   ./tools/start_robot.sh
   ```

3. Terminal 2: start `er_goto`.

   ```bash
   ./tools/start_er_goto.sh
   ```

   The script builds `er_goto`, starts the camera in the `er_camera_session` screen session, checks the camera streams and `inference_node`, and then starts the node with an `er_goto>` prompt.
4. When the script prints the hand-over reminder, press **Y** on the gamepad. The robot ignores `er_goto` until Y switches it to `/cmd_vel` control.
5. Type where to go, as a command or just the object:

   ```
   er_goto> go to the black chair
   er_goto> walk to the lady
   er_goto> stop
   er_goto> quit
   ```

   The robot turns toward the target and walks while Gemini keeps looking. It stops about 0.8 m in front of the target, logs `Arrived at "black chair"`, and waits for the next command. It gives up, stops, and waits if the target is not found after about one full turn. A command typed while the robot walks replaces the current target: the robot stops, looks for the new one, and heads there. `stop` halts the robot and waits; `quit` or Ctrl+D stops it and exits.
6. Press **Y** again to return control to the gamepad.

Keep the gamepad in hand during the whole run. **Y** takes control back instantly, **B** pauses inference, and Ctrl+C in terminal 2 stops the robot and exits. The robot keeps the last `/cmd_vel` command if `er_goto` is killed without cleanup, so Y is the fallback. Camera frames are sent to Google's Gemini API.

Useful options:

| Command | Purpose |
| --- | --- |
| `./tools/start_er_goto.sh walk to the chair` | Run this command first, then show the prompt |
| `./tools/start_er_goto.sh --dry-run` | Publish to `/cmd_vel_test` instead of `/cmd_vel`; the robot does not need to run |
| `./tools/start_er_goto.sh --debug-images` | Save each camera frame with the Gemini point drawn to `~/er_goto_debug/<time>-<target>/` |
| `./tools/start_er_goto.sh --restart-camera` | Restart the camera session before running |
| `screen -S er_camera_session -X quit` | Stop the camera, which keeps running after the script exits |

Do not run `./tools/start_camera.sh` at the same time; it opens the same camera with the parkour configuration. Before the first floor run, follow the staged test plan in [er_goto/README.md](er_goto/README.md#test-plan): unit tests, saved photos, camera dry run, then the robot on a crane.

## License

Unless otherwise stated, original content authored by RoboParty in this repository is licensed under the [GNU General Public License v3.0 only](LICENSE) (`GPL-3.0-only`).

Git submodules, packaged binaries, and other third-party materials retain their own copyright and license terms. The repository-level GPLv3 statement does not replace those terms. See [LICENSES.md](LICENSES.md) for the current license inventory and redistribution notes.
