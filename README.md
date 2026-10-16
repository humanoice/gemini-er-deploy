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

This fork adds `er_goto`, which takes a text command such as "walk to the chair" and walks RPO to that object. The object is located with Google Gemini Robotics-ER and an Intel RealSense D435i on the RDK X5 controller. See [Walk to the Chair](#walk-to-the-chair).

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

`er_goto` repeats a stop-look-move loop. The robot stands still while Gemini Robotics-ER points at the chair in the camera image. Local code turns that point into a bearing and a distance from the aligned depth image. The robot then turns or walks for a short, bounded time and looks again, until it stands about 0.8 m in front of the chair.

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

3. Terminal 2: run `er_goto` with the command or just the object.

   ```bash
   ./tools/start_er_goto.sh walk to the chair
   ./tools/start_er_goto.sh red chair near the window
   ```

   The script builds `er_goto`, starts the camera in the `er_camera_session` screen session, checks the camera streams and `inference_node`, and then starts the node.
4. When the script prints the hand-over reminder, press **Y** on the gamepad. The robot ignores `er_goto` until Y switches it to `/cmd_vel` control.
5. The robot looks, turns toward the chair, and walks in steps of up to 1 m. It stops about 0.8 m in front of the chair, and the node exits with `Arrived at "chair"`. It gives up and stops if the chair is not found after about one full turn.
6. Press **Y** again to return control to the gamepad.

Keep the gamepad in hand during the whole run. **Y** takes control back instantly, **B** pauses inference, and Ctrl+C in terminal 2 stops the robot. The robot keeps the last `/cmd_vel` command if `er_goto` is killed without cleanup, so Y is the fallback. Camera frames are sent to Google's Gemini API.

Useful options:

| Command | Purpose |
| --- | --- |
| `./tools/start_er_goto.sh --dry-run chair` | Publish to `/cmd_vel_test` instead of `/cmd_vel`; the robot does not need to run |
| `./tools/start_er_goto.sh --debug-images chair` | Save each camera frame with the Gemini point drawn to `~/er_goto_debug/` |
| `./tools/start_er_goto.sh --restart-camera chair` | Restart the camera session before running |
| `screen -S er_camera_session -X quit` | Stop the camera, which keeps running between commands |

Do not run `./tools/start_camera.sh` at the same time; it opens the same camera with the parkour configuration. Before the first floor run, follow the staged test plan in [er_goto/README.md](er_goto/README.md#test-plan): unit tests, saved photos, camera dry run, then the robot on a crane.

## License

Unless otherwise stated, original content authored by RoboParty in this repository is licensed under the [GNU General Public License v3.0 only](LICENSE) (`GPL-3.0-only`).

Git submodules, packaged binaries, and other third-party materials retain their own copyright and license terms. The repository-level GPLv3 statement does not replace those terms. See [LICENSES.md](LICENSES.md) for the current license inventory and redistribution notes.
