"""
One-off script: replay episode_000000.parquet through the real G1 WBC controller
(gear_wbc) inside the PickPlaceBottleLoco robocasa sim, and dump an offscreen
video of it — for visual sanity-checking of the LerobotReplayPolicy pipeline
that's already running via ROS2 in run_g1_control_loop.py / run_teleop_policy_loop.py.

This uses the same building blocks (SyncEnv/G1SyncEnv via sync_sim_utils.get_env,
get_policies, LerobotReplayPolicy) but drives them directly in one process so we
can grab frames with cv2 instead of going through ROS2 topics.

Caveat: our dataset has no recorded seed / initial mujoco state (no
`observation.sim.*` columns), so the scene resets to a *random* kitchen
layout/object placement rather than the exact one from the original episode.
Only the G1 body motion (from the recorded `action` / `action.eef` /
`teleop.*` columns) is faithfully replayed.
"""

import cv2
import numpy as np
import pandas as pd
import rclpy

rclpy.init(args=None)

from decoupled_wbc.control.main.teleop.configs.configs import SyncSimDataCollectionConfig
from decoupled_wbc.control.policy.lerobot_replay_policy import LerobotReplayPolicy
from decoupled_wbc.control.robot_model.instantiation import get_robot_type_and_model
from decoupled_wbc.control.utils.sync_sim_utils import get_env, get_policies

PARQUET_PATH = "/root/episode_000000.parquet"
WIDTH, HEIGHT = 640, 480
FPS = 20
CAMERAS = {
    "robot0_rs_tppview": "/root/replay_check_tpp.mp4",  # third-person, for whole-body/balance check
    "robot0_oak_egoview": "/root/replay_check_ego.mp4",  # matches dataset's recorded ego_view exactly
}

config = SyncSimDataCollectionConfig(
    robot="G1",
    task_name="PickPlaceBottleLoco",
    wbc_version="gear_wbc",
    renderer="mjviewer",
    enable_waist=False,
)

robot_type, robot_model = get_robot_type_and_model(config.robot, enable_waist_ik=config.enable_waist)

print("Building sync env (offscreen)...")
sync_env = get_env(
    config,
    onscreen=False,
    offscreen=True,
    camera_names=list(CAMERAS.keys()),
    render_camera=list(CAMERAS.keys())[0],
)

print("Building WBC policy...")
wbc_policy, _teleop_policy = get_policies(config, robot_type, robot_model, activate_keyboard_listener=False)

replay_policy = LerobotReplayPolicy(robot_model=robot_model, parquet_path=PARQUET_PATH)
n_frames = replay_policy._max_ctr
print(f"Episode has {n_frames} frames")

sync_env.reset()

fourcc = cv2.VideoWriter_fourcc(*"mp4v")
writers = {
    cam: cv2.VideoWriter(path, fourcc, FPS, (WIDTH, HEIGHT)) for cam, path in CAMERAS.items()
}

for t in range(n_frames):
    replay_action = replay_policy.get_action()
    wbc_goal = {
        "target_upper_body_pose": replay_action["target_upper_body_pose"],
        "wrist_pose": replay_action["wrist_pose"],
        "navigate_cmd": replay_action["navigate_cmd"],
        "base_height_command": replay_action["base_height_cmd"],
    }
    obs = sync_env.observe()
    wbc_policy.set_observation(obs)
    wbc_policy.set_goal(wbc_goal)
    wbc_action = wbc_policy.get_action()
    sync_env.queue_action(wbc_action)

    for cam, writer in writers.items():
        img = sync_env.base_env.sim.render(width=WIDTH, height=HEIGHT, camera_name=cam)[::-1]
        img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        writer.write(img_bgr)

    if t % 50 == 0:
        print(f"step {t}/{n_frames}")

for writer in writers.values():
    writer.release()
sync_env.close()
for cam, path in CAMERAS.items():
    print(f"Saved {cam} video to {path}")
