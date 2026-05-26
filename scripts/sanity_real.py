#!/usr/bin/env python3
"""sanity_real — omx_f bringup 이 떠 있는 상태에서 단발 IDLE → CONSOLE 검증.

reactor / 카메라 / dashboard 전부 미사용. 첫 동작 검증 전용.
풀 데모 (run_demo.sh --robot=real) 전에 반드시 1회 실행.

전제: `ros2 launch omx_reactor omx_real.launch.py` 가 다른 터미널에서 떠 있음.

사용법:
  cd ~/omx_reactor && source install/setup.bash
  OMX_ROBOT=real python3 scripts/sanity_real.py
"""
from __future__ import annotations

import os
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory


def main():
    if os.environ.get('OMX_ROBOT') != 'real':
        print('★ OMX_ROBOT=real 안 set 됨 — exit.', file=sys.stderr)
        return 1

    # trajectories 모듈은 import 시점에 환경 변수 확인 → 반드시 OMX_ROBOT 후 import
    from omx_reactor.trajectories import traj_idle, traj_console

    rclpy.init()
    node = Node('sanity_real')
    client = ActionClient(node, FollowJointTrajectory,
                          '/arm_controller/follow_joint_trajectory')

    print('→ arm_controller action 서버 대기 (max 10s)...')
    if not client.wait_for_server(timeout_sec=10.0):
        print('★ action 서버 없음 — bringup 떠 있는지 확인.', file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 2

    def send_and_wait(traj, label):
        print(f'→ {label} 전송...')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            print(f'  ★ {label} goal rejected', file=sys.stderr)
            return False
        res_fut = gh.get_result_async()
        # trajectory 자체 길이 + 여유 2s 대기
        max_t = max(p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
                    for p in traj.points)
        rclpy.spin_until_future_complete(node, res_fut, timeout_sec=max_t + 2.0)
        print(f'  ✓ {label} done')
        return True

    ok = send_and_wait(traj_idle(), 'IDLE')
    time.sleep(1.0)
    if ok:
        send_and_wait(traj_console(), 'CONSOLE')

    print('→ 종료. 동작 안 했으면 ros2 topic echo /joint_states 로 현재 위치 점검.')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
