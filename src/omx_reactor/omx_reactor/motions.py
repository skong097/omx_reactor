"""motions — Motion dataclass 정의 + MOTIONS 리스트 (Task 9 에서 채움)."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from omx_reactor.context import Context

# trajectory 의 실제 반환 타입은 trajectory_msgs/JointTrajectory 이나,
# 본 모듈은 ROS 무의존 유지를 위해 Callable[[], object] 로 둠.
# 테스트 시 Callable[[], None] 으로 대체 가능.


@dataclass(frozen=True)
class Motion:
    """Plug-in 단위 모션. 새 모션 추가 = MOTIONS 리스트에 항목 1개 + trajectory factory 1개."""
    id: str
    trigger: Callable[[Context], bool]   # ctx -> 매치 여부
    priority: int                        # 높을수록 interrupt 가능
    cooldown_sec: float                  # 같은 id 재실행 최소 간격
    trajectory: Callable[[], object]     # () -> JointTrajectory


# trajectories import 는 함수 정의 후에 한다 (순환 방지 필요 없음 — trajectories.py 는 motions 안 참조)
from omx_reactor.trajectories import (
    traj_dance, traj_freeze, traj_console, traj_idle, traj_hello, traj_bye,
    traj_hand_out, traj_hands_up, traj_hands_up_wave,
    traj_point_back, traj_nod, traj_cheer, traj_heart, traj_strong, traj_sad, traj_twinkle,
    traj_gripper_open, traj_gripper_close,
)

MOTIONS: list[Motion] = [
    # 감정 4분면 (priority 10)
    Motion('DANCE',
           trigger=lambda c: bool(c.emotion and c.emotion.quadrant == 'Q1'),
           priority=10, cooldown_sec=10.0, trajectory=traj_dance),
    Motion('FREEZE',
           trigger=lambda c: bool(c.emotion and c.emotion.quadrant == 'Q2'),
           priority=10, cooldown_sec=5.0, trajectory=traj_freeze),
    Motion('CONSOLE',
           trigger=lambda c: bool(c.emotion and c.emotion.quadrant == 'Q3'),
           priority=10, cooldown_sec=5.0, trajectory=traj_console),
    Motion('IDLE',
           trigger=lambda c: bool(c.emotion and
                                  (c.emotion.quadrant == 'Q4' or c.emotion.in_deadband)),
           priority=0,  cooldown_sec=0.0, trajectory=traj_idle),

    # 세션 경계 (priority 100 — 분면 모션 interrupt)
    Motion('HELLO',
           trigger=lambda c: c.session_event == 'new_track',
           priority=100, cooldown_sec=0.0, trajectory=traj_hello),
    Motion('BYE',
           trigger=lambda c: c.session_event == 'track_gone',
           priority=100, cooldown_sec=0.0, trajectory=traj_bye),

    # Gesture mimic (HELLO/BYE 보다 낮지만 분면 모션 interrupt) — priority 80~95
    Motion('HAND_OUT',
           trigger=lambda c: bool(c.gesture and c.gesture.event == 'hand_visible'),
           priority=80, cooldown_sec=5.0, trajectory=traj_hand_out),
    Motion('TWINKLE',
           trigger=lambda c: bool(c.gesture and c.gesture.event == 'twinkle'),
           priority=82, cooldown_sec=5.0, trajectory=traj_twinkle),
    Motion('HANDS_UP',
           trigger=lambda c: bool(c.gesture and c.gesture.event == 'hands_up'),
           priority=85, cooldown_sec=5.0, trajectory=traj_hands_up),
    Motion('HANDS_UP_WAVE',
           trigger=lambda c: bool(c.gesture and c.gesture.event == 'hands_up_wave'),
           priority=90, cooldown_sec=5.0, trajectory=traj_hands_up_wave),
    Motion('POINT_BACK',
           trigger=lambda c: bool(c.gesture and c.gesture.event == 'pointing_up'),
           priority=82, cooldown_sec=5.0, trajectory=traj_point_back),
    Motion('NOD',
           trigger=lambda c: bool(c.gesture and c.gesture.event == 'thumb_up'),
           priority=82, cooldown_sec=5.0, trajectory=traj_nod),
    Motion('SAD',
           trigger=lambda c: bool(c.gesture and c.gesture.event == 'thumb_down'),
           priority=82, cooldown_sec=5.0, trajectory=traj_sad),
    Motion('CHEER',
           trigger=lambda c: bool(c.gesture and c.gesture.event == 'victory'),
           priority=85, cooldown_sec=5.0, trajectory=traj_cheer),
    Motion('HEART',
           trigger=lambda c: bool(c.gesture and c.gesture.event == 'ilove_you'),
           priority=82, cooldown_sec=5.0, trajectory=traj_heart),
    Motion('STRONG',
           trigger=lambda c: bool(c.gesture and c.gesture.event == 'closed_fist'),
           priority=82, cooldown_sec=5.0, trajectory=traj_strong),
    # Gripper — 별 controller (reactor 가 trajectory.joint_names 으로 dispatch 분기)
    Motion('GRIPPER_OPEN',
           trigger=lambda c: bool(c.gesture and c.gesture.event == 'gripper_open'),
           priority=95, cooldown_sec=3.0, trajectory=traj_gripper_open),
    Motion('GRIPPER_CLOSE',
           trigger=lambda c: bool(c.gesture and c.gesture.event == 'gripper_close'),
           priority=95, cooldown_sec=3.0, trajectory=traj_gripper_close),
]
