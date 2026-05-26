"""trajectories — 6 P0 모션의 JointTrajectory factory.

OMX joint 매핑:
  joint1: base 회전 (좌우)
  joint2: shoulder pitch
  joint3: elbow pitch
  joint4: wrist pitch

안전 제약:
  - 각도: ±1.2 rad (~±69°) 이내 (test_positions_within_safe_range 게이트)
  - velocity: <= 2.0 rad/s 의 expressive motion 한계 — 2.5 rad/s gate 로 regression-protect
    (Dynamixel XM430 default ~4.8 rad/s 의 ~40% — visual crispness + safety 균형)
  - self-collision 회피 책임은 작성자

환경 변수:
  - OMX_ROBOT=sim (default) | real
    모듈 import 시점에 한 번 평가됨. downstream module (e.g., motions.py)
    이 factory 함수 reference 를 캡처하면 그 snapshot 을 공유 — 테스트에서
    env 를 바꾸려면 trajectories 와 downstream module 둘 다 reload 필요.
"""
from __future__ import annotations

import os
import random

from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# OMX_ROBOT=sim (default) 또는 'real' — 실 omx_f hardware 는 5축 + gripper_joint_1
_IS_REAL = os.environ.get('OMX_ROBOT', 'sim').lower() == 'real'

JOINT_NAMES = (['joint1', 'joint2', 'joint3', 'joint4']
               + (['joint5'] if _IS_REAL else []))
HOME_4 = [0.0, -1.0, 0.5, 0.5]                  # OMX 의 일반 home 자세 (4축 base)
HOME = HOME_4 + ([0.0] if _IS_REAL else [])     # real 은 joint5=0 padding


def _point(positions: list[float], t_sec: float) -> JointTrajectoryPoint:
    p = JointTrajectoryPoint()
    # real 모드면 4축 positions 에 joint5=0.0 auto-padding (이미 5개면 그대로)
    pos = list(positions)
    if _IS_REAL and len(pos) == 4:
        pos.append(0.0)
    p.positions = pos
    p.time_from_start = Duration(sec=int(t_sec),
                                 nanosec=int((t_sec - int(t_sec)) * 1e9))
    return p


def _traj(points: list[JointTrajectoryPoint]) -> JointTrajectory:
    t = JointTrajectory()
    t.joint_names = list(JOINT_NAMES)
    t.points = points
    return t


def _chain(*trajectories: JointTrajectory) -> JointTrajectory:
    """Concatenate JointTrajectory objects with cumulative time offsets.

    All inputs must share joint_names. The i-th input's point times are
    shifted by the sum of the (i-1) preceding inputs' final time_from_start.

    Returns a new JointTrajectory; inputs are not mutated.
    """
    assert trajectories, '_chain requires at least one trajectory'
    base_names = list(trajectories[0].joint_names)
    for t in trajectories[1:]:
        assert list(t.joint_names) == base_names, \
            f'_chain joint_names mismatch: {list(t.joint_names)} vs {base_names}'

    out_points: list[JointTrajectoryPoint] = []
    offset = 0.0
    for t in trajectories:
        for p in t.points:
            p_time = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
            out_points.append(_point(list(p.positions), p_time + offset))
        last = t.points[-1]
        offset += last.time_from_start.sec + last.time_from_start.nanosec * 1e-9

    out = _traj(out_points)
    out.joint_names = base_names
    return out


def traj_idle() -> JointTrajectory:
    """home pose 정지."""
    return _traj([_point(HOME, 0.5)])


def traj_hello() -> JointTrajectory:
    """home → 정면 + joint4 위로 ±30° × 2 (손 흔듦), 3s."""
    return _traj([
        _point(HOME,                   0.5),
        _point([0.0, -1.0, 0.5, 1.0],  1.0),
        _point([0.0, -1.0, 0.5, 0.0],  1.5),
        _point([0.0, -1.0, 0.5, 1.0],  2.0),
        _point([0.0, -1.0, 0.5, 0.0],  2.5),
        _point(HOME,                   3.0),
    ])


def traj_bye() -> JointTrajectory:
    """HELLO 슬로우 (1.5×), 4.5s."""
    return _traj([
        _point(HOME,                   0.75),
        _point([0.0, -1.0, 0.5, 1.0],  1.5),
        _point([0.0, -1.0, 0.5, 0.0],  2.25),
        _point([0.0, -1.0, 0.5, 1.0],  3.0),
        _point([0.0, -1.0, 0.5, 0.0],  3.75),
        _point(HOME,                   4.5),
    ])


def traj_freeze() -> JointTrajectory:
    """현 자세 + joint2 살짝 down 0.02rad (기죽음), 1s."""
    return _traj([
        _point([0.0, -1.02, 0.5, 0.5], 1.0),
    ])


def traj_console() -> JointTrajectory:
    """joint1 정면, joint4 up/down ±0.25rad × 4 (쓰담쓰담), 4s."""
    return _traj([
        _point(HOME,                    0.4),
        _point([0.0, -1.0, 0.5, 0.75],  1.0),
        _point([0.0, -1.0, 0.5, 0.25],  1.6),
        _point([0.0, -1.0, 0.5, 0.75],  2.2),
        _point([0.0, -1.0, 0.5, 0.25],  2.8),
        _point([0.0, -1.0, 0.5, 0.75],  3.4),
        _point(HOME,                    4.0),
    ])


def traj_hand_out() -> JointTrajectory:
    """사용자가 손 내밀면 OMX 도 손 내밂 (mimic) — joint2 수평 + joint3 펴기 reach
    + 1s hold + home 복귀. 3.5s total. velocity peak ~0.91 rad/s.
    """
    REACH = [0.0, 0.0, 0.0, 0.5]
    return _traj([
        _point(HOME,  0.4),
        _point(REACH, 1.5),
        _point(REACH, 2.5),    # 1s hold
        _point(HOME,  3.5),
    ])


def traj_hands_up() -> JointTrajectory:
    """만세 — 팔 위로 쭉 펴기. joint2 -1.2 (shoulder 위 limit) + joint3 0 + joint4 0 (elbow/wrist 일자).
    + 0.4s hold + home 복귀. 2.7s.
    """
    UP = [0.0, -1.2, 0.0, 0.0]
    return _traj([
        _point(HOME, 0.4),
        _point(UP,   1.5),
        _point(UP,   1.9),    # hold
        _point(HOME, 2.7),
    ])


def traj_hands_up_wave() -> JointTrajectory:
    """팔 위로 쭉 + 좌우 흔들기 (인사/안녕) — UP 자세 유지 + joint1 ±0.7 swing × 2회. 5.2s."""
    UP      = [0.0,  -1.2, 0.0, 0.0]
    WAVE_R  = [0.7,  -1.2, 0.0, 0.0]
    WAVE_L  = [-0.7, -1.2, 0.0, 0.0]
    return _traj([
        _point(HOME,   0.4),
        _point(UP,     1.5),
        _point(WAVE_R, 2.1),
        _point(WAVE_L, 2.7),
        _point(WAVE_R, 3.3),
        _point(WAVE_L, 3.9),
        _point(UP,     4.5),
        _point(HOME,   5.2),
    ])


def traj_point_back() -> JointTrajectory:
    """가리킴 — joint1 한쪽 + joint3/joint4 펴기 (gripper 정면 reach), 2.5s."""
    PT = [0.6, -0.5, 0.0, 0.0]
    return _traj([
        _point(HOME, 0.4),
        _point(PT,   1.5),
        _point(PT,   1.9),
        _point(HOME, 2.5),
    ])


def traj_nod() -> JointTrajectory:
    """끄덕 — joint4 위아래 × 3 (좋아 응답), 3.3s. velocity peak 2.0 rad/s."""
    NOD_DN = [0.0, -1.0, 0.5, 1.0]
    NOD_UP = [0.0, -1.0, 0.5, 0.0]
    return _traj([
        _point(HOME,   0.4),
        _point(NOD_DN, 0.9),
        _point(NOD_UP, 1.4),
        _point(NOD_DN, 1.9),
        _point(NOD_UP, 2.4),
        _point(HOME,   3.3),
    ])


def traj_cheer() -> JointTrajectory:
    """축하 (V 사인) — joint1 ±0.5 × 4 + joint2 살짝 위, 3.7s. velocity peak 2.0 rad/s."""
    UP_R = [0.5,  -1.2, 0.0, 0.0]
    UP_L = [-0.5, -1.2, 0.0, 0.0]
    UP_C = [0.0,  -1.2, 0.0, 0.0]
    return _traj([
        _point(HOME, 0.4),
        _point(UP_C, 1.0),
        _point(UP_R, 1.5),
        _point(UP_L, 2.0),
        _point(UP_R, 2.5),
        _point(UP_L, 3.0),
        _point(UP_C, 3.4),
        _point(HOME, 3.7),
    ])


def traj_heart() -> JointTrajectory:
    """사랑해 손 — 부드러운 좌우 swing × 2 (천천히), 4.0s."""
    HRT_R = [0.4, -0.8, 0.3, 0.4]
    HRT_L = [-0.4, -0.8, 0.3, 0.4]
    return _traj([
        _point(HOME,  0.5),
        _point(HRT_R, 1.5),
        _point(HRT_L, 2.5),
        _point(HRT_R, 3.5),
        _point(HOME,  4.0),
    ])


def traj_strong() -> JointTrajectory:
    """강한 자세 (주먹) — joint1 한쪽 + joint2 위 + joint3/joint4 펴기 (힘찬 reach), 2.5s hold."""
    ST = [0.5, -1.2, 0.0, 0.0]
    return _traj([
        _point(HOME, 0.4),
        _point(ST,   1.2),
        _point(ST,   2.0),    # hold
        _point(HOME, 2.5),
    ])


def traj_handshake() -> JointTrajectory:
    """오른손 악수 — REACH 1.5s + shake 4회 1s + REACH center 0.2s + HOLD 3s + slow HOME 2.8s.

    Total ~8.5s. velocity peak 2.4 rad/s (shake direction reversal).
    사람이 손을 떼더라도 3s HOLD 후 천천히 home 복귀 (release tolerance, force/vision 없음).
    """
    REACH    = [0.0, -0.5,  0.0, 0.0]
    SHAKE_UP = [0.0, -0.5, -0.3, 0.0]
    SHAKE_DN = [0.0, -0.5,  0.3, 0.0]
    return _traj([
        _point(HOME,     0.4),     # settle
        _point(REACH,    1.5),     # reach forward
        _point(SHAKE_UP, 1.75),
        _point(SHAKE_DN, 2.0),
        _point(SHAKE_UP, 2.25),
        _point(SHAKE_DN, 2.5),
        _point(REACH,    2.7),     # shake 중심 복귀
        _point(REACH,    5.7),     # HOLD 3s (release tolerance)
        _point(HOME,     8.5),     # 천천히 home 복귀 (2.8s)
    ])


def traj_sad() -> JointTrajectory:
    """슬픔 (엄지 아래) — joint2 down + joint3/joint4 굽힘 (머리 숙이듯), 3.0s."""
    SD = [0.0, -0.3, 1.0, 1.0]
    return _traj([
        _point(HOME, 0.5),
        _point(SD,   1.5),
        _point(SD,   2.3),    # hold
        _point(HOME, 3.0),
    ])


def traj_twinkle() -> JointTrajectory:
    """반짝반짝 — joint1 ±0.4 빠르게 alternating × 4 (base 좌우), 3.6s. velocity peak 2.0 rad/s."""
    TW_R = [0.4,  -1.0, 0.5, 0.5]
    TW_L = [-0.4, -1.0, 0.5, 0.5]
    return _traj([
        _point(HOME, 0.3),
        _point(TW_R, 0.7),
        _point(TW_L, 1.1),
        _point(TW_R, 1.5),
        _point(TW_L, 1.9),
        _point(TW_R, 2.3),
        _point(TW_L, 2.7),
        _point(TW_R, 3.1),
        _point(HOME, 3.6),
    ])


# ─── DANCE composite — random sample from happy sub-motion pool ──────────
_DANCE_POOL: tuple = (
    traj_twinkle, traj_hands_up, traj_hands_up_wave,
    traj_cheer, traj_heart, traj_nod, traj_strong,
)
_DANCE_SAMPLE_N: int = 3


def traj_dance(*, _rng: random.Random | None = None) -> JointTrajectory:
    """Happy 7 풀에서 3개 random sample (without replacement) → _chain.
    매 호출마다 다른 안무. 모든 sub-motion 이 HOME 시작/종료라 chain 경계
    velocity = 0 → 2.5 rad/s 게이트 안전.

    Args:
      _rng: 테스트용. None 이면 module random 사용 (런타임 경로).
    """
    rng = _rng if _rng is not None else random
    picks = rng.sample(_DANCE_POOL, _DANCE_SAMPLE_N)
    return _chain(*(f() for f in picks))


# ─── gripper trajectory — 별 controller (/gripper_controller/follow_joint_trajectory) ───
# joint_names = sim:'gripper_left_joint' | real:'gripper_joint_1'
# reactor 가 trajectory.joint_names[0].startswith('gripper') 로 dispatch 분기 (real/sim 동일)

GRIPPER_JOINT_NAMES = (['gripper_joint_1'] if _IS_REAL
                       else ['gripper_left_joint'])
GRIPPER_OPEN_ANGLE = 0.019    # OMX default open (rad)
GRIPPER_CLOSE_ANGLE = -0.010  # OMX default close


def _gripper_traj(angle: float, t_sec: float = 0.5) -> JointTrajectory:
    t = JointTrajectory()
    t.joint_names = list(GRIPPER_JOINT_NAMES)
    t.points = [_point([angle], t_sec)]
    return t


def traj_gripper_open() -> JointTrajectory:
    """gripper 열기 — gripper_left_joint = +0.019 rad."""
    return _gripper_traj(GRIPPER_OPEN_ANGLE, t_sec=0.5)


def traj_gripper_close() -> JointTrajectory:
    """gripper 닫기 — gripper_left_joint = -0.010 rad."""
    return _gripper_traj(GRIPPER_CLOSE_ANGLE, t_sec=0.5)
