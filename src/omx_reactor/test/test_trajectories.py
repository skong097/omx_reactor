import pytest

from omx_reactor.trajectories import (
    traj_idle, traj_hello, traj_bye, traj_dance, traj_freeze, traj_console,
    traj_hand_out, traj_hands_up, traj_hands_up_wave,
    traj_point_back, traj_nod, traj_cheer, traj_heart, traj_strong, traj_sad, traj_twinkle,
    traj_gripper_open, traj_gripper_close,
    JOINT_NAMES, GRIPPER_JOINT_NAMES,
)


# arm controller trajectory (joint1~4) — 모든 검증 항목 적용
ALL_FACTORIES = [traj_idle, traj_hello, traj_bye, traj_dance, traj_freeze, traj_console,
                 traj_hand_out, traj_hands_up, traj_hands_up_wave,
                 traj_point_back, traj_nod, traj_cheer, traj_heart, traj_strong, traj_sad,
                 traj_twinkle]


GRIPPER_FACTORIES = [traj_gripper_open, traj_gripper_close]


@pytest.mark.parametrize('factory', GRIPPER_FACTORIES)
def test_gripper_factory_uses_gripper_joint(factory):
    t = factory()
    assert list(t.joint_names) == GRIPPER_JOINT_NAMES
    assert len(t.points) >= 1
    for p in t.points:
        assert len(p.positions) == 1   # gripper 는 single joint


@pytest.mark.parametrize('factory', ALL_FACTORIES)
def test_joint_names_match(factory):
    t = factory()
    assert list(t.joint_names) == JOINT_NAMES


@pytest.mark.parametrize('factory', ALL_FACTORIES)
def test_time_strictly_monotonic(factory):
    t = factory()
    times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
             for p in t.points]
    assert all(b > a for a, b in zip(times, times[1:])), \
        f'{factory.__name__} times not strictly monotonic: {times}'


@pytest.mark.parametrize('factory', ALL_FACTORIES)
def test_positions_within_safe_range(factory):
    t = factory()
    for p in t.points:
        for j, val in zip(t.joint_names, p.positions):
            assert -1.2 <= val <= 1.2, \
                f'{factory.__name__} {j}={val} out of safe ±1.2rad range'


@pytest.mark.parametrize('factory', ALL_FACTORIES)
def test_positions_length_matches_joints(factory):
    t = factory()
    n = len(t.joint_names)
    for p in t.points:
        assert len(p.positions) == n


def test_at_least_one_point():
    for f in ALL_FACTORIES:
        assert len(f().points) >= 1


# velocity gate — expressive motion limit, hardware (Dynamixel XM430) default ~4.8 rad/s
# 2.5 rad/s = current peak (2.0 in DANCE/HELLO) + 0.5 headroom for future tuning
MAX_VELOCITY_RAD_S = 2.5


@pytest.mark.parametrize('factory', ALL_FACTORIES)
def test_max_velocity_within_limit(factory):
    """Each segment's per-joint velocity must be <= MAX_VELOCITY_RAD_S.

    Single-point factories (IDLE, FREEZE) are exempt (no segments).
    Note: first segment's velocity from arbitrary prior pose is not gated here
    (this would require runtime tracking) — design assumes prior motion ends at HOME.
    """
    t = factory()
    if len(t.points) < 2:
        return
    times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
             for p in t.points]
    for i in range(len(t.points) - 1):
        dt = times[i + 1] - times[i]
        for j, (a, b) in enumerate(zip(t.points[i].positions,
                                       t.points[i + 1].positions)):
            v = abs(b - a) / dt
            assert v <= MAX_VELOCITY_RAD_S, \
                (f'{factory.__name__} joint{j+1} segment {i}->{i+1}: '
                 f'v={v:.2f} > {MAX_VELOCITY_RAD_S} rad/s')


# ─── _chain helper tests ────────────────────────────────────────────────
from omx_reactor.trajectories import _chain, _traj, _point, HOME, JOINT_NAMES


def _t(*pairs):
    """Helper: build a JointTrajectory from (positions, t_sec) pairs."""
    return _traj([_point(list(pos), t) for pos, t in pairs])


def test_chain_offsets_second_trajectory_times():
    """Second trajectory's point times are shifted by first trajectory's last time."""
    a = _t((HOME, 0.4), (HOME, 1.0))                       # last = 1.0
    b = _t((HOME, 0.4), (HOME, 0.8))                       # last = 0.8 (orig)
    result = _chain(a, b)
    times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
             for p in result.points]
    # a's 2 points then b's 2 points shifted by 1.0
    assert times == pytest.approx([0.4, 1.0, 1.4, 1.8])


def test_chain_time_strictly_monotonic():
    """Chained trajectory's times must be strictly monotonic."""
    a = _t((HOME, 0.4), (HOME, 1.0))
    b = _t((HOME, 0.4), (HOME, 0.8))
    c = _t((HOME, 0.3), (HOME, 0.7))
    result = _chain(a, b, c)
    times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
             for p in result.points]
    assert all(t2 > t1 for t1, t2 in zip(times, times[1:])), times


def test_chain_preserves_joint_names():
    """Chained trajectory inherits joint_names from inputs."""
    a = _t((HOME, 0.4))
    b = _t((HOME, 0.5))
    result = _chain(a, b)
    assert list(result.joint_names) == JOINT_NAMES


def test_chain_rejects_mismatched_joint_names():
    """_chain raises if inputs have different joint_names."""
    a = _t((HOME, 0.4))
    b = _traj([_point([0.0], 0.5)])
    b.joint_names = ['gripper_left_joint']
    with pytest.raises(AssertionError):
        _chain(a, b)


# ─── traj_dance random sequence tests ──────────────────────────────────
import random as _random


def test_dance_deterministic_with_seed():
    """Same seed → identical trajectory positions sequence."""
    a = traj_dance(_rng=_random.Random(42))
    b = traj_dance(_rng=_random.Random(42))
    pos_a = [tuple(p.positions) for p in a.points]
    pos_b = [tuple(p.positions) for p in b.points]
    assert pos_a == pos_b


def test_dance_varies_across_seeds():
    """Different seeds → different sub-motion samples (different point counts or positions)."""
    a = traj_dance(_rng=_random.Random(0))
    b = traj_dance(_rng=_random.Random(1))
    pos_a = [tuple(p.positions) for p in a.points]
    pos_b = [tuple(p.positions) for p in b.points]
    assert pos_a != pos_b


def test_dance_composes_three_sub_motions():
    """traj_dance chains 3 sub-motions — point count must be sum of 3 pool members'
    point counts (every pool member has ≥3 points, so total ≥9)."""
    t = traj_dance(_rng=_random.Random(0))
    assert len(t.points) >= 9
