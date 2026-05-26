"""real-mode (OMX_ROBOT=real) joint name 분기 + 5축 padding 검증."""
import importlib
import os

import pytest


@pytest.fixture
def real_trajectories(monkeypatch):
    """OMX_ROBOT=real 환경에서 trajectories 모듈을 fresh import."""
    monkeypatch.setenv('OMX_ROBOT', 'real')
    import omx_reactor.trajectories as t
    importlib.reload(t)
    yield t
    importlib.reload(t)   # restore module to current env (monkeypatch already restored OMX_ROBOT)


def test_real_joint_names_5axis(real_trajectories):
    assert real_trajectories.JOINT_NAMES == \
        ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']


def test_real_gripper_joint_name(real_trajectories):
    assert real_trajectories.GRIPPER_JOINT_NAMES == ['gripper_joint_1']


_ARM_FACTORY_NAMES = [
    'traj_idle', 'traj_hello', 'traj_bye', 'traj_console',
    'traj_hand_out', 'traj_hands_up', 'traj_hands_up_wave',
    'traj_point_back', 'traj_nod', 'traj_cheer', 'traj_heart',
    'traj_strong', 'traj_handshake', 'traj_sad', 'traj_twinkle',
    'traj_freeze',
]


@pytest.mark.parametrize('factory_name', _ARM_FACTORY_NAMES)
def test_real_arm_trajectory_has_5_positions(real_trajectories, factory_name):
    """각 arm trajectory point 의 positions 길이가 5 (joint5=0 padding)."""
    f = getattr(real_trajectories, factory_name)
    t = f()
    assert list(t.joint_names) == \
        ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
    for p in t.points:
        assert len(p.positions) == 5, f'{factory_name}: {p.positions}'
        assert p.positions[4] == 0.0, f'{factory_name} joint5 must be 0'


def test_real_gripper_uses_gripper_joint_1(real_trajectories):
    for f in (real_trajectories.traj_gripper_open,
              real_trajectories.traj_gripper_close):
        t = f()
        assert list(t.joint_names) == ['gripper_joint_1']
        for p in t.points:
            assert len(p.positions) == 1


_DANCE_POOL_NAMES = [
    'traj_twinkle', 'traj_hands_up', 'traj_hands_up_wave',
    'traj_cheer', 'traj_heart', 'traj_nod', 'traj_strong',
]


@pytest.mark.parametrize('factory_name', _DANCE_POOL_NAMES)
def test_real_dance_pool_member_is_5axis(real_trajectories, factory_name):
    """DANCE pool 의 모든 멤버가 padding 후 5축이어야 _chain 경계가 안전."""
    f = getattr(real_trajectories, factory_name)
    t = f()
    assert list(t.joint_names) == \
        ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
    for p in t.points:
        assert len(p.positions) == 5
        assert p.positions[4] == 0.0
