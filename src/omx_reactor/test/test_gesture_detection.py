"""TDD — hand visibility detection + cooldown.

gesture_detection.py 는 mediapipe / ROS 무의존 pure logic.
mediapipe HandLandmarker 결과 (landmarks 존재 여부 bool) 를 받아
'hand_visible' 이벤트 emit + cooldown 관리.
"""
import pytest

from omx_reactor.gesture_detection import HandVisibilityDetector


def test_first_visible_emits():
    det = HandVisibilityDetector(cooldown_sec=5.0)
    assert det.update(visible=True, t_now=0.0) == 'hand_visible'


def test_visible_continuous_within_cooldown_no_emit():
    det = HandVisibilityDetector(cooldown_sec=5.0)
    det.update(visible=True, t_now=0.0)
    assert det.update(visible=True, t_now=1.0) is None
    assert det.update(visible=True, t_now=4.9) is None


def test_visible_after_cooldown_emits_again():
    det = HandVisibilityDetector(cooldown_sec=5.0)
    det.update(visible=True, t_now=0.0)
    assert det.update(visible=True, t_now=5.01) == 'hand_visible'


def test_not_visible_no_emit():
    det = HandVisibilityDetector(cooldown_sec=5.0)
    assert det.update(visible=False, t_now=0.0) is None
    assert det.update(visible=False, t_now=1.0) is None


def test_visible_lost_then_visible_again_within_cooldown_no_emit():
    """grace 안 손 깜빡임 + 복귀 — cooldown 안이라 emit X (spam 방지)."""
    det = HandVisibilityDetector(cooldown_sec=5.0)
    det.update(visible=True, t_now=0.0)
    det.update(visible=False, t_now=1.0)
    assert det.update(visible=True, t_now=2.0) is None


def test_visible_lost_then_visible_after_cooldown_emits():
    """오래 떠난 후 손 다시 보임 — emit."""
    det = HandVisibilityDetector(cooldown_sec=5.0)
    det.update(visible=True, t_now=0.0)
    det.update(visible=False, t_now=1.0)
    assert det.update(visible=True, t_now=6.0) == 'hand_visible'


def test_cooldown_zero_emits_every_transition():
    """cooldown=0 시 매 visibility=True 마다 emit (테스트 친화)."""
    det = HandVisibilityDetector(cooldown_sec=0.0)
    assert det.update(visible=True, t_now=0.0) == 'hand_visible'
    assert det.update(visible=True, t_now=0.1) == 'hand_visible'


# ─── classify_hand_state — Open_Palm + wrist 위치 + wave 결합 ───
from omx_reactor.gesture_detection import classify_hand_state, WaveDetector


def test_classify_open_palm_middle_no_wave_is_hand_visible():
    # wrist_y=0.6 > up_threshold 0.55 → middle
    assert classify_hand_state('Open_Palm', wrist_y_normalized=0.6, is_waving=False) == 'hand_visible'


def test_classify_open_palm_up_no_wave_is_hands_up():
    assert classify_hand_state('Open_Palm', wrist_y_normalized=0.3, is_waving=False) == 'hands_up'


def test_classify_open_palm_up_wave_is_hands_up_wave():
    assert classify_hand_state('Open_Palm', wrist_y_normalized=0.3, is_waving=True) == 'hands_up_wave'


def test_classify_open_palm_up_threshold_boundary():
    # default up_threshold=0.55 — exclusive
    assert classify_hand_state('Open_Palm', wrist_y_normalized=0.54, is_waving=False) == 'hands_up'
    assert classify_hand_state('Open_Palm', wrist_y_normalized=0.56, is_waving=False) == 'hand_visible'


def test_classify_open_palm_middle_wave_is_twinkle():
    assert classify_hand_state('Open_Palm', wrist_y_normalized=0.6, is_waving=True) == 'twinkle'


def test_classify_gesture_direct_mappings():
    assert classify_hand_state('Pointing_Up', wrist_y_normalized=0.5, is_waving=False) == 'pointing_up'
    assert classify_hand_state('Thumb_Up', wrist_y_normalized=0.5, is_waving=False) == 'thumb_up'
    assert classify_hand_state('Thumb_Down', wrist_y_normalized=0.5, is_waving=False) == 'thumb_down'
    assert classify_hand_state('Victory', wrist_y_normalized=0.5, is_waving=False) == 'victory'
    assert classify_hand_state('ILoveYou', wrist_y_normalized=0.5, is_waving=False) == 'ilove_you'
    assert classify_hand_state('Closed_Fist', wrist_y_normalized=0.5, is_waving=False) == 'closed_fist'


def test_classify_gripper_distance_fallback():
    """gesture None + thumb-index distance — gripper open/close fallback."""
    assert classify_hand_state(None, wrist_y_normalized=0.5, is_waving=False,
                                thumb_index_distance=0.30) == 'gripper_open'
    assert classify_hand_state(None, wrist_y_normalized=0.5, is_waving=False,
                                thumb_index_distance=0.02) == 'gripper_close'
    # 중간 거리 — None
    assert classify_hand_state(None, wrist_y_normalized=0.5, is_waving=False,
                                thumb_index_distance=0.10) is None


def test_classify_gripper_skipped_when_known_gesture():
    """known gesture 인식 시 distance 무관 (gesture 매핑 우선)."""
    # Thumb_Up + distance 큼 — gesture 매핑 thumb_up (gripper_open 아님)
    assert classify_hand_state('Thumb_Up', wrist_y_normalized=0.5, is_waving=False,
                                thumb_index_distance=0.30) == 'thumb_up'


def test_classify_none_gesture_returns_none():
    """gesture None + distance None → None (fallback 데이터 없음)."""
    assert classify_hand_state(None, wrist_y_normalized=0.5, is_waving=False) is None
    assert classify_hand_state('', wrist_y_normalized=0.3, is_waving=False) is None


# ─── WaveDetector — wrist x 시계열 진동 감지 ───

def test_wave_constant_no_oscillation():
    wd = WaveDetector(window_size=10, oscillation_threshold=0.2)
    for _ in range(10):
        wd.update(wrist_x_normalized=0.5)
    assert wd.is_waving() is False


def test_wave_strong_oscillation_detected():
    wd = WaveDetector(window_size=10, oscillation_threshold=0.2)
    for x in [0.3, 0.7, 0.3, 0.7, 0.3, 0.7, 0.3, 0.7, 0.3, 0.7]:
        wd.update(wrist_x_normalized=x)
    assert wd.is_waving() is True


def test_wave_below_threshold_no_detect():
    wd = WaveDetector(window_size=10, oscillation_threshold=0.3)
    # peak-to-peak = 0.2 < threshold 0.3
    for x in [0.4, 0.6, 0.4, 0.6]:
        wd.update(wrist_x_normalized=x)
    assert wd.is_waving() is False


def test_wave_window_drops_old_samples():
    wd = WaveDetector(window_size=4, oscillation_threshold=0.2)
    # 초기 진동 → window 가 새 정적 값으로 채워지면 wave 해제
    for x in [0.3, 0.7, 0.3, 0.7]:
        wd.update(wrist_x_normalized=x)
    assert wd.is_waving() is True
    for _ in range(5):   # window=4 보다 더 — 옛 진동 다 빠짐
        wd.update(wrist_x_normalized=0.5)
    assert wd.is_waving() is False


def test_wave_insufficient_samples_no_detect():
    """window_size 채우기 전엔 wave X (단조롭다 판정 어려움 — false positive 방지)."""
    wd = WaveDetector(window_size=10, oscillation_threshold=0.2)
    wd.update(wrist_x_normalized=0.3)
    wd.update(wrist_x_normalized=0.7)
    assert wd.is_waving() is False


# ─── handedness branch — HANDSHAKE trigger ─────────────────────────────
def test_classify_open_palm_right_hand_middle_is_handshake_offer():
    """Open_Palm + wrist 중간 + 오른손 + not waving → handshake_offer (NEW)."""
    assert classify_hand_state(
        gesture='Open_Palm',
        wrist_y_normalized=0.7,    # 중간 (> up_threshold 0.55)
        is_waving=False,
        handedness='Right',
    ) == 'handshake_offer'


def test_classify_open_palm_left_hand_middle_is_hand_visible():
    """Open_Palm + wrist 중간 + 왼손 → 기존 hand_visible 유지."""
    assert classify_hand_state(
        gesture='Open_Palm',
        wrist_y_normalized=0.7,
        is_waving=False,
        handedness='Left',
    ) == 'hand_visible'


def test_classify_open_palm_no_handedness_middle_is_hand_visible():
    """Open_Palm + wrist 중간 + handedness 미감지 (None) → hand_visible 폴백."""
    assert classify_hand_state(
        gesture='Open_Palm',
        wrist_y_normalized=0.7,
        is_waving=False,
        handedness=None,
    ) == 'hand_visible'


def test_classify_open_palm_right_hand_middle_waving_is_twinkle():
    """Open_Palm + wrist 중간 + 오른손 + waving → twinkle (waving 우선)."""
    assert classify_hand_state(
        gesture='Open_Palm',
        wrist_y_normalized=0.7,
        is_waving=True,
        handedness='Right',
    ) == 'twinkle'
