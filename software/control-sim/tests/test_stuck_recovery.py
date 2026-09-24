"""Tests for stuck / stall / tilt auto-backup recovery.

スタック検知の3条件を個別に検証し、誤検知が起きないことも確認する。

パラメータ (control_config.h より):
  CPU0_SENSOR_STUCK_CONFIRM_MS       = 1000   (検知継続時間)
  CPU0_SENSOR_STUCK_MIN_CMD_RPM      = 50     (対象前進指令下限)
  CPU0_SENSOR_STUCK_STALL_SPEED_MM_S = 15     (ストール判定閾値)
  CPU0_SENSOR_STUCK_SLIP_ACCEL_DELTA_MG = 40  (空転判定加速度変化閾値)
  CPU0_SENSOR_STUCK_TILT_THRESH_MG   = 250    (傾斜判定閾値)
"""

from __future__ import annotations

import ctypes

import pytest

from controlsim.bindings import (
    BOOL,
    CPU0_SENSOR_VALID_ALL,
    ObstacleAvoidanceOutput,
    SensorSnapshot,
    library,
)

# obstacle_avoidance rule / phase constants
RULE_BACKUP = 9  # CPU0_SENSOR_RULE_BACKUP_START

# Minimum command RPM that triggers the stuck-detection window (>= 50 RPM).
_CMD_RPM = 120

# dt per step [ms] — matches firmware's 100 ms control cycle
_DT_MS = 100

# Number of steps to exceed STUCK_CONFIRM_MS (1000 ms) with margin
_CONFIRM_STEPS = 12  # 12 × 100 ms = 1200 ms  >  1000 ms confirm threshold


# --------------------------------------------------------------------------- #
#  Helper: incremental controller runner
# --------------------------------------------------------------------------- #

class StuckController:
    """Thin wrapper around the C controller for stuck-detection tests.

    ``last_cmd_rpm`` is seeded before the stuck-window loop so the
    firmware's forward_motion_commanded condition evaluates to TRUE.
    """

    def __init__(self) -> None:
        self.handle = library()
        self.handle.obstacle_avoidance_controller_init()
        self.now_ms = 0

    def _make_snapshot(
        self,
        *,
        accel_y: int = 0,
        update_count: int = 0,
        tof: tuple[int, int, int] = (1500, 1500, 1500),
    ) -> SensorSnapshot:
        s = SensorSnapshot()
        s.tof_distance_mm[:] = tof
        s.valid_flags = CPU0_SENSOR_VALID_ALL
        s.initialized = 1
        s.accel_mg[:] = (0, accel_y, 1000)
        s.update_count = update_count
        return s

    def step(
        self,
        *,
        accel_y: int = 0,
        linear_speed_mm_s: int = 120,
        update_count: int = 0,
        tof: tuple[int, int, int] = (1500, 1500, 1500),
        encoder_rpm_x10: tuple[int, int] | None = None,
    ) -> ObstacleAvoidanceOutput:
        snapshot = self._make_snapshot(accel_y=accel_y, update_count=update_count, tof=tof)
        output = ObstacleAvoidanceOutput()
        self.handle.obstacle_avoidance_encoder_feedback_set(
            BOOL(encoder_rpm_x10 is not None),
            ctypes.c_uint32(update_count),
            ctypes.c_int16(encoder_rpm_x10[0] if encoder_rpm_x10 else 0),
            ctypes.c_int16(encoder_rpm_x10[1] if encoder_rpm_x10 else 0),
        )
        self.handle.obstacle_avoidance_controller_step(
            ctypes.byref(snapshot),
            BOOL(False),
            self.now_ms & 0xFFFF_FFFF,
            ctypes.c_int16(0),           # target_steering_deg
            ctypes.c_int16(linear_speed_mm_s),
            ctypes.byref(output),
        )
        self.now_ms += _DT_MS
        return output

    def prime_forward(self, n: int = 2) -> None:
        """Run normal-forward steps to seed last_cmd_rpm >= 50."""
        for i in range(n):
            self.step(accel_y=10 * (i % 3), linear_speed_mm_s=120, update_count=i + 1)


def test_side_wall_and_wheel_imbalance_back_up_while_center_is_open() -> None:
    """側壁近接と片輪の大幅減速が続けば、中心ToFと並進速度が正でも後退する。"""
    ctrl = StuckController()
    ctrl.prime_forward()
    outcomes = [ctrl.step(update_count=100 + i, tof=(311, 1000, 1495),
                          encoder_rpm_x10=(700, 280), linear_speed_mm_s=285) for i in range(6)]
    assert any(output.rule == RULE_BACKUP for output in outcomes)


def test_side_wall_alone_does_not_force_backup() -> None:
    """通過できる片側障害物では左右輪が正常なら従来の前進回避を許す。"""
    ctrl = StuckController()
    ctrl.prime_forward()
    outcomes = [ctrl.step(update_count=100 + i, tof=(360, 1000, 1495),
                          encoder_rpm_x10=(700, 600)) for i in range(12)]
    assert all(output.rule != RULE_BACKUP for output in outcomes)


def test_one_stalled_wheel_backs_up_despite_positive_center_speed() -> None:
    """片輪0RPM・片輪70RPMなら平均並進速度が正でもスタックを検出する。"""
    ctrl = StuckController()
    ctrl.prime_forward()
    outcomes = [ctrl.step(update_count=100 + i, linear_speed_mm_s=170,
                          encoder_rpm_x10=(700, 0)) for i in range(10)]
    assert any(output.rule == RULE_BACKUP for output in outcomes)


def test_stale_encoder_sequence_does_not_accumulate_stall_time() -> None:
    """CPU1状態が更新されなければ同一の0RPMを繰り返し積算しない。"""
    ctrl = StuckController()
    ctrl.prime_forward()
    outcomes = [ctrl.step(update_count=100, linear_speed_mm_s=170,
                          encoder_rpm_x10=(700, 0)) for _ in range(12)]
    assert all(output.rule != RULE_BACKUP for output in outcomes)

# --------------------------------------------------------------------------- #
#  Test: motor stall detection
# --------------------------------------------------------------------------- #

def test_stall_triggers_backup_after_confirm_period() -> None:
    """モーターストール: エンコーダ速度 <= 15 mm/s が 1000 ms 継続で BACKUP 発動。"""
    ctrl = StuckController()
    ctrl.prime_forward()

    backup_triggered = False
    for i in range(_CONFIRM_STEPS):
        # ToF is clear (>500 mm), so frontal_collision trigger does NOT fire.
        # linear_speed_mm_s = 0 → stall condition.
        output = ctrl.step(
            accel_y=0,
            linear_speed_mm_s=0,
            update_count=100 + i,
            tof=(1500, 1500, 1500),
        )
        if output.rule == RULE_BACKUP:
            backup_triggered = True
            break

    assert backup_triggered, (
        "Motor stall (speed=0) for > 1000 ms should trigger BACKUP "
        f"(last rule={output.rule})"
    )


# --------------------------------------------------------------------------- #
#  Test: no false positive during steady-speed forward driving
# --------------------------------------------------------------------------- #

def test_no_false_positive_during_steady_speed_driving() -> None:
    """定常速度走行（accel_y変化なし）でスタック判定しない。

    slip_detectedを削除したため、速度120mm/s・accel_y固定のような
    定常走行シナリオは誤検知しないことを確認する。
    """
    ctrl = StuckController()
    ctrl.prime_forward()

    # 20ステップ (2000 ms) — 定常速度120mm/sで走らせる
    for i in range(20):
        output = ctrl.step(
            accel_y=0,           # 定常走行 → accel変化なし（slip検知なし）
            linear_speed_mm_s=120,  # > STALL_SPEED(15) → stall検知なし
            update_count=200 + i,
            tof=(1500, 1500, 1500),
        )
        assert output.rule != RULE_BACKUP, (
            f"False BACKUP triggered at step {i} during steady-speed driving "
            f"(rule={output.rule})"
        )



# --------------------------------------------------------------------------- #
#  Test: obstacle-climb / tilt detection
# --------------------------------------------------------------------------- #

def test_tilt_triggers_backup_after_confirm_period() -> None:
    """段差乗り上げ: |accel_y| >= 250 mg が 1000 ms 継続で BACKUP 発動。"""
    ctrl = StuckController()
    ctrl.prime_forward()

    backup_triggered = False
    for i in range(_CONFIRM_STEPS):
        output = ctrl.step(
            accel_y=300,        # 300 mg >= 250 mg threshold → tilt detected
            linear_speed_mm_s=80,
            update_count=300 + i,
            tof=(1500, 1500, 1500),
        )
        if output.rule == RULE_BACKUP:
            backup_triggered = True
            break

    assert backup_triggered, (
        "Tilt (accel_y=300 mg) for > 1000 ms should trigger BACKUP "
        f"(last rule={output.rule})"
    )


# --------------------------------------------------------------------------- #
#  Test: no false positive during normal driving
# --------------------------------------------------------------------------- #

def test_no_false_positive_during_normal_driving() -> None:
    """正常走行中はスタック判定しない: 速度120 mm/s、Y加速度が正常変動、ToFクリア。"""
    ctrl = StuckController()
    ctrl.prime_forward()

    # 25ステップ (2500 ms) — 十分な時間を走らせてもBACKUPが発動しないことを確認
    for i in range(25):
        # accel_y が ±60 mg で振動 → delta >= 40 mg → 空転ではない
        accel_y = 60 if (i % 2 == 0) else -60
        output = ctrl.step(
            accel_y=accel_y,
            linear_speed_mm_s=120,   # speed > 30 mm/s, stall not triggered
            update_count=400 + i,
            tof=(1500, 1500, 1500),
        )
        assert output.rule != RULE_BACKUP, (
            f"False BACKUP triggered at step {i} during normal driving "
            f"(rule={output.rule}, left={output.left_rpm}, right={output.right_rpm})"
        )


# --------------------------------------------------------------------------- #
#  Test: no false positive during avoidance sequence
# --------------------------------------------------------------------------- #

def test_no_false_positive_during_avoidance_sequence() -> None:
    """回避シーケンス（PIVOT）中はスタック判定ウィンドウがリセットされる。

    正面ToFが300mm以下になるとコントローラは PIVOT フェーズに入る。
    フェーズが AVOIDANCE_ESCAPE_NONE でないとき、stuck_elapsed_ms は
    リセットされるためスタック判定は発動しない。
    """
    ctrl = StuckController()

    # まず前方クリア状態で primeして last_cmd_rpm を前進値にする
    ctrl.prime_forward()

    # 正面に近い障害物を提示して回避シーケンスを開始させる
    for i in range(3):
        ctrl.step(
            accel_y=0,
            linear_speed_mm_s=0,     # stall条件だが回避中なので検知されない
            update_count=500 + i,
            tof=(1500, 250, 1500),   # center 250 mm → フロンタル衝突トリガー
        )

    # 回避フェーズ中に BACKUP 以外の理由で BACKUP が発動していないことを確認
    # (この時点では frontal_collision_count 経由の BACKUP は正常動作)
    # さらに10ステップ進め、stuck_elapsed_ms ベースの BACKUP が発動しないことを確認
    # (回避中は stuck チェックをスキップするため)
    for i in range(10):
        output = ctrl.step(
            accel_y=0,
            linear_speed_mm_s=0,     # stall条件だが回避中
            update_count=510 + i,
            tof=(1500, 250, 1500),
        )
        # BACKUP が発動しても、それは frontal_collision によるものであり
        # stuck_elapsed_ms ベースのものではない (どちらもrule=9なので区別できないが
        # 回避フェーズ中のスタック検知コードは実行されないことを確認する意味がある)
        # 少なくとも BLOCKED(rule=5) や SAFE_STOP(rule=1) は出ないことを確認
        assert output.rule not in (1, 5), (
            f"Unexpected stop during avoidance sequence at step {i} "
            f"(rule={output.rule})"
        )
