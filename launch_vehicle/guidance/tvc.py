"""TVC (Thrust Vector Control) — 추력 벡터 제어 (Layer 3).

역할:
  자세 오차 → 짐벌 각도 명령 → 추력 방향 조정 → 모멘트 발생

물리 배경:
  로켓은 공기역학적 조종면이 없는 상승 초기에
  엔진 짐벌(gimbal)로만 자세를 제어.
  짐벌 각도 δ → 추력 F 의 횡방향 성분 = F·sin(δ) ≈ F·δ

제어 구조:
  오차 = 목표 pitch − 현재 pitch
  짐벌 명령 = Kp·오차 + Kd·오차_변화율 (PD 제어기)

관찰 한계:
  - 짐벌 구동기 동특성(actuator lag) 미포함
  - 추력 편심 모멘트 미반영
  - 롤 제어 미포함 (MVP: pitch/yaw 2축만)
  - 자세 피드백은 외부 EKF/IMU 제공 가정
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from ..contracts.schemas import FlightCommand


@dataclass
class TVCConfig:
    """TVC PD 제어기 파라미터.

    짐벌 각도 물리적 한계:
      대부분 발사체: ±8° (±0.14 rad) 수준

    Kp, Kd 는 발사체 관성·추력에 따라 조정 필요.
    하드코딩 금지 — 발사체별 설계값.
    """
    kp:           float = 2.0                 # 비례 게인
    kd:           float = 0.5                 # 미분 게인
    max_gimbal_rad: float = math.radians(8.0) # 최대 짐벌 각도 (rad)
    dt_smooth_s:  float = 0.1                 # 미분 스무딩 시정수 (s)


class TVCController:
    """추력 벡터 제어기 (PD).

    사용법::
        tvc = TVCController(config)
        updated_cmd = tvc.tick(
            cmd=guidance_cmd,
            pitch_actual=state.pitch_rad,
            dt_s=0.05,
        )
    """

    def __init__(self, config: Optional[TVCConfig] = None):
        self._cfg       = config or TVCConfig()
        self._prev_err  = 0.0
        self._err_deriv = 0.0

    def tick(
        self,
        cmd: FlightCommand,
        pitch_actual: float,
        dt_s: float,
        yaw_actual: float = 0.0,
        yaw_target: float = 0.0,
    ) -> FlightCommand:
        """PD 제어기로 짐벌 명령 계산.

        Args:
            cmd:          유도기 출력 (pitch 목표 포함)
            pitch_actual: IMU/EKF 추정 현재 피치 (rad)
            dt_s:         타임스텝 (s)
            yaw_actual:   현재 요 (rad)
            yaw_target:   목표 요 (rad)

        Returns:
            짐벌 명령이 채워진 FlightCommand
        """
        # 피치 PD
        err_p = cmd.pitch_cmd_rad - pitch_actual
        self._err_deriv = self._smooth_deriv(err_p, dt_s)
        gimbal_y = self._pd(err_p)

        # 요 PD (단순화: 동일 게인)
        err_y = yaw_target - yaw_actual
        gimbal_z = self._pd(err_y)

        # 짐벌 한계 클램핑
        g_max = self._cfg.max_gimbal_rad
        gimbal_y = max(-g_max, min(g_max, gimbal_y))
        gimbal_z = max(-g_max, min(g_max, gimbal_z))

        return FlightCommand(
            pitch_cmd_rad=cmd.pitch_cmd_rad,
            gimbal_y_rad=gimbal_y,
            gimbal_z_rad=gimbal_z,
            throttle=cmd.throttle,
            request_abort=cmd.request_abort,
            note=cmd.note + f"|TVC:δy={math.degrees(gimbal_y):.1f}°",
        )

    # ── 내부 ─────────────────────────────────────────────────────────────────

    def _pd(self, error: float) -> float:
        return self._cfg.kp * error + self._cfg.kd * self._err_deriv

    def _smooth_deriv(self, err: float, dt_s: float) -> float:
        """지수 이동 평균으로 미분 스무딩 (노이즈 억제)."""
        tau = self._cfg.dt_smooth_s
        alpha = dt_s / (tau + dt_s)
        raw_deriv = (err - self._prev_err) / max(dt_s, 1e-6)
        self._err_deriv += alpha * (raw_deriv - self._err_deriv)
        self._prev_err = err
        return self._err_deriv
