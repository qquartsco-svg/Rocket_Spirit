"""Gravity Turn 유도 — 중력 선회 피치 프로그램 (Layer 3).

원리:
  자연 중력 선회 = 수직 상승 후 소량 피치 킥 → 공기역학적으로
  속도벡터가 중력 방향으로 자동 회전 → 최소 에너지 상승 궤도.

단계별 유도 전략:
  1. 수직 상승 (0 ~ tower_clear_m):
     pitch = π/2 (수직 고정)

  2. 피치 킥 (tower_clear_m → kick_end_m):
     선형 감소: π/2 → kick_target_rad
     (빠를수록 적극적 선회, 느릴수록 보수적)

  3. 자연 선회 (kick_end_m ~):
     목표: 속도벡터 방향 추적 (flight path angle γ)
     pitch_cmd = γ = arctan(vz / vh)  (vh = sqrt(vx²+vy²))
     → 추력이 속도벡터 방향 = 자연 중력 선회

  4. 등동압 구간 (Max-Q):
     스로틀 감소로 동압 제한 (유도는 속도벡터 추적 유지)

관찰 한계:
  - 실제 closed-loop 최적 유도(PEG, Q-law) 대체 불가
  - 바람·대기 교란 피드백 없음
  - 목표 궤도 파라미터 기반 실시간 수정 미포함 (MVP 범위)
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from ..contracts.schemas import (
    FlightCommand, FlightPhase, RocketState
)


@dataclass
class GravityTurnConfig:
    """Gravity Turn 유도 파라미터.

    pitch_kick_target_rad: 피치 킥 목표각 (수직에서 동쪽 방향 기울기)
    kick_start_m:          피치 킥 시작 고도 (m)
    kick_end_m:            피치 킥 완료 고도 (m)
    throttle_nominal:      정상 스로틀 [0, 1]
    throttle_maxq:         Max-Q 구간 스로틀 감소값
    maxq_throttle_limit:   Max-Q 구간 동압 상한 (Pa)
    """
    pitch_kick_target_rad: float = math.radians(75.0)  # 수직에서 15° 기울기
    kick_start_m:          float = 500.0     # 피치 킥 시작 고도 (m)
    kick_end_m:            float = 5_000.0   # 피치 킥 완료 고도 (m)
    throttle_nominal:      float = 1.0
    throttle_maxq:         float = 0.72      # Max-Q 스로틀 감소 (Falcon 9 참고)
    maxq_throttle_limit_pa: float = 35_000.0  # 동압 임계 (Pa)
    # 최소 피치 (수평에 가까워지는 상한)
    pitch_floor_rad:       float = math.radians(5.0)   # 5° (거의 수평)


class GravityTurnGuidance:
    """Gravity Turn 유도기.

    매 틱 RocketState → FlightCommand 생성.
    stateless (설정만 보유, 상태 적분 없음).

    사용법::
        guidance = GravityTurnGuidance(config)
        cmd = guidance.tick(state, phase, q_pa=18000)
    """

    def __init__(self, config: Optional[GravityTurnConfig] = None):
        self._cfg = config or GravityTurnConfig()

    def tick(
        self,
        state: RocketState,
        phase: FlightPhase,
        q_pa: float = 0.0,
    ) -> FlightCommand:
        """한 틱 유도 명령 생성.

        Args:
            state: 현재 발사체 상태
            phase: 현재 비행 단계
            q_pa:  현재 동압 (Pa)

        Returns:
            FlightCommand — 피치·스로틀 명령
        """
        pitch = self._pitch_cmd(state)
        throttle = self._throttle_cmd(phase, q_pa)

        return FlightCommand(
            pitch_cmd_rad=pitch,
            throttle=throttle,
            note=f"GravityTurn|h={state.altitude_m:.0f}m|M={state.mach:.2f}",
        )

    # ── 내부 ─────────────────────────────────────────────────────────────────

    def _pitch_cmd(self, state: RocketState) -> float:
        """고도·속도 기반 피치 명령 계산.

        3단계 프로그램:
          1. 수직 상승 (h < kick_start_m): pitch = π/2
          2. 피치 킥 (kick_start ≤ h < kick_end): 선형 감소
          3. 자연 선회 (h ≥ kick_end): 속도벡터 방향
        """
        h = state.altitude_m
        cfg = self._cfg

        if h < cfg.kick_start_m:
            # ① 수직 상승
            return math.pi / 2.0

        if h < cfg.kick_end_m:
            # ② 피치 킥 (선형 보간)
            alpha = (h - cfg.kick_start_m) / (cfg.kick_end_m - cfg.kick_start_m)
            return math.pi / 2.0 + alpha * (cfg.pitch_kick_target_rad - math.pi / 2.0)

        # ③ 자연 선회 — 속도벡터 방향 (flight path angle γ)
        vh = math.sqrt(state.vx_ms**2 + state.vy_ms**2)  # 수평 속도
        vz = state.vz_ms                                   # 수직 속도

        if vh < 1.0 and vz > 0.0:
            return math.pi / 2.0  # 거의 수직 → 수직 유지

        # gamma = arctan2(vz, vh): 수평 기준 비행경로각
        gamma = math.atan2(vz, vh)
        # pitch = gamma 로 추력 벡터 정렬
        return max(cfg.pitch_floor_rad, min(math.pi / 2.0, gamma))

    def _throttle_cmd(self, phase: FlightPhase, q_pa: float) -> float:
        """비행 단계·동압 기반 스로틀 명령.

        MAX_Q 구간에서 스로틀 감소 → 동압 제어.
        """
        cfg = self._cfg

        if phase in (FlightPhase.HOLD, FlightPhase.COUNTDOWN,
                     FlightPhase.ABORT, FlightPhase.NOMINAL):
            return 0.0

        if phase == FlightPhase.MAX_Q:
            # 동압이 임계 이상이면 감소
            if q_pa > cfg.maxq_throttle_limit_pa:
                return cfg.throttle_maxq
            return cfg.throttle_nominal

        if phase in (FlightPhase.MECO, FlightPhase.STAGE_SEP,
                     FlightPhase.COAST):
            return 0.0

        return cfg.throttle_nominal
