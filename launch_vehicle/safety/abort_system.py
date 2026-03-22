"""중단 시스템 — 비상 중단 모드 관리 (Layer 4).

중단 모드:
  ENGINE_SHUTDOWN:  엔진 정지 → 탄도 낙하 (해양 투하 가정)
  SAFE_DESTRUCT:    Range Safety 명령 → 자폭 (파편 최소화)
  COAST_ABORT:      상단 분리 후 코스트 → 낙하

트리거 조건:
  - 동압 구조 한계 초과 (q > q_limit)
  - Range Safety 복도 이탈
  - 추진 이상 (추력 급락)
  - 자세 이탈 (pitch 오차 과대)
  - 외부 명령 (Range Control Officer)

건전성 Ω_flight 기반 통합:
  Ω = Ω_propulsion × Ω_structural × Ω_trajectory × Ω_range
  Ω < abort_omega → ABORT 권고

관찰 한계:
  - LES(Launch Escape System) 등 유인 모드 미포함
  - 자폭 폭발 물리 미모델링 (이벤트 플래그만)
  - 실제 Range Safety 절차·법적 요구사항 대체 불가
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional

from ..contracts.schemas import (
    AbortMode, FlightHealth, FlightPhase, RocketState
)


@dataclass
class AbortSystemConfig:
    """중단 시스템 파라미터."""
    # 동압 한계 (Pa) — 구조 하중 기준
    q_abort_pa:          float = 80_000.0   # 이 이상 → 구조 위험
    # 자세 이탈 한계 (rad)
    pitch_error_abort_rad: float = math.radians(20.0)  # 20° 이탈
    # Ω 중단 임계
    omega_abort_threshold: float = 0.25
    # 추력 급락 감지 (정상 대비 비율)
    thrust_loss_fraction:  float = 0.70     # 정상의 70% 미만 → 이상


@dataclass
class AbortEvent:
    """중단 이벤트 기록."""
    t_s:         float
    mode:        AbortMode
    phase:       FlightPhase
    trigger:     str
    omega:       float


class AbortSystem:
    """비상 중단 시스템.

    FlightHealth + 직접 센서값으로 중단 필요 여부 판정.

    사용법::
        abort_sys = AbortSystem(config)
        mode = abort_sys.evaluate(health, state, phase, q_pa)
        if mode != AbortMode.NONE:
            fsm.force_phase(FlightPhase.ABORT)
    """

    def __init__(self, config: Optional[AbortSystemConfig] = None):
        self._cfg    = config or AbortSystemConfig()
        self._events: List[AbortEvent] = []

    def evaluate(
        self,
        health: FlightHealth,
        state: RocketState,
        phase: FlightPhase,
        q_pa: float,
        pitch_error_rad: float = 0.0,
        range_destruct: bool   = False,
        external_abort: bool   = False,
    ) -> AbortMode:
        """중단 필요 여부 평가.

        Returns:
            AbortMode.NONE: 정상
            그 외: 해당 중단 모드 권고
        """
        if phase in (FlightPhase.HOLD, FlightPhase.COUNTDOWN,
                     FlightPhase.NOMINAL, FlightPhase.ABORT):
            return AbortMode.NONE

        trigger = self._check_triggers(
            health, q_pa, pitch_error_rad, range_destruct, external_abort
        )
        if trigger is None:
            return AbortMode.NONE

        mode = self._select_mode(phase, trigger)
        event = AbortEvent(
            t_s=state.t_s,
            mode=mode,
            phase=phase,
            trigger=trigger,
            omega=health.omega,
        )
        self._events.append(event)
        return mode

    @property
    def events(self) -> List[AbortEvent]:
        return list(self._events)

    # ── 내부 ─────────────────────────────────────────────────────────────────

    def _check_triggers(
        self,
        health: FlightHealth,
        q_pa: float,
        pitch_err: float,
        range_destruct: bool,
        external_abort: bool,
    ) -> Optional[str]:
        """트리거 조건 순서 확인 (우선순위 순)."""
        if external_abort:
            return "external_command"
        if range_destruct:
            return "range_safety_destruct"
        if q_pa > self._cfg.q_abort_pa:
            return f"q_limit:{q_pa:.0f}Pa"
        if abs(pitch_err) > self._cfg.pitch_error_abort_rad:
            return f"attitude_diverge:{math.degrees(pitch_err):.1f}deg"
        if health.omega < self._cfg.omega_abort_threshold:
            return f"omega_low:{health.omega:.3f}"
        return None

    def _select_mode(self, phase: FlightPhase, trigger: str) -> AbortMode:
        """비행 단계·트리거 기반 최적 중단 모드 선택."""
        if "range_safety" in trigger or "external" in trigger:
            return AbortMode.SAFE_DESTRUCT

        if phase in (FlightPhase.LIFTOFF, FlightPhase.ASCENDING,
                     FlightPhase.MAX_Q):
            return AbortMode.ENGINE_SHUTDOWN

        if phase in (FlightPhase.UPPER_BURN, FlightPhase.COAST,
                     FlightPhase.ORBIT_INSERT):
            return AbortMode.COAST_ABORT

        return AbortMode.ENGINE_SHUTDOWN
