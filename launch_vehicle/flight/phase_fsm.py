"""비행 단계 FSM — 발사체 비행 단계 전이 관리 (Layer 2).

상태 전이 그래프:
  HOLD → COUNTDOWN → LIFTOFF → ASCENDING → MAX_Q
       → MECO → STAGE_SEP → UPPER_BURN → COAST
       → ORBIT_INSERT → NOMINAL
  (모든 단계) → ABORT

전이 조건:
  HOLD → COUNTDOWN:    external trigger (go_command)
  COUNTDOWN → LIFTOFF: T-0 도달
  LIFTOFF → ASCENDING: 고도 > tower_clear_m
  ASCENDING → MAX_Q:   동압 q 증가 → 감소 전환점 (자동)
  MAX_Q → MECO:        추진제 고갈 또는 MECO 명령
  MECO → STAGE_SEP:    타이머 (coast_before_sep_s)
  STAGE_SEP → UPPER_BURN: 점화 지연 후 자동
  UPPER_BURN → COAST:  2단 추진제 고갈 또는 MECO
  COAST → ORBIT_INSERT: 외부 명령 (apogee_kick)
  ORBIT_INSERT → NOMINAL: 완료 신호

설계 철학:
  - FSM 자체는 물리를 들고 있지 않음 — 관찰값(context)만 수신
  - 전이 조건은 FlightContext 로부터 읽음
  - abort 는 어느 단계든 즉시 전이 가능
  - 타이머 기반 전이 + 조건 기반 전이 혼용 (실제 비행과 유사)
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from ..contracts.schemas import AbortMode, FlightPhase


@dataclass
class FlightFSMConfig:
    """비행 FSM 파라미터.

    모든 임계값은 하드코딩 금지 — 발사체 설계에 따라 달라짐.
    """
    # 단계 전이 조건
    tower_clear_m:      float = 100.0   # 탑 클리어 고도 (m)
    countdown_duration_s: float = 10.0  # 카운트다운 시간 (s)
    coast_before_sep_s: float = 2.0     # MECO 후 단분리까지 코스트 시간 (s)
    ignition_delay_s:   float = 1.0     # 단분리 후 상단 점화 지연 (s)
    # Max-Q 판정
    q_maxq_threshold_pa: float = 5_000.0  # Max-Q 진입 동압 임계 (Pa)
    # 안전 한계
    max_q_abort_pa:     float = 100_000.0  # 이 이상이면 구조 한계 (Pa) → ABORT 고려
    max_altitude_m:     float = 1_000_000.0  # 궤도 이탈 상한 (참고)


@dataclass
class FlightContext:
    """FSM 전이 판단에 필요한 관찰값 묶음.

    물리값은 외부(적분기·모니터)에서 계산 후 주입.
    """
    t_s:            float
    altitude_m:     float
    dynamic_q_pa:   float
    propellant_kg:  float       # 현재 단 잔여 추진제
    speed_ms:       float
    # 외부 명령 플래그
    go_command:     bool = False   # 발사 허가
    meco_command:   bool = False   # 수동 MECO
    apogee_kick:    bool = False   # 궤도 삽입 명령
    orbit_complete: bool = False   # 궤도 삽입 완료
    abort_trigger:  bool = False   # 비상 중단 명령


class FlightPhaseFSM:
    """발사체 비행 단계 유한 상태 기계.

    사용법::
        fsm = FlightPhaseFSM(config)
        ctx = FlightContext(t_s=0, altitude_m=0, ...)
        phase = fsm.update(ctx)
    """

    def __init__(self, config: Optional[FlightFSMConfig] = None):
        self._cfg   = config or FlightFSMConfig()
        self._phase = FlightPhase.HOLD

        # 내부 타이머
        self._countdown_started_t: float = -1.0
        self._meco_t:  float = -1.0
        self._sep_t:   float = -1.0
        self._q_prev:  float = 0.0
        self._in_maxq: bool  = False
        # Max-Q 구간 완료 플래그: 한 번 통과한 이후 재진입 방지.
        # 이유: 동압이 고도 상승 후 감소 전환 시 미세 진동으로
        #       ASCENDING↔MAX_Q 를 반복하는 오진입 현상 제거.
        self._max_q_complete: bool = False

    # ── 공개 API ──────────────────────────────────────────────────────────────

    @property
    def phase(self) -> FlightPhase:
        return self._phase

    def update(self, ctx: FlightContext) -> FlightPhase:
        """컨텍스트 관찰값으로 단계 전이 판정 후 현재 단계 반환.

        Args:
            ctx: 현재 비행 관찰값

        Returns:
            현재(또는 전이된) FlightPhase
        """
        # ABORT 는 어느 단계에서든 최우선
        if ctx.abort_trigger and self._phase not in (FlightPhase.ABORT, FlightPhase.NOMINAL):
            self._transition(FlightPhase.ABORT, ctx.t_s, "abort_trigger")
            return self._phase

        if self._phase == FlightPhase.HOLD:
            self._from_hold(ctx)
        elif self._phase == FlightPhase.COUNTDOWN:
            self._from_countdown(ctx)
        elif self._phase == FlightPhase.LIFTOFF:
            self._from_liftoff(ctx)
        elif self._phase == FlightPhase.ASCENDING:
            self._from_ascending(ctx)
        elif self._phase == FlightPhase.MAX_Q:
            self._from_maxq(ctx)
        elif self._phase == FlightPhase.MECO:
            self._from_meco(ctx)
        elif self._phase == FlightPhase.STAGE_SEP:
            self._from_stage_sep(ctx)
        elif self._phase == FlightPhase.UPPER_BURN:
            self._from_upper_burn(ctx)
        elif self._phase == FlightPhase.COAST:
            self._from_coast(ctx)
        elif self._phase == FlightPhase.ORBIT_INSERT:
            self._from_orbit_insert(ctx)

        return self._phase

    def force_phase(self, phase: FlightPhase) -> None:
        """외부에서 직접 단계 강제 설정 (테스트·디버그용)."""
        self._phase = phase

    # ── 전이 핸들러 ───────────────────────────────────────────────────────────

    def _from_hold(self, ctx: FlightContext) -> None:
        if ctx.go_command:
            self._countdown_started_t = ctx.t_s
            self._transition(FlightPhase.COUNTDOWN, ctx.t_s, "go_command")

    def _from_countdown(self, ctx: FlightContext) -> None:
        elapsed = ctx.t_s - self._countdown_started_t
        if elapsed >= self._cfg.countdown_duration_s:
            self._transition(FlightPhase.LIFTOFF, ctx.t_s, "T-0")

    def _from_liftoff(self, ctx: FlightContext) -> None:
        if ctx.altitude_m >= self._cfg.tower_clear_m:
            self._transition(FlightPhase.ASCENDING, ctx.t_s, "tower_clear")

    def _from_ascending(self, ctx: FlightContext) -> None:
        # Max-Q 진입: 동압이 임계 이상 AND 아직 Max-Q 구간 미통과
        # _max_q_complete=True 이면 재진입 금지 (동압 감소 후 미세 상승 오진입 방지)
        if (ctx.dynamic_q_pa >= self._cfg.q_maxq_threshold_pa
                and not self._in_maxq
                and not self._max_q_complete):
            self._in_maxq = True
            self._transition(FlightPhase.MAX_Q, ctx.t_s, "q_threshold")
        elif ctx.meco_command or ctx.propellant_kg <= 0.0:
            self._meco_t = ctx.t_s
            self._transition(FlightPhase.MECO, ctx.t_s, "meco")

    def _from_maxq(self, ctx: FlightContext) -> None:
        # Max-Q 탈출: 동압이 감소세로 전환되면 다시 ASCENDING
        # _max_q_complete=True 로 마킹 → 이후 ASCENDING에서 MAX_Q 재진입 차단
        if ctx.dynamic_q_pa < self._q_prev and self._in_maxq:
            self._in_maxq = False
            self._max_q_complete = True  # Max-Q 구간 완료
            self._transition(FlightPhase.ASCENDING, ctx.t_s, "q_past_peak")
        elif ctx.meco_command or ctx.propellant_kg <= 0.0:
            self._meco_t = ctx.t_s
            self._transition(FlightPhase.MECO, ctx.t_s, "meco")
        self._q_prev = ctx.dynamic_q_pa

    def _from_meco(self, ctx: FlightContext) -> None:
        elapsed = ctx.t_s - self._meco_t
        if elapsed >= self._cfg.coast_before_sep_s:
            self._sep_t = ctx.t_s
            self._transition(FlightPhase.STAGE_SEP, ctx.t_s, "sep_timer")

    def _from_stage_sep(self, ctx: FlightContext) -> None:
        elapsed = ctx.t_s - self._sep_t
        if elapsed >= self._cfg.ignition_delay_s:
            self._transition(FlightPhase.UPPER_BURN, ctx.t_s, "upper_ignition")

    def _from_upper_burn(self, ctx: FlightContext) -> None:
        if ctx.meco_command or ctx.propellant_kg <= 0.0:
            self._transition(FlightPhase.COAST, ctx.t_s, "upper_meco")

    def _from_coast(self, ctx: FlightContext) -> None:
        if ctx.apogee_kick:
            self._transition(FlightPhase.ORBIT_INSERT, ctx.t_s, "apogee_kick")

    def _from_orbit_insert(self, ctx: FlightContext) -> None:
        if ctx.orbit_complete or ctx.propellant_kg <= 0.0:
            self._transition(FlightPhase.NOMINAL, ctx.t_s, "orbit_complete")

    # ── 내부 ─────────────────────────────────────────────────────────────────

    def _transition(self, target: FlightPhase, t_s: float, reason: str) -> None:
        self._phase = target
