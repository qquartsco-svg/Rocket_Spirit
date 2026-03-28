"""LaunchAgent — 발사체 통합 오케스트레이터.

레이어 통합:
  Layer 0 (contracts): RocketState, TelemetryFrame, FlightPhase
  Layer 1 (physics):   AtmosphereEngine, PropulsionEngine, AerodynamicsEngine,
                       VariableMassIntegrator
  Layer 2 (flight):    FlightPhaseFSM, StagingManager
  Layer 3 (guidance):  GravityTurnGuidance, TVCController
  Layer 4 (safety):    RangeSafetySystem, AbortSystem, FlightChain

에코시스템 브리지 (선택적):
  TAM (StarScream)  → 호버 readiness 를 발사 사전 인가로 수신
  Air Jordan        → 저고도(< 20 km) 공력 보조 판정
  AgedCare_Stack    → 유인 발사(MVP-4) 탑승자 안전 게이트

틱 파이프라인 (매 dt_s):
  1.  대기 상태 추정 (고도 → AtmosphereEngine)
  2.  유도·제어 명령 생성 (상태·단계 → GravityTurn + TVC)
  3.  추진 상태 갱신 (스로틀·대기 → PropulsionEngine)
  4.  공력 계산 (대기·속도 → AerodynamicsEngine)
  5.  RK4 적분 (추력·항력·질량 → VariableMassIntegrator)
  6.  비행 단계 전이 판정 (FlightPhaseFSM)
  7.  단분리 처리 (StagingManager)
  8.  Range Safety 점검
  9.  건전성 Ω 계산 [+ Air Jordan 보조]
  10. 중단 평가
  11. 텔레메트리 기록 (FlightChain)

설계 원칙:
  - 하나의 tick() 이 하나의 물리적 시간스텝에 대응
  - 상태는 함수적으로 갱신 (RocketState 불변 → 새 객체)
  - 외부에서 주입 가능한 모든 엔진 (Edge AI 패턴)
  - 브리지는 ImportError 시 자동 폴백 — 독립 동작 보장
"""
from __future__ import annotations

import math
from typing import Any, List, Optional

from .bridges.tam_bridge import TamLaunchReadiness, optional_tam_launch_readiness
from .bridges.air_jordan_bridge import optional_air_jordan_aero
from .adapters.aged_care_adapter import (
    AgedCareLaunchSafety,
    evaluate_crew_launch_gate,
)

from .contracts.schemas import (
    FlightCommand, FlightHealth, FlightPhase, PhysicsConfig,
    RocketState, TelemetryFrame, VehicleConfig, DEFAULT_PHYSICS,
    AbortMode,
)
from .physics.atmosphere import AtmosphereEngine
from .physics.propulsion import PropulsionEngine, PropulsionConfig
from .physics.aero import AerodynamicsEngine, VehicleAeroConfig
from .physics.integrator import VariableMassIntegrator
from .flight.phase_fsm import FlightPhaseFSM, FlightFSMConfig, FlightContext
from .flight.staging import StagingManager
from .guidance.gravity_turn import GravityTurnGuidance, GravityTurnConfig
from .guidance.tvc import TVCController, TVCConfig
from .safety.range_safety import RangeSafetySystem, RangeSafetyConfig, RangeSafetyReport
from .safety.abort_system import AbortSystem, AbortSystemConfig
from .audit.flight_chain import FlightChain


class LaunchAgent:
    """발사체 통합 오케스트레이터.

    사용법::
        vehicle = VehicleConfig(...)
        agent = LaunchAgent(vehicle)

        # 발사 허가
        agent.command_go()

        # 시뮬레이션 루프
        while agent.phase not in (FlightPhase.NOMINAL, FlightPhase.ABORT):
            frame = agent.tick()
            print(frame.summary_dict())

        print(agent.chain.summary())
    """

    def __init__(
        self,
        vehicle: VehicleConfig,
        dt_s: float = 0.1,
        # 레이어별 설정 (선택적 — 기본값 사용 가능)
        fsm_config:      Optional[FlightFSMConfig]      = None,
        guidance_config: Optional[GravityTurnConfig]    = None,
        tvc_config:      Optional[TVCConfig]             = None,
        range_config:    Optional[RangeSafetyConfig]    = None,
        abort_config:    Optional[AbortSystemConfig]    = None,
        propulsion_config: Optional[PropulsionConfig]   = None,
        chain_interval:  int = 10,    # N틱마다 체인 기록
        physics: PhysicsConfig = DEFAULT_PHYSICS,
        # ── 에코시스템 브리지 (선택) ─────────────────────────────────────
        tam_readiness:   Optional[TamLaunchReadiness]   = None,
        crew_safety:     Optional[AgedCareLaunchSafety] = None,
        human_rated_mvp4: bool = False,
    ):
        self._vehicle = vehicle
        self._dt_s    = dt_s
        self._phys    = physics

        # ── Layer 1: 물리 엔진 ────────────────────────────────────────────
        self._atm_eng  = AtmosphereEngine(physics)
        aero_cfg = VehicleAeroConfig(ref_area_m2=vehicle.ref_area_m2)
        self._aero_eng = AerodynamicsEngine(aero_cfg)
        self._integrator = VariableMassIntegrator(
            dry_mass_kg=sum(s.dry_mass_kg for s in vehicle.stages)
                        + vehicle.payload_mass_kg + vehicle.fairing_mass_kg,
            physics=physics,
        )

        # 추진 엔진은 첫 단으로 초기화
        first_stage = vehicle.stages[0]
        self._prop_eng = PropulsionEngine(first_stage, propulsion_config, physics)

        # ── Layer 2: 비행 FSM + 단 관리 ──────────────────────────────────
        self._fsm     = FlightPhaseFSM(fsm_config)
        self._staging = StagingManager(vehicle)

        # ── Layer 3: 유도·제어 ────────────────────────────────────────────
        self._guidance = GravityTurnGuidance(guidance_config)
        self._tvc      = TVCController(tvc_config)

        # ── Layer 4: 안전·감사 ────────────────────────────────────────────
        self._range_ss = RangeSafetySystem(range_config, physics)
        self._abort_sys = AbortSystem(abort_config)
        self._chain    = FlightChain(vehicle.vehicle_id, chain_interval)

        # ── 에코시스템 브리지 상태 ────────────────────────────────────────
        self._tam_readiness   = tam_readiness
        self._crew_safety     = crew_safety
        self._human_rated     = human_rated_mvp4
        self._air_jordan_evidence: dict = {}

        # ── 초기 상태 ─────────────────────────────────────────────────────
        self._state = RocketState(
            total_mass_kg=vehicle.total_liftoff_mass_kg,
            propellant_mass_kg=sum(s.propellant_mass_kg for s in vehicle.stages),
            t_s=0.0,
        )
        self._t_s   = 0.0
        self._go    = False   # 발사 허가 플래그

        # 초기 이벤트 기록
        init_ev: dict = {
            "vehicle":   vehicle.vehicle_id,
            "mass_kg":   vehicle.total_liftoff_mass_kg,
            "human_rated": human_rated_mvp4,
        }
        if tam_readiness is not None:
            init_ev["tam_mode"]    = tam_readiness.tam_mode
            init_ev["tam_omega"]   = tam_readiness.tam_omega_total
            init_ev["tam_auth"]    = tam_readiness.launch_authorized
        self._chain.record_event(0.0, "init", init_ev)

    # ── 공개 제어 ─────────────────────────────────────────────────────────────

    def command_go(self) -> None:
        """발사 허가 명령 (Go command)."""
        self._go = True
        self._chain.record_event(self._t_s, "go_command", {"t_s": self._t_s})

    def command_go_from_tam(
        self,
        report: Any = None,
        mode: str = "HOVER",
    ) -> bool:
        """TAM readiness 기반 발사 인가.

        TAM FlightReadinessReport 가 발사 최소 조건(HOVER, Ω≥0.80, blockers=없음)을
        만족할 때만 Go 명령을 발행한다.

        Args:
            report: TAM FlightReadinessReport (또는 duck-typed 동등 객체).
                    None 이면 기존 self._tam_readiness 를 사용.
            mode:   현재 TAM 모드 문자열.

        Returns:
            True = 발사 인가됨, False = 조건 미달 (차단).
        """
        if report is not None:
            self._tam_readiness = optional_tam_launch_readiness(report, mode)

        r = self._tam_readiness
        if r is None or not r.launch_authorized:
            reason = "tam_not_authorized" if r else "tam_readiness_missing"
            self._chain.record_event(self._t_s, "go_from_tam_denied",
                                     {"reason": reason,
                                      "tam_omega": r.tam_omega_total if r else 0.0})
            return False

        # 유인 MVP-4 추가 게이트
        if self._human_rated:
            crew_ok, crew_blockers = evaluate_crew_launch_gate(
                self._crew_safety, self._human_rated
            )
            if not crew_ok:
                self._chain.record_event(self._t_s, "go_from_tam_denied",
                                         {"reason": "crew_gate_failed",
                                          "blockers": crew_blockers})
                return False

        self._go = True
        self._chain.record_event(self._t_s, "go_from_tam",
                                 {"tam_mode":  r.tam_mode,
                                  "tam_omega": r.tam_omega_total,
                                  "tam_verdict": r.tam_verdict})
        return True

    def command_abort(self) -> None:
        """외부 중단 명령."""
        self._fsm.force_phase(FlightPhase.ABORT)
        self._chain.record_event(self._t_s, "abort_commanded", {"t_s": self._t_s})

    @property
    def phase(self) -> FlightPhase:
        return self._fsm.phase

    @property
    def state(self) -> RocketState:
        return self._state

    @property
    def chain(self) -> FlightChain:
        return self._chain

    # ── 메인 틱 ──────────────────────────────────────────────────────────────

    def tick(self) -> TelemetryFrame:
        """한 틱 실행 — 전체 레이어 파이프라인.

        Returns:
            TelemetryFrame — 이 틱의 스냅샷 (텔레메트리 + 건전성)
        """
        state = self._state
        dt    = self._dt_s

        # ── 1. 대기 상태 추정 ─────────────────────────────────────────────
        atm = self._atm_eng.query(state.altitude_m, state.speed_ms)

        # ── 2. 유도·제어 명령 (이전 상태 기반) ───────────────────────────
        raw_cmd = self._guidance.tick(state, self._fsm.phase, atm.dynamic_q_pa)
        cmd = self._tvc.tick(raw_cmd, state.pitch_rad, dt)

        # ── 3. 추진 상태 갱신 ─────────────────────────────────────────────
        prop = self._prop_eng.tick(atm, cmd.throttle, dt)

        # ── 4. 공력 계산 ──────────────────────────────────────────────────
        aero = self._aero_eng.tick(atm, state.speed_ms)

        # ── 5. RK4 적분 ───────────────────────────────────────────────────
        new_state = self._integrator.step(
            state=state,
            thrust_n=prop.thrust_n,
            pitch_rad=cmd.pitch_cmd_rad,
            yaw_rad=state.yaw_rad,
            drag_n=aero.drag_n,
            mass_flow_kgs=prop.mass_flow_kgs,
            dt_s=dt,
        )
        # 동압·마하 수 파생값 갱신
        new_atm = self._atm_eng.query(new_state.altitude_m, new_state.speed_ms)
        new_state = RocketState.from_vector(
            vec=new_state.as_vector(),
            propellant_kg=new_state.propellant_mass_kg,
            t_s=new_state.t_s,
            pitch_rad=cmd.pitch_cmd_rad,
            yaw_rad=new_state.yaw_rad,
            mach=new_atm.mach,
            q_pa=new_atm.dynamic_q_pa,
        )

        # ── 6. 비행 단계 전이 판정 ────────────────────────────────────────
        fctx = FlightContext(
            t_s=new_state.t_s,
            altitude_m=new_state.altitude_m,
            dynamic_q_pa=new_atm.dynamic_q_pa,
            propellant_kg=prop.propellant_mass_kg,
            speed_ms=new_state.speed_ms,
            go_command=self._go,
            meco_command=(not prop.is_ignited and
                          self._fsm.phase in (FlightPhase.ASCENDING, FlightPhase.MAX_Q,
                                              FlightPhase.UPPER_BURN)),
        )
        prev_phase = self._fsm.phase
        self._fsm.update(fctx)
        new_phase = self._fsm.phase

        # 단계 전이 이벤트 기록
        if new_phase != prev_phase:
            self._chain.record_event(new_state.t_s, "phase_transition",
                                     {"from": prev_phase.value, "to": new_phase.value})

        # ── 7. 단분리 처리 ────────────────────────────────────────────────
        stage_idx = self._staging.stage_idx
        if new_phase == FlightPhase.STAGE_SEP and prev_phase != FlightPhase.STAGE_SEP:
            event = self._staging.separate(new_state.t_s)
            if event:
                self._chain.record_event(new_state.t_s, "stage_separation",
                                         {"stage": event.stage_id,
                                          "ejected_kg": event.mass_ejected_kg})
                next_stage = self._staging.active_stage
                if next_stage:
                    self._prop_eng.reset_stage(next_stage)

        # 1단 점화 (COUNTDOWN → LIFTOFF 전환 시)
        if new_phase == FlightPhase.LIFTOFF and prev_phase == FlightPhase.COUNTDOWN:
            self._prop_eng.ignite()
            self._chain.record_event(new_state.t_s, "engine_ignition",
                                     {"stage": 1})

        # 페어링 분리 (고도 100 km 이상)
        if new_state.altitude_m > 100_000.0:
            ejected = self._staging.eject_fairing(new_state.t_s)
            if ejected > 0:
                self._chain.record_event(new_state.t_s, "fairing_separation",
                                         {"mass_kg": ejected})

        # 2단 점화
        if new_phase == FlightPhase.UPPER_BURN and prev_phase == FlightPhase.STAGE_SEP:
            self._prop_eng.ignite()

        # ── 8. Range Safety 점검 ──────────────────────────────────────────
        # HOLD/COUNTDOWN/LIFTOFF 단계: IIP 가 발사장 근처에 있는 것은 정상.
        # 충분한 고도 확보 전까지 exclusion/corridor 체크를 유보.
        _rss_active = (new_state.altitude_m > 1_000.0 and new_phase not in (
            FlightPhase.HOLD, FlightPhase.COUNTDOWN, FlightPhase.LIFTOFF
        ))
        if _rss_active:
            rss_report: RangeSafetyReport = self._range_ss.check(new_state)
        else:
            from .safety.range_safety import RangeSafetyReport as _RSSReport
            rss_report = _RSSReport(
                in_corridor=True, iip_x_m=0.0, iip_y_m=0.0,
                iip_downrange_m=0.0, iip_crossrange_m=0.0,
                corridor_ok=True, iip_safe=True, destruct_required=False,
            )

        # ── 9. 건전성 Ω 계산 ──────────────────────────────────────────────
        health = self._compute_health(new_state, new_atm, prop, rss_report)

        # ── 10. 중단 평가 ─────────────────────────────────────────────────
        abort_mode = self._abort_sys.evaluate(
            health=health,
            state=new_state,
            phase=new_phase,
            q_pa=new_atm.dynamic_q_pa,
            range_destruct=rss_report.destruct_required,
        )
        if abort_mode != AbortMode.NONE:
            self._fsm.force_phase(FlightPhase.ABORT)
            self._chain.record_event(new_state.t_s, "abort",
                                     {"mode": abort_mode.value,
                                      "omega": health.omega})

        # ── 상태 업데이트 ─────────────────────────────────────────────────
        self._state = new_state
        self._t_s   = new_state.t_s
        self._go    = False  # go 는 한 틱만 활성

        # ── 11. 텔레메트리 기록 ───────────────────────────────────────────
        frame = TelemetryFrame(
            t_s=new_state.t_s,
            phase=self._fsm.phase,
            state=new_state,
            atm=new_atm,
            propulsion=prop,
            aero=aero,
            health=health,
            command=cmd,
            stage_idx=self._staging.stage_idx,
        )
        block = self._chain.record(frame)

        return frame

    # ── 내부: 건전성 계산 ─────────────────────────────────────────────────────

    def _compute_health(
        self,
        state: RocketState,
        atm,
        prop,
        rss: RangeSafetyReport,
    ) -> FlightHealth:
        """Ω_rocket = Ω_p × Ω_s × Ω_t × Ω_r [× Ω_aero (저고도, Air Jordan)].

        컴포넌트:
          Ω_propulsion:  추진제 잔량 비율
          Ω_structural:  동압 안전 마진
          Ω_trajectory:  속도/자세 발산 정도
          Ω_range:       Range Safety 상태
          Ω_aero:        Air Jordan 공력 보조 (저고도만, 없으면 1.0)
        """
        alerts: List[str] = []
        evidence: dict = {}

        # Ω_propulsion: 추진제 잔량
        prop_frac = self._prop_eng.propellant_fraction
        ω_p = max(0.1, prop_frac)

        # Ω_structural: 동압 안전 마진
        q_limit = 80_000.0
        q = atm.dynamic_q_pa
        if q > q_limit:
            ω_s = 0.10
            alerts.append(f"[위험] 동압 한계 초과: {q/1000:.1f} kPa")
        elif q > q_limit * 0.7:
            ω_s = 0.60
            alerts.append(f"[주의] 동압 높음: {q/1000:.1f} kPa")
        else:
            ω_s = 1.0

        # Ω_trajectory: 단순화 (속도 존재 여부)
        ω_t = 1.0 if state.speed_ms >= 0 else 0.5

        # Ω_range: Range Safety
        ω_r = 1.0 if rss.corridor_ok and rss.iip_safe else 0.20
        if not rss.corridor_ok:
            alerts.append(f"[위험] 비행복도 이탈")

        # Ω_aero: Air Jordan 저고도 공력 보조 (선택적)
        ω_aero = 1.0
        aj = optional_air_jordan_aero(
            altitude_m=state.altitude_m,
            speed_ms=state.speed_ms,
            mach=state.mach,
            mass_kg=state.total_mass_kg,
        )
        if aj is not None:
            _hm, _ld, aj_ev = aj
            evidence.update(aj_ev)
            # lift_ratio < 0.8 → 공력 이상 경고
            lr = aj_ev.get("aj_lift_ratio", 1.0)
            if lr < 0.5:
                ω_aero = 0.60
                alerts.append(f"[주의] Air Jordan 양력비 낮음: {lr:.2f}")
            self._air_jordan_evidence = aj_ev

        omega = ω_p * ω_s * ω_t * ω_r * ω_aero

        if omega >= 0.80:
            verdict = "NOMINAL"
        elif omega >= 0.50:
            verdict = "CAUTION"
        elif omega >= 0.25:
            verdict = "WARNING"
        else:
            verdict = "ABORT"

        return FlightHealth(
            omega=omega,
            verdict=verdict,
            alerts=alerts,
            abort_required=(verdict == "ABORT"),
            omega_propulsion=ω_p,
            omega_structural=ω_s,
            omega_trajectory=ω_t,
            omega_range=ω_r,
        )

    # ── 요약 ─────────────────────────────────────────────────────────────────

    @property
    def tam_readiness(self) -> Optional[TamLaunchReadiness]:
        return self._tam_readiness

    @property
    def air_jordan_evidence(self) -> dict:
        return dict(self._air_jordan_evidence)

    def summary(self) -> str:
        s = self._state
        lines = [
            f"[LaunchAgent] {self._vehicle.vehicle_id}",
            f"  단계:      {self._fsm.phase.value}",
            f"  미션시각:  {self._t_s:.1f} s",
            f"  고도:      {s.altitude_m/1000:.3f} km",
            f"  속도:      {s.speed_ms:.1f} m/s  (M={s.mach:.2f})",
            f"  동압:      {s.dynamic_q_pa/1000:.2f} kPa",
            f"  잔여질량:  {s.total_mass_kg:.0f} kg",
            f"  사거리:    {s.downrange_m/1000:.3f} km",
            f"  감사블록:  {self._chain.length}개",
        ]
        if self._tam_readiness is not None:
            r = self._tam_readiness
            lines.append(
                f"  TAM:       {r.tam_mode} | Ω={r.tam_omega_total:.3f} "
                f"| auth={r.launch_authorized}"
            )
        if self._air_jordan_evidence:
            ev = self._air_jordan_evidence
            lines.append(
                f"  Air Jordan: lr={ev.get('aj_lift_ratio', '-'):.2f} "
                f"| LD={ev.get('aj_ld', '-'):.1f}"
            )
        return "\n".join(lines)
