"""LaunchVehicle_Stack 전체 테스트 스위트.

§1  데이터 계약 (Layer 0)
§2  대기 모델 — US Std Atm 1976
§3  추진 모델
§4  공력 모델
§5  가변 질량 RK4 적분기
§6  비행 단계 FSM
§7  단분리 관리자
§8  Gravity Turn 유도
§9  TVC 제어기
§10 Range Safety
§11 중단 시스템
§12 비행 기록 체인 (FlightChain)
§13 LaunchAgent 통합

실행:
    cd LaunchVehicle_Stack
    python -m pytest tests/ -v
"""
from __future__ import annotations

import math
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import pytest

from launch_vehicle.contracts.schemas import (
    PhysicsConfig, AtmosphereState, PropulsionState, AeroState,
    RocketState, StageConfig, VehicleConfig, FlightCommand,
    FlightPhase, AbortMode, TelemetryFrame, FlightHealth,
    DEFAULT_PHYSICS,
)
from launch_vehicle.physics.atmosphere import AtmosphereEngine
from launch_vehicle.physics.propulsion import PropulsionEngine, PropulsionConfig
from launch_vehicle.physics.aero import AerodynamicsEngine, VehicleAeroConfig
from launch_vehicle.physics.integrator import VariableMassIntegrator
from launch_vehicle.flight.phase_fsm import FlightPhaseFSM, FlightFSMConfig, FlightContext
from launch_vehicle.flight.staging import StagingManager
from launch_vehicle.guidance.gravity_turn import GravityTurnGuidance, GravityTurnConfig
from launch_vehicle.guidance.tvc import TVCController, TVCConfig
from launch_vehicle.safety.range_safety import RangeSafetySystem, RangeSafetyConfig
from launch_vehicle.safety.abort_system import AbortSystem, AbortSystemConfig
from launch_vehicle.audit.flight_chain import FlightChain, FlightBlock
from launch_vehicle.launch_agent import LaunchAgent


# ─────────────────────────────────────────────────────────────────────────────
# 공통 픽스처
# ─────────────────────────────────────────────────────────────────────────────

def make_stage(stage_id: int = 1, prop_kg: float = 400_000.0) -> StageConfig:
    return StageConfig(
        stage_id=stage_id,
        dry_mass_kg=25_000.0,
        propellant_mass_kg=prop_kg,
        engine_count=9,
        isp_sl_s=282.0,
        isp_vac_s=311.0,
        thrust_sl_n=7_607_000.0,
        thrust_vac_n=8_227_000.0,
        nozzle_exit_area_m2=11.5,
        max_throttle=1.0,
        min_throttle=0.57,
        burn_time_design_s=162.0,
    )


def make_stage2(prop_kg: float = 90_000.0) -> StageConfig:
    return StageConfig(
        stage_id=2,
        dry_mass_kg=4_000.0,
        propellant_mass_kg=prop_kg,
        engine_count=1,
        isp_sl_s=340.0,
        isp_vac_s=348.0,
        thrust_sl_n=934_000.0,
        thrust_vac_n=934_000.0,
        nozzle_exit_area_m2=2.0,
        max_throttle=1.0,
        min_throttle=0.39,
        burn_time_design_s=397.0,
    )


def make_vehicle() -> VehicleConfig:
    return VehicleConfig(
        vehicle_id="TestLV-001",
        stages=[make_stage(), make_stage2()],
        payload_mass_kg=22_800.0,
        fairing_mass_kg=1_900.0,
        body_diameter_m=3.66,
    )


def make_state(**kw) -> RocketState:
    defaults = dict(
        x_m=0.0, y_m=0.0, z_m=0.0,
        vx_ms=0.0, vy_ms=0.0, vz_ms=0.0,
        total_mass_kg=500_000.0,
        propellant_mass_kg=400_000.0,
        t_s=0.0,
    )
    defaults.update(kw)
    return RocketState(**defaults)


# ─────────────────────────────────────────────────────────────────────────────
# §1  데이터 계약
# ─────────────────────────────────────────────────────────────────────────────

class TestSchemas:
    def test_physics_config_defaults(self):
        p = PhysicsConfig()
        assert p.g0_ms2 == pytest.approx(9.80665)
        assert p.R_earth_m == pytest.approx(6_371_000.0)

    def test_rocket_state_as_vector_length(self):
        s = make_state()
        assert len(s.as_vector()) == 7

    def test_rocket_state_from_vector_roundtrip(self):
        s = make_state(x_m=100.0, vz_ms=50.0, total_mass_kg=400_000.0)
        vec = s.as_vector()
        s2 = RocketState.from_vector(vec, propellant_kg=300_000.0, t_s=10.0)
        assert s2.x_m == pytest.approx(100.0)
        assert s2.vz_ms == pytest.approx(50.0)

    def test_rocket_state_speed_ms_computed(self):
        s = make_state(vx_ms=3.0, vy_ms=4.0, vz_ms=0.0)
        assert s.speed_ms == pytest.approx(5.0)

    def test_stage_config_total_mass(self):
        st = make_stage()
        assert st.total_mass_kg == pytest.approx(25_000 + 400_000)

    def test_stage_mass_flow(self):
        st = make_stage()
        mdot = st.mass_flow_kgs(throttle=1.0)
        # F_vac / (Isp_vac * g0)
        expected = 8_227_000 / (311.0 * 9.80665)
        assert mdot == pytest.approx(expected, rel=1e-4)

    def test_vehicle_total_liftoff_mass(self):
        v = make_vehicle()
        # 1단 + 2단 + payload + fairing
        expected = (25_000 + 400_000) + (4_000 + 90_000) + 22_800 + 1_900
        assert v.total_liftoff_mass_kg == pytest.approx(expected)

    def test_vehicle_ref_area_computed(self):
        v = make_vehicle()
        expected = math.pi * (3.66 / 2) ** 2
        assert v.ref_area_m2 == pytest.approx(expected)

    def test_flight_health_defaults(self):
        h = FlightHealth()
        assert h.omega == pytest.approx(1.0)
        assert h.verdict == "NOMINAL"

    def test_telemetry_summary_dict_keys(self):
        state = make_state()
        health = FlightHealth()
        cmd = FlightCommand()
        frame = TelemetryFrame(
            t_s=0.0, phase=FlightPhase.HOLD,
            state=state, atm=None, propulsion=None, aero=None,
            health=health, command=cmd, stage_idx=0,
        )
        d = frame.summary_dict()
        for key in ("t_s", "phase", "alt_km", "speed_ms", "mach",
                    "q_kpa", "mass_kg", "omega", "verdict"):
            assert key in d

    def test_abort_mode_enum_values(self):
        assert AbortMode.NONE.value == "none"
        assert AbortMode.SAFE_DESTRUCT.value == "safe_destruct"

    def test_flight_phase_enum_has_abort(self):
        assert FlightPhase.ABORT in list(FlightPhase)


# ─────────────────────────────────────────────────────────────────────────────
# §2  대기 모델
# ─────────────────────────────────────────────────────────────────────────────

class TestAtmosphereEngine:
    def setup_method(self):
        self.atm = AtmosphereEngine()

    def test_sea_level_temperature(self):
        s = self.atm.query(0.0)
        assert s.temperature_k == pytest.approx(288.15, abs=0.01)

    def test_sea_level_pressure(self):
        s = self.atm.query(0.0)
        assert s.pressure_pa == pytest.approx(101_325.0, rel=1e-4)

    def test_sea_level_density(self):
        s = self.atm.query(0.0)
        assert s.density_kgm3 == pytest.approx(1.225, rel=1e-3)

    def test_density_decreases_with_altitude(self):
        rho0 = self.atm.query(0.0).density_kgm3
        rho10k = self.atm.query(10_000.0).density_kgm3
        assert rho10k < rho0

    def test_tropopause_temperature(self):
        # 11,000 m 에서 기온은 약 216.65 K
        s = self.atm.query(11_000.0)
        assert s.temperature_k == pytest.approx(216.65, abs=1.0)

    def test_speed_of_sound_positive(self):
        for h in [0, 5000, 20000, 50000]:
            s = self.atm.query(h)
            assert s.speed_of_sound_ms > 0

    def test_mach_computed(self):
        s = self.atm.query(0.0, speed_ms=340.0)
        assert s.mach == pytest.approx(340.0 / s.speed_of_sound_ms)

    def test_dynamic_q_at_rest(self):
        s = self.atm.query(0.0, speed_ms=0.0)
        assert s.dynamic_q_pa == pytest.approx(0.0)

    def test_dynamic_q_increases_with_speed(self):
        q100 = self.atm.query(0.0, speed_ms=100.0).dynamic_q_pa
        q200 = self.atm.query(0.0, speed_ms=200.0).dynamic_q_pa
        assert q200 == pytest.approx(4 * q100, rel=1e-3)  # q ∝ v²

    def test_gravity_decreases_with_altitude(self):
        g0 = self.atm.gravity(0.0)
        g100k = self.atm.gravity(100_000.0)
        assert g100k < g0

    def test_altitude_clamped_at_86km(self):
        s1 = self.atm.query(86_000.0)
        s2 = self.atm.query(100_000.0)
        assert s1.temperature_k == pytest.approx(s2.temperature_k, abs=0.1)

    def test_density_method(self):
        rho = self.atm.density(0.0)
        assert rho == pytest.approx(1.225, rel=1e-3)


# ─────────────────────────────────────────────────────────────────────────────
# §3  추진 모델
# ─────────────────────────────────────────────────────────────────────────────

class TestPropulsionEngine:
    def setup_method(self):
        self.stage = make_stage(prop_kg=1_000.0)
        self.atm   = AtmosphereEngine()
        self.eng   = PropulsionEngine(self.stage)

    def _atm_sl(self):
        return self.atm.query(0.0)

    def test_initial_not_ignited(self):
        assert not self.eng.is_ignited

    def test_ignite_succeeds(self):
        assert self.eng.ignite()

    def test_ignite_no_propellant_fails(self):
        s = make_stage(prop_kg=0.0)
        e = PropulsionEngine(s)
        assert not e.ignite()

    def test_thrust_positive_when_ignited(self):
        self.eng.ignite()
        atm = self._atm_sl()
        state = self.eng.tick(atm, throttle_cmd=1.0, dt_s=0.1)
        assert state.thrust_n > 0

    def test_propellant_decreases_after_tick(self):
        self.eng.ignite()
        atm = self._atm_sl()
        before = self.eng.propellant_remaining_kg
        self.eng.tick(atm, throttle_cmd=1.0, dt_s=1.0)
        after = self.eng.propellant_remaining_kg
        assert after < before

    def test_shutdown_stops_thrust(self):
        self.eng.ignite()
        self.eng.shutdown()
        atm = self._atm_sl()
        state = self.eng.tick(atm, throttle_cmd=1.0, dt_s=0.1)
        assert state.thrust_n == pytest.approx(0.0)

    def test_vacuum_thrust_higher_than_sl(self):
        """진공 추력 > 해면 추력."""
        self.eng.ignite()
        atm_vac = self.atm.query(200_000.0)  # 거의 진공
        atm_sl  = self._atm_sl()
        state_vac = self.eng.tick(atm_vac, 1.0, 0.1)
        thrust_vac = state_vac.thrust_n
        self.eng.reset_stage(make_stage(prop_kg=1_000.0))
        self.eng.ignite()
        state_sl = self.eng.tick(atm_sl, 1.0, 0.1)
        assert thrust_vac > state_sl.thrust_n

    def test_throttle_reduces_thrust(self):
        self.eng.ignite()
        atm = self._atm_sl()
        s1 = self.eng.tick(atm, 1.0, 0.1)
        self.eng.reset_stage(make_stage(prop_kg=1_000.0))
        self.eng.ignite()
        # 다시 점화 후 낮은 스로틀
        for _ in range(10):  # 지연 수렴 대기
            s2 = self.eng.tick(atm, 0.57, 0.1)
        assert s2.thrust_n < s1.thrust_n

    def test_propellant_exhaustion_shutdown(self):
        """추진제 고갈 → 자동 종료."""
        s = make_stage(prop_kg=10.0)
        e = PropulsionEngine(s)
        e.ignite()
        atm = self._atm_sl()
        for _ in range(1000):
            state = e.tick(atm, 1.0, 0.1)
        assert not e.is_ignited

    def test_propellant_fraction_starts_1(self):
        assert self.eng.propellant_fraction == pytest.approx(1.0)


# ─────────────────────────────────────────────────────────────────────────────
# §4  공력 모델
# ─────────────────────────────────────────────────────────────────────────────

class TestAerodynamicsEngine:
    def setup_method(self):
        cfg = VehicleAeroConfig(ref_area_m2=10.52)
        self.aero = AerodynamicsEngine(cfg)
        self.atm  = AtmosphereEngine()

    def test_zero_speed_zero_drag(self):
        atm = self.atm.query(0.0, speed_ms=0.0)
        s = self.aero.tick(atm, speed_ms=0.0)
        assert s.drag_n == pytest.approx(0.0)

    def test_drag_increases_with_speed(self):
        atm100 = self.atm.query(0.0, speed_ms=100.0)
        atm200 = self.atm.query(0.0, speed_ms=200.0)
        s100 = self.aero.tick(atm100, speed_ms=100.0)
        s200 = self.aero.tick(atm200, speed_ms=200.0)
        assert s200.drag_n > s100.drag_n

    def test_cd_subsonic(self):
        cd = self.aero.cd_at_mach(0.5)
        assert 0.1 < cd < 0.3

    def test_cd_transonic_peak(self):
        """천음속(M≈1.1)에서 Cd 최대."""
        cd_sub  = self.aero.cd_at_mach(0.5)
        cd_tran = self.aero.cd_at_mach(1.1)
        cd_sup  = self.aero.cd_at_mach(3.0)
        assert cd_tran > cd_sub
        assert cd_tran > cd_sup

    def test_cd_hypersonic_converges(self):
        cd8 = self.aero.cd_at_mach(8.0)
        cd10 = self.aero.cd_at_mach(10.0)
        assert abs(cd8 - cd10) < 0.05  # 점근

    def test_aero_state_ref_area(self):
        atm = self.atm.query(0.0, speed_ms=300.0)
        s = self.aero.tick(atm, speed_ms=300.0)
        assert s.ref_area_m2 == pytest.approx(10.52)

    def test_drag_decreases_with_altitude_same_speed(self):
        """같은 속도, 고도 높을수록 밀도 낮아 항력 감소."""
        v = 1000.0
        atm_sl = self.atm.query(0.0, speed_ms=v)
        atm_hi = self.atm.query(30_000.0, speed_ms=v)
        s_sl = self.aero.tick(atm_sl, v)
        s_hi = self.aero.tick(atm_hi, v)
        assert s_hi.drag_n < s_sl.drag_n


# ─────────────────────────────────────────────────────────────────────────────
# §5  가변 질량 RK4 적분기
# ─────────────────────────────────────────────────────────────────────────────

class TestVariableMassIntegrator:
    def setup_method(self):
        self.integ = VariableMassIntegrator(dry_mass_kg=5_000.0)

    def test_no_thrust_gravity_pulls_down(self):
        """추력 없으면 중력으로 수직 속도 감소."""
        s0 = make_state(z_m=1000.0, vz_ms=0.0, total_mass_kg=10_000.0)
        s1 = self.integ.step(s0, thrust_n=0.0, pitch_rad=math.pi/2,
                             yaw_rad=0.0, drag_n=0.0, mass_flow_kgs=0.0, dt_s=0.1)
        assert s1.vz_ms < 0.0  # 중력으로 하강

    def test_vertical_thrust_increases_altitude(self):
        """충분한 추력 → 고도 증가."""
        s0 = make_state(z_m=0.0, vz_ms=0.0, total_mass_kg=100_000.0)
        thrust = 2.0 * 100_000.0 * 9.80665  # 중력의 2배
        s1 = self.integ.step(s0, thrust_n=thrust, pitch_rad=math.pi/2,
                             yaw_rad=0.0, drag_n=0.0, mass_flow_kgs=0.0, dt_s=0.1)
        assert s1.z_m > 0.0
        assert s1.vz_ms > 0.0

    def test_mass_decreases_with_flow(self):
        s0 = make_state(total_mass_kg=100_000.0)
        s1 = self.integ.step(s0, thrust_n=0.0, pitch_rad=math.pi/2,
                             yaw_rad=0.0, drag_n=0.0, mass_flow_kgs=100.0, dt_s=1.0)
        assert s1.total_mass_kg < s0.total_mass_kg

    def test_mass_not_below_dry(self):
        """질량은 dry_mass 이하로 내려가지 않음."""
        s0 = make_state(total_mass_kg=5_001.0)  # dry_mass + 1 kg 추진제
        for _ in range(1000):
            s0 = self.integ.step(s0, thrust_n=0.0, pitch_rad=math.pi/2,
                                 yaw_rad=0.0, drag_n=0.0,
                                 mass_flow_kgs=100.0, dt_s=1.0)
        assert s0.total_mass_kg >= 5_000.0

    def test_altitude_not_negative(self):
        """지표면(z=0) 아래로 내려가지 않음."""
        s0 = make_state(z_m=0.0, vz_ms=-100.0, total_mass_kg=10_000.0)
        s1 = self.integ.step(s0, thrust_n=0.0, pitch_rad=math.pi/2,
                             yaw_rad=0.0, drag_n=0.0, mass_flow_kgs=0.0, dt_s=1.0)
        assert s1.z_m >= 0.0

    def test_horizontal_thrust(self):
        """수평 추력 → x 방향 이동."""
        s0 = make_state(z_m=1_000.0, total_mass_kg=100_000.0)
        thrust = 200_000.0
        s1 = self.integ.step(s0, thrust_n=thrust, pitch_rad=0.0,  # 수평
                             yaw_rad=0.0, drag_n=0.0, mass_flow_kgs=0.0, dt_s=1.0)
        assert s1.vx_ms > 0.0

    def test_drag_opposes_velocity(self):
        """항력은 속도 반대 방향 → 속도 감소."""
        s0 = make_state(z_m=1_000.0, vz_ms=100.0, total_mass_kg=100_000.0)
        s1 = self.integ.step(s0, thrust_n=0.0, pitch_rad=math.pi/2,
                             yaw_rad=0.0, drag_n=50_000.0, mass_flow_kgs=0.0, dt_s=0.1)
        # 항력 + 중력 → vz 감소
        assert s1.vz_ms < s0.vz_ms

    def test_tsiolkovsky_sanity(self):
        """Tsiolkovsky 근사 검증.

        Δv = Isp·g₀·ln(m0/m1)
        m0=100000, m1=50000, Isp=311 → Δv ≈ 2115 m/s
        """
        m0 = 100_000.0
        m1 = 50_000.0
        isp = 311.0
        g0 = 9.80665
        delta_v_expected = isp * g0 * math.log(m0 / m1)

        integ = VariableMassIntegrator(dry_mass_kg=10_000.0)
        mdot = 1_000.0
        thrust = mdot * isp * g0
        s = make_state(z_m=200_000.0, total_mass_kg=m0)  # 진공 고도
        v0 = s.vz_ms
        elapsed = (m0 - m1) / mdot  # 연소 시간

        dt = 0.1
        steps = int(elapsed / dt)
        for _ in range(steps):
            s = integ.step(s, thrust_n=thrust, pitch_rad=math.pi/2,
                           yaw_rad=0.0, drag_n=0.0, mass_flow_kgs=mdot, dt_s=dt)

        delta_v_simulated = s.vz_ms - v0
        # 중력 손실 있으므로 Tsiolkovsky 보다 작음, 단 50% 이내 오차
        assert delta_v_simulated > delta_v_expected * 0.5


# ─────────────────────────────────────────────────────────────────────────────
# §6  비행 단계 FSM
# ─────────────────────────────────────────────────────────────────────────────

class TestFlightPhaseFSM:
    def setup_method(self):
        cfg = FlightFSMConfig(
            countdown_duration_s=10.0,
            tower_clear_m=100.0,
            coast_before_sep_s=2.0,
            ignition_delay_s=1.0,
        )
        self.fsm = FlightPhaseFSM(cfg)

    def _ctx(self, **kw) -> FlightContext:
        defaults = dict(t_s=0.0, altitude_m=0.0, dynamic_q_pa=0.0,
                        propellant_kg=100_000.0, speed_ms=0.0)
        defaults.update(kw)
        return FlightContext(**defaults)

    def test_initial_hold(self):
        assert self.fsm.phase == FlightPhase.HOLD

    def test_hold_to_countdown_on_go(self):
        ctx = self._ctx(go_command=True)
        self.fsm.update(ctx)
        assert self.fsm.phase == FlightPhase.COUNTDOWN

    def test_countdown_to_liftoff(self):
        self.fsm.update(self._ctx(t_s=0.0, go_command=True))
        self.fsm.update(self._ctx(t_s=10.0))
        assert self.fsm.phase == FlightPhase.LIFTOFF

    def test_liftoff_to_ascending_at_tower_clear(self):
        self.fsm.update(self._ctx(t_s=0.0, go_command=True))
        self.fsm.update(self._ctx(t_s=10.0))
        self.fsm.update(self._ctx(t_s=11.0, altitude_m=100.0))
        assert self.fsm.phase == FlightPhase.ASCENDING

    def test_ascending_to_meco_on_propellant_exhaustion(self):
        self.fsm.force_phase(FlightPhase.ASCENDING)
        ctx = self._ctx(propellant_kg=0.0, meco_command=True)
        self.fsm.update(ctx)
        assert self.fsm.phase == FlightPhase.MECO

    def test_meco_to_stage_sep_after_timer(self):
        self.fsm.force_phase(FlightPhase.MECO)
        # _meco_t 초기화를 위해 먼저 ASCENDING → MECO 전환
        self.fsm._meco_t = 0.0
        self.fsm.update(self._ctx(t_s=2.5, propellant_kg=100.0))
        assert self.fsm.phase == FlightPhase.STAGE_SEP

    def test_stage_sep_to_upper_burn_after_delay(self):
        self.fsm.force_phase(FlightPhase.STAGE_SEP)
        self.fsm._sep_t = 0.0
        self.fsm.update(self._ctx(t_s=1.5))
        assert self.fsm.phase == FlightPhase.UPPER_BURN

    def test_abort_overrides_any_phase(self):
        for phase in (FlightPhase.ASCENDING, FlightPhase.UPPER_BURN, FlightPhase.COAST):
            self.fsm.force_phase(phase)
            self.fsm.update(self._ctx(abort_trigger=True))
            assert self.fsm.phase == FlightPhase.ABORT
            self.fsm.force_phase(FlightPhase.HOLD)  # 초기화

    def test_nominal_not_aborted(self):
        """NOMINAL 단계는 abort 처리 안 됨."""
        self.fsm.force_phase(FlightPhase.NOMINAL)
        self.fsm.update(self._ctx(abort_trigger=True))
        assert self.fsm.phase == FlightPhase.NOMINAL

    def test_force_phase(self):
        self.fsm.force_phase(FlightPhase.COAST)
        assert self.fsm.phase == FlightPhase.COAST


# ─────────────────────────────────────────────────────────────────────────────
# §7  단분리 관리자
# ─────────────────────────────────────────────────────────────────────────────

class TestStagingManager:
    def setup_method(self):
        self.vehicle = make_vehicle()
        self.mgr = StagingManager(self.vehicle)

    def test_initial_stage_idx_zero(self):
        assert self.mgr.stage_idx == 0

    def test_active_stage_is_first(self):
        assert self.mgr.active_stage.stage_id == 1

    def test_has_next_stage(self):
        assert self.mgr.has_next_stage

    def test_separate_advances_stage(self):
        self.mgr.separate(t_s=160.0)
        assert self.mgr.stage_idx == 1

    def test_separate_reduces_total_mass(self):
        before = self.mgr.total_mass_kg
        event = self.mgr.separate(t_s=160.0)
        after = self.mgr.total_mass_kg
        assert after < before
        assert event.mass_ejected_kg == pytest.approx(25_000.0)  # 1단 dry mass

    def test_separate_records_event(self):
        self.mgr.separate(t_s=100.0)
        assert len(self.mgr.events) == 1

    def test_no_more_stages_returns_none(self):
        self.mgr.separate(160.0)
        result = self.mgr.separate(400.0)
        assert result is None

    def test_fairing_ejection(self):
        before = self.mgr.total_mass_kg
        m = self.mgr.eject_fairing(120.0)
        assert m == pytest.approx(1_900.0)
        assert self.mgr.total_mass_kg == pytest.approx(before - 1_900.0)

    def test_fairing_ejection_only_once(self):
        self.mgr.eject_fairing(120.0)
        m2 = self.mgr.eject_fairing(130.0)
        assert m2 == 0.0


# ─────────────────────────────────────────────────────────────────────────────
# §8  Gravity Turn 유도
# ─────────────────────────────────────────────────────────────────────────────

class TestGravityTurnGuidance:
    def setup_method(self):
        cfg = GravityTurnConfig(
            kick_start_m=500.0,
            kick_end_m=5_000.0,
            pitch_kick_target_rad=math.radians(75.0),
        )
        self.g = GravityTurnGuidance(cfg)

    def test_vertical_below_kick_start(self):
        state = make_state(z_m=100.0)
        cmd = self.g.tick(state, FlightPhase.LIFTOFF)
        assert cmd.pitch_cmd_rad == pytest.approx(math.pi / 2.0)

    def test_pitch_kicks_between_kick_altitudes(self):
        state = make_state(z_m=2_500.0)  # 중간 고도
        cmd = self.g.tick(state, FlightPhase.ASCENDING)
        # π/2 와 target 사이여야 함
        assert math.radians(74) < cmd.pitch_cmd_rad < math.pi / 2.0

    def test_gravity_turn_above_kick_end(self):
        """kick_end 이상에서는 속도벡터 방향 추종."""
        state = make_state(z_m=10_000.0, vx_ms=1_000.0, vz_ms=500.0)
        cmd = self.g.tick(state, FlightPhase.ASCENDING)
        expected = math.atan2(500.0, 1_000.0)
        assert cmd.pitch_cmd_rad == pytest.approx(expected, abs=0.05)

    def test_zero_throttle_in_hold(self):
        state = make_state()
        cmd = self.g.tick(state, FlightPhase.HOLD)
        assert cmd.throttle == pytest.approx(0.0)

    def test_reduced_throttle_at_maxq(self):
        state = make_state(z_m=15_000.0)
        cmd_normal = self.g.tick(state, FlightPhase.ASCENDING, q_pa=10_000.0)
        cmd_maxq   = self.g.tick(state, FlightPhase.MAX_Q,    q_pa=40_000.0)
        assert cmd_maxq.throttle < cmd_normal.throttle

    def test_nominal_throttle_one_in_ascending(self):
        state = make_state(z_m=5_000.0)
        cmd = self.g.tick(state, FlightPhase.ASCENDING)
        assert cmd.throttle == pytest.approx(1.0)


# ─────────────────────────────────────────────────────────────────────────────
# §9  TVC 제어기
# ─────────────────────────────────────────────────────────────────────────────

class TestTVCController:
    def setup_method(self):
        cfg = TVCConfig(kp=2.0, kd=0.5, max_gimbal_rad=math.radians(8.0))
        self.tvc = TVCController(cfg)

    def test_no_error_no_gimbal(self):
        cmd = FlightCommand(pitch_cmd_rad=math.pi/2, throttle=1.0)
        result = self.tvc.tick(cmd, pitch_actual=math.pi/2, dt_s=0.1)
        assert abs(result.gimbal_y_rad) < 1e-3

    def test_positive_error_positive_gimbal(self):
        cmd = FlightCommand(pitch_cmd_rad=math.pi/2 + 0.1, throttle=1.0)
        result = self.tvc.tick(cmd, pitch_actual=math.pi/2, dt_s=0.1)
        assert result.gimbal_y_rad > 0.0

    def test_gimbal_clamped_at_limit(self):
        cmd = FlightCommand(pitch_cmd_rad=math.pi/2 + 1.0, throttle=1.0)
        result = self.tvc.tick(cmd, pitch_actual=0.0, dt_s=0.1)
        assert abs(result.gimbal_y_rad) <= math.radians(8.0) + 1e-9

    def test_throttle_preserved(self):
        cmd = FlightCommand(pitch_cmd_rad=math.pi/2, throttle=0.72)
        result = self.tvc.tick(cmd, pitch_actual=math.pi/2, dt_s=0.1)
        assert result.throttle == pytest.approx(0.72)


# ─────────────────────────────────────────────────────────────────────────────
# §10 Range Safety
# ─────────────────────────────────────────────────────────────────────────────

class TestRangeSafety:
    def setup_method(self):
        cfg = RangeSafetyConfig(
            corridor_half_width_m=50_000.0,
            corridor_direction_rad=0.0,
            max_downrange_m=500_000.0,
            iip_exclusion_radius_m=200_000.0,
        )
        self.rss = RangeSafetySystem(cfg)

    def test_on_pad_iip_near_origin(self):
        state = make_state(z_m=10.0, vz_ms=1.0)
        report = self.rss.check(state)
        assert report.iip_downrange_m < 200_000.0  # IIP ≈ 발사장 근처

    def test_in_corridor_nominal(self):
        """d=0(동쪽 along), y_m=0 → crossrange=|iip_y|≈0 → 복도 내."""
        cfg = RangeSafetyConfig(
            corridor_half_width_m=500_000.0,  # 500 km 복도
            corridor_direction_rad=0.0,        # along = x(동쪽), cross = |y|
            max_downrange_m=2_000_000.0,       # 2000 km 이내
            iip_exclusion_radius_m=1_000.0,
        )
        rss = RangeSafetySystem(cfg)
        # 동쪽 비행, 남북 이탈 없음 → crossrange ≈ 0
        state = make_state(z_m=50_000.0, vx_ms=3_000.0, vz_ms=2_000.0,
                           x_m=50_000.0, y_m=0.0)
        report = rss.check(state)
        assert report.corridor_ok

    def test_out_of_corridor_triggers_destruct(self):
        """대폭 이탈 시 자폭 권고."""
        state = make_state(z_m=50_000.0, vx_ms=5_000.0, vz_ms=100.0,
                           x_m=200_000.0)  # 동쪽으로 크게 이탈
        report = self.rss.check(state)
        assert report.destruct_required

    def test_iip_computation_stationary(self):
        """수직 속도만 있으면 IIP ≈ 현재 수평 위치."""
        state = make_state(z_m=10_000.0, vz_ms=0.0, vx_ms=0.0, vy_ms=0.0)
        report = self.rss.check(state)
        assert abs(report.iip_x_m) < 10.0
        assert abs(report.iip_y_m) < 10.0


# ─────────────────────────────────────────────────────────────────────────────
# §11 중단 시스템
# ─────────────────────────────────────────────────────────────────────────────

class TestAbortSystem:
    def setup_method(self):
        cfg = AbortSystemConfig(
            q_abort_pa=80_000.0,
            pitch_error_abort_rad=math.radians(20.0),
            omega_abort_threshold=0.25,
        )
        self.sys = AbortSystem(cfg)

    def _health(self, omega=1.0):
        h = FlightHealth()
        object.__setattr__(h, 'omega', omega) if False else None
        return FlightHealth(omega=omega, verdict="NOMINAL" if omega >= 0.8 else "ABORT")

    def test_nominal_no_abort(self):
        mode = self.sys.evaluate(
            health=FlightHealth(omega=0.9, verdict="NOMINAL"),
            state=make_state(),
            phase=FlightPhase.ASCENDING,
            q_pa=10_000.0,
        )
        assert mode == AbortMode.NONE

    def test_q_limit_triggers_abort(self):
        mode = self.sys.evaluate(
            health=FlightHealth(omega=0.9, verdict="NOMINAL"),
            state=make_state(),
            phase=FlightPhase.ASCENDING,
            q_pa=90_000.0,  # 한계 초과
        )
        assert mode != AbortMode.NONE

    def test_low_omega_triggers_abort(self):
        mode = self.sys.evaluate(
            health=FlightHealth(omega=0.10, verdict="ABORT"),
            state=make_state(),
            phase=FlightPhase.ASCENDING,
            q_pa=1_000.0,
        )
        assert mode != AbortMode.NONE

    def test_range_safety_selects_destruct(self):
        mode = self.sys.evaluate(
            health=FlightHealth(omega=0.9, verdict="NOMINAL"),
            state=make_state(),
            phase=FlightPhase.ASCENDING,
            q_pa=1_000.0,
            range_destruct=True,
        )
        assert mode == AbortMode.SAFE_DESTRUCT

    def test_external_abort_selects_destruct(self):
        mode = self.sys.evaluate(
            health=FlightHealth(omega=0.9, verdict="NOMINAL"),
            state=make_state(),
            phase=FlightPhase.ASCENDING,
            q_pa=1_000.0,
            external_abort=True,
        )
        assert mode == AbortMode.SAFE_DESTRUCT

    def test_hold_phase_no_abort(self):
        """HOLD 단계는 중단 평가 제외."""
        mode = self.sys.evaluate(
            health=FlightHealth(omega=0.0, verdict="ABORT"),
            state=make_state(),
            phase=FlightPhase.HOLD,
            q_pa=999_999.0,
        )
        assert mode == AbortMode.NONE

    def test_event_recorded_on_abort(self):
        self.sys.evaluate(
            health=FlightHealth(omega=0.0, verdict="ABORT"),
            state=make_state(),
            phase=FlightPhase.ASCENDING,
            q_pa=100_000.0,
        )
        assert len(self.sys.events) >= 1


# ─────────────────────────────────────────────────────────────────────────────
# §12 비행 기록 체인
# ─────────────────────────────────────────────────────────────────────────────

class TestFlightChain:
    def setup_method(self):
        self.chain = FlightChain("TEST-LV", record_interval=1)

    def _frame(self, t_s: float) -> TelemetryFrame:
        return TelemetryFrame(
            t_s=t_s,
            phase=FlightPhase.ASCENDING,
            state=make_state(t_s=t_s),
            atm=None, propulsion=None, aero=None,
            health=FlightHealth(),
            command=FlightCommand(),
            stage_idx=0,
        )

    def test_empty_chain_integrity(self):
        assert self.chain.verify_integrity()

    def test_record_adds_block(self):
        self.chain.record(self._frame(1.0))
        assert self.chain.length == 1

    def test_multiple_records(self):
        for i in range(5):
            self.chain.record(self._frame(float(i)))
        assert self.chain.length == 5

    def test_integrity_after_records(self):
        for i in range(10):
            self.chain.record(self._frame(float(i)))
        assert self.chain.verify_integrity()

    def test_head_hash_changes_after_record(self):
        h0 = self.chain.head_hash
        self.chain.record(self._frame(0.0))
        assert self.chain.head_hash != h0

    def test_record_event_immediate(self):
        chain = FlightChain("TEST", record_interval=100)  # 긴 간격
        chain.record_event(0.0, "stage_sep", {"stage": 1})
        assert chain.length == 1  # 간격 무관 즉시 기록

    def test_interval_skips_ticks(self):
        chain = FlightChain("TEST", record_interval=5)
        for i in range(4):
            chain.record(self._frame(float(i)))
        assert chain.length == 0  # 5번째 틱 전까지 기록 안 됨

    def test_summary_contains_vehicle_id(self):
        s = self.chain.summary()
        assert "TEST-LV" in s

    def test_export_json_creates_file(self, tmp_path):
        self.chain.record(self._frame(0.0))
        out = str(tmp_path / "test_chain.json")
        self.chain.export_json(out)
        import os
        assert os.path.exists(out)


# ─────────────────────────────────────────────────────────────────────────────
# §13 LaunchAgent 통합
# ─────────────────────────────────────────────────────────────────────────────

class TestLaunchAgent:
    def setup_method(self):
        self.vehicle = make_vehicle()
        self.agent = LaunchAgent(
            self.vehicle,
            dt_s=0.1,
            chain_interval=5,
        )

    def test_initial_phase_hold(self):
        assert self.agent.phase == FlightPhase.HOLD

    def test_initial_state_on_ground(self):
        s = self.agent.state
        assert s.altitude_m == pytest.approx(0.0)
        assert s.speed_ms == pytest.approx(0.0)

    def test_chain_records_init_event(self):
        assert self.agent.chain.length >= 1

    def test_tick_advances_time(self):
        self.agent.command_go()
        frame = self.agent.tick()
        assert frame.t_s > 0.0

    def test_go_command_triggers_countdown(self):
        self.agent.command_go()
        self.agent.tick()
        assert self.agent.phase == FlightPhase.COUNTDOWN

    def test_propulsion_ignites_at_liftoff(self):
        """카운트다운 → 발사 후 추진력 발생."""
        self.agent.command_go()
        for _ in range(110):  # 카운트다운(10s÷0.1dt=100)+여유
            self.agent.tick()
        # LIFTOFF 또는 그 이후 단계여야 함
        assert self.agent.phase in (
            FlightPhase.LIFTOFF, FlightPhase.ASCENDING,
            FlightPhase.MAX_Q, FlightPhase.MECO,
        )

    def test_altitude_increases_after_liftoff(self):
        self.agent.command_go()
        for _ in range(200):  # 발사 후 일정 시간
            self.agent.tick()
        assert self.agent.state.altitude_m > 0.0

    def test_frame_health_has_omega(self):
        self.agent.command_go()
        frame = self.agent.tick()
        assert 0.0 <= frame.health.omega <= 1.0

    def test_abort_command_sets_abort_phase(self):
        self.agent.command_go()
        self.agent.tick()
        self.agent.command_abort()
        assert self.agent.phase == FlightPhase.ABORT

    def test_chain_integrity_after_ticks(self):
        self.agent.command_go()
        for _ in range(50):
            self.agent.tick()
        assert self.agent.chain.verify_integrity()

    def test_summary_output(self):
        s = self.agent.summary()
        assert self.vehicle.vehicle_id in s
        assert "단계" in s

    def test_telemetry_frame_has_all_fields(self):
        self.agent.command_go()
        frame = self.agent.tick()
        assert frame.phase is not None
        assert frame.state is not None
        assert frame.health is not None
        assert frame.command is not None

    def test_short_simulation_no_crash(self):
        """100틱 시뮬레이션 오류 없이 완주."""
        self.agent.command_go()
        for _ in range(100):
            if self.agent.phase in (FlightPhase.NOMINAL, FlightPhase.ABORT):
                break
            self.agent.tick()
        assert True  # 예외 없이 완주

    def test_second_stage_index_after_separation(self):
        """StagingManager.separate() 직접 호출 → stage_idx = 1."""
        event = self.agent._staging.separate(t_s=160.0)
        assert event is not None
        assert self.agent._staging.stage_idx == 1
