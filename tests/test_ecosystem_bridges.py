"""에코시스템 브리지 및 어댑터 테스트.

§1  TAM 브리지 (tam_bridge)
§2  Air Jordan 브리지 (air_jordan_bridge)
§3  AgedCare 어댑터 (aged_care_adapter)
§4  LaunchAgent 에코시스템 통합
§5  command_go_from_tam 시나리오
"""
from __future__ import annotations

import pytest
from launch_vehicle import (
    LaunchAgent, FlightPhase, StageConfig, VehicleConfig,
    TamLaunchReadiness, optional_tam_launch_readiness,
    AgedCareLaunchSafety, evaluate_crew_launch_gate,
)
from launch_vehicle.bridges.tam_bridge import readiness_from_tam_report
from launch_vehicle.bridges.air_jordan_bridge import optional_air_jordan_aero
from launch_vehicle.adapters.aged_care_adapter import (
    launch_safety_from_safety_state,
    launch_safety_from_omega_report,
)


# ── 픽스처 ────────────────────────────────────────────────────────────────────

@pytest.fixture
def tiny_vehicle() -> VehicleConfig:
    stage = StageConfig(
        stage_id=1, dry_mass_kg=500.0, propellant_mass_kg=3000.0,
        engine_count=1, isp_sl_s=280.0, isp_vac_s=310.0,
        thrust_sl_n=80_000.0, thrust_vac_n=90_000.0,
        nozzle_exit_area_m2=0.5, max_throttle=1.0, min_throttle=0.5,
        burn_time_design_s=60.0,
    )
    return VehicleConfig(vehicle_id="RS-TEST-001", stages=[stage])


class _FakeTamReport:
    """duck-typed TAM FlightReadinessReport 모의."""
    def __init__(self, omega=0.92, verdict="HEALTHY", takeoff=True, blockers=()):
        self.omega_total = omega
        self.verdict = verdict
        self.takeoff_possible = takeoff
        self.blockers = blockers
        self.omega_propulsion = 0.85
        self.omega_morph = 0.90
        self.omega_power = 0.80
        self.omega_cog = 0.94


# ── §1 TAM 브리지 ─────────────────────────────────────────────────────────────

class TestTamBridge:
    def test_hover_high_omega_authorized(self):
        report = _FakeTamReport(omega=0.90, verdict="HEALTHY", takeoff=True)
        r = readiness_from_tam_report(report, "HOVER")
        assert r.launch_authorized is True
        assert r.tam_mode == "HOVER"
        assert r.tam_omega_total == pytest.approx(0.90)

    def test_ground_drive_not_authorized(self):
        report = _FakeTamReport(omega=0.90, verdict="HEALTHY", takeoff=True)
        r = readiness_from_tam_report(report, "GROUND_DRIVE")
        assert r.launch_authorized is False

    def test_low_omega_not_authorized(self):
        report = _FakeTamReport(omega=0.60, verdict="STABLE", takeoff=True)
        r = readiness_from_tam_report(report, "HOVER")
        assert r.launch_authorized is False

    def test_blockers_prevent_authorization(self):
        report = _FakeTamReport(omega=0.92, verdict="HEALTHY", takeoff=True,
                                blockers=("thrust_to_weight_low",))
        r = readiness_from_tam_report(report, "HOVER")
        assert r.launch_authorized is False
        assert len(r.blockers) > 0

    def test_takeoff_false_prevents_authorization(self):
        report = _FakeTamReport(omega=0.92, verdict="HEALTHY", takeoff=False)
        r = readiness_from_tam_report(report, "HOVER")
        assert r.launch_authorized is False

    def test_optional_returns_none_for_none_report(self):
        result = optional_tam_launch_readiness(None, "HOVER")
        assert result is None

    def test_flight_cruise_also_authorized(self):
        report = _FakeTamReport(omega=0.88, verdict="HEALTHY", takeoff=True)
        r = readiness_from_tam_report(report, "FLIGHT_CRUISE")
        assert r.launch_authorized is True

    def test_omega_components_extracted(self):
        report = _FakeTamReport()
        r = readiness_from_tam_report(report, "HOVER")
        assert r.tam_omega_propulsion == pytest.approx(0.85)
        assert r.tam_omega_morph == pytest.approx(0.90)
        assert r.tam_omega_cog == pytest.approx(0.94)


# ── §2 Air Jordan 브리지 ──────────────────────────────────────────────────────

class TestAirJordanBridge:
    def test_high_altitude_returns_none(self):
        # > 20 km 는 공력 모델 스킵
        result = optional_air_jordan_aero(
            altitude_m=25_000.0, speed_ms=800.0, mach=2.5, mass_kg=80_000.0
        )
        assert result is None

    def test_low_altitude_returns_none_or_tuple_when_not_installed(self):
        # Air Jordan 미설치 시 None, 설치 시 tuple
        result = optional_air_jordan_aero(
            altitude_m=5_000.0, speed_ms=200.0, mach=0.6, mass_kg=80_000.0
        )
        assert result is None or (isinstance(result, tuple) and len(result) == 3)

    def test_zero_altitude_liftoff(self):
        result = optional_air_jordan_aero(
            altitude_m=0.0, speed_ms=10.0, mach=0.03, mass_kg=120_000.0
        )
        assert result is None or isinstance(result, tuple)


# ── §3 AgedCare 어댑터 ───────────────────────────────────────────────────────

class TestAgedCareAdapter:
    def test_safe_crew_snapshot(self):
        class FakeSafety:
            omega = 0.95
            verdict = "SAFE"
            emergency_triggered = False
            manual_override = False

        snap = launch_safety_from_safety_state(FakeSafety())
        assert snap.omega_care == pytest.approx(0.95)
        assert snap.verdict == "SAFE"
        assert snap.emergency_triggered is False

    def test_emergency_crew_snapshot(self):
        class FakeEmergency:
            omega = 0.20
            verdict = "EMERGENCY"
            emergency_triggered = True
            manual_override = False

        snap = launch_safety_from_safety_state(FakeEmergency())
        assert snap.emergency_triggered is True

    def test_omega_report_conversion(self):
        class FakeOmega:
            omega_total = 0.72
            verdict = "STABLE"

        snap = launch_safety_from_omega_report(FakeOmega())
        assert snap.omega_care == pytest.approx(0.72)
        assert snap.emergency_triggered is False

    def test_crew_gate_unmanned_always_ok(self):
        ok, blockers = evaluate_crew_launch_gate(None, human_rated_mvp4=False)
        assert ok is True
        assert blockers == []

    def test_crew_gate_manned_missing_snapshot_blocked(self):
        ok, blockers = evaluate_crew_launch_gate(None, human_rated_mvp4=True)
        assert ok is False
        assert "crew_missing_care_snapshot" in blockers

    def test_crew_gate_manned_safe_passes(self):
        snap = AgedCareLaunchSafety(
            omega_care=0.90, verdict="SAFE",
            emergency_triggered=False, manual_override=False
        )
        ok, blockers = evaluate_crew_launch_gate(snap, human_rated_mvp4=True)
        assert ok is True
        assert blockers == []

    def test_crew_gate_emergency_blocked(self):
        snap = AgedCareLaunchSafety(
            omega_care=0.85, verdict="SAFE", emergency_triggered=True
        )
        ok, blockers = evaluate_crew_launch_gate(snap, human_rated_mvp4=True)
        assert ok is False
        assert "crew_care_emergency" in blockers

    def test_crew_gate_caution_verdict_blocked(self):
        snap = AgedCareLaunchSafety(
            omega_care=0.85, verdict="CAUTION", emergency_triggered=False
        )
        ok, blockers = evaluate_crew_launch_gate(snap, human_rated_mvp4=True)
        assert ok is False

    def test_crew_gate_low_omega_care_blocked(self):
        snap = AgedCareLaunchSafety(
            omega_care=0.60, verdict="SAFE", emergency_triggered=False
        )
        ok, blockers = evaluate_crew_launch_gate(snap, human_rated_mvp4=True)
        assert ok is False


# ── §4 LaunchAgent 에코시스템 통합 ───────────────────────────────────────────

class TestLaunchAgentEcosystem:
    def test_agent_created_without_bridges(self, tiny_vehicle):
        agent = LaunchAgent(tiny_vehicle)
        assert agent.phase == FlightPhase.HOLD
        assert agent.tam_readiness is None

    def test_agent_created_with_tam_readiness(self, tiny_vehicle):
        tam_r = TamLaunchReadiness(
            tam_omega_total=0.92,
            tam_mode="HOVER",
            tam_takeoff_possible=True,
            tam_verdict="HEALTHY",
            launch_authorized=True,
        )
        agent = LaunchAgent(tiny_vehicle, tam_readiness=tam_r)
        assert agent.tam_readiness is not None
        assert agent.tam_readiness.launch_authorized is True

    def test_agent_summary_includes_tam(self, tiny_vehicle):
        tam_r = TamLaunchReadiness(
            tam_omega_total=0.88, tam_mode="HOVER",
            tam_takeoff_possible=True, tam_verdict="HEALTHY",
            launch_authorized=True,
        )
        agent = LaunchAgent(tiny_vehicle, tam_readiness=tam_r)
        s = agent.summary()
        assert "TAM" in s
        assert "HOVER" in s

    def test_agent_tick_runs_without_bridges(self, tiny_vehicle):
        agent = LaunchAgent(tiny_vehicle)
        agent.command_go()
        frame = agent.tick()
        assert frame is not None


# ── §5 command_go_from_tam 시나리오 ──────────────────────────────────────────

class TestCommandGoFromTam:
    def test_authorized_tam_allows_go(self, tiny_vehicle):
        agent = LaunchAgent(tiny_vehicle)
        report = _FakeTamReport(omega=0.92, verdict="HEALTHY", takeoff=True)
        authorized = agent.command_go_from_tam(report, "HOVER")
        assert authorized is True

    def test_ground_drive_tam_denies_go(self, tiny_vehicle):
        agent = LaunchAgent(tiny_vehicle)
        report = _FakeTamReport(omega=0.92, verdict="HEALTHY", takeoff=True)
        authorized = agent.command_go_from_tam(report, "GROUND_DRIVE")
        assert authorized is False

    def test_low_omega_tam_denies_go(self, tiny_vehicle):
        agent = LaunchAgent(tiny_vehicle)
        report = _FakeTamReport(omega=0.55, verdict="STABLE", takeoff=False)
        authorized = agent.command_go_from_tam(report, "HOVER")
        assert authorized is False

    def test_none_report_denies_go(self, tiny_vehicle):
        agent = LaunchAgent(tiny_vehicle)
        authorized = agent.command_go_from_tam(None, "HOVER")
        assert authorized is False

    def test_manned_mvp4_without_crew_safety_denied(self, tiny_vehicle):
        agent = LaunchAgent(tiny_vehicle, human_rated_mvp4=True)
        report = _FakeTamReport(omega=0.92, verdict="HEALTHY", takeoff=True)
        authorized = agent.command_go_from_tam(report, "HOVER")
        assert authorized is False

    def test_manned_mvp4_with_safe_crew_authorized(self, tiny_vehicle):
        crew = AgedCareLaunchSafety(
            omega_care=0.90, verdict="SAFE",
            emergency_triggered=False, manual_override=False
        )
        agent = LaunchAgent(tiny_vehicle, human_rated_mvp4=True, crew_safety=crew)
        report = _FakeTamReport(omega=0.92, verdict="HEALTHY", takeoff=True)
        authorized = agent.command_go_from_tam(report, "HOVER")
        assert authorized is True

    def test_go_from_tam_recorded_in_chain(self, tiny_vehicle):
        agent = LaunchAgent(tiny_vehicle)
        report = _FakeTamReport(omega=0.92, verdict="HEALTHY", takeoff=True)
        agent.command_go_from_tam(report, "HOVER")
        # go_from_tam 이벤트 블록 확인 (phase_events 인터페이스 사용)
        go_blocks = agent.chain.phase_events("go_from_tam")
        assert len(go_blocks) >= 1
