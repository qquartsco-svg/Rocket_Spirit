from __future__ import annotations

from launch_vehicle import (
    LaunchAgent,
    FlightPhase,
    StageConfig,
    VehicleConfig,
    __version__,
)


def test_root_exports_core_types():
    assert LaunchAgent is not None
    assert FlightPhase.HOLD.value == "hold"
    assert __version__ == "0.1.1"


def test_root_exports_support_simple_construction():
    stage = StageConfig(
        stage_id=1,
        dry_mass_kg=1000.0,
        propellant_mass_kg=5000.0,
        engine_count=1,
        isp_sl_s=250.0,
        isp_vac_s=300.0,
        thrust_sl_n=100_000.0,
        thrust_vac_n=110_000.0,
        nozzle_exit_area_m2=1.0,
        max_throttle=1.0,
        min_throttle=0.5,
        burn_time_design_s=60.0,
    )
    vehicle = VehicleConfig(vehicle_id="API-001", stages=[stage])
    agent = LaunchAgent(vehicle)
    assert agent.phase == FlightPhase.HOLD
