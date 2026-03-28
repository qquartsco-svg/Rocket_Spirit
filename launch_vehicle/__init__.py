"""Rocket_Spirit / LaunchVehicle_Stack public package API.

브랜딩 이름은 ``Rocket_Spirit`` 이지만 실제 배포 패키지 이름은
``launch_vehicle`` 이다.

에코시스템 연결:
  TAM (StarScream)  — 지상→호버 readiness → 발사 인가
  Air Jordan        — 저고도 공력 보조 판정 (고도 < 20 km)
  AgedCare_Stack    — 유인 발사(MVP-4) 탑승자 안전 게이트
  VPF               — 발사대 지상 물리 (LaunchAgent 내 독립 사용)
"""

from .launch_agent import LaunchAgent
from .contracts.schemas import (
    AbortMode,
    AeroState,
    AtmosphereState,
    DEFAULT_PHYSICS,
    FlightCommand,
    FlightHealth,
    FlightPhase,
    PhysicsConfig,
    PropulsionState,
    RocketState,
    StageConfig,
    TelemetryFrame,
    VehicleConfig,
)
from .audit.flight_chain import FlightBlock, FlightChain
from .bridges.tam_bridge import TamLaunchReadiness, optional_tam_launch_readiness
from .adapters.aged_care_adapter import (
    AgedCareLaunchSafety,
    evaluate_crew_launch_gate,
    snapshot_from_safety_state,
)

__version__ = "0.1.1"

__all__ = [
    "__version__",
    # 오케스트레이터
    "LaunchAgent",
    # 계약
    "AbortMode",
    "AeroState",
    "AtmosphereState",
    "DEFAULT_PHYSICS",
    "FlightBlock",
    "FlightChain",
    "FlightCommand",
    "FlightHealth",
    "FlightPhase",
    "PhysicsConfig",
    "PropulsionState",
    "RocketState",
    "StageConfig",
    "TelemetryFrame",
    "VehicleConfig",
    # 에코시스템 브리지
    "TamLaunchReadiness",
    "optional_tam_launch_readiness",
    # 어댑터
    "AgedCareLaunchSafety",
    "evaluate_crew_launch_gate",
    "snapshot_from_safety_state",
]
