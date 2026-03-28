"""TAM (StarScream) → LaunchVehicle 발사 인가 브리지.

시나리오:
    휠체어 플랫폼이 TAM 오케스트레이션을 통해 HOVER 상태에 도달하면,
    LaunchVehicle 은 TAM readiness 를 발사 사전 인가(Pre-Launch Auth) 신호로
    수신한다. 두 스택은 역할이 엄격히 분리된다:

        TAM (StarScream)  ─ 지상 이동 → 변형 → 호버 판정 (Go/No-Go)
        LaunchVehicle     ─ 호버 이후 → 상승 → 궤도 삽입

    TAM 이 없어도 LaunchAgent 는 독립 동작 가능 (ImportError 폴백).

사용법::
    from launch_vehicle.bridges.tam_bridge import optional_tam_launch_readiness
    from launch_vehicle.launch_agent import LaunchAgent

    agent = LaunchAgent(vehicle)
    readiness = optional_tam_launch_readiness(tam_report, tam_mode="HOVER")
    if readiness and readiness.launch_authorized:
        agent.command_go_from_tam(readiness)
"""
from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional


# ── 발사 인가 스냅샷 ─────────────────────────────────────────────────────────

@dataclass(frozen=True)
class TamLaunchReadiness:
    """TAM FlightReadinessReport 에서 추출한 발사 사전 인가 스냅샷."""
    tam_omega_total: float = 0.0
    tam_mode: str = "GROUND_DRIVE"
    tam_takeoff_possible: bool = False
    tam_verdict: str = "CRITICAL"
    # 개별 Ω 컴포넌트 (있으면 LaunchAgent 건전성에 반영)
    tam_omega_propulsion: float = 0.0
    tam_omega_morph: float = 0.0
    tam_omega_power: float = 0.0
    tam_omega_cog: float = 0.0
    # 발사 인가 최종 판정
    launch_authorized: bool = False
    # 차단 코드 (TAM blockers 그대로 전달)
    blockers: tuple = ()


# ── 최소 발사 인가 기준 ───────────────────────────────────────────────────────
_LAUNCH_AUTH_MODES = frozenset({"HOVER", "FLIGHT_CRUISE"})
_MIN_TAM_OMEGA = 0.80
_REQUIRED_VERDICTS = frozenset({"HEALTHY", "STABLE"})


def readiness_from_tam_report(report: Any, mode: str) -> TamLaunchReadiness:
    """duck-typed TAM FlightReadinessReport → TamLaunchReadiness 변환.

    Args:
        report: TAM FlightReadinessReport (또는 동일 인터페이스 덕타입).
        mode:   현재 TAM AerialPlatformMode 문자열.

    Returns:
        TamLaunchReadiness (launch_authorized=True 면 발사 진행 가능).
    """
    omega  = float(getattr(report, "omega_total", 0.0))
    verdict = str(getattr(report, "verdict", "CRITICAL"))
    takeoff = bool(getattr(report, "takeoff_possible", False))
    blockers = getattr(report, "blockers", ())

    authorized = (
        mode in _LAUNCH_AUTH_MODES
        and omega >= _MIN_TAM_OMEGA
        and takeoff
        and verdict in _REQUIRED_VERDICTS
        and len(blockers) == 0
    )

    return TamLaunchReadiness(
        tam_omega_total=omega,
        tam_mode=mode,
        tam_takeoff_possible=takeoff,
        tam_verdict=verdict,
        tam_omega_propulsion=float(getattr(report, "omega_propulsion", 0.0)),
        tam_omega_morph=float(getattr(report, "omega_morph", 0.0)),
        tam_omega_power=float(getattr(report, "omega_power", 0.0)),
        tam_omega_cog=float(getattr(report, "omega_cog", 0.0)),
        launch_authorized=authorized,
        blockers=tuple(blockers),
    )


def optional_tam_launch_readiness(
    report: Any,
    mode: str,
) -> Optional[TamLaunchReadiness]:
    """TAM report 가 있으면 변환, None 이면 None 반환.

    TAM 스택이 설치되지 않거나 report 가 None 이면 None 을 돌려준다.
    LaunchAgent 는 None 을 독립 발사 모드로 해석한다.
    """
    if report is None:
        return None
    return readiness_from_tam_report(report, mode)


def _ensure_tam_on_path() -> None:
    """Transformable_Air_Mobility_Stack 경로를 sys.path 에 추가."""
    here = Path(__file__).resolve()
    package_root = here.parents[2]
    for parent in package_root.parents:
        if parent.name in ("00_BRAIN", "_staging"):
            break
    candidate_base = package_root.parent
    for base in (candidate_base, candidate_base.parent / "_staging"):
        candidate = base / "Transformable_Air_Mobility_Stack"
        if candidate.is_dir():
            s = str(candidate)
            if s not in sys.path:
                sys.path.insert(0, s)
            break


def try_import_tam_readiness(
    mode: str = "HOVER",
    **build_kwargs: Any,
) -> Optional[TamLaunchReadiness]:
    """TAM 패키지를 동적 import 해서 readiness 를 직접 계산.

    TAM 이 설치된 모노레포 환경에서 사용.
    ``build_kwargs`` 는 TAM GroundAirTickInput 파라미터.
    """
    _ensure_tam_on_path()
    try:
        from transformable_air_mobility import (
            run_ground_air_tick,
        )
        from transformable_air_mobility.pipeline import GroundAirTickInput
    except ImportError:
        return None

    inp = GroundAirTickInput(mode=mode, **build_kwargs)  # type: ignore[arg-type]
    report, next_mode = run_ground_air_tick(inp)
    return readiness_from_tam_report(report, next_mode)
