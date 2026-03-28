"""AgedCare_Stack → LaunchVehicle 유인 발사 안전 어댑터.

역할:
    유인 발사 시나리오 (MVP-4) 에서 탑승자(고령자·취약자)의 안전 상태를
    발사 인가 조건으로 반영한다.

    MVP-1 (무인): AgedCareLaunchSafety 없어도 발사 가능.
    MVP-4 (유인): care_verdict=="SAFE" + omega_care>=0.80 필수.

    AgedCare_Stack 이 설치되지 않아도 동작 (duck-typing + ImportError 폴백).

차단 코드:
    "crew_care_emergency"         — 탑승자 비상 상태
    "crew_care_manual_override"   — 수동 중단 요청
    "crew_care_verdict_not_safe"  — 탑승자 상태 CAUTION/WARNING 이상
    "crew_missing_care_snapshot"  — 유인 MVP-4 인데 스냅샷 없음

사용법::
    from launch_vehicle.adapters.aged_care_adapter import (
        AgedCareLaunchSafety, snapshot_from_safety_state
    )
    snap = snapshot_from_safety_state(care_state)
    agent = LaunchAgent(vehicle, crew_safety=snap, human_rated_mvp4=True)
"""
from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional


@dataclass(frozen=True)
class AgedCareLaunchSafety:
    """탑승자 안전 스냅샷 — 유인 발사 인가 게이트용."""
    omega_care: float = 1.0          # 케어 건전성 [0,1]
    verdict: str = "SAFE"            # SAFE / CAUTION / WARNING / EMERGENCY
    emergency_triggered: bool = False
    manual_override: bool = False    # 탑승자/케어스태프 수동 중단 요청


def _verdict_scale(verdict: str) -> float:
    v = (verdict or "").strip().upper()
    return {"SAFE": 1.0, "CAUTION": 0.55, "WARNING": 0.25, "EMERGENCY": 0.0}.get(v, 0.45)


def launch_safety_from_safety_state(safety: Any) -> AgedCareLaunchSafety:
    """duck-typed SafetyState → AgedCareLaunchSafety 변환."""
    omega = float(getattr(safety, "omega", getattr(safety, "omega_care", 1.0)))
    verdict = str(getattr(safety, "verdict", "SAFE"))
    emergency = bool(getattr(safety, "emergency_triggered",
                             getattr(safety, "emergency", False)))
    override = bool(getattr(safety, "manual_override", False))
    return AgedCareLaunchSafety(
        omega_care=max(0.0, min(1.0, omega)),
        verdict=verdict,
        emergency_triggered=emergency,
        manual_override=override,
    )


def launch_safety_from_omega_report(report: Any) -> AgedCareLaunchSafety:
    """duck-typed OmegaReport → AgedCareLaunchSafety 변환."""
    omega = float(getattr(report, "omega_total", getattr(report, "omega", 1.0)))
    verdict = str(getattr(report, "verdict", "SAFE"))
    # OmegaReport 의 CRITICAL 은 비상에 준함
    emergency = verdict.strip().upper() in ("CRITICAL", "EMERGENCY")
    return AgedCareLaunchSafety(
        omega_care=max(0.0, min(1.0, omega)),
        verdict=verdict,
        emergency_triggered=emergency,
    )


def _ensure_aged_care_on_path() -> None:
    here = Path(__file__).resolve()
    package_root = here.parents[2]
    candidate_base = package_root.parent
    for base in (candidate_base, candidate_base.parent / "_staging"):
        candidate = base / "AgedCare_Stack"
        if candidate.is_dir():
            s = str(candidate)
            if s not in sys.path:
                sys.path.insert(0, s)
            break


def snapshot_from_safety_state(safety: Any) -> Optional[AgedCareLaunchSafety]:
    """안전 스냅샷 변환 — duck-typing (AgedCare 미설치 시 None)."""
    if safety is None:
        return None
    return launch_safety_from_safety_state(safety)


def try_import_aged_care_safety() -> Optional[AgedCareLaunchSafety]:
    """AgedCare_Stack 에서 직접 SafetyState 를 가져와 변환."""
    _ensure_aged_care_on_path()
    try:
        from aged_care.safety import get_current_safety_state  # type: ignore
        state = get_current_safety_state()
        return launch_safety_from_safety_state(state)
    except (ImportError, AttributeError):
        return None


def evaluate_crew_launch_gate(
    crew_safety: Optional[AgedCareLaunchSafety],
    human_rated_mvp4: bool,
) -> tuple[bool, list]:
    """유인 발사 게이트 판정.

    Returns:
        (crew_launch_ok, blockers_list)
    """
    blockers = []
    if not human_rated_mvp4:
        return True, []  # 무인 모드 — 게이트 불필요

    if crew_safety is None:
        blockers.append("crew_missing_care_snapshot")
        return False, blockers

    if crew_safety.emergency_triggered:
        blockers.append("crew_care_emergency")
    if crew_safety.manual_override:
        blockers.append("crew_care_manual_override")
    v = crew_safety.verdict.strip().upper()
    if v != "SAFE":
        blockers.append(f"crew_care_verdict_not_safe:{v}")

    ok = (
        len(blockers) == 0
        and crew_safety.omega_care >= 0.80
        and v == "SAFE"
    )
    return ok, blockers
