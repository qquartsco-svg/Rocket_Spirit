"""Air_Jordan 비행 물리 엔진 → LaunchVehicle 대기권 구간 판정 브리지.

역할:
    Air Jordan 은 날개·공력 물리를 계산하는 비행 엔진이다.
    LaunchVehicle 은 로켓 추진·탄도 물리를 다루지만, 저고도 대기권 구간
    (이륙~Max-Q, 고도 0~20 km)에서는 동압·항력·공력 부하를 추가 검증하는
    데 Air Jordan 의 lift_ratio 와 LD 값을 활용한다.

    고도 > 20 km 이면 Air Jordan 호출을 건너뛰고 LaunchVehicle 자체 대기
    모델을 사용한다. (희박 대기 구간에서 날개 공력 모델은 유효하지 않음)

우선순위 연결 체계:
    Air Jordan (저고도 < 20 km)
      ↓ unavailable
    LaunchVehicle 자체 대기 모델 (보수 폴백)

사용법::
    from launch_vehicle.bridges.air_jordan_bridge import optional_air_jordan_aero
    result = optional_air_jordan_aero(altitude_m=5000, speed_ms=350,
                                      mach=1.05, mass_kg=120_000)
    if result:
        hover_margin, ld, evidence = result
"""
from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Dict, Optional, Tuple


# 대기권 적용 고도 한계 — 이 위에서는 날개 공력 무의미
_MAX_AERO_ALTITUDE_M = 20_000.0


def _aircraft_name_for_rocket(mass_kg: float) -> str:
    """로켓 질량 → Air Jordan 기준 기체 분류."""
    # 소형 로켓(<5t), 중형(<50t), 대형
    if mass_kg <= 5_000.0:
        return "cessna"    # 공력 기준점만 필요
    if mass_kg <= 50_000.0:
        return "f16"
    return "b737"


def _alpha_deg_for_rocket(mach: float) -> float:
    """마하수 기반 공격각 추정 (로켓 피치 프로그램 단순화)."""
    if mach < 0.3:
        return 90.0   # 수직 이륙
    if mach < 0.8:
        return 60.0   # 초기 중력 선회
    if mach < 1.5:
        return 40.0   # 초음속 전환
    return 15.0       # 고속 상승


def _ensure_air_jordan_on_path() -> None:
    here = Path(__file__).resolve()
    package_root = here.parents[2]
    candidate_base = package_root.parent
    for base in (candidate_base, candidate_base.parent / "_staging"):
        candidate = base / "Air_Jordan"
        if candidate.is_dir():
            s = str(candidate)
            if s not in sys.path:
                sys.path.insert(0, s)
            break


def optional_air_jordan_aero(
    altitude_m: float,
    speed_ms: float,
    mach: float,
    mass_kg: float,
) -> Optional[Tuple[float, float, Dict[str, Any]]]:
    """Air Jordan 공력 판정 (저고도 전용).

    Args:
        altitude_m: 현재 고도 (m)
        speed_ms:   비행 속도 (m/s)
        mach:       마하수
        mass_kg:    현재 발사체 질량 (kg)

    Returns:
        (hover_margin_proxy, lift_over_drag, evidence) or None if unavailable/高고도.
    """
    # 고고도에서는 Air Jordan 공력 모델 무의미 → 스킵
    if altitude_m > _MAX_AERO_ALTITUDE_M:
        return None

    _ensure_air_jordan_on_path()
    try:
        from flight_engine import analyze_from_snapshot
    except ImportError:
        return None

    snap = {
        "flight_aircraft":  _aircraft_name_for_rocket(mass_kg),
        "flight_airfoil":   "2412",
        "flight_altitude_m": altitude_m,
        "flight_v_ms":      max(speed_ms, 8.0),
        "flight_alpha_deg": _alpha_deg_for_rocket(mach),
    }
    try:
        result = analyze_from_snapshot(snap)
    except Exception:
        return None

    hover_margin = float(result.lift_ratio) - 1.0
    ld = float(result.forces.lift_over_drag)
    evidence = {
        "aero_source":          "air_jordan",
        "aj_altitude_m":        altitude_m,
        "aj_mach":              mach,
        "aj_lift_ratio":        float(result.lift_ratio),
        "aj_ld":                ld,
        "aj_is_flying":         bool(result.is_flying),
        "aj_efficiency_rating": result.efficiency_rating,
    }
    return hover_margin, ld, evidence
