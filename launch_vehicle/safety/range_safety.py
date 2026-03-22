"""Range Safety — 비행 복도 감시·IIP 추정 (Layer 4).

역할:
  매 틱 발사체 궤적이 허용 비행 복도 내에 있는지 확인.
  IIP (Instantaneous Impact Point) 를 추정하여
  금지 구역 진입 시 자폭 명령 권고.

IIP 근사 계산:
  - 단순화: 지구 평면 가정 + 탄도 투사 (진공, 중력만)
  - 현재 위치·속도 → 자유 낙하 시 지상 충돌 예상 위치
  - 정확한 IIP 는 대기·지구 곡률·잔여 추진력 포함 → 근사값

탄도 IIP 수식:
  z(t) = z₀ + vz₀·t − ½g·t²  → z(t*)=0 에서 t* 계산
  x_IIP = x₀ + vx₀·t*
  y_IIP = y₀ + vy₀·t*

비행 복도:
  원점(발사장) 중심, 방위각 범위 내, 최대 이탈 거리 제한.
  단순 직사각형 또는 원뿔형 복도.

관찰 한계:
  - 실제 Range Safety 는 회전 지구·대기·파편 궤도 포함
  - 본 구현은 근사 IIP + 단순 corridor check (MVP)
  - 법적·운용적 Range Safety 절차 대체 불가
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

from ..contracts.schemas import PhysicsConfig, RocketState, DEFAULT_PHYSICS


@dataclass
class RangeSafetyConfig:
    """비행 복도 파라미터.

    corridor_half_width_m: 비행 경로 중심선에서 최대 허용 이탈 거리 (m)
    corridor_direction_rad: 비행 방위각 (rad, 북=0, 동=π/2)
    max_downrange_m:        최대 허용 사거리 (m)
    iip_exclusion_radius_m: IIP 금지 구역 반경 (m, 발사장 주변)
    """
    corridor_half_width_m:  float = 50_000.0    # ±50 km 복도
    corridor_direction_rad: float = 0.0          # 북방향 발사 (참고값)
    max_downrange_m:        float = 500_000.0    # 500 km 최대 사거리
    iip_exclusion_radius_m: float = 200_000.0   # IIP 금지 구역 반경


@dataclass
class RangeSafetyReport:
    """Range Safety 평가 결과."""
    in_corridor:      bool
    iip_x_m:          float   # IIP 동 좌표 (m, 발사장 기준)
    iip_y_m:          float   # IIP 북 좌표 (m)
    iip_downrange_m:  float   # IIP 사거리
    iip_crossrange_m: float   # IIP 복도 이탈 거리
    corridor_ok:      bool    # 비행 복도 내 여부
    iip_safe:         bool    # IIP 금지 구역 밖 여부
    destruct_required: bool   # 자폭 명령 권고
    note:             str = ""


class RangeSafetySystem:
    """Range Safety 시스템.

    사용법::
        rss = RangeSafetySystem(config)
        report = rss.check(state)
        if report.destruct_required:
            trigger_abort()
    """

    def __init__(
        self,
        config: Optional[RangeSafetyConfig] = None,
        physics: PhysicsConfig = DEFAULT_PHYSICS,
    ):
        self._cfg  = config or RangeSafetyConfig()
        self._phys = physics

    def check(self, state: RocketState) -> RangeSafetyReport:
        """현재 상태에서 Range Safety 평가.

        Args:
            state: 현재 발사체 상태

        Returns:
            RangeSafetyReport
        """
        iip_x, iip_y = self._compute_iip(state)
        iip_dr = math.sqrt(iip_x**2 + iip_y**2)

        # 복도 이탈 계산 (복도 방향에 대한 수직 거리)
        d = self._cfg.corridor_direction_rad
        cos_d, sin_d = math.cos(d), math.sin(d)
        # 복도 방향 단위벡터 (along), 법선벡터 (cross)
        iip_cross = abs(-sin_d * iip_x + cos_d * iip_y)
        iip_along = cos_d * iip_x + sin_d * iip_y

        corridor_ok = (
            iip_cross <= self._cfg.corridor_half_width_m
            and iip_along <= self._cfg.max_downrange_m
        )
        iip_safe = iip_dr >= self._cfg.iip_exclusion_radius_m

        destruct = not corridor_ok or not iip_safe

        notes = []
        if not corridor_ok:
            notes.append(f"이탈: crossrange={iip_cross/1000:.1f}km")
        if not iip_safe:
            notes.append(f"IIP 금지구역: {iip_dr/1000:.1f}km")

        return RangeSafetyReport(
            in_corridor=corridor_ok,
            iip_x_m=iip_x,
            iip_y_m=iip_y,
            iip_downrange_m=iip_dr,
            iip_crossrange_m=iip_cross,
            corridor_ok=corridor_ok,
            iip_safe=iip_safe,
            destruct_required=destruct,
            note=" | ".join(notes),
        )

    # ── 내부 ─────────────────────────────────────────────────────────────────

    def _compute_iip(self, state: RocketState) -> Tuple[float, float]:
        """탄도 자유 낙하 기반 근사 IIP 계산.

        현재 고도·속도에서 엔진 정지 + 진공 탄도 가정.
        z(t) = z₀ + vz₀·t − ½g·t²  →  t* (z=0 해)
        x_IIP = x₀ + vx₀·t*
        y_IIP = y₀ + vy₀·t*
        """
        g  = self._phys.g0_ms2   # 근사: 고도 무관 (단순화)
        z0 = state.z_m
        vz = state.vz_ms

        # z(t) = 0 풀기: t² + (2vz/g)·t − 2z₀/g = 0
        a_coeff = -0.5 * g
        b_coeff = vz
        c_coeff = z0

        discriminant = b_coeff**2 - 4 * a_coeff * c_coeff
        if discriminant < 0 or a_coeff == 0:
            # 현재 고도 0 이하 (발사대 상태 등) → IIP = 현재 위치
            return state.x_m, state.y_m

        t_star = (-b_coeff - math.sqrt(discriminant)) / (2 * a_coeff)
        if t_star < 0:
            t_star = (-b_coeff + math.sqrt(discriminant)) / (2 * a_coeff)
        t_star = max(0.0, t_star)

        iip_x = state.x_m + state.vx_ms * t_star
        iip_y = state.y_m + state.vy_ms * t_star
        return iip_x, iip_y
