"""공력 모델 — 항력·동압 계산 (Layer 1).

핵심 방정식:
  D = ½ρv²·Cd(M)·A_ref

Cd(M) 모델:
  마하수 구간별 조각선형(piecewise-linear) 보간.
  실제 발사체 Cd 곡선의 정성적 특성 반영:
    - 아음속 (M < 0.8):     Cd ≈ 0.20 (평탄)
    - 천음속 (0.8 ~ 1.2):  Cd 급등 → 최대 ≈ 0.45 (파 항력)
    - 초음속 (1.2 ~ 3.0):  Cd 감소 → 0.25 수준
    - 고초음속 (M > 5.0):  Cd ≈ 0.18 (점근)

관찰 한계:
  - CFD / 풍동 시험 데이터 미사용 (근사 곡선)
  - 받음각 항력 (angle-of-attack drag) 미포함 (슬렌더 바디 가정)
  - 양력·모멘트 미포함 (순수 질점 모델)
  - 마찰·베이스 항력 분리 없음 (총 Cd 단일값)

설계 철학:
  - Cd 테이블은 외부에서 주입 가능 (VehicleAeroConfig)
  - piecewise linear 보간 → 검증 가능한 단순 모델
  - 실측 Cd 데이터 보유 시 테이블 교체만으로 정밀도 향상
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple

from ..contracts.schemas import AeroState, AtmosphereState


# ── Cd 테이블 (마하수, Cd) 기본값 ────────────────────────────────────────────
# 슬렌더 발사체 근사 Cd 곡선 (정성적)
# (M, Cd_total)

_DEFAULT_CD_TABLE: Tuple[Tuple[float, float], ...] = (
    (0.00, 0.18),   # 정지 (마찰 항력 지배)
    (0.50, 0.18),
    (0.80, 0.20),   # 아음속 끝
    (0.90, 0.28),   # 천음속 Cd 급등 시작
    (1.00, 0.40),   # 충격파 형성
    (1.10, 0.45),   # Cd 최고점 (파항력 최대)
    (1.20, 0.42),
    (1.50, 0.35),   # 초음속 감소
    (2.00, 0.28),
    (3.00, 0.23),
    (5.00, 0.20),   # 극초음속
    (8.00, 0.18),
    (10.0, 0.18),   # 점근
)


@dataclass
class VehicleAeroConfig:
    """발사체 공력 사양.

    cd_table: (마하수, Cd) 조각선형 테이블
              첫 번째 열은 오름차순 마하수여야 함.
    ref_area_m2: 기준 면적 (통상 π·(d/2)²)
    """
    ref_area_m2: float
    cd_table: List[Tuple[float, float]] = field(
        default_factory=lambda: list(_DEFAULT_CD_TABLE)
    )


class AerodynamicsEngine:
    """발사체 공력 모델.

    stateless — 매 틱 순수 함수적으로 호출 가능.

    사용법::
        cfg = VehicleAeroConfig(ref_area_m2=10.52)
        aero = AerodynamicsEngine(cfg)
        state = aero.tick(atm_state, speed_ms=500)
    """

    def __init__(self, config: VehicleAeroConfig):
        self._cfg = config
        # Cd 테이블 정렬 보장
        self._cd_table = sorted(config.cd_table, key=lambda x: x[0])

    # ── 공개 API ──────────────────────────────────────────────────────────────

    def tick(self, atm: AtmosphereState, speed_ms: float) -> AeroState:
        """한 틱 공력 상태 계산.

        Args:
            atm:      현재 대기 상태
            speed_ms: 현재 속도 크기 (m/s)

        Returns:
            AeroState — 항력·Cd·동압
        """
        mach = atm.mach  # 이미 atm 에서 계산됨
        cd   = self._cd_interpolate(mach)
        q    = atm.dynamic_q_pa
        drag = q * cd * self._cfg.ref_area_m2

        return AeroState(
            drag_n=drag,
            cd=cd,
            mach=mach,
            q_pa=q,
            ref_area_m2=self._cfg.ref_area_m2,
        )

    def cd_at_mach(self, mach: float) -> float:
        """마하수에 대한 Cd 직접 조회 (단위 테스트·시각화용)."""
        return self._cd_interpolate(mach)

    def max_q_altitude(self, vehicle_mass_kg: float,
                       thrust_n: float,
                       step_m: float = 100.0) -> float:
        """Max-Q 발생 예상 고도 탐색 (간편 추정).

        실제 Max-Q 는 비행 중 계산되므로 이 함수는 참고값.
        단순 수직 상승 + 일정 추력 가정.
        """
        from .atmosphere import AtmosphereEngine
        atm_eng = AtmosphereEngine()
        q_prev = 0.0
        h = 0.0
        # 대략적 가속 추정
        while h < 60_000.0:
            atm = atm_eng.query(h)
            g   = atm_eng.gravity(h)
            net_f = thrust_n - vehicle_mass_kg * g - atm.dynamic_q_pa * self._cd_interpolate(atm.mach) * self._cfg.ref_area_m2
            acc = max(0.0, net_f / vehicle_mass_kg)
            v_approx = (2 * acc * h) ** 0.5 if h > 0 else 0.0
            q = 0.5 * atm.density_kgm3 * v_approx ** 2
            if q < q_prev and h > 5_000.0:
                return h - step_m
            q_prev = q
            h += step_m
        return h

    # ── 내부 ─────────────────────────────────────────────────────────────────

    def _cd_interpolate(self, mach: float) -> float:
        """마하수 조각선형 보간 Cd.

        mach ≤ 첫 항목: 첫 항목 Cd 사용
        mach ≥ 마지막: 마지막 Cd 사용
        """
        table = self._cd_table
        if mach <= table[0][0]:
            return table[0][1]
        if mach >= table[-1][0]:
            return table[-1][1]
        for i in range(len(table) - 1):
            m0, cd0 = table[i]
            m1, cd1 = table[i + 1]
            if m0 <= mach < m1:
                t = (mach - m0) / (m1 - m0)
                return cd0 + t * (cd1 - cd0)
        return table[-1][1]
