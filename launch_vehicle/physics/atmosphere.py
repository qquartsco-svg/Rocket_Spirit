"""대기 모델 — US Standard Atmosphere 1976 (Layer 1).

구현 범위:
  0 ~ 86,000 m (기하 고도 기준)
  7개 고도 레이어 (troposphere ~ mesosphere)

출력:
  AtmosphereState(고도, 기온, 기압, 밀도, 음속, 마하수, 동압)

관찰 한계:
  - 표준 대기 (실제 날씨, 바람, 습도 미반영)
  - 86 km 초과 시 86 km 경계값 유지
  - 열권 이상(열전달·복사 영향) 미모델링

수식 출처:
  NOAA/NASA/USAF Standard Atmosphere 1976
  https://ntrs.nasa.gov/citations/19770009539

설계 철학:
  - 순수 stdlib — 외부 의존 없음
  - 모든 상수는 클래스 레벨 상수 (하드코딩 아님 — 물리 상수는 설계값)
  - query() 는 순수 함수 (같은 입력 → 같은 출력, 부작용 없음)
"""
from __future__ import annotations

import math
from typing import Tuple

from ..contracts.schemas import AtmosphereState, PhysicsConfig, DEFAULT_PHYSICS


# ── 표준 대기 레이어 정의 ─────────────────────────────────────────────────────
# (기저 고도_m, 기저 온도_K, 온도 기울기_K/m, 기저 압력_Pa)
# 출처: US Standard Atmosphere 1976, Table 4

_LAYERS: Tuple[Tuple[float, float, float, float], ...] = (
    (     0.0, 288.15, -0.0065,  101_325.0),   # 대류권 (Troposphere)
    ( 11_000.0, 216.65,  0.0000,   22_632.1),   # 권계면1 (Tropopause)
    ( 20_000.0, 216.65,  0.0010,    5_474.89),  # 성층권1 (Stratosphere L1)
    ( 32_000.0, 228.65,  0.0028,      868.019), # 성층권2 (Stratosphere L2)
    ( 47_000.0, 270.65,  0.0000,      110.906), # 성층계면 (Stratopause)
    ( 51_000.0, 270.65, -0.0028,       66.9389),# 중간권1 (Mesosphere L1)
    ( 71_000.0, 214.65, -0.0020,        3.95642),# 중간권2 (Mesosphere L2)
    ( 86_000.0, 186.87,  0.0000,        0.37338),# 상한 (경계값 유지)
)


class AtmosphereEngine:
    """US Standard Atmosphere 1976 기반 대기 모델.

    순수 계산 엔진 — 상태를 들고 있지 않음 (stateless).

    사용법::
        atm = AtmosphereEngine()
        state = atm.query(altitude_m=10_000, speed_ms=300)
        print(state.mach, state.density_kgm3)
    """

    def __init__(self, physics: PhysicsConfig = DEFAULT_PHYSICS):
        self._phys = physics

    # ── 공개 API ──────────────────────────────────────────────────────────────

    def query(self, altitude_m: float, speed_ms: float = 0.0) -> AtmosphereState:
        """고도와 속도로 대기 상태 추정.

        Args:
            altitude_m: 기하 고도 (m, 음수는 0으로 클램핑)
            speed_ms:   현재 속도 크기 (m/s, 마하·동압 계산용)

        Returns:
            AtmosphereState — 추정된 대기 상태
        """
        h = max(0.0, min(altitude_m, 86_000.0))
        T, P = self._temperature_pressure(h)
        rho  = self._density(P, T)
        a    = self._speed_of_sound(T)
        mach = speed_ms / a if a > 0.0 else 0.0
        q    = 0.5 * rho * speed_ms ** 2

        return AtmosphereState(
            altitude_m=h,
            temperature_k=T,
            pressure_pa=P,
            density_kgm3=rho,
            speed_of_sound_ms=a,
            mach=mach,
            dynamic_q_pa=q,
        )

    def density(self, altitude_m: float) -> float:
        """밀도만 필요할 때 사용하는 간편 호출."""
        h = max(0.0, min(altitude_m, 86_000.0))
        T, P = self._temperature_pressure(h)
        return self._density(P, T)

    # ── 내부 계산 ─────────────────────────────────────────────────────────────

    def _temperature_pressure(self, h: float) -> Tuple[float, float]:
        """고도 h(m) 에서 기온(K)·기압(Pa) 계산.

        레이어별 공식:
          등온 레이어 (lapse_rate == 0):
            P = P_b × exp(−g₀ × (h − H_b) / (R × T_b))
          기온 기울기 레이어 (lapse_rate ≠ 0):
            T = T_b + L × (h − H_b)
            P = P_b × (T / T_b)^(−g₀ / (L × R))
        """
        g0 = self._phys.g0_ms2
        R  = self._phys.R_air_J_kgK

        # 현재 레이어 찾기
        layer = _LAYERS[-1]
        for i in range(len(_LAYERS) - 1):
            if h < _LAYERS[i + 1][0]:
                layer = _LAYERS[i]
                break

        H_b, T_b, L, P_b = layer
        dh = h - H_b

        if abs(L) < 1e-12:  # 등온 레이어
            T = T_b
            P = P_b * math.exp(-g0 * dh / (R * T_b))
        else:               # 기온 기울기 레이어
            T = T_b + L * dh
            exponent = -g0 / (L * R)
            P = P_b * (T / T_b) ** exponent

        return T, P

    def _density(self, pressure_pa: float, temperature_k: float) -> float:
        """이상 기체 상태방정식: ρ = P / (R × T)."""
        return pressure_pa / (self._phys.R_air_J_kgK * temperature_k)

    def _speed_of_sound(self, temperature_k: float) -> float:
        """건조 공기 음속: a = √(γ × R × T)."""
        return math.sqrt(self._phys.gamma_air * self._phys.R_air_J_kgK * temperature_k)

    # ── 유틸리티 ──────────────────────────────────────────────────────────────

    def gravity(self, altitude_m: float) -> float:
        """고도별 중력 가속도: g(h) = g₀ × (R_e / (R_e + h))².

        발사체 상승 중 중력 약화를 반영.
        """
        R_e = self._phys.R_earth_m
        g0  = self._phys.g0_ms2
        return g0 * (R_e / (R_e + altitude_m)) ** 2
