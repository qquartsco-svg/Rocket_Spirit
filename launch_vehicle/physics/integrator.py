"""가변 질량 RK4 적분기 — 3-DoF 질점 + 질량 소모 (Layer 1).

적분 대상 상태벡터:
  s⃗ = [x, y, z, vx, vy, vz, m]  (7-state translational + mass)

운동 방정식:
  dx/dt = vx
  dy/dt = vy
  dz/dt = vz
  dvx/dt = (F_thrust_x + F_drag_x) / m
  dvy/dt = (F_thrust_y + F_drag_y) / m
  dvz/dt = (F_thrust_z + F_drag_z) / m − g(z)
  dm/dt  = −ṁ

좌표계 (ENU, 발사장 기준):
  x: East (동)
  y: North (북)
  z: Up (수직 고도)

추력 벡터 분해:
  pitch: 지면과 수직축 사이 각도 (rad), 초기=π/2 (수직)
  yaw:   방위각 (rad), 0 = 북
  pitch 킥 → x-z 평면 내 회전
  F_thrust_x = F × cos(yaw) × sin(π/2 − pitch)  = F × cos(yaw) × cos(pitch)
  F_thrust_y = F × sin(yaw) × cos(pitch)
  F_thrust_z = F × sin(pitch)

항력 벡터 분해:
  항력은 속도 반대 방향:
  F_drag = −D × v̂  (v̂ = v⃗/|v⃗|)

설계 선택:
  - 4차 룽게-쿠타 (RK4): 오차 O(dt⁴), 고정 타임스텝
  - dt_s 권장: 0.05 ~ 0.1 s (발사체 시간스케일)
  - 지표면 충돌 방지: z < 0 → vz = 0 클램핑
  - 질량 하한: dry_mass_kg 이하로 감소 안 함

관찰 한계:
  - 회전 역학(모멘트·자세 적분) 미포함 — pitch/yaw 는 직접 지정
  - 지구 자전·코리올리 미반영
  - 상단 대기(86km+) 에서의 희박화 미모델링
"""
from __future__ import annotations

import math
from typing import Tuple

from ..contracts.schemas import (
    PhysicsConfig, RocketState, DEFAULT_PHYSICS
)


# 상태벡터 타입 별칭
_Vec7 = Tuple[float, float, float, float, float, float, float]


class VariableMassIntegrator:
    """가변 질량 3-DoF RK4 적분기.

    사용법::
        integrator = VariableMassIntegrator(dry_mass_kg=5_000)
        new_state = integrator.step(
            state,
            thrust_n=6_000_000,
            pitch_rad=1.2,
            yaw_rad=0.0,
            drag_n=50_000,
            mass_flow_kgs=2_000,
            dt_s=0.05,
        )
    """

    def __init__(
        self,
        dry_mass_kg: float,
        physics: PhysicsConfig = DEFAULT_PHYSICS,
    ):
        self._dry_mass = dry_mass_kg
        self._phys     = physics

    def step(
        self,
        state: RocketState,
        thrust_n: float,
        pitch_rad: float,
        yaw_rad: float,
        drag_n: float,
        mass_flow_kgs: float,
        dt_s: float,
    ) -> RocketState:
        """RK4 단 1회 적분 → 새 RocketState 반환.

        Args:
            state:          현재 상태
            thrust_n:       순간 추력 크기 (N)
            pitch_rad:      현재 피치각 (rad)
            yaw_rad:        현재 요각 (rad)
            drag_n:         항력 크기 (N, 속도 반대방향)
            mass_flow_kgs:  질량 유량 (kg/s)
            dt_s:           타임스텝 (s)

        Returns:
            새 RocketState — 갱신된 추정 상태
        """
        s0 = state.as_vector()

        k1 = self._deriv(s0, thrust_n, pitch_rad, yaw_rad, drag_n, mass_flow_kgs)
        k2 = self._deriv(_add(_scale(k1, dt_s * 0.5), s0),
                         thrust_n, pitch_rad, yaw_rad, drag_n, mass_flow_kgs)
        k3 = self._deriv(_add(_scale(k2, dt_s * 0.5), s0),
                         thrust_n, pitch_rad, yaw_rad, drag_n, mass_flow_kgs)
        k4 = self._deriv(_add(_scale(k3, dt_s), s0),
                         thrust_n, pitch_rad, yaw_rad, drag_n, mass_flow_kgs)

        # RK4 가중 평균
        ds = _scale(
            _add(_add(_add(k1, _scale(k2, 2.0)), _scale(k3, 2.0)), k4),
            dt_s / 6.0
        )
        s1_raw = _add(s0, ds)

        # 클램핑: 지표면 아래 진입 방지 + 질량 하한
        s1 = list(s1_raw)
        if s1[2] < 0.0:         # z < 0
            s1[2] = 0.0
            if s1[5] < 0.0:
                s1[5] = 0.0     # vz 클램핑
        s1[6] = max(self._dry_mass, s1[6])  # m ≥ dry_mass

        s1_t = tuple(s1)

        # 파생값 계산
        vx, vy, vz = s1_t[3], s1_t[4], s1_t[5]
        speed = math.sqrt(vx**2 + vy**2 + vz**2)
        propellant_remaining = max(0.0, s1_t[6] - self._dry_mass)

        return RocketState.from_vector(
            vec=s1_t,
            propellant_kg=propellant_remaining,
            t_s=state.t_s + dt_s,
            pitch_rad=pitch_rad,
            yaw_rad=yaw_rad,
        )

    # ── 내부 ─────────────────────────────────────────────────────────────────

    def _deriv(
        self,
        s: _Vec7,
        thrust_n: float,
        pitch_rad: float,
        yaw_rad: float,
        drag_n: float,
        mdot: float,
    ) -> _Vec7:
        """ds⃗/dt 계산 — 미분 방정식 우변."""
        x, y, z, vx, vy, vz, m = s

        # 중력 (고도별 감소 반영)
        g = self._phys.g0_ms2 * (
            self._phys.R_earth_m / (self._phys.R_earth_m + max(0.0, z))
        ) ** 2

        # 추력 벡터 분해 (ENU, pitch=π/2 → 수직)
        # pitch: 수직축(z)과의 각도 기준 (0=수평, π/2=수직)
        Ft_z = thrust_n * math.sin(pitch_rad)
        Ft_h = thrust_n * math.cos(pitch_rad)       # 수평 성분
        Ft_x = Ft_h * math.cos(yaw_rad)
        Ft_y = Ft_h * math.sin(yaw_rad)

        # 항력 벡터 분해 (속도 반대)
        speed = math.sqrt(vx**2 + vy**2 + vz**2)
        if speed > 1e-6:
            Fd_x = -drag_n * vx / speed
            Fd_y = -drag_n * vy / speed
            Fd_z = -drag_n * vz / speed
        else:
            Fd_x = Fd_y = Fd_z = 0.0

        m_safe = max(self._dry_mass, m)

        # 가속도
        ax = (Ft_x + Fd_x) / m_safe
        ay = (Ft_y + Fd_y) / m_safe
        az = (Ft_z + Fd_z) / m_safe - g

        return (vx, vy, vz, ax, ay, az, -mdot)


# ── 벡터 유틸리티 (순수 함수) ────────────────────────────────────────────────

def _add(a: _Vec7, b: _Vec7) -> _Vec7:
    return tuple(x + y for x, y in zip(a, b))  # type: ignore[return-value]

def _scale(a: _Vec7, s: float) -> _Vec7:
    return tuple(x * s for x in a)  # type: ignore[return-value]
