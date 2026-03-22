"""추진 모델 — 로켓 엔진 추력·질량유량 계산 (Layer 1).

핵심 방정식:
  추력 (고도 보정):
    F(h) = F_vac − P_atm(h) × A_exit
         = throttle × (F_vac − P_atm × A_nozzle)

  질량 유량 (진공 기준 고정):
    ṁ = F_vac / (Isp_vac × g₀) × throttle

  유효 비추력 (고도 함수):
    Isp(h) = F(h) / (ṁ × g₀)

설계 선택:
  - ṁ 는 진공 기준으로 고정 (단순화 — 노즐 유량은 챔버 압력 지배)
  - 추력만 고도에 따라 보정 (배압 손실)
  - 스로틀 명령 → 실제 스로틀에 1차 지연 (tau_s) 적용

관찰 한계:
  - 연소 불안정·추력 편차 미모델링
  - 엔진 시동/종료 과도 구간 단순화 (ignition_delay 상수)
  - 다중 엔진 고장 케이스 미포함 (MVP 범위 밖)
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from ..contracts.schemas import (
    AtmosphereState, PhysicsConfig, PropulsionState,
    StageConfig, DEFAULT_PHYSICS
)


@dataclass
class PropulsionConfig:
    """추진 엔진 동적 파라미터.

    throttle_tau_s: 스로틀 1차 지연 시정수 (s)
    min_ignition_throttle: 점화 유지 최소 스로틀
    """
    throttle_tau_s:        float = 0.3   # 스로틀 응답 지연 (s)
    min_ignition_throttle: float = 0.3   # 연소 유지 최소값


class PropulsionEngine:
    """로켓 추진 엔진 모델.

    1개 단(Stage)의 추진 상태를 매 틱 갱신.
    단분리 시 reset_stage() 로 다음 단 전환.

    사용법::
        eng = PropulsionEngine(stage_cfg, atm_engine)
        eng.ignite()
        for each tick:
            state = eng.tick(atm_state, throttle_cmd, dt_s)
    """

    def __init__(
        self,
        stage: StageConfig,
        config: Optional[PropulsionConfig] = None,
        physics: PhysicsConfig = DEFAULT_PHYSICS,
    ):
        self._stage   = stage
        self._cfg     = config or PropulsionConfig()
        self._phys    = physics

        # 내부 상태 (가변)
        self._propellant_kg:  float = stage.propellant_mass_kg
        self._throttle_actual: float = 0.0   # 실제 스로틀 (지연 반영)
        self._burn_time_s:    float = 0.0
        self._ignited:        bool  = False

    # ── 공개 제어 ─────────────────────────────────────────────────────────────

    def ignite(self) -> bool:
        """점화 시도.

        Returns:
            True: 점화 성공 (추진제 있음)
            False: 점화 실패 (추진제 고갈)
        """
        if self._propellant_kg <= 0.0:
            return False
        self._ignited = True
        self._throttle_actual = self._cfg.min_ignition_throttle
        return True

    def shutdown(self) -> None:
        """엔진 정지 (MECO / 단분리 전)."""
        self._ignited = False
        self._throttle_actual = 0.0

    def reset_stage(self, next_stage: StageConfig) -> None:
        """단분리 후 다음 단으로 전환."""
        self._stage            = next_stage
        self._propellant_kg    = next_stage.propellant_mass_kg
        self._throttle_actual  = 0.0
        self._burn_time_s      = 0.0
        self._ignited          = False

    # ── 메인 틱 ──────────────────────────────────────────────────────────────

    def tick(
        self,
        atm: AtmosphereState,
        throttle_cmd: float,
        dt_s: float,
    ) -> PropulsionState:
        """추진 상태 한 틱 갱신.

        Args:
            atm:          현재 대기 상태 (배압 보정용)
            throttle_cmd: 스로틀 명령 [0, 1]
            dt_s:         타임스텝 (s)

        Returns:
            PropulsionState — 현재 추진 관찰값
        """
        if not self._ignited or self._propellant_kg <= 0.0:
            self._ignited = False
            return self._zero_state()

        # 스로틀 1차 지연 (τ = throttle_tau_s)
        throttle_cmd_clamped = max(
            self._stage.min_throttle,
            min(self._stage.max_throttle, throttle_cmd)
        )
        tau = self._cfg.throttle_tau_s
        alpha = dt_s / (tau + dt_s)  # 이산화 지수 이동 평균
        self._throttle_actual += alpha * (throttle_cmd_clamped - self._throttle_actual)

        # 고도 보정 추력
        thrust = self._thrust_at_altitude(atm.pressure_pa)

        # 질량 유량 (진공 기준 → 고도에 관계없이 동일)
        mdot = self._stage.mass_flow_kgs(self._throttle_actual, self._phys)

        # 추진제 소모
        consumed = mdot * dt_s
        self._propellant_kg = max(0.0, self._propellant_kg - consumed)
        self._burn_time_s  += dt_s

        # 추진제 고갈 → 자동 종료
        if self._propellant_kg <= 0.0:
            self.shutdown()

        # 유효 Isp (고도 보정)
        isp_eff = thrust / (mdot * self._phys.g0_ms2) if mdot > 1e-6 else 0.0

        return PropulsionState(
            thrust_n=thrust,
            mass_flow_kgs=mdot,
            isp_s=isp_eff,
            throttle_fraction=self._throttle_actual,
            propellant_mass_kg=self._propellant_kg,
            is_ignited=self._ignited,
            burn_time_s=self._burn_time_s,
        )

    # ── 내부 계산 ─────────────────────────────────────────────────────────────

    def _thrust_at_altitude(self, p_ambient_pa: float) -> float:
        """배압 손실 반영 고도 보정 추력.

        F(h) = throttle × (F_vac − P_amb × A_exit)

        F_sl = F_vac − P_sl × A_exit  에서 A_exit 역산:
          A_exit = (F_vac − F_sl) / P_sl
        """
        st  = self._stage
        P_sl = self._phys.P_sl_pa
        A_exit = st.nozzle_exit_area_m2
        F_vac = st.thrust_vac_n

        # 배압 손실
        F_altitude = F_vac - p_ambient_pa * A_exit
        F_altitude = max(0.0, F_altitude)  # 음수 추력 방지

        return self._throttle_actual * F_altitude

    def _zero_state(self) -> PropulsionState:
        """엔진 꺼진 상태."""
        return PropulsionState(
            thrust_n=0.0,
            mass_flow_kgs=0.0,
            isp_s=0.0,
            throttle_fraction=0.0,
            propellant_mass_kg=self._propellant_kg,
            is_ignited=False,
            burn_time_s=self._burn_time_s,
        )

    # ── 조회 ─────────────────────────────────────────────────────────────────

    @property
    def propellant_remaining_kg(self) -> float:
        return self._propellant_kg

    @property
    def is_ignited(self) -> bool:
        return self._ignited

    @property
    def burn_time_s(self) -> float:
        return self._burn_time_s

    @property
    def propellant_fraction(self) -> float:
        """잔여 추진제 비율 [0, 1]."""
        if self._stage.propellant_mass_kg <= 0.0:
            return 0.0
        return self._propellant_kg / self._stage.propellant_mass_kg
