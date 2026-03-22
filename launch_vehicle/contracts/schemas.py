"""LaunchVehicle_Stack 핵심 데이터 계약 (Layer 0).

설계 철학:
  - 모든 상태는 '관찰/추정값'이다 — 그라운드 트루스가 아님
  - 물리량은 항상 SI 단위 (m, kg, s, rad, N, Pa)
  - frozen=True 로 상태 객체 불변성 보장 (함수형 갱신 패턴)
  - 하드코딩 금지 — 모든 파라미터는 Config 데이터클래스로
  - 단위 충돌 방지: 각도는 rad, 속도는 m/s, 압력은 Pa

계층 구조:
  PhysicsConfig  ← 상수 모음 (중력, 지구 반경 등)
  AtmosphereState  ← 관찰: 고도별 대기 상태
  PropulsionState  ← 관찰: 현재 추진 상태
  AeroState        ← 관찰: 현재 공력 상태
  RocketState      ← 추정: 발사체 주 상태벡터 (7-state translational + mass)
  StageConfig      ← 설계값: 단 사양
  VehicleConfig    ← 설계값: 발사체 전체 사양
  FlightCommand    ← 명령: 유도·제어 출력
  FlightPhase      ← 열거형: 비행 단계
  AbortMode        ← 열거형: 중단 모드
  TelemetryFrame   ← 기록: 한 틱의 텔레메트리 스냅샷
  FlightHealth     ← 판정: 비행 건전성 종합
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple


# ── 물리 상수 (설정 가능) ────────────────────────────────────────────────────

@dataclass
class PhysicsConfig:
    """물리 상수 — 기본값은 지구 표준 기반.

    시스템 의존성을 외부에서 주입 가능하도록 분리.
    달/화성 등 다른 천체 지원 시 서브클래싱 또는 오버라이드.
    """
    g0_ms2:       float = 9.80665       # 표준 중력 가속도 (m/s²)
    R_earth_m:    float = 6_371_000.0   # 지구 평균 반경 (m)
    R_air_J_kgK:  float = 287.058       # 건조 공기 기체 상수 (J/kg·K)
    gamma_air:    float = 1.4           # 공기 비열비
    P_sl_pa:      float = 101_325.0     # 해면 표준 기압 (Pa)
    T_sl_k:       float = 288.15        # 해면 표준 온도 (K)
    rho_sl_kgm3:  float = 1.225         # 해면 표준 밀도 (kg/m³)


# 전역 기본 인스턴스 (모듈 임포트 시점에 고정)
DEFAULT_PHYSICS = PhysicsConfig()


# ── 비행 단계 열거형 ─────────────────────────────────────────────────────────

class FlightPhase(Enum):
    """발사체 비행 단계 FSM 상태.

    전이 경로:
      HOLD → COUNTDOWN → LIFTOFF → ASCENDING → MAX_Q
      → MECO → STAGE_SEP → UPPER_BURN → COAST
      → ORBIT_INSERT → NOMINAL
              ↓ (어느 단계든)
           ABORT
    """
    HOLD         = "hold"          # 발사 대기 (카운트다운 전)
    COUNTDOWN    = "countdown"     # T-10 ~ T-0
    LIFTOFF      = "liftoff"       # T+0 → 탑이 클리어 (~100m)
    ASCENDING    = "ascending"     # 능동 상승 (1단 연소 중)
    MAX_Q        = "max_q"         # 최대 동압 구간 (스로틀 감소)
    MECO         = "meco"          # Main Engine Cut-Off (1단 연소 완료)
    STAGE_SEP    = "stage_sep"     # 단분리 이벤트
    UPPER_BURN   = "upper_burn"    # 2단 연소
    COAST        = "coast"         # 자유 비행 (추진력 없음)
    ORBIT_INSERT = "orbit_insert"  # 궤도 삽입 연소 (circularization)
    NOMINAL      = "nominal"       # 페이로드 분리 완료 — 임무 성공
    ABORT        = "abort"         # 비상 중단 (모든 단계 전이 가능)


# ── 중단 모드 열거형 ─────────────────────────────────────────────────────────

class AbortMode(Enum):
    """발사체 중단 모드.

    고도/단계에 따라 적합한 모드가 달라진다.
    LES(Launch Escape System) 등 유인 모드는 별도 확장.
    """
    NONE             = "none"            # 중단 불필요
    ENGINE_SHUTDOWN  = "engine_shutdown" # 엔진 정지 후 탄도 낙하
    SAFE_DESTRUCT    = "safe_destruct"   # Range Safety 파괴 명령
    COAST_ABORT      = "coast_abort"     # 상단 분리 후 코스트


# ── 대기 상태 (관찰값) ───────────────────────────────────────────────────────

@dataclass(frozen=True)
class AtmosphereState:
    """US Standard Atmosphere 1976 기반 추정 대기 상태.

    altitude_m 는 지오메트릭 고도 (기하 고도) 기준.
    Mach 수는 현재 속도와 이 상태의 음속으로 산출.

    관찰 한계:
      - 표준 대기 모델 (실제 날씨·바람 미반영)
      - 유효 범위: 0 ~ 86,000 m
      - 86 km 이상은 경계값으로 유지
    """
    altitude_m:    float   # 기하 고도 (m)
    temperature_k: float   # 기온 (K)
    pressure_pa:   float   # 기압 (Pa)
    density_kgm3:  float   # 밀도 (kg/m³)
    speed_of_sound_ms: float  # 음속 (m/s)
    mach:          float   # 현재 속도 기반 마하수 (속도 주입 필요)
    dynamic_q_pa:  float   # 동압 q = ½ρv² (Pa)


# ── 추진 상태 (관찰값) ───────────────────────────────────────────────────────

@dataclass(frozen=True)
class PropulsionState:
    """현재 추진 엔진 관찰 상태.

    실제 추력은 고도(주변 압력)에 따라 변한다.
    F(h) ≈ F_vac − P_ambient × A_exit

    관찰 한계:
      - 연소 불안정, 추력 편차 미모델링 (확정값으로 취급하지 말 것)
      - throttle_fraction 은 명령값이며 실제 응답에 지연 있을 수 있음
    """
    thrust_n:          float   # 실제 추력 (N)
    mass_flow_kgs:     float   # 질량 유량 ṁ (kg/s)
    isp_s:             float   # 유효 비추력 (s)
    throttle_fraction: float   # 스로틀 명령값 [0, 1]
    propellant_mass_kg: float  # 잔여 추진제 질량 (kg)
    is_ignited:        bool    # 점화 여부
    burn_time_s:       float   # 현 단 연소 경과 시간 (s)


# ── 공력 상태 (관찰값) ───────────────────────────────────────────────────────

@dataclass(frozen=True)
class AeroState:
    """현재 공력 관찰 상태.

    D = ½ρv²·Cd(M)·A_ref
    항력 방향은 속도 벡터 반대 방향.

    관찰 한계:
      - Cd 는 마하수 구간별 조각선형 보간 (실제 CFD 대체 불가)
      - 양력 미포함 (슬렌더 바디 근사)
      - 천음속(M≈1) 구간은 Cd 급등 모델로 근사
    """
    drag_n:     float   # 항력 크기 (N)
    cd:         float   # 항력 계수 (무차원)
    mach:       float   # 마하수
    q_pa:       float   # 동압 (Pa)
    ref_area_m2: float  # 기준 면적 (m²)


# ── 발사체 주 상태벡터 (추정값) ──────────────────────────────────────────────

@dataclass(frozen=True)
class RocketState:
    """발사체 3-DoF 질점 + 질량 상태 추정값.

    좌표계: ENU (East-North-Up) 발사장 기준
      x_m: 동쪽 수평 거리 (m)
      y_m: 북쪽 수평 거리 (m) — 발사 방위각 기준
      z_m: 수직 고도 (m, 발사대 해발)

    적분 상태벡터: s⃗ = [x, y, z, vx, vy, vz, m]
    ds⃗/dt = f(s⃗, t, controls)

    추정 한계:
      - 센서 노이즈, 바람, 대기 가변성 미반영
      - pitch/yaw/roll 은 제어 명령 기반 근사
    """
    # 위치 (m)
    x_m: float = 0.0
    y_m: float = 0.0
    z_m: float = 0.0        # 고도
    # 속도 (m/s)
    vx_ms: float = 0.0
    vy_ms: float = 0.0
    vz_ms: float = 0.0
    # 질량
    total_mass_kg:       float = 0.0
    propellant_mass_kg:  float = 0.0
    # 자세 (rad) — 제어 루프 출력 기반 근사
    pitch_rad: float = math.pi / 2.0  # 초기: 수직 (90°)
    yaw_rad:   float = 0.0
    roll_rad:  float = 0.0
    # 파생 관측량
    altitude_m:   float = 0.0   # z_m 과 동일 (편의용)
    speed_ms:     float = 0.0   # |v⃗|
    downrange_m:  float = 0.0   # sqrt(x²+y²)
    mach:         float = 0.0
    dynamic_q_pa: float = 0.0
    # 미션 시계
    t_s: float = 0.0

    def __post_init__(self):
        # frozen dataclass 에서 파생 필드 초기화
        object.__setattr__(self, "altitude_m", self.z_m)
        object.__setattr__(self, "speed_ms",
                           (self.vx_ms**2 + self.vy_ms**2 + self.vz_ms**2)**0.5)
        object.__setattr__(self, "downrange_m",
                           (self.x_m**2 + self.y_m**2)**0.5)

    def as_vector(self) -> Tuple[float, ...]:
        """적분 상태벡터 [x, y, z, vx, vy, vz, m] 반환."""
        return (self.x_m, self.y_m, self.z_m,
                self.vx_ms, self.vy_ms, self.vz_ms,
                self.total_mass_kg)

    @classmethod
    def from_vector(cls, vec: Tuple[float, ...],
                    propellant_kg: float,
                    t_s: float,
                    pitch_rad: float = math.pi / 2.0,
                    yaw_rad: float = 0.0,
                    mach: float = 0.0,
                    q_pa: float = 0.0) -> "RocketState":
        """적분 결과 벡터에서 RocketState 생성."""
        x, y, z, vx, vy, vz, m = vec
        return cls(
            x_m=x, y_m=y, z_m=max(0.0, z),
            vx_ms=vx, vy_ms=vy, vz_ms=vz,
            total_mass_kg=m,
            propellant_mass_kg=propellant_kg,
            pitch_rad=pitch_rad,
            yaw_rad=yaw_rad,
            mach=mach,
            dynamic_q_pa=q_pa,
            t_s=t_s,
        )


# ── 단 사양 (설계값) ─────────────────────────────────────────────────────────

@dataclass
class StageConfig:
    """발사체 1개 단(Stage) 사양.

    모든 추력/비추력 값은 이론값이며, 실제 성능에는 편차가 있다.
    Isp_sl / Isp_vac 의 차이는 노즐 팽창비에 기인.
    """
    stage_id:           int     # 단 번호 (1부터 시작)
    dry_mass_kg:        float   # 구조 질량 (추진제 제외, kg)
    propellant_mass_kg: float   # 총 추진제 질량 (kg)
    engine_count:       int     # 엔진 수
    isp_sl_s:           float   # 해면 비추력 (s)
    isp_vac_s:          float   # 진공 비추력 (s)
    thrust_sl_n:        float   # 해면 총 추력 (N)
    thrust_vac_n:       float   # 진공 총 추력 (N)
    nozzle_exit_area_m2: float  # 노즐 출구 면적 (m²) — 고도보정 추력 계산용
    max_throttle:       float   # 최대 스로틀 [0, 1]
    min_throttle:       float   # 최소 스로틀 (연소 중 하한)
    burn_time_design_s: float   # 설계 연소 시간 (s)
    ignition_delay_s:   float = 0.5  # 점화 지연 (s)

    @property
    def total_mass_kg(self) -> float:
        return self.dry_mass_kg + self.propellant_mass_kg

    def mass_flow_kgs(self, throttle: float, physics: PhysicsConfig = DEFAULT_PHYSICS) -> float:
        """스로틀 기반 질량 유량 ṁ = F_vac / (Isp_vac × g0) × throttle."""
        return (self.thrust_vac_n / (self.isp_vac_s * physics.g0_ms2)) * throttle


# ── 발사체 전체 사양 (설계값) ────────────────────────────────────────────────

@dataclass
class VehicleConfig:
    """발사체 전체 사양 — 단 스택 + 페이로드.

    stages[0] = 1단, stages[1] = 2단, ...
    """
    vehicle_id:     str
    stages:         List[StageConfig] = field(default_factory=list)
    payload_mass_kg: float = 0.0         # 페이로드 질량 (kg)
    fairing_mass_kg: float = 0.0         # 페어링 질량 (kg)
    body_diameter_m: float = 3.66        # 기준 직경 (m)
    ref_area_m2:     float = field(init=False)  # π·(d/2)² (자동 계산)
    physics: PhysicsConfig = field(default_factory=PhysicsConfig)

    def __post_init__(self):
        self.ref_area_m2 = math.pi * (self.body_diameter_m / 2.0) ** 2

    @property
    def total_liftoff_mass_kg(self) -> float:
        return (sum(s.total_mass_kg for s in self.stages)
                + self.payload_mass_kg + self.fairing_mass_kg)

    def stage(self, idx: int) -> Optional[StageConfig]:
        return self.stages[idx] if 0 <= idx < len(self.stages) else None


# ── 비행 명령 (제어 출력) ────────────────────────────────────────────────────

@dataclass
class FlightCommand:
    """유도·제어 루프의 한 틱 출력.

    pitch_cmd_rad: 목표 피치각 (rad)
    gimbal_y_rad:  짐벌 피치축 명령 (rad, TVC)
    gimbal_z_rad:  짐벌 요축 명령 (rad, TVC)
    throttle:      스로틀 명령 [0, 1]
    request_abort: 중단 요청 (AbortMode)
    note:          진단 메시지 (선택)
    """
    pitch_cmd_rad:  float      = math.pi / 2.0
    gimbal_y_rad:   float      = 0.0
    gimbal_z_rad:   float      = 0.0
    throttle:       float      = 1.0
    request_abort:  AbortMode  = AbortMode.NONE
    note:           str        = ""


# ── 비행 건전성 판정 ─────────────────────────────────────────────────────────

@dataclass
class FlightHealth:
    """비행 건전성 종합 판정 — Ω 패턴 응용.

    Ω_flight = Ω_propulsion × Ω_structural × Ω_trajectory × Ω_range

    판정 기준:
      NOMINAL   Ω ≥ 0.80
      CAUTION   Ω ≥ 0.50
      WARNING   Ω ≥ 0.25
      ABORT     Ω < 0.25
    """
    omega:          float = 1.0
    verdict:        str   = "NOMINAL"   # NOMINAL/CAUTION/WARNING/ABORT
    alerts:         List[str] = field(default_factory=list)
    abort_required: bool  = False
    abort_mode:     AbortMode = AbortMode.NONE

    # 구성 Ω 인수
    omega_propulsion:  float = 1.0   # 추진 건전성
    omega_structural:  float = 1.0   # 동압·구조 하중
    omega_trajectory:  float = 1.0   # 궤도 편차
    omega_range:       float = 1.0   # Range Safety (IIP 복도)


# ── 텔레메트리 프레임 (기록 단위) ────────────────────────────────────────────

@dataclass(frozen=True)
class TelemetryFrame:
    """한 틱의 텔레메트리 스냅샷 — FlightChain 기록 단위.

    block_hash 는 FlightChain 에서 채워지며,
    이후 체인 연결의 prev_hash 로 사용된다.
    """
    t_s:          float
    phase:        FlightPhase
    state:        RocketState
    atm:          Optional[AtmosphereState]
    propulsion:   Optional[PropulsionState]
    aero:         Optional[AeroState]
    health:       FlightHealth
    command:      FlightCommand
    stage_idx:    int          # 현재 활성 단 인덱스
    block_hash:   str = ""     # FlightChain 이 채움

    def summary_dict(self) -> Dict:
        """JSON 직렬화 가능 요약."""
        s = self.state
        return {
            "t_s":        round(self.t_s, 3),
            "phase":      self.phase.value,
            "alt_km":     round(s.altitude_m / 1000, 3),
            "speed_ms":   round(s.speed_ms, 1),
            "mach":       round(s.mach, 3),
            "q_kpa":      round(s.dynamic_q_pa / 1000, 3),
            "mass_kg":    round(s.total_mass_kg, 1),
            "prop_kg":    round(s.propellant_mass_kg, 1),
            "downrange_km": round(s.downrange_m / 1000, 3),
            "omega":      round(self.health.omega, 4),
            "verdict":    self.health.verdict,
            "throttle":   round(self.command.throttle, 3),
            "stage":      self.stage_idx,
        }
