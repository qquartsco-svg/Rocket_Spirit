# 🚀 Rocket_Sprit

> **로켓의 영혼은 무엇인가.**
>
> 인간이 중력을 거스르고 대기권을 가로질러 나아가려 할 때,
> 그 안에 담긴 것은 추력과 항력의 방정식만이 아니다.
> 수백만 번의 계산, 수십 년의 실패, 그리고
> "이번엔 닿을 수 있다"는 궤적이다.
>
> 이 스택은 그 영혼을 코드로 쓴다.
> 그리고 계속 쓰여진다.

---

## 이 시스템이 무엇인가

**Rocket_Sprit** 는 발사체가 지상을 떠나 궤도에 접근하기까지의
물리적 여정을 레이어로 분리한 비행 시뮬레이션 스택이다.

대기를 가로지르는 것, 질량을 태워 속도로 변환하는 것,
단을 분리하고, 중력을 선회하고, 모든 순간을 서명된 기록으로 남기는 것 —
이 과정들이 하나의 일관된 구조 안에서 흐른다.

이 스택은 v0.1.0에서 시작하여 궤도 전파, 귀환 연소, 성좌 배치로
계속 확장되어 나가는 시스템이다.

```
T-0 에서 페이로드가 궤도에 접근하기까지:

  발사대 (HOLD)
     │
     ▼
  카운트다운 (COUNTDOWN) ── T-10 ~ T-0
     │
     ▼ 점화
  이륙 (LIFTOFF) ── 탑 클리어, 1단 최대 추력
     │
     ▼
  상승 (ASCENDING) ── 중력 선회 시작, 피치 킥
     │
     ▼ 동압 최고점
  Max-Q (MAX_Q) ── 스로틀 감소, 구조 하중 관리
     │
     ▼
  1단 연소 종료 (MECO) ── Main Engine Cut-Off
     │
     ▼ 코스트
  단분리 (STAGE_SEP) ── 1단 사출, 질량 감소
     │
     ▼ 2단 점화
  2단 연소 (UPPER_BURN) ── 진공 비추력, 고효율
     │
     ▼
  자유 비행 (COAST) ── 추력 없음, 관성
     │
     ▼ 근지점
  궤도 삽입 (ORBIT_INSERT) ── 원형화 연소
     │
     ▼
  임무 완료 (NOMINAL) ── 페이로드 분리

  (모든 단계) ──→  ABORT ── 범위 안전·구조 한계·Ω 임계
```

---

## 왜 레이어인가

로켓은 하나의 덩어리가 아니다.
물리, 제어, 안전, 기록 — 각자의 언어로 흐르며,
하나가 실패해도 나머지가 살아있어야 한다.

```
┌─────────────────────────────────────────────────────┐
│  Layer 4 — 임무 안전                                 │
│  Range Safety · Abort System · FlightChain(SHA-256) │
├─────────────────────────────────────────────────────┤
│  Layer 3 — 유도·제어                                 │
│  Gravity Turn · TVC (추력 벡터 제어)                │
├─────────────────────────────────────────────────────┤
│  Layer 2 — 비행 단계 관리                            │
│  FlightPhase FSM · StagingManager                   │
├─────────────────────────────────────────────────────┤
│  Layer 1 — 물리 엔진                                 │
│  AtmosphereEngine · PropulsionEngine                │
│  AerodynamicsEngine · VariableMassIntegrator (RK4)  │
├─────────────────────────────────────────────────────┤
│  Layer 0 — 데이터 계약                               │
│  RocketState · StageConfig · TelemetryFrame         │
└─────────────────────────────────────────────────────┘
           │
      LaunchAgent
  (전체 파이프라인 오케스트레이터)
```

---

## 핵심 물리

### 가변 질량 동역학 (Tsiolkovsky)

```
상태벡터:  s⃗ = [x, y, z, vx, vy, vz, m]
           3D 질점 병진 운동 + 질량 소모 (7-state)

운동 방정식:
  dv⃗/dt = (F⃗_thrust + F⃗_drag) / m − g(z)·ẑ
  dm/dt  = −ṁ

적분: 4차 룽게-쿠타 (RK4), dt = 0.1 s
```

> 이 모델은 3D 질점 병진 동역학으로 관측된다.
> 강체 회전(6-DoF)은 현재 범위 밖이며,
> 자세 명령(pitch/yaw)은 추력 방향 입력으로 처리된다.

### 추력 (고도 보정)

```
F(h) = throttle × (F_vac − P_atm(h) × A_exit)

해면: 배압 손실 최대 → 추력 감소
진공: 손실 없음   → 추력 최대
```

### 대기 모델 (US Standard Atmosphere 1976)

```
7개 레이어, 0 ~ 86 km

ρ(h) → 항력   D = ½ρv²·Cd(M)·A_ref
T(h) → 음속   a = √(γRT)  →  Mach
g(h) = g₀·(R_e / (R_e + h))²
```

> 86 km 이상은 표준 대기 상한 경계값이 유지된다.
> IIP(순간 충격점)는 탄도 근사로 추정되며,
> 정밀 범위 안전 연산과는 구분된다.

### 비행 건전성 Ω

```
Ω_flight = Ω_추진 × Ω_구조 × Ω_궤도 × Ω_범위

NOMINAL  Ω ≥ 0.80
CAUTION  Ω ≥ 0.50
WARNING  Ω ≥ 0.25
ABORT    Ω < 0.25  →  AbortSystem 발동
```

---

## FlightChain — 비행 기록 서명

모든 텔레메트리는 SHA-256 연결 해시로 서명된다.

```
블록 구조:
  H_i = SHA-256( i ‖ t_s ‖ payload_json ‖ H_{i-1} )
  Genesis: H_0 = SHA-256("GENESIS|LaunchVehicle|{vehicle_id}")

기록 대상 (매 N틱 + 이벤트 즉시):
  - 미션 시각 t_s
  - 비행 단계 (FlightPhase)
  - 위치·속도·마하수·동압
  - 건전성 Ω·verdict
  - 스로틀 명령

위변조 탐지:
  ∀ i: block_i.hash == recompute(block_i)
  ∀ i>0: block_i.prev_hash == block_{i-1}.hash
```

MECO, 단분리, ABORT 등 핵심 이벤트는
주기와 무관하게 즉시 체인에 기록된다.

```python
# 비행 후 무결성 검증
assert agent.chain.verify_integrity()
agent.chain.export_json("flight_log.json")
print(agent.chain.summary())
# [FlightChain] RST-001
#   블록수: 42 | 무결성: ✓ INTACT
#   Genesis: a3f9b2c1d4e7…
#   Head:    7f2a1b9c3d0e…
```

> FlightChain은 SYD_DRIFT CommandChain 패턴을
> 발사체 텔레메트리용으로 재구현한 것이다.
> Python 수준 불변성 보장이며,
> OS 파일 수준 보호와 실시간 다운링크 암호화는 별도 영역이다.

---

## 빠른 시작

```python
from launch_vehicle.contracts.schemas import StageConfig, VehicleConfig
from launch_vehicle.launch_agent import LaunchAgent

# 1단 정의
stage1 = StageConfig(
    stage_id=1,
    dry_mass_kg=25_000,
    propellant_mass_kg=400_000,
    engine_count=9,
    isp_sl_s=282, isp_vac_s=311,
    thrust_sl_n=7_607_000, thrust_vac_n=8_227_000,
    nozzle_exit_area_m2=11.5,
    max_throttle=1.0, min_throttle=0.57,
    burn_time_design_s=162,
)

# 2단 정의
stage2 = StageConfig(
    stage_id=2,
    dry_mass_kg=4_000,
    propellant_mass_kg=92_670,
    engine_count=1,
    isp_sl_s=340, isp_vac_s=348,
    thrust_sl_n=934_000, thrust_vac_n=934_000,
    nozzle_exit_area_m2=1.12,
    max_throttle=1.0, min_throttle=1.0,
    burn_time_design_s=397,
)

vehicle = VehicleConfig(
    vehicle_id="RST-001",
    stages=[stage1, stage2],
    payload_mass_kg=22_800,
    fairing_mass_kg=1_900,
    body_diameter_m=3.66,
)

# 발사
agent = LaunchAgent(vehicle, dt_s=0.1)
agent.command_go()

while agent.phase.value not in ("nominal", "abort"):
    frame = agent.tick()
    print(f"[{frame.t_s:7.1f}s] {frame.phase.value:12s} "
          f"h={frame.state.altitude_m/1000:7.1f}km  "
          f"v={frame.state.speed_ms:6.0f}m/s  "
          f"M={frame.state.mach:.2f}  "
          f"Ω={frame.health.omega:.3f}")

print(agent.chain.summary())
```

```
[    0.1s] countdown    h=    0.0km  v=     0m/s  M=0.00  Ω=1.000
[   10.1s] liftoff      h=    0.0km  v=     0m/s  M=0.00  Ω=1.000
[   10.2s] ascending    h=    0.0km  v=    15m/s  M=0.04  Ω=1.000
[   30.0s] ascending    h=    4.2km  v=   312m/s  M=0.92  Ω=1.000
[   55.0s] max_q        h=   13.1km  v=   520m/s  M=1.62  Ω=0.950
...
```

---

## 아키텍처 철학

### "상태는 추정이다"

핵심 상태 객체(`RocketState`, `AtmosphereState`, `PropulsionState`, `AeroState`)는
`frozen=True` 불변 데이터클래스로 설계되어 있다.
설정 파라미터는 별도 `*Config` 클래스로 분리된다.

물리 관측값은 그라운드 트루스가 아니다 — 모델의 추정값으로 흐른다.
그래서 언어도 그에 맞게 쓴다:

- `measured` X → `estimated` / `observed` ✓
- `actual thrust` X → `modeled thrust` ✓
- `real atmosphere` X → `standard atmosphere approximation` ✓

### "하드코딩은 없다"

모든 물리 파라미터는 `*Config` 데이터클래스로 분리된다.
`PhysicsConfig`의 중력·대기 파라미터를 교체하면
다른 행성 환경으로의 확장 경로가 열린다.
`VehicleAeroConfig`에 실측 Cd 테이블을 넣으면 모델 정밀도가 달라진다.

### "연결은 선택적이다"

```python
# 궤도 전파기 (MECO 이후) — 미설치 시 없어도 작동
try:
    from lucifer_engine import OrbitalPropagator
    ...
except ImportError:
    pass  # 궤도 단계는 trajectory 출력만

# 자율 비행 스택 — 선택 연동
try:
    from autonomy_runtime_stack import ...
```

---

## 파일 구조

```
Rocket_Sprit/
├── launch_vehicle/
│   ├── contracts/
│   │   └── schemas.py          # Layer 0: 전체 데이터 계약
│   ├── physics/
│   │   ├── atmosphere.py       # US Std Atm 1976 (0~86 km)
│   │   ├── propulsion.py       # 추력·Isp·질량유량 (고도 보정)
│   │   ├── aero.py             # 항력·Cd(M) 조각선형 보간
│   │   └── integrator.py       # 가변질량 RK4 (7-state)
│   ├── flight/
│   │   ├── phase_fsm.py        # 11단계 비행 FSM
│   │   └── staging.py          # 단분리·페어링 관리
│   ├── guidance/
│   │   ├── gravity_turn.py     # 중력 선회 유도 (3단계 피치 프로그램)
│   │   └── tvc.py              # 추력 벡터 제어 (PD)
│   ├── safety/
│   │   ├── range_safety.py     # IIP 탄도 추정·비행 복도 감시
│   │   └── abort_system.py     # 4트리거 × 3중단 모드
│   ├── audit/
│   │   └── flight_chain.py     # SHA-256 비행 기록 체인
│   └── launch_agent.py         # 전체 오케스트레이터
└── tests/
    └── test_launch_vehicle.py  # 112 케이스 (§1~§13)
```

---

## 테스트

```bash
cd Rocket_Sprit
python -m pytest tests/ -v
# 112 passed in 0.15s
```

각 섹션은 수치 계약·상태 전이·통합 tick 검증 중심으로 구성된다:

- §1  데이터 계약 (12 케이스)
- §2  대기 모델 (13 케이스)
- §3  추진 모델 (10 케이스)
- §4  공력 모델 (7 케이스)
- §5  가변 질량 RK4 적분기 (8 케이스)
- §6  비행 단계 FSM (10 케이스)
- §7  단분리 관리자 (9 케이스)
- §8  Gravity Turn 유도 (6 케이스)
- §9  TVC 제어기 (4 케이스)
- §10 Range Safety (4 케이스)
- §11 중단 시스템 (7 케이스)
- §12 FlightChain (9 케이스)
- §13 LaunchAgent 통합 (13 케이스)

---

## 현재 모델 범위

이 스택이 현재 다루지 않는 영역:

| 미포함 영역 | 현재 모델 범위 |
|-------------|----------------|
| 연소 불안정·추력 편차 | 결정론적 모델 — 확률적 노이즈 없음 |
| 지구 자전·코리올리 | 3D 질점 병진 (강체 6-DoF 아님) |
| 실측 Cd 데이터 | 근사 조각선형 곡선 (CFD·풍동 외) |
| 정밀 IIP 계산 | 탄도 추정 (대기 재진입·파편 제외) |
| LES (유인 탈출 시스템) | 무인 발사체 기준 설계 |
| 86 km 이상 대기 | 표준 대기 상한 → 경계값 유지 |

이 범위는 로드맵을 따라 단계적으로 확장된다.

---

## 확장 로드맵

```
v0.1.0 ✅  현재 — 5레이어 기반 구조, 2단 상승 비행 시뮬레이션
v0.2.0     궤도 삽입 후 Lucifer_Engine 연동 (N-body 전파)
v0.3.0     닫힌 루프 유도 (PEG 계열, 최적 제어)
v0.4.0     재사용 발사체 — 귀환 연소·착지 제어
v0.5.0     다중 발사체 시나리오 (성좌 배치 시뮬레이션)
```

---

## 연결 생태계

```
Rocket_Sprit
    │
    ├── Lucifer_Engine         ← 궤도 전파 (MECO 이후 N-body 추적)
    ├── SYD_DRIFT              ← SHA-256 감사 체인 (FlightChain 원형)
    ├── Autonomy_Runtime_Stack ← 제어 루프 패턴 (Tick/FSM/계약)
    └── AgedCare_Stack         ← 영혼-바디 철학 공유
                                  (AI = 영혼, 플랫폼 = 바디)
```

---

⚠️ **연구·교육용 시뮬레이션**
이 소프트웨어는 연구·교육 목적의 시뮬레이션으로 설계되어 있다.
실제 발사 운용, 항공우주 인증, 안전 시스템 용도로 사용될 수 없다.

---

*"중력은 규칙이다. 그리고 규칙은 궤적으로 이해된다."*
