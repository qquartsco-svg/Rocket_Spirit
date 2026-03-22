# 🚀 Rocket_Sprit

> **로켓의 영혼은 무엇인가.**
>
> 인간이 중력을 거스르고 대기권을 찢고 나가려 할 때,
> 그 안에 담긴 것은 추력과 항력의 방정식이 아니다.
> 수백만 번의 계산, 수십 년의 실패, 그리고
> "이번엔 닿을 수 있다"는 믿음이다.
>
> 이 스택은 그 영혼을 코드로 쓴다.

---

## 이 시스템이 무엇인가

**Rocket_Sprit** 는 발사체가 지상을 떠나 궤도에 닿기까지의
물리적 여정을 레이어로 분리한 비행 시뮬레이션 스택이다.

대기를 뚫는 것, 질량을 태워 속도로 바꾸는 것,
단을 분리하고, 중력을 선회하고, 안전하게 기록으로 남기는 것 —
이 모든 과정이 하나의 일관된 구조 안에 놓인다.

```
T-0 에서 페이로드가 궤도에 오르기까지:

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
  Max-Q (MAX_Q) ── 스로틀 감소, 구조 하중 제어
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
물리, 제어, 안전, 기록 — 각자의 언어로 작동하며,
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

운동 방정식:
  dv⃗/dt = (F⃗_thrust + F⃗_drag) / m − g(z)·ẑ
  dm/dt  = −ṁ

적분: 4차 룽게-쿠타 (RK4), dt = 0.1 s
```

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

### 비행 건전성 Ω

```
Ω_flight = Ω_추진 × Ω_구조 × Ω_궤도 × Ω_범위

NOMINAL  Ω ≥ 0.80
CAUTION  Ω ≥ 0.50
WARNING  Ω ≥ 0.25
ABORT    Ω < 0.25  →  AbortSystem 발동
```

---

## 빠른 시작

```python
from launch_vehicle.contracts.schemas import StageConfig, VehicleConfig
from launch_vehicle.flight.phase_fsm import FlightFSMConfig
from launch_vehicle.launch_agent import LaunchAgent

# 발사체 정의
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

vehicle = VehicleConfig(
    vehicle_id="RST-001",
    stages=[stage1],
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

이 시스템의 모든 상태(`RocketState`, `AtmosphereState` 등)는
`frozen=True` 불변 데이터클래스다.

물리 관측값은 그라운드 트루스가 아니다 — 모델의 추정값이다.
그래서 언어도 그에 맞게 쓴다:

- `measured` X → `estimated` / `observed` ✓
- `actual thrust` X → `modeled thrust` ✓
- `real atmosphere` X → `standard atmosphere approximation` ✓

### "하드코딩은 없다"

모든 물리 파라미터는 `*Config` 데이터클래스로 분리된다.
`PhysicsConfig`를 바꾸면 달 환경도 시뮬레이션할 수 있다.
`VehicleAeroConfig`에 실측 Cd 테이블을 넣으면 정밀도가 올라간다.

### "연결은 선택적이다"

```
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
│   │   ├── range_safety.py     # IIP 추정·비행 복도 감시
│   │   └── abort_system.py     # 4트리거 × 3중단 모드
│   ├── audit/
│   │   └── flight_chain.py     # SHA-256 블랙박스
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

커버리지:
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

## 확장 로드맵

```
v0.1.0 ✅  현재 — 5레이어 기반 구조, 2단 시뮬레이션
v0.2.0     궤도 삽입 후 Lucifer_Engine 연동 (N-body 전파)
v0.3.0     닫힌 루프 유도 (PEG 계열, 최적 제어)
v0.4.0     재사용 발사체 — 귀환 연소·착지 제어
v0.5.0     다중 발사체 시나리오 (성좌 배치 시뮬레이션)
```

---

## 설계 한계 (솔직한 고백)

이 시스템이 모델링하지 않는 것:

| 미포함 | 이유 |
|--------|------|
| 연소 불안정·추력 편차 | 확정적 모델 — 노이즈 없음 |
| 지구 자전·코리올리 | 3-DoF 질점 (6-DoF 아님) |
| 실제 Cd 데이터 | CFD·풍동 데이터 없음, 근사 곡선 |
| 정밀 IIP | 탄도 근사 (대기·파편 미포함) |
| LES (유인 탈출) | 무인 발사체 기준 MVP |
| 86 km 이상 대기 | 표준 대기 상한 → 경계값 유지 |

이 모든 한계는 알고 있기 때문에 적혀 있다.
모르는 것이 더 위험하다.

---

## 연결 생태계

```
Rocket_Sprit
    │
    ├── Lucifer_Engine      ← 궤도 전파 (MECO 이후 추적)
    ├── SYD_DRIFT           ← SHA-256 감사 체인 (FlightChain 원형)
    ├── Autonomy_Runtime_Stack ← 제어 루프 패턴 (Tick/FSM/계약)
    └── AgedCare_Stack      ← 영혼-바디 철학 공유
                               (AI = 영혼, 플랫폼 = 바디)
```

---

⚠️ **프로토타입 공개**
이 소프트웨어는 연구·교육 목적의 시뮬레이션이다.
실제 발사 운용, 항공우주 인증, 안전 시스템으로 사용할 수 없다.

---

*"중력은 규칙이다. 하지만 규칙은 이해하면 넘을 수 있다."*
