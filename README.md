# Rocket_Spirit

로켓의 영혼은 무엇인가.

인간이 중력을 거스르고 대기권을 가로질러 나아가려 할 때, 그 안에 담긴 것은 추력과 항력의 방정식만이 아니다. 수백만 번의 계산, 수십 년의 실패, 그리고 "이번엔 닿을 수 있다"는 궤적이다.

이 스택은 그 영혼을 코드로 쓴다. 그리고 계속 쓰여진다.

`Rocket_Spirit` 는 브랜딩 이름이다. 실제 저장소/패키지 이름은 `LaunchVehicle_Stack` / `launch_vehicle` 이다.

---

## 무엇인가

`Rocket_Spirit` 는 발사체가 지상을 떠나 궤도에 접근하기까지의 물리적 여정을 레이어로 분리한 비행 시뮬레이션 스택이다.

v0.1.1 부터는 단독 동작에 더해 전체 에코시스템과 연결된다:

```
[휠체어] → WTS 변형 → TAM (StarScream) 호버 판정 → Rocket_Spirit 발사 인가 → 궤도
```

---

## 에코시스템 연결 (v0.1.1)

| 시스템 | 역할 | 연결 형태 |
|--------|------|----------|
| **TAM (StarScream)** | 지상→호버 readiness → 발사 인가 | `bridges/tam_bridge.py` |
| **Air Jordan** | 저고도(< 20 km) 공력 보조 판정 | `bridges/air_jordan_bridge.py` |
| **AgedCare_Stack** | 유인 발사(MVP-4) 탑승자 안전 게이트 | `adapters/aged_care_adapter.py` |
| **VPF** | 발사대 지상 물리 | 독립 참조 |
| **Wheelchair_Transform** | 변형 시퀀스 | TAM을 통해 간접 연결 |

### TAM → 발사 인가 흐름

```python
from launch_vehicle import LaunchAgent, optional_tam_launch_readiness

# TAM FlightReadinessReport (StarScream)을 발사 인가로 변환
readiness = optional_tam_launch_readiness(tam_report, mode="HOVER")

agent = LaunchAgent(vehicle, tam_readiness=readiness)
if agent.command_go_from_tam(tam_report, "HOVER"):
    # HOVER + Ω_total ≥ 0.80 + blockers 없음 → 발사 인가
    while agent.phase.value not in ("nominal", "abort"):
        frame = agent.tick()
```

TAM 발사 인가 조건:
- `mode` ∈ `{HOVER, FLIGHT_CRUISE}`
- `omega_total ≥ 0.80`
- `takeoff_possible = True`
- `verdict` ∈ `{HEALTHY, STABLE}`
- `blockers = []`

### 유인 발사 MVP-4

```python
from launch_vehicle import AgedCareLaunchSafety

crew = AgedCareLaunchSafety(
    omega_care=0.90,
    verdict="SAFE",
    emergency_triggered=False,
    manual_override=False,
)
agent = LaunchAgent(vehicle, crew_safety=crew, human_rated_mvp4=True)
# TAM 인가 + crew verdict=="SAFE" + omega_care>=0.80 → 유인 발사 허가
```

---

## 비행 여정

`T-0` 에서 페이로드가 궤도에 접근하기까지:

```text
[에코시스템] TAM HOVER 인가
        |
        v
발사대 (HOLD)
        |
        v
카운트다운 (COUNTDOWN)  --  T-10 ~ T-0
        |
        v 점화
이륙 (LIFTOFF)  --  탑 클리어, 1단 최대 추력
        |              [Air Jordan 공력 보조: 저고도]
        v
상승 (ASCENDING)  --  중력 선회 시작, 피치 킥
        |
        v 동압 최고점
Max-Q (MAX_Q)  --  스로틀 감소, 구조 하중 관리
        |              [Air Jordan 종료: 고도 > 20 km]
        v
1단 연소 종료 (MECO)
        |
        v 코스트
단분리 (STAGE_SEP)
        |
        v 2단 점화
2단 연소 (UPPER_BURN)
        |
        v
자유 비행 (COAST)
        |
        v 근지점
궤도 삽입 (ORBIT_INSERT)
        |
        v
임무 완료 (NOMINAL)

(모든 단계) ----> ABORT
```

---

## 레이어 구조

```text
 에코시스템 브리지
  TAM · Air Jordan · AgedCare · VPF
        |
        v
Layer 4 -- 임무 안전
  Range Safety · Abort System · FlightChain (SHA-256)
Layer 3 -- 유도·제어
  Gravity Turn · TVC
Layer 2 -- 비행 단계 관리
  FlightPhase FSM (11 단계) · StagingManager
Layer 1 -- 물리 엔진
  AtmosphereEngine · PropulsionEngine · AerodynamicsEngine · VariableMassIntegrator
Layer 0 -- 데이터 계약
  RocketState · StageConfig · TelemetryFrame

              |
          LaunchAgent
```

---

## 핵심 물리

가변 질량 동역학:

```text
s = [x, y, z, vx, vy, vz, m]
dv/dt = (F_thrust + F_drag) / m - g(z) * ẑ
dm/dt = -ṁ
```

- 적분: 4차 룽게-쿠타 (RK4), `dt = 0.1 s`
- 현재 모델: 3D 질점 병진 + 질량 소모
- 범위 밖: 강체 회전 (6-DoF)

추력 고도 보정:

```text
F(h) = throttle × (F_vac − P_atm(h) × A_exit)
```

대기 모델: US Standard Atmosphere 1976, 0 ~ 86 km

항력:

```text
D = ½ · ρ · v² · Cd(M) · A_ref
```

중력:

```text
g(h) = g₀ × (Rₑ / (Rₑ + h))²
```

---

## 비행 건전성 Ω

```text
Ω_rocket = Ω_propulsion × Ω_structural × Ω_trajectory × Ω_range [× Ω_aero]
```

| 컴포넌트 | 설명 |
|---------|------|
| **Ω_propulsion** | 추진제 잔량 비율 |
| **Ω_structural** | 동압 안전 마진 (q < 80 kPa) |
| **Ω_trajectory** | 속도/자세 발산 정도 |
| **Ω_range** | Range Safety 복도 상태 |
| **Ω_aero** | Air Jordan 공력 보조 (저고도, 선택적) |

판정 임계:

| 판정 | Ω 범위 | 의미 |
|------|--------|------|
| `NOMINAL` | ≥ 0.80 | 정상 |
| `CAUTION` | ≥ 0.50 | 주의 |
| `WARNING` | ≥ 0.25 | 경고 |
| `ABORT` | < 0.25 | 중단 |

---

## FlightChain (SHA-256 비행 감사)

모든 텔레메트리는 SHA-256 연결 해시로 기록된다:

```text
H_i = SHA-256(i | t_s | payload_json | H_{i-1})
H_0 = SHA-256("GENESIS|LaunchVehicle|{vehicle_id}")
```

기록 대상: 비행 단계, 위치, 속도, 마하수, 동압, Ω, 스로틀,
단분리·MECO·ABORT·TAM Go 이벤트

```python
assert agent.chain.verify_integrity()
agent.chain.export_json("flight_log.json")
print(agent.chain.summary())
```

---

## 빠른 시작

### 무인 독립 발사

```python
from launch_vehicle import LaunchAgent, StageConfig, VehicleConfig

stage1 = StageConfig(
    stage_id=1, dry_mass_kg=25_000, propellant_mass_kg=400_000,
    engine_count=9, isp_sl_s=282, isp_vac_s=311,
    thrust_sl_n=7_607_000, thrust_vac_n=8_227_000,
    nozzle_exit_area_m2=11.5, max_throttle=1.0, min_throttle=0.57,
    burn_time_design_s=162,
)
vehicle = VehicleConfig(vehicle_id="RST-001", stages=[stage1],
                        payload_mass_kg=22_800, fairing_mass_kg=1_900,
                        body_diameter_m=3.66)

agent = LaunchAgent(vehicle, dt_s=0.1)
agent.command_go()

while agent.phase.value not in ("nominal", "abort"):
    frame = agent.tick()
    print(f"[{frame.t_s:7.1f}s] {frame.phase.value:12s} "
          f"h={frame.state.altitude_m/1000:7.1f}km  "
          f"Ω={frame.health.omega:.3f}")

print(agent.chain.summary())
```

### TAM 연동 발사

```python
from launch_vehicle import LaunchAgent, VehicleConfig, optional_tam_launch_readiness

# TAM FlightReadinessReport (StarScream) → 발사 인가
readiness = optional_tam_launch_readiness(tam_report, mode="HOVER")
agent = LaunchAgent(vehicle, tam_readiness=readiness)

if agent.command_go_from_tam(tam_report, "HOVER"):
    print(f"발사 인가: TAM Ω={readiness.tam_omega_total:.3f}")
    # ... 발사 루프
```

---

## 파일 구조

```text
LaunchVehicle_Stack/
├── launch_vehicle/
│   ├── __init__.py              # Public API (v0.1.1)
│   ├── launch_agent.py          # 통합 오케스트레이터
│   ├── contracts/schemas.py     # 데이터 계약 (Layer 0)
│   ├── physics/                 # 물리 엔진 (Layer 1)
│   │   ├── atmosphere.py
│   │   ├── propulsion.py
│   │   ├── aero.py
│   │   └── integrator.py
│   ├── flight/                  # 비행 단계 관리 (Layer 2)
│   │   ├── phase_fsm.py
│   │   └── staging.py
│   ├── guidance/                # 유도·제어 (Layer 3)
│   │   ├── gravity_turn.py
│   │   └── tvc.py
│   ├── safety/                  # 안전·감사 (Layer 4)
│   │   ├── range_safety.py
│   │   ├── abort_system.py
│   │   └── (audit/flight_chain.py)
│   ├── bridges/                 # 에코시스템 브리지
│   │   ├── tam_bridge.py        ← TAM (StarScream)
│   │   └── air_jordan_bridge.py ← Air Jordan
│   └── adapters/                # 시스템 어댑터
│       └── aged_care_adapter.py ← AgedCare_Stack (MVP-4)
├── tests/
│   ├── test_launch_vehicle.py   # §1~§13 (112 테스트)
│   ├── test_ecosystem_bridges.py # 에코시스템 브리지 (31 테스트)
│   └── test_public_api.py
├── scripts/
│   ├── regenerate_signature.py
│   ├── verify_signature.py
│   ├── cleanup_generated.py
│   └── release_check.py
├── VERSION, CHANGELOG.md, BLOCKCHAIN_INFO.md
└── pyproject.toml
```

---

## 테스트

```bash
cd LaunchVehicle_Stack
python -m pytest tests/ -v
```

현재 결과: **145 passed**

| 섹션 | 내용 |
|------|------|
| §1 | 데이터 계약 |
| §2 | 대기 모델 |
| §3 | 추진 모델 |
| §4 | 공력 모델 |
| §5 | 가변 질량 RK4 적분기 |
| §6 | 비행 단계 FSM |
| §7 | 단분리 관리자 |
| §8 | Gravity Turn 유도 |
| §9 | TVC 제어기 |
| §10 | Range Safety |
| §11 | 중단 시스템 |
| §12 | FlightChain |
| §13 | LaunchAgent 통합 |
| §14 | 에코시스템 브리지 (TAM, Air Jordan, AgedCare) |

---

## 현재 범위 및 확장 경로

| 영역 | 현재 | 다음 확장 |
|------|------|---------|
| 대기 | US Std Atm 1976, 0~86 km | NRLMSISE-00, 풍속 |
| 추진 | 결정론적 | 엔진 아웃, 노이즈 |
| 공력 | 근사 Cd(M) | Cd(M,α) 테이블 |
| 동역학 | 3D 질점 | 6-DoF 강체 |
| IIP | 탄도 근사 | 몬테카를로 분포 |
| 지구 | 구형, 자전 없음 | 자전·코리올리 |
| 연결 | TAM, Air Jordan, AgedCare | Lucifer_Engine 궤도 |

---

## 로드맵

- `v0.1.1`: 현재 — 에코시스템 연결 (TAM, Air Jordan, AgedCare)
- `v0.2.0`: `Lucifer_Engine` 궤도 전파 연동 (MECO 이후 궤적)
- `v0.3.0`: PEG 정밀 유도 + 몬테카를로
- `v0.4.0`: 재사용 발사체 귀환 (powered descent)
- `v0.5.0`: 다중 발사체 / 성좌 배치
- 이후: 6-DoF, 바람, LES, 자전 지구

---

## 설계 철학

**상태는 추정이다.** 핵심 상태 객체는 `frozen=True` 불변 데이터클래스다. `measured` 대신 `estimated`, `actual thrust` 대신 `modeled thrust`.

**하드코딩은 없다.** 물리 상수와 설계값은 `*Config` 로 분리. `PhysicsConfig` 교체 시 다른 행성 환경 확장 가능.

**연결은 선택적이다.** 모든 브리지는 `ImportError` 폴백 — TAM, Air Jordan, AgedCare 없이도 독립 동작.

**한 틱이 하나의 물리 시간스텝.** `LaunchAgent.tick()` 은 11단계 파이프라인을 원자적으로 실행한다.

---

## 연결 생태계

```text
Wheelchair_Transform_System
        ↓ (변형·잠금 스냅샷)
TAM / StarScream                 ← 지상→호버 오케스트레이션
        ↓ (FlightReadinessReport, Ω ≥ 0.80, HOVER)
Rocket_Spirit                    ← 발사 인가 → 상승 → 궤도
        ├── Air_Jordan            ← 저고도 공력 보조
        ├── AgedCare_Stack        ← 유인 MVP-4 게이트
        └── Lucifer_Engine (예정) ← MECO 이후 궤도 전파
```

---

## 한계

이 소프트웨어는 연구·교육 목적의 시뮬레이션이다. 실제 발사 운용, 항공우주 인증, 안전 시스템 용도로 사용할 수 없다.

> 중력은 규칙이다. 그리고 규칙은 궤적으로 이해된다.
> 이번엔, 지면을 떠난다.
