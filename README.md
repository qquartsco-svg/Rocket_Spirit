# Rocket_Spirit

로켓의 영혼은 무엇인가.

인간이 중력을 거스르고 대기권을 가로질러 나아가려 할 때, 그 안에 담긴 것은 추력과 항력의 방정식만이 아니다. 수백만 번의 계산, 수십 년의 실패, 그리고 "이번엔 닿을 수 있다"는 궤적이다.

이 스택은 그 영혼을 코드로 쓴다. 그리고 계속 쓰여진다.

`Rocket_Spirit` 는 브랜딩 이름이다. 실제 저장소/패키지 이름은 `LaunchVehicle_Stack` / `launch_vehicle` 이다.

## 무엇인가

`Rocket_Spirit` 는 발사체가 지상을 떠나 궤도에 접근하기까지의 물리적 여정을 레이어로 분리한 비행 시뮬레이션 스택이다.

대기를 가로지르는 것, 질량을 태워 속도로 변환하는 것, 단을 분리하고, 중력을 선회하고, 모든 순간을 서명된 기록으로 남기는 것. 이 과정들이 하나의 일관된 구조 안에서 흐른다.

이 스택은 `v0.1.0` 에서 시작해 궤도 전파, 귀환 연소, 성좌 배치로 계속 확장되어 나가는 시스템이다.

## 비행 여정

`T-0` 에서 페이로드가 궤도에 접근하기까지:

```text
발사대 (HOLD)
   |
   v
카운트다운 (COUNTDOWN) -- T-10 ~ T-0
   |
   v 점화
이륙 (LIFTOFF) -- 탑 클리어, 1단 최대 추력
   |
   v
상승 (ASCENDING) -- 중력 선회 시작, 피치 킥
   |
   v 동압 최고점
Max-Q (MAX_Q) -- 스로틀 감소, 구조 하중 관리
   |
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

## 왜 레이어인가

로켓은 하나의 덩어리가 아니다. 물리, 제어, 안전, 기록은 각자의 언어로 흐르며, 하나가 실패해도 나머지가 살아있어야 한다.

```text
Layer 4 -- 임무 안전
  Range Safety · Abort System · FlightChain(SHA-256)
Layer 3 -- 유도·제어
  Gravity Turn · TVC
Layer 2 -- 비행 단계 관리
  FlightPhase FSM · StagingManager
Layer 1 -- 물리 엔진
  AtmosphereEngine · PropulsionEngine · AerodynamicsEngine · VariableMassIntegrator
Layer 0 -- 데이터 계약
  RocketState · StageConfig · TelemetryFrame

                |
            LaunchAgent
```

## 핵심 물리

가변 질량 동역학:

```text
s = [x, y, z, vx, vy, vz, m]
dv/dt = (F_thrust + F_drag) / m - g(z) * z_hat
dm/dt = -m_dot
```

- 적분: 4차 룽게-쿠타(`RK4`)
- 기본 시간 스텝: `dt = 0.1 s`
- 현재 모델: `3D 질점 병진 + 질량 소모`
- 범위 밖: 강체 회전(`6-DoF`)

추력의 고도 보정:

```text
F(h) = throttle * (F_vac - P_atm(h) * A_exit)
```

대기 모델:

- US Standard Atmosphere 1976
- 0 ~ 86 km
- 86 km 이상은 경계값 유지

항력:

```text
D = 1/2 * rho * v^2 * Cd(M) * A_ref
```

중력:

```text
g(h) = g0 * (Re / (Re + h))^2
```

IIP 는 탄도 근사로 추정되며, 정밀 범위 안전 연산과는 구분된다.

## 비행 건전성 Ω

```text
Omega_flight = Omega_propulsion
             * Omega_structural
             * Omega_trajectory
             * Omega_range
```

- `NOMINAL`: `Ω >= 0.80`
- `CAUTION`: `Ω >= 0.50`
- `WARNING`: `Ω >= 0.25`
- `ABORT`: `Ω < 0.25`

## FlightChain

모든 텔레메트리는 SHA-256 연결 해시로 기록된다.

```text
H_i = SHA-256(i | t_s | payload_json | H_{i-1})
H_0 = SHA-256("GENESIS|LaunchVehicle|{vehicle_id}")
```

기록 대상:

- 비행 단계
- 위치, 속도, 마하수, 동압
- 건전성 `Ω`
- 스로틀 명령
- 단분리, MECO, ABORT 같은 중요 이벤트

```python
assert agent.chain.verify_integrity()
agent.chain.export_json("flight_log.json")
print(agent.chain.summary())
```

`FlightChain` 은 `SYD_DRIFT` 의 `CommandChain` 패턴을 발사체 텔레메트리용으로 재구현한 것이다.

## 빠른 시작

```python
from launch_vehicle import LaunchAgent, StageConfig, VehicleConfig

stage1 = StageConfig(
    stage_id=1,
    dry_mass_kg=25_000,
    propellant_mass_kg=400_000,
    engine_count=9,
    isp_sl_s=282,
    isp_vac_s=311,
    thrust_sl_n=7_607_000,
    thrust_vac_n=8_227_000,
    nozzle_exit_area_m2=11.5,
    max_throttle=1.0,
    min_throttle=0.57,
    burn_time_design_s=162,
)

stage2 = StageConfig(
    stage_id=2,
    dry_mass_kg=4_000,
    propellant_mass_kg=92_670,
    engine_count=1,
    isp_sl_s=340,
    isp_vac_s=348,
    thrust_sl_n=934_000,
    thrust_vac_n=934_000,
    nozzle_exit_area_m2=1.12,
    max_throttle=1.0,
    min_throttle=1.0,
    burn_time_design_s=397,
)

vehicle = VehicleConfig(
    vehicle_id="RST-001",
    stages=[stage1, stage2],
    payload_mass_kg=22_800,
    fairing_mass_kg=1_900,
    body_diameter_m=3.66,
)

agent = LaunchAgent(vehicle, dt_s=0.1)
agent.command_go()

while agent.phase.value not in ("nominal", "abort"):
    frame = agent.tick()
    print(
        f"[{frame.t_s:7.1f}s] {frame.phase.value:12s} "
        f"h={frame.state.altitude_m/1000:7.1f}km  "
        f"v={frame.state.speed_ms:6.0f}m/s  "
        f"M={frame.state.mach:.2f}  "
        f"Ω={frame.health.omega:.3f}"
    )

print(agent.chain.summary())
```

## 아키텍처 철학

### 상태는 추정이다

핵심 상태 객체 `RocketState`, `AtmosphereState`, `PropulsionState`, `AeroState` 는 `frozen=True` 불변 데이터클래스로 설계되어 있다. 설정 파라미터는 별도 `*Config` 클래스로 분리된다.

언어도 그에 맞춘다.

- `measured` 보다 `estimated / observed`
- `actual thrust` 보다 `modeled thrust`
- `real atmosphere` 보다 `standard atmosphere approximation`

### 하드코딩은 없다

물리 상수와 설계값은 `*Config` 로 분리되어 있다. `PhysicsConfig` 를 교체하면 다른 행성 환경으로의 확장 경로가 열리고, `VehicleAeroConfig` 에 실측 `Cd` 테이블을 넣으면 정밀도가 달라진다.

### 연결은 선택적이다

```python
try:
    from lucifer_engine import OrbitalPropagator
except ImportError:
    pass

try:
    from autonomy_runtime_stack import ...
except ImportError:
    pass
```

## 파일 구조

```text
LaunchVehicle_Stack/
├── launch_vehicle/
│   ├── contracts/schemas.py
│   ├── physics/atmosphere.py
│   ├── physics/propulsion.py
│   ├── physics/aero.py
│   ├── physics/integrator.py
│   ├── flight/phase_fsm.py
│   ├── flight/staging.py
│   ├── guidance/gravity_turn.py
│   ├── guidance/tvc.py
│   ├── safety/range_safety.py
│   ├── safety/abort_system.py
│   ├── audit/flight_chain.py
│   └── launch_agent.py
└── tests/test_launch_vehicle.py
```

## 테스트

```bash
cd LaunchVehicle_Stack
python -m pytest tests/ -v
```

현재 결과:

- `112 passed`

섹션:

- §1 데이터 계약
- §2 대기 모델
- §3 추진 모델
- §4 공력 모델
- §5 가변 질량 RK4 적분기
- §6 비행 단계 FSM
- §7 단분리 관리자
- §8 Gravity Turn 유도
- §9 TVC 제어기
- §10 Range Safety
- §11 중단 시스템
- §12 FlightChain
- §13 LaunchAgent 통합

## 현재 범위와 확장 경로

| 영역 | 현재 상태 | 다음 확장 경로 |
|---|---|---|
| 대기 모델 | US Std Atm 1976, 0~86 km | NRLMSISE-00, 풍속 프로파일 |
| 추진 모델 | 결정론적 | 엔진 아웃, 확률 노이즈 |
| 공력 모델 | 근사 `Cd(M)` | `Cd(M, alpha)` 테이블 |
| 동역학 | 3D 질점 병진 + 질량 소모 | 강체 `6-DoF` |
| IIP 계산 | 탄도 근사 | 몬테카를로 IIP 분포 |
| 지구 모델 | 구형 지구, 자전 없음 | 자전·코리올리 보정 |
| 대기 상한 | 86 km 경계값 유지 | `Lucifer_Engine` 전달 |
| 탈출 시스템 | 무인 발사체 기준 | LES 확장 |

## 레이어별 확장

- Layer 0:
  - `WindState`
  - `EngineFailureState`
  - 6-DoF 상태 계약
- Layer 1:
  - 풍속 주입
  - 엔진별 추력 마스크
  - `Cd(M, alpha)` 확장
  - 13-state 적분기
- Layer 2:
  - 재사용 발사체 FSM
  - hot staging
  - 병렬 부스터
- Layer 3:
  - PEG
  - LQR / MPC
  - powered descent guidance
- Layer 4:
  - IIP 몬테카를로
  - 실시간 텔레메트리 포맷
  - 외부 검증 체인

## 로드맵

- `v0.1.0`: 현재
- `v0.2.0`: `Lucifer_Engine` 궤도 전파 연동
- `v0.3.0`: PEG + 몬테카를로
- `v0.4.0`: 재사용 발사체 귀환
- `v0.5.0`: 다중 발사체 / 성좌 배치
- 이후: `6-DoF`, 바람, LES

## 연결 생태계

```text
Rocket_Spirit
  ├── Lucifer_Engine
  ├── SYD_DRIFT
  ├── Autonomy_Runtime_Stack
  └── AgedCare_Stack
```

## 한계

이 소프트웨어는 연구·교육 목적의 시뮬레이션이다. 실제 발사 운용, 항공우주 인증, 안전 시스템 용도로 사용할 수 없다.

> 중력은 규칙이다. 그리고 규칙은 궤적으로 이해된다.
