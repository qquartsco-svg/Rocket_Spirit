# Changelog — Rocket_Spirit / LaunchVehicle_Stack

모든 중요 변경사항은 이 파일에 기록된다.
형식: [버전] 날짜 — 변경 요약

---

## [0.1.2] 2026-03-28

### 버그 수정 (시뮬레이션 완주 불가 5개 이슈)

#### Fix 1 — Range Safety: `iip_safe` 가 자폭 트리거를 오발동 (Critical)
- **원인**: 수직 상승 구간(T+0~MECO) 탄도 IIP 는 항상 발사장 근처 → `iip_safe=False` 가 정상 상태인데, `destruct = not corridor_ok or not iip_safe` 로직이 매 틱 자폭 명령을 생성
- **수정**: `destruct = not corridor_ok` 로 변경. `iip_safe` 는 정보용 플래그로만 유지
- 파일: `launch_vehicle/safety/range_safety.py`

#### Fix 2 — LaunchAgent: `ω_r` 계산에서 `iip_safe` 제거 + Air Jordan 고도 가드 (Critical)
- **원인**: `ω_r = 1.0 if rss.corridor_ok and rss.iip_safe else 0.20` → `iip_safe=False` 시 Ω_r=0.20 → Ω<0.25 → 전체 ABORT
- **수정**: `ω_r = 1.0 if rss.corridor_ok else 0.20`
- 추가: Air Jordan 공력 보조를 `altitude_m > 500m` 이상에서만 활성화 (이륙 직후 오경보 방지)
- 파일: `launch_vehicle/launch_agent.py`

#### Fix 3 — LaunchAgent: `Ω_propulsion` 연소 중 자연 추진제 감소를 비상으로 오판 (Critical)
- **원인**: `ω_p = max(0.1, prop_frac)` → 75% 추진제 소모 시 `ω_p=0.247<0.25` → Ω<0.25 → ABORT
- **수정**: 엔진 점화 중이면 `ω_p = 1.0`(잔량 > 0) / 비연소 시 `ω_p = max(0.30, prop_frac)` (MECO 순간 ABORT 방지)
- 파일: `launch_vehicle/launch_agent.py`

#### Fix 4 — FlightPhaseFSM: MAX_Q ↔ ASCENDING 위상 진동 (Major)
- **원인**: 동압 감소 시 MAX_Q → ASCENDING 전이 후 여전히 q > 임계 → 즉시 재진입 반복
- **수정**: `_max_q_complete: bool` 플래그 추가 — 한 번 통과한 MAX_Q 구간 재진입 차단
- 파일: `launch_vehicle/flight/phase_fsm.py`

#### Fix 5 — Range Safety: `max_downrange_m` 기본값이 LEO 사거리에 비해 과소 (Critical)
- **원인**: `max_downrange_m=500km` → 궤도 발사 로켓 IIP 가 500km 초과 시 corridor_ok=False → ABORT
- **수정**: `max_downrange_m=10,000km` (LEO 궤도삽입 전 단계까지 포괄)
- 파일: `launch_vehicle/safety/range_safety.py`

#### Fix 6 — LaunchAgent: apogee_kick / orbit_complete 미설정으로 COAST → NOMINAL 불가 (Major)
- **원인**: `FlightContext(apogee_kick=False, orbit_complete=False)` 고정 → COAST 에서 NOMINAL 도달 불가
- **수정**: 자동 트리거 로직 추가
  - `apogee_kick = (phase==COAST and vz<0 and alt>=80km)` (정점 통과 자동 감지)
  - `orbit_complete = (phase==ORBIT_INSERT and (v>=7km/s or prop==0))` (궤도 속도 또는 추진제 고갈)
- 파일: `launch_vehicle/launch_agent.py`

### 테스트 추가
- `TestLaunchAgent::test_full_simulation_reaches_nominal` — 2단 로켓 전체 시뮬레이션 회귀 테스트 (HOLD→NOMINAL, ABORT 없음 검증)

### 결과
- 이전: 시뮬레이션 T+29s에서 무조건 ABORT (Range Safety 오발동)
- 이후: HOLD→LIFTOFF→ASCENDING→MAX_Q→MECO→STAGE_SEP→UPPER_BURN→COAST→ORBIT_INSERT→NOMINAL 전 단계 완주
- 테스트: **146 passed** (신규 1개 포함)

---

## [0.1.1] 2026-03-28

### 추가
- `launch_vehicle/bridges/tam_bridge.py` — TAM (StarScream) 호버 readiness → 발사 인가 브리지
  - `TamLaunchReadiness` 데이터클래스 (frozen)
  - `readiness_from_tam_report()` duck-typed 변환
  - `optional_tam_launch_readiness()` 폴백 지원
  - `try_import_tam_readiness()` 모노레포 동적 import
- `launch_vehicle/bridges/air_jordan_bridge.py` — Air Jordan 저고도 공력 보조 브리지
  - 고도 < 20 km 구간 공력 판정 (저고도 전용)
  - `optional_air_jordan_aero()` ImportError 폴백
- `launch_vehicle/adapters/aged_care_adapter.py` — 유인 발사 탑승자 안전 어댑터 (MVP-4)
  - `AgedCareLaunchSafety` 스냅샷
  - `evaluate_crew_launch_gate()` 유인 발사 게이트 판정
  - `snapshot_from_safety_state()` duck-typing 변환
- `LaunchAgent.command_go_from_tam()` — TAM readiness 기반 발사 인가
- `LaunchAgent.tam_readiness` / `air_jordan_evidence` 속성
- `LaunchAgent._compute_health()` — Air Jordan Ω_aero 보조 항 추가
- `LaunchAgent.__init__()` — `tam_readiness`, `crew_safety`, `human_rated_mvp4` 파라미터
- `tests/test_ecosystem_bridges.py` — 에코시스템 브리지 전용 테스트 31개
- `VERSION`, `CHANGELOG.md`, `BLOCKCHAIN_INFO.md`, `PHAM_BLOCKCHAIN_LOG.md`
- `scripts/` — 서명·검증·릴리즈 자동화 스크립트

### 변경
- `__init__.py` — 버전 0.1.1, 에코시스템 브리지/어댑터 공개 API 추가
- `pyproject.toml` — 버전 0.1.1 동기화

### 수정
- `tests/test_public_api.py` — 버전 어서션 0.1.1 로 갱신

---

## [0.1.0] 2026-03-28

### 초기 릴리즈
- Layer 0: 데이터 계약 (`contracts/schemas.py`) — RocketState, VehicleConfig 등
- Layer 1: 물리 엔진 — US 표준 대기, 추진, 공력, RK4 적분
- Layer 2: 비행 FSM (11단계) + 단분리 관리
- Layer 3: 중력 선회 유도 + TVC 제어
- Layer 4: Range Safety, 중단 시스템, SHA-256 FlightChain
- `LaunchAgent` 통합 오케스트레이터 (11단계 틱 파이프라인)
- 테스트 112개 (§1~§13 전 레이어)
