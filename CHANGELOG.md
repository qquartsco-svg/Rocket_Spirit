# Changelog — Rocket_Spirit / LaunchVehicle_Stack

모든 중요 변경사항은 이 파일에 기록된다.
형식: [버전] 날짜 — 변경 요약

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
