# Rocket_Spirit — 릴리즈 감사 로그

## v0.1.2 — 2026-03-28

- 시뮬레이션 완주 불가 버그 6개 수정 (Fix 1~6)
- Range Safety iip_safe 오발동 제거 (Fix 1, 2)
- Ω_propulsion 오진단 수정: 연소 중 자연 추진제 감소 ≠ 비상 (Fix 3)
- FSM MAX_Q ↔ ASCENDING 위상 진동 제거 (Fix 4)
- max_downrange_m 10,000km로 확장 (Fix 5)
- apogee_kick / orbit_complete 자동 트리거 (Fix 6)
- 테스트 146개 (기존 145 + 전체 시뮬레이션 회귀 테스트 1)
- SIGNATURE.sha256 재생성

## v0.1.1 — 2026-03-28

- 에코시스템 브리지 추가: TAM, Air Jordan, AgedCare
- `command_go_from_tam()` — 호버 readiness 기반 발사 인가
- 테스트 145개 (기존 114 + 에코시스템 브리지 31)
- SIGNATURE.sha256 재생성

## v0.1.0 — 2026-03-28

- 초기 릴리즈: 4레이어 발사체 시뮬레이션 스택
- 112개 테스트, FlightChain SHA-256 감사
