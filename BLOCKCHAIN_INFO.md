# Rocket_Spirit — 무결성 & 감사 체계

## SIGNATURE.sha256

`SIGNATURE.sha256` 는 이 저장소의 핵심 파일에 대한 SHA-256 해시 매니페스트다.
릴리즈 시점의 파일 상태를 기록하며, 이후 변조를 탐지하는 데 사용된다.

```
scripts/verify_signature.py      # 검증
scripts/regenerate_signature.py  # 재생성 (변경 후)
```

## FlightChain (비행 감사 체인)

`launch_vehicle/audit/flight_chain.py` 는 각 텔레메트리 틱을 SHA-256 연결
블록으로 기록하는 비행 내부 감사 체인이다. 블록체인과 동일한 구조이나,
외부 네트워크 없이 단일 비행 단위로 독립 동작한다.

- 각 블록: 이전 블록 해시 + 틱 데이터 SHA-256
- `FlightChain.verify_integrity()` 로 체인 무결성 검증
- 이벤트(단계 전이, 단분리, Go 명령, 중단 등) 즉시 기록

## 검증 범위

SIGNATURE.sha256 추적 대상:
- `launch_vehicle/**/*.py`
- `tests/**/*.py`
- `scripts/**/*.py`
- `README.md`, `README_EN.md`, `CHANGELOG.md`, `VERSION`, `pyproject.toml`

제외:
- `.git/`, `.pytest_cache/`, `__pycache__/`, `*.pyc`
- `SIGNATURE.sha256` 자체

## PHAM_BLOCKCHAIN_LOG.md

릴리즈 히스토리와 각 버전의 무결성 서명 요약을 기록한다.
