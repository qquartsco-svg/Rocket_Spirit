"""비행 기록 체인 — SHA-256 블랙박스 (Layer 4).

SYD_DRIFT CommandChain 패턴을 발사체 텔레메트리용으로 재구현.

블록 구조:
  H_i = SHA-256( str(i) ‖ str(t_s) ‖ payload_json ‖ H_{i-1} )
  Genesis: H_0 = SHA-256("GENESIS|LaunchVehicle|{vehicle_id}")

기록 대상 (TelemetryFrame 요약):
  - 미션 시각 t_s
  - 비행 단계 (FlightPhase)
  - 위치·속도·마하수·동압
  - 건전성 Ω·verdict
  - 스로틀 명령

위변조 탐지:
  ∀ i: block_i.hash == recompute(block_i)
  ∀ i>0: block_i.prev_hash == block_{i-1}.hash

관찰 한계:
  - Python 수준 불변성 (OS 파일 수정은 별도 보호 필요)
  - 실시간 다운링크 암호화 미포함
  - 주기 기록 (매 N틱마다) — 연속 기록 시 메모리 증가 주의
"""
from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

from ..contracts.schemas import TelemetryFrame


GENESIS_PREFIX = "GENESIS|LaunchVehicle"


def _sha256(data: str) -> str:
    return hashlib.sha256(data.encode("utf-8")).hexdigest()


@dataclass(frozen=True)
class FlightBlock:
    """불변 비행 기록 블록."""
    index:      int
    t_s:        float
    payload:    str      # JSON
    prev_hash:  str
    hash:       str

    @classmethod
    def create(cls, index: int, t_s: float,
               payload: Dict[str, Any], prev_hash: str) -> "FlightBlock":
        payload_json = json.dumps(payload, sort_keys=True, ensure_ascii=False)
        raw = f"{index}|{t_s:.6f}|{payload_json}|{prev_hash}"
        return cls(
            index=index, t_s=t_s,
            payload=payload_json,
            prev_hash=prev_hash,
            hash=_sha256(raw),
        )

    def verify(self) -> bool:
        raw = f"{self.index}|{self.t_s:.6f}|{self.payload}|{self.prev_hash}"
        return _sha256(raw) == self.hash

    def to_dict(self) -> Dict[str, Any]:
        return {
            "index":     self.index,
            "t_s":       self.t_s,
            "payload":   json.loads(self.payload),
            "prev_hash": self.prev_hash[:16] + "…",
            "hash":      self.hash[:16] + "…",
        }


class FlightChain:
    """발사체 비행 기록 체인.

    TelemetryFrame 을 주기적으로 기록하여
    사후 분석·무결성 검증을 지원.

    사용법::
        chain = FlightChain(vehicle_id="LV-001")
        block = chain.record(telemetry_frame)
        assert chain.verify_integrity()
        chain.export_json("flight_log.json")
    """

    def __init__(self, vehicle_id: str, record_interval: int = 10):
        """
        Args:
            vehicle_id:      발사체 식별자
            record_interval: N틱마다 한 번 기록 (0 = 매 틱)
        """
        self._vehicle_id    = vehicle_id
        self._interval      = max(0, record_interval)
        self._genesis_hash  = _sha256(f"{GENESIS_PREFIX}|{vehicle_id}")
        self._blocks: List[FlightBlock] = []
        self._tick_count    = 0

    def record(self, frame: TelemetryFrame) -> Optional[FlightBlock]:
        """TelemetryFrame 을 블록으로 기록.

        record_interval 틱마다 한 번 실제 기록.
        나머지 틱은 None 반환.

        Returns:
            FlightBlock 또는 None (간격 틱)
        """
        self._tick_count += 1
        if self._interval > 0 and self._tick_count % self._interval != 0:
            return None

        payload = frame.summary_dict()
        prev_hash = self._blocks[-1].hash if self._blocks else self._genesis_hash
        block = FlightBlock.create(
            index=len(self._blocks),
            t_s=frame.t_s,
            payload=payload,
            prev_hash=prev_hash,
        )
        self._blocks.append(block)
        return block

    def record_event(self, t_s: float, event_type: str,
                     data: Optional[Dict] = None) -> FlightBlock:
        """중요 이벤트 즉시 기록 (간격 무관).

        단분리, MECO, 중단 등 핵심 이벤트 보장 기록.
        """
        payload = {"event": event_type, "t_s": t_s}
        if data:
            payload.update(data)
        prev_hash = self._blocks[-1].hash if self._blocks else self._genesis_hash
        block = FlightBlock.create(
            index=len(self._blocks),
            t_s=t_s,
            payload=payload,
            prev_hash=prev_hash,
        )
        self._blocks.append(block)
        return block

    # ── 검증 ─────────────────────────────────────────────────────────────────

    def verify_integrity(self) -> bool:
        """전체 체인 무결성 검증."""
        prev = self._genesis_hash
        for block in self._blocks:
            if not block.verify():
                return False
            if block.prev_hash != prev:
                return False
            prev = block.hash
        return True

    # ── 조회 ─────────────────────────────────────────────────────────────────

    @property
    def length(self) -> int:
        return len(self._blocks)

    @property
    def head_hash(self) -> str:
        return self._blocks[-1].hash if self._blocks else self._genesis_hash

    def phase_events(self, event_type: str) -> List[FlightBlock]:
        result = []
        for b in self._blocks:
            p = json.loads(b.payload)
            if p.get("event") == event_type or p.get("phase") == event_type:
                result.append(b)
        return result

    def summary(self) -> str:
        ok = "✓ INTACT" if self.verify_integrity() else "✗ TAMPERED"
        return (
            f"[FlightChain] {self._vehicle_id}\n"
            f"  블록수: {self.length} | 무결성: {ok}\n"
            f"  Genesis: {self._genesis_hash[:16]}…\n"
            f"  Head:    {self.head_hash[:16]}…\n"
        )

    def export_json(self, path: str) -> None:
        data = {
            "vehicle_id":   self._vehicle_id,
            "genesis_hash": self._genesis_hash,
            "block_count":  self.length,
            "integrity":    self.verify_integrity(),
            "blocks": [b.to_dict() for b in self._blocks],
        }
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
