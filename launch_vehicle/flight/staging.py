"""단분리 관리자 — 다단 발사체 단 전환 (Layer 2).

역할:
  - 현재 활성 단(Stage) 추적
  - 단분리 이벤트: 소모 단 질량 제거 → 다음 단 전환
  - 단분리 후 발사체 총 질량 갱신
  - 페어링 분리 이벤트 (고도 임계 기반)

단분리 시점:
  FlightPhaseFSM 이 MECO → STAGE_SEP 전이 시
  StagingManager.separate() 호출

질량 계산:
  분리 전: m_total = Σ(단_질량) + payload + fairing
  분리 후: m_total = m_total − 분리_단_dry_mass

관찰 한계:
  - 단분리 충격·분리 속도 델타V 미포함 (순간 질량 제거 가정)
  - 연결부 질량(어댑터) 미포함
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

from ..contracts.schemas import StageConfig, VehicleConfig


@dataclass
class StagingEvent:
    """단분리 이벤트 기록."""
    stage_id:     int
    t_s:          float
    mass_ejected_kg: float    # 분리된 단 질량 (dry)
    remaining_mass_kg: float  # 분리 후 발사체 총 질량


class StagingManager:
    """다단 발사체 단 전환 관리자.

    VehicleConfig 의 stages 를 순서대로 소진.
    현재 활성 단의 추진제 상태와 총 발사체 질량을 추적.

    사용법::
        mgr = StagingManager(vehicle_config)
        # 단분리 시점에:
        event = mgr.separate(t_s=150.0)
        new_stage = mgr.active_stage
    """

    def __init__(self, vehicle: VehicleConfig):
        self._vehicle     = vehicle
        self._stage_idx   = 0
        self._events: List[StagingEvent] = []

        # 발사체 총 질량 (추진제 포함)
        self._current_mass_kg = vehicle.total_liftoff_mass_kg
        self._fairing_ejected = False

    # ── 공개 API ──────────────────────────────────────────────────────────────

    @property
    def stage_idx(self) -> int:
        """현재 활성 단 인덱스 (0-기반)."""
        return self._stage_idx

    @property
    def active_stage(self) -> Optional[StageConfig]:
        """현재 활성 단 설정."""
        return self._vehicle.stage(self._stage_idx)

    @property
    def has_next_stage(self) -> bool:
        return self._stage_idx + 1 < len(self._vehicle.stages)

    @property
    def total_mass_kg(self) -> float:
        return self._current_mass_kg

    @property
    def events(self) -> List[StagingEvent]:
        return list(self._events)

    def separate(self, t_s: float) -> Optional[StagingEvent]:
        """단분리 실행.

        소모된 단의 dry_mass 를 총 질량에서 제거하고
        다음 단으로 전환.

        Returns:
            StagingEvent (단분리 성공) 또는 None (마지막 단)
        """
        if not self.has_next_stage:
            return None

        ejected_stage = self._vehicle.stages[self._stage_idx]
        # 소모 단의 dry_mass 만 제거 (추진제는 이미 소모됨)
        ejected_mass  = ejected_stage.dry_mass_kg
        self._current_mass_kg -= ejected_mass
        self._stage_idx += 1

        event = StagingEvent(
            stage_id=ejected_stage.stage_id,
            t_s=t_s,
            mass_ejected_kg=ejected_mass,
            remaining_mass_kg=self._current_mass_kg,
        )
        self._events.append(event)
        return event

    def eject_fairing(self, t_s: float) -> float:
        """페어링 분리 (고도 임계 도달 시).

        Returns:
            분리된 페어링 질량 (kg)
        """
        if self._fairing_ejected:
            return 0.0
        mass = self._vehicle.fairing_mass_kg
        self._current_mass_kg -= mass
        self._fairing_ejected = True
        return mass

    def consume_propellant(self, mass_kg: float) -> None:
        """추진제 소모 반영 (총 질량 감소).

        적분기에서 질량이 줄어들 때 동기화.
        """
        self._current_mass_kg = max(
            self._dry_mass_remaining(),
            self._current_mass_kg - mass_kg,
        )

    # ── 내부 ─────────────────────────────────────────────────────────────────

    def _dry_mass_remaining(self) -> float:
        """현재 스택에 남은 최소 질량 (dry mass 합계 + payload)."""
        remaining_stages = self._vehicle.stages[self._stage_idx:]
        return (sum(s.dry_mass_kg for s in remaining_stages)
                + self._vehicle.payload_mass_kg
                + (self._vehicle.fairing_mass_kg if not self._fairing_ejected else 0.0))
