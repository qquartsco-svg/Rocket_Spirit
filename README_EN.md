# Rocket_Spirit — LaunchVehicle_Stack

What is the soul of a rocket?

When a human decides to cross the atmosphere and defy gravity, what fills that vessel is not merely equations of thrust and drag — it is millions of calculations, decades of failure, and a trajectory that says *this time, we reach*.

This stack writes that soul in code. And it keeps being written.

`Rocket_Spirit` is the branding name. The actual repository/package name is `LaunchVehicle_Stack` / `launch_vehicle`.

---

## What It Is

`Rocket_Spirit` is a layered flight simulation stack covering the physical journey from ground to orbit insertion.

From v0.1.1, it connects to the full ecosystem:

```
[Wheelchair] → WTS morphing → TAM (StarScream) hover gate → Rocket_Spirit launch → Orbit
```

---

## Ecosystem Connections (v0.1.1)

| System | Role | Bridge |
|--------|------|--------|
| **TAM (StarScream)** | Ground→hover readiness → launch authorization | `bridges/tam_bridge.py` |
| **Air Jordan** | Low-altitude (< 20 km) aerodynamic supplement | `bridges/air_jordan_bridge.py` |
| **AgedCare_Stack** | Crewed launch (MVP-4) passenger safety gate | `adapters/aged_care_adapter.py` |
| **VPF** | Launch pad ground physics | independent reference |
| **Wheelchair_Transform** | Morphing sequence | indirect via TAM |

### TAM → Launch Authorization Flow

```python
from launch_vehicle import LaunchAgent, optional_tam_launch_readiness

readiness = optional_tam_launch_readiness(tam_report, mode="HOVER")
agent = LaunchAgent(vehicle, tam_readiness=readiness)

if agent.command_go_from_tam(tam_report, "HOVER"):
    # TAM HOVER + Ω_total ≥ 0.80 + no blockers → launch authorized
    while agent.phase.value not in ("nominal", "abort"):
        frame = agent.tick()
```

**TAM launch authorization conditions:**
- `mode` ∈ `{HOVER, FLIGHT_CRUISE}`
- `omega_total ≥ 0.80`
- `takeoff_possible = True`
- `verdict` ∈ `{HEALTHY, STABLE}`
- `blockers = []`

### Crewed Launch MVP-4

```python
from launch_vehicle import AgedCareLaunchSafety

crew = AgedCareLaunchSafety(
    omega_care=0.90, verdict="SAFE",
    emergency_triggered=False, manual_override=False,
)
agent = LaunchAgent(vehicle, crew_safety=crew, human_rated_mvp4=True)
# TAM authorized + crew verdict=="SAFE" + omega_care≥0.80 → crewed launch enabled
```

---

## Flight Journey

```text
[Ecosystem] TAM HOVER authorization
        |
        v
Launch Pad (HOLD)
        |
        v
Countdown (COUNTDOWN)  —  T-10 ~ T-0
        |
        v  ignition
Liftoff (LIFTOFF)  —  tower clear, max thrust
        |              [Air Jordan aero: low altitude]
        v
Ascending (ASCENDING)  —  gravity turn, pitch kick
        |
        v  peak dynamic pressure
Max-Q (MAX_Q)  —  throttle back, structural load management
        |              [Air Jordan ends: altitude > 20 km]
        v
MECO (Main Engine Cut-Off)
        |
        v  coast
Stage Separation (STAGE_SEP)
        |
        v  upper stage ignition
Upper Burn (UPPER_BURN)
        |
        v
Coast (COAST)
        |
        v  perigee burn
Orbit Insertion (ORBIT_INSERT)
        |
        v
Mission Nominal (NOMINAL)

(any phase) ─────────> ABORT
```

---

## Layer Architecture

```text
 Ecosystem Bridges
  TAM · Air Jordan · AgedCare · VPF
        |
        v
Layer 4 — Mission Safety
  Range Safety · Abort System · FlightChain (SHA-256)
Layer 3 — Guidance & Control
  Gravity Turn · TVC
Layer 2 — Flight Phase Management
  FlightPhase FSM (11 states) · StagingManager
Layer 1 — Physics Engines
  AtmosphereEngine · PropulsionEngine · AerodynamicsEngine · VariableMassIntegrator
Layer 0 — Data Contracts
  RocketState · StageConfig · TelemetryFrame

              |
          LaunchAgent
```

---

## Core Physics

Variable mass dynamics:

```text
s = [x, y, z, vx, vy, vz, m]
dv/dt = (F_thrust + F_drag) / m − g(z) · ẑ
dm/dt = −ṁ
```

Integration: 4th-order Runge-Kutta, `dt = 0.1 s`

Altitude-corrected thrust:

```text
F(h) = throttle × (F_vac − P_atm(h) × A_exit)
```

Atmosphere: US Standard Atmosphere 1976, 0 – 86 km

Drag: `D = ½ · ρ · v² · Cd(M) · A_ref`

Gravity: `g(h) = g₀ × (Rₑ / (Rₑ + h))²`

---

## Flight Health Ω

```text
Ω_rocket = Ω_propulsion × Ω_structural × Ω_trajectory × Ω_range [× Ω_aero]
```

| Component | Description |
|-----------|-------------|
| **Ω_propulsion** | Propellant fraction remaining |
| **Ω_structural** | Dynamic pressure margin (q < 80 kPa) |
| **Ω_trajectory** | Speed/attitude divergence |
| **Ω_range** | Range Safety corridor status |
| **Ω_aero** | Air Jordan aero supplement (low alt, optional) |

| Verdict | Ω | Meaning |
|---------|---|---------|
| `NOMINAL` | ≥ 0.80 | Nominal |
| `CAUTION` | ≥ 0.50 | Monitor |
| `WARNING` | ≥ 0.25 | Alert |
| `ABORT` | < 0.25 | Abort |

---

## FlightChain (SHA-256 Audit)

Every telemetry tick is recorded as a SHA-256 linked block:

```text
H_i = SHA-256(i | t_s | payload_json | H_{i-1})
H_0 = SHA-256("GENESIS|LaunchVehicle|{vehicle_id}")
```

```python
assert agent.chain.verify_integrity()
agent.chain.export_json("flight_log.json")
print(agent.chain.summary())
```

---

## Quick Start

### Unmanned Standalone Launch

```python
from launch_vehicle import LaunchAgent, StageConfig, VehicleConfig

stage = StageConfig(
    stage_id=1, dry_mass_kg=25_000, propellant_mass_kg=400_000,
    engine_count=9, isp_sl_s=282, isp_vac_s=311,
    thrust_sl_n=7_607_000, thrust_vac_n=8_227_000,
    nozzle_exit_area_m2=11.5, max_throttle=1.0, min_throttle=0.57,
    burn_time_design_s=162,
)
vehicle = VehicleConfig(vehicle_id="RST-001", stages=[stage],
                        payload_mass_kg=22_800, fairing_mass_kg=1_900,
                        body_diameter_m=3.66)

agent = LaunchAgent(vehicle, dt_s=0.1)
agent.command_go()

while agent.phase.value not in ("nominal", "abort"):
    frame = agent.tick()
    print(f"[{frame.t_s:7.1f}s] {frame.phase.value:12s} "
          f"h={frame.state.altitude_m/1000:7.1f}km  Ω={frame.health.omega:.3f}")

print(agent.chain.summary())
```

### TAM-Connected Launch

```python
from launch_vehicle import LaunchAgent, VehicleConfig, optional_tam_launch_readiness

readiness = optional_tam_launch_readiness(tam_report, mode="HOVER")
agent = LaunchAgent(vehicle, tam_readiness=readiness)

if agent.command_go_from_tam(tam_report, "HOVER"):
    print(f"Launch authorized: TAM Ω={readiness.tam_omega_total:.3f}")
```

---

## Tests

```bash
cd LaunchVehicle_Stack
python -m pytest tests/ -v
```

Current: **145 passed**

| Section | Coverage |
|---------|----------|
| §1–§13 | Data contracts, physics, FSM, guidance, safety, audit, LaunchAgent |
| §14 | Ecosystem bridges: TAM, Air Jordan, AgedCare |

---

## Scope & Roadmap

| Domain | Current | Next |
|--------|---------|------|
| Atmosphere | US Std Atm 1976, 0–86 km | NRLMSISE-00, wind |
| Propulsion | Deterministic | Engine-out, noise |
| Aero | Cd(M) approx | Cd(M,α) table |
| Dynamics | 3D point mass | 6-DoF rigid body |
| IIP | Ballistic approx | Monte Carlo |
| Earth | Spherical, no rotation | Coriolis |
| Ecosystem | TAM, Air Jordan, AgedCare | Lucifer_Engine orbit |

**Roadmap:**
- `v0.1.1` — current: ecosystem bridges (TAM, Air Jordan, AgedCare)
- `v0.2.0` — Lucifer_Engine orbital propagation (post-MECO)
- `v0.3.0` — PEG guidance + Monte Carlo
- `v0.4.0` — Reusable vehicle return (powered descent)
- `v0.5.0` — Multi-vehicle / constellation deployment

---

## Ecosystem Map

```text
Wheelchair_Transform_System
        ↓  (morph/lock snapshot)
TAM / StarScream              ←  Ground→Hover orchestration
        ↓  (FlightReadinessReport, Ω ≥ 0.80, HOVER)
Rocket_Spirit                 ←  Launch auth → Ascent → Orbit
        ├── Air_Jordan         ←  Low-altitude aero supplement
        ├── AgedCare_Stack     ←  Crewed MVP-4 gate
        └── Lucifer_Engine     ←  Post-MECO orbital propagation (planned)
```

---

## Design Philosophy

**State is estimated.** All state objects use `frozen=True` immutable dataclasses. Say `estimated` not `measured`, `modeled thrust` not `actual thrust`.

**No hardcoding.** Physics constants and design values are separated into `*Config` classes. Swap `PhysicsConfig` for Mars gravity; inject measured `Cd` tables for precision.

**Connections are optional.** Every bridge uses `ImportError` fallback — LaunchAgent operates standalone without TAM, Air Jordan, or AgedCare installed.

**One tick = one physical timestep.** `LaunchAgent.tick()` atomically executes the 11-step pipeline.

---

## Disclaimer

This software is for research and educational simulation only. It cannot be used for actual launch operations, aerospace certification, or safety-critical systems.

> Gravity is a rule. And rules are understood through trajectories.
> This time, we leave the ground.
