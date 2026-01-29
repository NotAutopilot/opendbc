# Pre-AP Tesla NAP vs Tinkla Comprehensive Analysis

## Executive Summary

This document analyzes the differences between the NAP (Not Autopilot) implementation and the working Tinkla reference implementation for pre-AP Tesla Model S support.

---

## 1. DBC Analysis

### DBC File Comparison

| Aspect | Tinkla | NAP | Issue? |
|--------|--------|-----|--------|
| Pre-AP DBC | `tesla_can` | `tesla_preap` | Maybe |
| GAS_SENSOR message | Added dynamically | In DBC | OK |
| Core messages | Same addresses | Same addresses | OK |
| Signal names | Consistent | Consistent | OK |

**Finding**: Both DBCs have the same core messages at the same addresses. The `tesla_preap.dbc` is actually more complete as it includes GAS_SENSOR natively. **No DBC issues found.**

### Key Messages Present in Both
- EPAS_sysStatus (880) - Steering status
- DI_torque1 (264) - Motor torque
- DI_torque2 (280) - Gear, torque estimate
- ESP_B (341) - Vehicle speed
- SDM1 (513) - Seatbelt (pre-AP)
- GTW_carState (792) - Door states
- DI_state (872) - Cruise state
- STW_ACTN_RQ (69) - Stalk buttons
- BrakeMessage (522) - Brake status

---

## 2. Missing Modules/Features

### Files in Tinkla but NOT in NAP

| File | Purpose | Priority |
|------|---------|----------|
| `HUD_module.py` | IC/HUD integration, lane display | LOW (optional) |
| `ck_fingerprint.py` | CK fingerprint detection | LOW |
| `ibooster_tools/` | iBooster utilities | MEDIUM |
| `pedal_calibrator/` | Pedal calibration tools | MEDIUM |
| `radar_tools/` | Radar processing utilities | LOW |
| `tinkla/` | Tinkla-specific utilities | LOW |

### Missing CAN Messages (compared to Tinkla)

1. **DAS_lanes** - Lane line info to IC display
2. **DAS_object** - Lead vehicle info to IC
3. **DAS_bodyControls** - Turn signals, hazards
4. **DAS_telemetry** - Road info telemetry
5. **DAS_warningMatrix** - Warning display messages
6. **ECU_BrakeCommand** - iBooster brake control (partially implemented)

---

## 3. Logic Inconsistencies

### 3.1 carstate.py Issues

| Issue | Status | Notes |
|-------|--------|-------|
| Double-pull in carstate | FIXED | Now only in PCC_module |
| Seatbelt KeyError | FIXED | try/except added |
| Door state case | FIXED | Now lowercase |
| torqueLevel signal name | FIXED | Using EPAS_torsionBarTorque |

### 3.2 PCC_module.py Comparison

| Feature | Tinkla | NAP | Status |
|---------|--------|-----|--------|
| Double-pull detection | Yes | Yes | OK |
| Pedal value calculation | Full impl | Full impl | OK |
| Zero torque calibration | Yes | Yes | OK |
| Pedal hysteresis | Yes | Yes | OK |
| iBooster coordination | Yes | Partial | NEEDS WORK |
| Lead vehicle tracking | Yes | Partial | NEEDS WORK |
| Regen braking limits | Yes | Yes | OK |

**Missing in NAP PCC_module**:
- Full radar/lead vehicle integration
- Proper leadOne tracking from radarState
- Some edge case handling

### 3.3 ACC_module.py Comparison

| Feature | Tinkla | NAP | Status |
|---------|--------|-----|--------|
| Double-pull detection | Yes | Yes | OK |
| Virtual button presses | Yes | Yes | OK |
| Speed limit integration | Yes | Yes | OK |
| Fleet speed averaging | Yes | Yes | OK |
| Autoresume logic | Yes | Yes | OK |
| Lead tracking | Yes | Partial | NEEDS WORK |

### 3.4 LONG_module.py Issues

| Feature | Tinkla | NAP | Status |
|---------|--------|-----|--------|
| PCC/ACC coordination | Yes | Yes | OK |
| Speed limit handling | Yes | Yes | OK |
| iBooster commands | Yes | Partial | NEEDS WORK |
| AP1 long control | Yes | Yes | OK |

**Issues Found**:
1. `create_ibst_command` in `teslacan_legacy.py` uses different message structure than Tinkla
2. Missing proper iBooster brake value scaling

### 3.5 teslacan_legacy.py vs Tinkla teslacan.py

| Feature | Tinkla | NAP | Status |
|---------|--------|-----|--------|
| Checksum calc | Yes | Yes | OK |
| Steering control | Yes | Yes | OK |
| Longitudinal control | Yes | Yes | OK |
| Pedal command | Yes | Yes | OK |
| iBooster command | Full CRC | Simple checksum | DIFFERENT |
| Lane messages | Yes | No | MISSING |
| Object messages | Yes | No | MISSING |
| Body controls | Yes | No | MISSING |
| Warning matrix | Yes | No | MISSING (optional) |

---

## 4. Event Names

NAP uses different event names than Tinkla:

| Tinkla Event | NAP Event |
|--------------|-----------|
| pccEnabled | napPedalCruiseEnabled |
| pccDisabled | napPedalCruiseDisabled |
| accEnabled | napAdaptiveCruiseEnabled |
| accDisabled | napAdaptiveCruiseDisabled |
| iBoosterBrakeNotOk | napIBoosterFault |
| promptMaxRegen | napMaxRegenActive |
| pedalCalibrationNeeded | napPedalCalibrationNeeded |

These need to be defined in cereal/car.capnp (or the event handling updated).

---

## 5. TODO List (Priority Order)

### HIGH Priority (Required for basic function)

- [ ] **Verify cereal events exist** - Check that napPedalCruiseEnabled, napPedalCruiseDisabled, etc. are defined in car.capnp
- [ ] **Test current implementation** - Deploy and test with actual hardware
- [ ] **Fix any remaining signal access errors** - Monitor for KeyErrors in logs

### MEDIUM Priority (For full functionality)

- [ ] **iBooster brake command** - Align with Tinkla's `create_ibst_command`:
  - Use proper CRC8 calculation
  - Use ECU_BrakeCommand message format
  - Add ECU_BrakeStatus reading in carstate
- [ ] **Lead vehicle tracking** - Add proper radarState integration in PCC_module
- [ ] **Pedal calibration** - Port pedal_calibrator from Tinkla or add calibration UI

### LOW Priority (Nice to have)

- [ ] **HUD integration** - Port HUD_module.py for IC display
- [ ] **Lane display** - Add DAS_lanes message generation
- [ ] **Object display** - Add DAS_object for lead car on IC
- [ ] **Body controls** - Add turn signal forwarding

---

## 6. Testing Checklist

### Basic Engagement
- [ ] Single stalk pull enables lateral only
- [ ] Double stalk pull enables lateral + longitudinal
- [ ] Cancel button disables everything
- [ ] Speed shows correctly (not frozen)
- [ ] Doors/seatbelt detection works

### Longitudinal Control
- [ ] Pedal commands sent correctly
- [ ] Speed adjustment buttons work (+/- 1, +/- 5)
- [ ] Speed limit integration works (if enabled)
- [ ] Gas pedal override works
- [ ] Brake pedal disables cruise

### iBooster (if equipped)
- [ ] iBooster status read correctly
- [ ] Brake commands sent correctly
- [ ] Standstill resume works

---

## 7. Quick Reference

### CAN Bus Layout (Pre-AP)
All messages on single bus (CANBUS.party = 0):
- No autopilot bus (CAN_AUTOPILOT = -1 in Tinkla)
- Pedal on bus 0 or 2 (configurable via NAPPedalCanBus)

### Key State Variables
```
carstate.py:
- cruiseEnabled: Master engagement (lateral control)
- enablePedal: Pedal hardware ready and stock CC allows
- pcc_available: PCC module availability flag

PCC_module.py:
- enable_pedal_cruise: Longitudinal control active
- pedal_speed_kph: Target cruise speed

ACC_module.py:
- enable_adaptive_cruise: ACC active (when no pedal)
- acc_speed_kph: Target ACC speed
```

---

## 8. Architecture Diagram

```
User Stalk Pull
      │
      ▼
┌─────────────┐
│ carstate.py │ ◄── Reads CAN signals
│             │     Sets cruiseEnabled
└─────┬───────┘
      │
      ▼
┌─────────────┐     ┌─────────────┐
│ PCC_module  │ OR  │ ACC_module  │
│ (pedal)     │     │ (no pedal)  │
└─────┬───────┘     └─────┬───────┘
      │                   │
      ▼                   ▼
┌─────────────────────────────────┐
│         LONG_module.py          │
│   (coordinates, speed limits)   │
└─────────────┬───────────────────┘
              │
              ▼
┌─────────────────────────────────┐
│      teslacan_legacy.py         │
│   (generates CAN messages)      │
└─────────────┬───────────────────┘
              │
              ▼
         CAN Bus
```
