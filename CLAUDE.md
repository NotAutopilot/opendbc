# NAP (Not Autopilot) Development Notes

## Project Overview
This is the opendbc_repo for the NAP branch, porting pre-AP Tesla Model S support from Tinkla (openpilot 0.9.6) to modern openpilot 0.10.3.

## Source of Truth
**Tinkla is the source of truth for pre-AP Tesla logic** - located at:
`/Users/jackbrandt/projects/notautopilot-development/openpilot-tinkla/selfdrive/car/tesla/`

When in doubt about pre-AP implementation details, check Tinkla first.

## Key Architecture Differences (NAP vs Tinkla)

### DBC Files
- **Tinkla**: Uses `tesla_can` DBC for ALL Tesla cars including pre-AP
- **NAP**: Uses separate `tesla_preap.dbc` for pre-AP cars
- Both DBCs have compatible core messages (EPAS_sysStatus, DI_state, STW_ACTN_RQ, etc.)
- `tesla_preap.dbc` includes GAS_SENSOR for pedal interceptor (Tinkla adds this dynamically)

### Code Organization
- **Tinkla**: All Tesla code in `selfdrive/car/tesla/`
- **NAP**: Tesla code in `opendbc/car/tesla/` (opendbc is a submodule)

### Module Responsibilities
- **carstate.py**: Read CAN signals, simple cruiseEnabled toggle on MAIN/CANCEL
- **PCC_module.py**: Double-pull detection for enable_pedal_cruise, pedal command generation
- **ACC_module.py**: Virtual stalk button presses when pedal not available
- **LONG_module.py**: Coordinates PCC and ACC, speed limit handling
- **teslacan_legacy.py**: CAN message generation for legacy cars

## Critical Implementation Notes

### Stalk Engagement (Pre-AP)
- Single pull of MAIN button → enables `cruiseEnabled` (lateral control only)
- Double pull within 750ms → enables `enable_pedal_cruise` (lateral + longitudinal)
- Double-pull detection is ONLY in PCC_module, NOT in carstate
- Cancel button → disables everything

### Pedal Interceptor
- GAS_SENSOR message (0x552) read from configurable bus (0 or 2)
- Signal names: INTERCEPTOR_GAS, INTERCEPTOR_GAS2, STATE, IDX
- STATE=0 means pedal hardware OK
- Pedal enabled when: hardware OK AND (stock CC off OR enablePedalOverCC)

### Disable Cruise Control Toggle (NAPDisableCruiseControl)
- When enabled AND PCC is active: actively sends CANCEL to prevent stock CC from engaging
- This prevents the pedal and stock CC from fighting each other
- Sends CANCEL at 10Hz whenever stock CC tries to engage while PCC is in use
- Located in LONG_module.py `_update_preap()` function

### iBooster Braking
- Optional hardware for friction braking
- ECU_BrakeCommand message for brake commands
- ECU_BrakeStatus for reading brake state
- When iBooster active, can start from standstill

### CruiseButtons Values
```python
IDLE = 0
CANCEL = 1       # FWD - Push stalk away from driver
MAIN = 2         # RWD - Pull stalk towards driver
RES_ACCEL = 16   # UP_1ST - Push up 1st detent (+1 speed)
RES_ACCEL_2ND = 4  # UP_2ND - Push up 2nd detent (+5 speed)
DECEL_SET = 32   # DN_1ST - Push down 1st detent (-1 speed)
DECEL_2ND = 8    # DN_2ND - Push down 2nd detent (-5 speed)
```

## Common Mistakes to Avoid

1. **Don't put double-pull logic in carstate.py** - it belongs in PCC_module
2. **Don't access Params every frame** - cache and refresh at 1Hz max
3. **Door states are lowercase** - use "open"/"closed" not "OPEN"/"CLOSED"
4. **Pre-AP has no RCM_status** - use SDM1 for seatbelt, wrap in try/except
5. **Signal name is EPAS_torsionBarTorque** - not EPAS_torqueLevel
6. **Pre-AP uses single CAN bus** - all buses point to CANBUS.party
7. **CRITICAL: CANParser must have explicit message lists** - Don't pass empty `[]` to CANParser!
   - Empty lists cause race conditions: signals may not exist when first accessed
   - Any KeyError in `update_legacy()` causes entire method to fail, returning stale data
   - Tinkla lists all messages explicitly with frequencies
   - Modern openpilot format: `[("ESP_B", 50), ("DI_state", 10), ...]`
   - This pre-registers all signals with default value 0.0

## NAP Parameter Mapping (Tinkla → NAP)
| Tinkla Param | NAP Param |
|--------------|-----------|
| TinklaEnablePedal | NAPPedalEnabled |
| TinklaPedalCanZero | NAPPedalCanBus |
| TinklaDisableCruiseControl | NAPDisableCruiseControl |
| TinklaHasIBooster | NAPIBoosterEnabled |
| TinklaBrakeFactor | NAPBrakeFactor |
| TinklaPedalProfile | NAPPedalProfile |

## File Locations

### NAP (this repo)
- `opendbc/car/tesla/carstate.py` - Car state parsing
- `opendbc/car/tesla/carcontroller.py` - CAN command generation
- `opendbc/car/tesla/values.py` - Constants and platform configs
- `opendbc/car/tesla/PCC_module.py` - Pedal cruise control
- `opendbc/car/tesla/ACC_module.py` - Adaptive cruise control
- `opendbc/car/tesla/LONG_module.py` - Longitudinal coordinator
- `opendbc/car/tesla/teslacan_legacy.py` - Legacy CAN commands
- `opendbc/dbc/tesla_preap.dbc` - Pre-AP DBC file

### Tinkla (reference)
- `/Users/jackbrandt/projects/notautopilot-development/openpilot-tinkla/selfdrive/car/tesla/`
