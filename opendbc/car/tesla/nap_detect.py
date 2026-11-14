"""NAP: pick the legacy Tesla platform from what is on the harness.

An AP1 car's DAS ECU sends DAS_steeringControl (0x488, 50 Hz) and DAS_control
(0x2B9, ~25 Hz) continuously. A Pre-AP car has no AP ECU, and nothing NAP sends
(pedal, radar emulation, DAS_bodyControls) exists before a safety mode is set.
EPAS firmware can't tell Pre-AP from AP1: Pre-AP retrofits often run AP1 EPAS firmware.

Among AP1 cars the EPAS firmware does tell a Model S from a Model X, as in stock
fingerprinting, so it is queried only once the AP ECU has been seen.
"""

AP_ECU_ADDRS = (0x488, 0x2B9)  # DAS_steeringControl, DAS_control
SDM1_ADDR = 0x201              # seatbelt/airbag status: sent by a Model S, not by a Model X (RCM_status instead)

# NAPCarType param values
NAP_CAR_TYPE_AUTO = 0
NAP_CAR_TYPE_PREAP = 1
NAP_CAR_TYPE_AP1_S = 2
NAP_CAR_TYPE_AP1_X = 3

PREAP_PLATFORM = "TESLA_MODEL_S_PREAP"
AP1_S_PLATFORM = "TESLA_MODEL_S_HW1"
AP1_X_PLATFORM = "TESLA_MODEL_X_HW1"
AP1_PLATFORMS = (AP1_S_PLATFORM, AP1_X_PLATFORM)
FORCED_PLATFORM = {
  NAP_CAR_TYPE_PREAP: PREAP_PLATFORM,
  NAP_CAR_TYPE_AP1_S: AP1_S_PLATFORM,
  NAP_CAR_TYPE_AP1_X: AP1_X_PLATFORM,
}


def _seen(finger: dict[int, dict[int, int]]) -> set[int]:
  # With the relay closed during fingerprinting, AP ECU frames can show up on
  # bus 0 as well as bus 2, so both are checked.
  return set(finger.get(0, {})) | set(finger.get(2, {}))


def ap_ecu_present(finger: dict[int, dict[int, int]]) -> bool:
  seen = _seen(finger)
  return all(addr in seen for addr in AP_ECU_ADDRS)


def detect_legacy_platform(finger: dict[int, dict[int, int]], fw_candidates=()) -> str:
  """Pre-AP without the AP ECU; otherwise the AP1 model.

  fw_candidates: platforms the FW query matched (empty when it did not run).
  """
  if not ap_ecu_present(finger):
    return PREAP_PLATFORM

  ap1_fw = [platform for platform in AP1_PLATFORMS if platform in fw_candidates]
  if len(ap1_fw) == 1:
    return ap1_fw[0]

  # EPAS firmware not recognised: fall back to which seatbelt message the car sends
  return AP1_S_PLATFORM if SDM1_ADDR in _seen(finger) else AP1_X_PLATFORM


def normalize_car_type(value) -> int | None:
  """None stays None (not a NAP device); anything unknown means Auto."""
  if value is None:
    return None
  return value if value == NAP_CAR_TYPE_AUTO or value in FORCED_PLATFORM else NAP_CAR_TYPE_AUTO


def read_nap_car_type() -> int | None:
  """NAPCarType, or None when unset or unreadable (stock fingerprinting).

  The manager writes it on NAP devices; replay and tests start without it.
  """
  try:
    from openpilot.common.params import Params
    return normalize_car_type(Params().get("NAPCarType"))
  except Exception:
    return None
