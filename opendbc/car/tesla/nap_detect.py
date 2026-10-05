"""NAP: pick the legacy Model S platform from what is on the harness.

An AP1 car's DAS ECU sends DAS_steeringControl (0x488, 50 Hz) and DAS_control
(0x2B9, ~25 Hz) continuously. A Pre-AP car has no AP ECU, and nothing NAP sends
(pedal, radar emulation, DAS_bodyControls) exists before a safety mode is set.
EPAS firmware can't tell them apart: Pre-AP retrofits often run AP1 EPAS firmware.
"""

AP_ECU_ADDRS = (0x488, 0x2B9)  # DAS_steeringControl, DAS_control

# NAPCarType param values
NAP_CAR_TYPE_AUTO = 0
NAP_CAR_TYPE_PREAP = 1
NAP_CAR_TYPE_AP1 = 2

PREAP_PLATFORM = "TESLA_MODEL_S_PREAP"
AP1_PLATFORM = "TESLA_MODEL_S_HW1"
FORCED_PLATFORM = {NAP_CAR_TYPE_PREAP: PREAP_PLATFORM, NAP_CAR_TYPE_AP1: AP1_PLATFORM}


def detect_legacy_platform(finger: dict[int, dict[int, int]]) -> str:
  """AP1 when the DAS ECU is on the harness, else Pre-AP.

  With the relay closed during fingerprinting, AP ECU frames can show up on
  bus 0 as well as bus 2, so both are checked.
  """
  seen = set(finger.get(0, {})) | set(finger.get(2, {}))
  return AP1_PLATFORM if all(addr in seen for addr in AP_ECU_ADDRS) else PREAP_PLATFORM


def normalize_car_type(value) -> int | None:
  """None stays None (not a NAP device); anything unknown means Auto."""
  if value is None:
    return None
  return value if value in (NAP_CAR_TYPE_AUTO, NAP_CAR_TYPE_PREAP, NAP_CAR_TYPE_AP1) else NAP_CAR_TYPE_AUTO


def read_nap_car_type() -> int | None:
  """NAPCarType, or None when unset or unreadable (stock fingerprinting).

  The manager writes it on NAP devices; replay and tests start without it.
  """
  try:
    from openpilot.common.params import Params
    return normalize_car_type(Params().get("NAPCarType"))
  except Exception:
    return None
