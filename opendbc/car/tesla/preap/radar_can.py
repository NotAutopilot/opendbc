"""Exact Pre-AP Bosch radar gateway payload transforms.

These match the panda tesla_preap GTW path: buses, DLCs, counters, checksums,
and bitfield patches. Missing or contradictory configuration yields no TX.
"""
from __future__ import annotations

from dataclasses import dataclass


RADAR_BUS = 1
BOSCH_TRIGGER_ADDR = 0x36E
BOSCH_STATUS_ADDR = 0x301
BOSCH_POINT_BASE_ADDRESS = 0x310
BOSCH_POINT_ADDRESS_STRIDE = 3
BOSCH_POINT_COUNT = 32

GTW_CAR_CONFIG_SRC = 0x398
GTW_CAR_CONFIG_DST = 0x2A9
STW_ANGLHP_SRC = 0x0E
STW_ANGLHP_DST = 0x199
ESP_115_SRC = 0x115
ESP_115_DST = 0x129
DI_ESP_CONTROL_DST = 0x1A9
DI_TORQUE2_SRC = 0x118
DI_TORQUE2_DST = 0x119
ESP_WHEEL_SPEEDS_DST = 0x169

READDR = (
  (0x45, 0x219, 8, "STW_ACTN_RQ"),
  (0x108, 0x109, 8, "DI_torque1"),
  (0x145, 0x149, 8, "ESP_145h"),
  (0x20A, 0x159, 8, "BrakeMessage"),
  (0x308, 0x209, 8, "GTW_odo"),
  (0x30A, 0x2D9, 8, "BC_status"),
  (0x405, 0x2B9, 8, "VIP_405HS"),
)


@dataclass(frozen=True)
class GatewayFrame:
  addr: int
  bus: int
  data: bytes


def _u32_le(data: bytes, offset: int) -> int:
  chunk = data[offset:offset + 4].ljust(4, b"\x00")
  return chunk[0] | (chunk[1] << 8) | (chunk[2] << 16) | (chunk[3] << 24)


def _pack_u32_le(value: int) -> bytes:
  value &= 0xFFFFFFFF
  return bytes((value & 0xFF, (value >> 8) & 0xFF, (value >> 16) & 0xFF, (value >> 24) & 0xFF))


def _crc8_j1850(lo: int, hi: int, msg_len: int) -> int:
  crc = 0xFF
  for index in range(msg_len):
    value = ((lo >> (index * 8)) & 0xFF) if index <= 3 else ((hi >> ((index - 4) * 8)) & 0xFF)
    crc ^= value
    for _ in range(8):
      crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
  return crc ^ 0xFF


def radar_config_allowed(radar_emulation: bool) -> bool:
  return bool(radar_emulation)


def transform_car_config(src: bytes, *, position: int = 0, epas_type: int = 0) -> bytes | None:
  if len(src) != 8:
    return None
  lo = _u32_le(src, 0)
  hi = _u32_le(src, 4)
  lo = (lo & 0xFFFFF33F) | 0x100 | 0x440
  hi = (hi & 0xCFFF0F0F) | 0x10000000 | ((position & 0x03) << 4) | (epas_type << 12)
  return _pack_u32_le(lo) + _pack_u32_le(hi)


def transform_stw_anglhp(src: bytes) -> bytes | None:
  if len(src) != 8:
    return None
  lo = _u32_le(src, 0)
  hi = _u32_le(src, 4)
  if ((lo >> 16) & 0xFF3F) == 0xFF3F:
    lo = (lo & 0x0000FFFF) | (0x0020 << 16)
    hi = (hi & 0x00FFFFF0) | 0x00000004
    crc = _crc8_j1850(lo, hi, 7)
    hi = hi | (crc << 24)
  return _pack_u32_le(lo) + _pack_u32_le(hi)


def synthesize_di_esp_control(src: bytes) -> bytes | None:
  if len(src) != 6:
    return None
  hi_src = _u32_le(src, 4)
  counter = ((hi_src & 0xF0) >> 4) & 0x0F
  syn_lo = 0x000C0000 | (counter << 28)
  checksum = (0x38 + 0x0C + (counter << 4)) & 0xFF
  return _pack_u32_le(syn_lo)[:4] + bytes((checksum,))


def synthesize_esp_wheel_speeds(src: bytes) -> bytes | None:
  if len(src) != 6:
    return None
  lo = _u32_le(src, 0)
  ws_counter = _u32_le(src, 4) & 0x0F
  raw_speed = (0xFFF0000 & lo) >> 16
  if raw_speed == 0xFFF:
    speed = 0x1FFF
  else:
    mph_x100 = raw_speed * 5 - 2500
    kph_x100 = mph_x100 * 1609 // 1000
    speed = 0 if kph_x100 < 0 else ((kph_x100 // 4) & 0x1FFF)
  ws_lo = speed | (speed << 13) | (speed << 26)
  ws_hi = ((speed >> 6) | (speed << 7) | (ws_counter << 20)) & 0x00FFFFFF
  checksum = 0x76
  for shift in (0, 8, 16, 24):
    checksum = (checksum + ((ws_lo >> shift) & 0xFF)) & 0xFF
  for shift in (0, 8, 16):
    checksum = (checksum + ((ws_hi >> shift) & 0xFF)) & 0xFF
  ws_hi = ws_hi | (checksum << 24)
  return _pack_u32_le(ws_lo) + _pack_u32_le(ws_hi)


def gateway_frames(addr: int, bus: int, data: bytes, *, radar_emulation: bool,
                   position: int = 0) -> tuple[GatewayFrame, ...]:
  if not radar_config_allowed(radar_emulation) or bus != 0:
    return ()

  frames: list[GatewayFrame] = []
  for src, dst, dlc, _name in READDR:
    if addr == src and len(data) == dlc:
      frames.append(GatewayFrame(dst, RADAR_BUS, bytes(data)))

  if addr == GTW_CAR_CONFIG_SRC:
    payload = transform_car_config(data, position=position)
    if payload is not None:
      frames.append(GatewayFrame(GTW_CAR_CONFIG_DST, RADAR_BUS, payload))

  if addr == STW_ANGLHP_SRC:
    payload = transform_stw_anglhp(data)
    if payload is not None:
      frames.append(GatewayFrame(STW_ANGLHP_DST, RADAR_BUS, payload))

  if addr == ESP_115_SRC:
    if len(data) == 6:
      frames.append(GatewayFrame(ESP_115_DST, RADAR_BUS, bytes(data)))
      payload = synthesize_di_esp_control(data)
      if payload is not None:
        frames.append(GatewayFrame(DI_ESP_CONTROL_DST, RADAR_BUS, payload))

  if addr == DI_TORQUE2_SRC:
    if len(data) == 6:
      frames.append(GatewayFrame(DI_TORQUE2_DST, RADAR_BUS, bytes(data)))
      payload = synthesize_esp_wheel_speeds(data)
      if payload is not None:
        frames.append(GatewayFrame(ESP_WHEEL_SPEEDS_DST, RADAR_BUS, payload))

  return tuple(frames)
