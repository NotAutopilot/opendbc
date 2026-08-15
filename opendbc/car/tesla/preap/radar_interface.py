from dataclasses import dataclass

from opendbc.can import CANParser
from opendbc.car import structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.tesla.preap.radar_can import (
  BOSCH_POINT_ADDRESS_STRIDE,
  BOSCH_POINT_BASE_ADDRESS,
  BOSCH_POINT_COUNT,
  BOSCH_STATUS_ADDR,
  BOSCH_TRIGGER_ADDR,
  RADAR_BUS,
)

BOSCH_ALERT_ADDR = 0x501


BOSCH_DBC = "tesla_radar_bosch_generated"
BOSCH_TRACK_MAX_MISSED_CYCLES = 2
BOSCH_TRACK_MAX_DISTANCE_DELTA_M = 10.0
BOSCH_TRACK_MAX_VELOCITY_DELTA_MPS = 10.0
BOSCH_MAX_DISTANCE_M = 250.0
BOSCH_MIN_PROB_EXIST = 50.0


@dataclass(frozen=True)
class BoschTrackObservation:
  d_rel: float
  y_rel: float
  v_rel: float
  measured: bool


@dataclass
class BoschTrack:
  point: structs.RadarData.RadarPoint
  missed_cycles: int = 0


class BoschTrackLifecycle:
  def __init__(self):
    self._slots: dict[int, BoschTrack] = {}
    self._next_track_id = 0

  @property
  def points(self):
    return [self._slots[slot].point for slot in sorted(self._slots)]

  def clear(self):
    self._slots.clear()

  def update(self, slot: int, observation: BoschTrackObservation | None):
    track = self._slots.get(slot)
    if observation is None:
      if track is None:
        return
      track.point.deprecated.measured = False
      track.missed_cycles += 1
      if track.missed_cycles > BOSCH_TRACK_MAX_MISSED_CYCLES:
        del self._slots[slot]
      return

    if track is None or self._is_discontinuous(track.point, observation):
      point = structs.RadarData.RadarPoint()
      point.trackId = self._next_track_id
      self._next_track_id += 1
      track = BoschTrack(point)
      self._slots[slot] = track

    track.missed_cycles = 0
    track.point.dRel = observation.d_rel
    track.point.yRel = observation.y_rel
    track.point.vRel = observation.v_rel
    track.point.deprecated.measured = observation.measured

  @staticmethod
  def _is_discontinuous(point, observation):
    distance_delta = abs(observation.d_rel - point.dRel)
    velocity_delta = abs(observation.v_rel - point.vRel)
    return (distance_delta > BOSCH_TRACK_MAX_DISTANCE_DELTA_M or
            velocity_delta > BOSCH_TRACK_MAX_VELOCITY_DELTA_MPS)


def get_bosch_radar_can_parser():
  messages = [
    ("TeslaRadarSguInfo", 8),
    # Optional: missing samples must not invalidate CAN health. A fresh
    # asserted ESP-MIA is taken only from current-cycle updated_messages.
    ("TeslaRadarAlertMatrix", float("nan")),
  ]
  for i in range(BOSCH_POINT_COUNT):
    messages.extend([
      (f"RadarPoint{i}_A", 8),
      (f"RadarPoint{i}_B", 8),
    ])
  return CANParser(BOSCH_DBC, messages, RADAR_BUS)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.dbc_name = BOSCH_DBC
    self.trigger_msg = BOSCH_TRIGGER_ADDR
    self.num_points = BOSCH_POINT_COUNT
    self.radar_offset = float(getattr(CP_SP, "radarOffset", 0.0) or 0.0)
    self.updated_messages = set()
    self.bosch_tracks = BoschTrackLifecycle()
    self.radar_off_can = bool(CP.radarUnavailable)
    self.rcp = None if self.radar_off_can else get_bosch_radar_can_parser()

  def update(self, can_packets):
    if self.radar_off_can or self.rcp is None:
      return super().update(None)

    self.updated_messages.update(self.rcp.update(can_packets))
    if self.trigger_msg not in self.updated_messages:
      return None

    ret = structs.RadarData()
    can_valid = bool(self.rcp.can_valid)
    if not can_valid:
      ret.errors.canError = True

    status_healthy = self._apply_status(ret)
    if not can_valid or not status_healthy:
      self.bosch_tracks.clear()
      ret.points = []
      self.updated_messages.clear()
      return ret

    for i in range(self.num_points):
      point_a_address = BOSCH_POINT_BASE_ADDRESS + i * BOSCH_POINT_ADDRESS_STRIDE
      slot_addresses = {point_a_address, point_a_address + 1}
      observation = None
      if slot_addresses <= self.updated_messages:
        observation = self._parse_bosch_track(self.rcp.vl[f"RadarPoint{i}_A"], self.rcp.vl[f"RadarPoint{i}_B"])
      self.bosch_tracks.update(i, observation)

    ret.points = self.bosch_tracks.points
    self.updated_messages.clear()
    return ret

  def _apply_status(self, ret) -> bool:
    if BOSCH_STATUS_ADDR not in self.updated_messages or "TeslaRadarSguInfo" not in self.rcp.vl:
      ret.errors.radarFault = True
      return False

    radar_status = self.rcp.vl["TeslaRadarSguInfo"]
    hw_fail = bool(radar_status.get("RADC_HWFail"))
    sgu_fail = bool(radar_status.get("RADC_SGUFail"))
    dirty = bool(radar_status.get("RADC_SensorDirty"))
    if hw_fail or sgu_fail:
      ret.errors.radarFault = True
      return False
    if dirty:
      ret.errors.radarUnavailableTemporary = True
    if BOSCH_ALERT_ADDR in self.updated_messages:
      alert = self.rcp.vl.get("TeslaRadarAlertMatrix", {})
      if bool(alert.get("RADC_a012_espMIA")):
        ret.errors.radarFault = True
        return False
    return True

  def _parse_bosch_track(self, msg_a, msg_b):
    if msg_a["Index"] != msg_b["Index2"] or not msg_a["Tracked"]:
      return None
    if msg_a["LongDist"] > BOSCH_MAX_DISTANCE_M or msg_a["LongDist"] <= 0 or msg_a["ProbExist"] < BOSCH_MIN_PROB_EXIST:
      return None
    return BoschTrackObservation(
      d_rel=msg_a["LongDist"],
      y_rel=msg_a["LatDist"] + self.radar_offset,
      v_rel=msg_a["LongSpeed"],
      measured=bool(msg_a["Meas"]),
    )
