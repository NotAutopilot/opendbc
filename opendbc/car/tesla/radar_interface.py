import math
from dataclasses import dataclass

from opendbc.can.parser import CANParser
from opendbc.car import Bus, structs
from opendbc.car.tesla.values import DBC, CANBUS, CAR
from opendbc.car.interfaces import RadarInterfaceBase

# Optional NAP config import (available on device/runtime)
try:
  from opendbc.car.tesla.preap.nap_conf import nap_conf
except ImportError:
  nap_conf = None


BOSCH_POINT_BASE_ADDRESS = 0x310
BOSCH_POINT_ADDRESS_STRIDE = 3
BOSCH_TRACK_MAX_MISSED_CYCLES = 2
BOSCH_TRACK_MAX_DISTANCE_DELTA_M = 10.0
BOSCH_TRACK_MAX_VELOCITY_DELTA_MPS = 10.0
PREAP_ALERT_MATRIX_ADDRESS = 0x501
PREAP_ESP_MIA_BYTE = 1
PREAP_ESP_MIA_MASK = 1 << 3
PREAP_VIN_VALIDITY_BYTE = 4
PREAP_VIN_VALIDITY_MASK = 1 << 4


@dataclass(frozen=True)
class PreAPRadarAlertHealth:
  vin_invalid: bool
  esp_input_error: bool


def parse_preap_radar_alert_matrix(dat: bytes) -> PreAPRadarAlertHealth:
  if len(dat) != 8:
    raise ValueError(f"expected 8-byte radar alert matrix, got {len(dat)} bytes")

  return PreAPRadarAlertHealth(
    vin_invalid=bool(dat[PREAP_VIN_VALIDITY_BYTE] & PREAP_VIN_VALIDITY_MASK),
    esp_input_error=bool(dat[PREAP_ESP_MIA_BYTE] & PREAP_ESP_MIA_MASK),
  )


@dataclass(frozen=True)
class BoschTrackObservation:
  d_rel: float
  y_rel: float
  v_rel: float
  a_rel: float
  yv_rel: float
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

  def update(self, slot: int, observation: BoschTrackObservation | None):
    track = self._slots.get(slot)
    if observation is None:
      if track is None:
        return

      track.missed_cycles += 1
      if track.missed_cycles > BOSCH_TRACK_MAX_MISSED_CYCLES:
        del self._slots[slot]
      else:
        track.point.measured = False
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
    track.point.aRel = observation.a_rel
    track.point.yvRel = observation.yv_rel
    track.point.measured = observation.measured

  @staticmethod
  def _is_discontinuous(point, observation):
    distance_delta = abs(observation.d_rel - point.dRel)
    velocity_delta = abs(observation.v_rel - point.vRel)
    return (distance_delta > BOSCH_TRACK_MAX_DISTANCE_DELTA_M or
            velocity_delta > BOSCH_TRACK_MAX_VELOCITY_DELTA_MPS)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.CP = CP

    self.continental_radar = CP.carFingerprint in (CAR.TESLA_MODEL_S_HW3, )
    self.bosch_radar = CP.carFingerprint in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1, CAR.TESLA_MODEL_S_HW2, CAR.TESLA_MODEL_S_PREAP)
    self.preap_radar = CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP

    messages = []
    if self.continental_radar:
      messages.append(('RadarStatus', 16))
      self.num_points = 40
      self.trigger_msg = 1119
      self.radar_point_frq = 16
    elif self.bosch_radar:
      messages.append(('TeslaRadarSguInfo', 8))
      if self.preap_radar:
        # Pre-AP radar alerts should stay alive-agnostic so absence does not look like a CAN fault.
        messages.append(('TeslaRadarAlertMatrix', math.nan))

      self.num_points = 32
      self.trigger_msg = 878
      self.radar_point_frq = 8

    if self.bosch_radar or self.continental_radar:
      for i in range(self.num_points):
        messages.extend([
          (f'RadarPoint{i}_A', self.radar_point_frq),
          (f'RadarPoint{i}_B', self.radar_point_frq),
        ])

    self.radar_off_can = CP.radarUnavailable
    if not CP.radarUnavailable:
      self.rcp = CANParser(DBC[CP.carFingerprint][Bus.radar], messages, CANBUS.radar)
    else:
      self.rcp = None
    print(f"[NAP] RadarInterface: radarUnavailable={CP.radarUnavailable}, radar_off_can={self.radar_off_can}, rcp={'active' if self.rcp else 'None'}")

    self.updated_messages = set()
    self.track_id = 0
    self.bosch_tracks = BoschTrackLifecycle()
    self.preap_alert_health = PreAPRadarAlertHealth(vin_invalid=False, esp_input_error=False)
    self.preap_alert_health_samples: tuple[PreAPRadarAlertHealth, ...] = ()
    # Keep parity with Tinkla radar lateral alignment behavior.
    # For behind-nosecone installs, users can configure horizontal offset in meters.
    if self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP and nap_conf is not None:
      self.radar_offset = float(nap_conf.radar_offset)
    else:
      self.radar_offset = 0.0

  def update(self, can_msgs):
    self.preap_alert_health_samples = ()
    if self.preap_radar:
      self.preap_alert_health_samples = tuple(
        parse_preap_radar_alert_matrix(dat)
        for _, frames in can_msgs
        for address, dat, src in frames
        if src == CANBUS.radar and address == PREAP_ALERT_MATRIX_ADDRESS and len(dat) == 8
      )
      if self.preap_alert_health_samples:
        self.preap_alert_health = self.preap_alert_health_samples[-1]

    if self.radar_off_can or (self.rcp is None):
      return super().update(None)

    values = self.rcp.update(can_msgs)
    self.updated_messages.update(values)

    if self.trigger_msg not in self.updated_messages:
      return None

    ret = structs.RadarData()

    if self.rcp is None:
      return ret

    # Errors
    if not self.rcp.can_valid:
      ret.errors.canError = True

    ret.errors.radarFault = False
    ret.errors.radarUnavailableTemporary = False
    if self.continental_radar:
      radar_status = self.rcp.vl['RadarStatus']
      if radar_status['shortTermUnavailable']:
        ret.errors.radarUnavailableTemporary = True
      if radar_status['sensorBlocked'] or radar_status['vehDynamicsError']:
        ret.errors.radarFault = True
    elif self.bosch_radar:
      radar_status = self.rcp.vl['TeslaRadarSguInfo']
      if self.preap_radar:
        ret.errors.radarVinInvalid = self.preap_alert_health.vin_invalid
        ret.errors.radarEspInputError = self.preap_alert_health.esp_input_error
        ret.errors.radarEcuError = bool(radar_status.get('RADC_HWFail', 0)) or (
          bool(radar_status.get('RADC_SGUFail', 0)) and not (ret.errors.radarVinInvalid or ret.errors.radarEspInputError)
        )
        ret.errors.radarFault = ret.errors.radarVinInvalid or ret.errors.radarEspInputError or ret.errors.radarEcuError
      elif radar_status['RADC_HWFail'] or radar_status['RADC_SGUFail']:
        ret.errors.radarFault = True
      if radar_status['RADC_SensorDirty']:
        ret.errors.radarUnavailableTemporary = True

    # Radar tracks
    for i in range(self.num_points):
      msg_a = self.rcp.vl[f'RadarPoint{i}_A']
      msg_b = self.rcp.vl[f'RadarPoint{i}_B']

      if self.bosch_radar:
        point_a_address = BOSCH_POINT_BASE_ADDRESS + i * BOSCH_POINT_ADDRESS_STRIDE
        slot_addresses = {point_a_address, point_a_address + 1}
        observation = None
        if slot_addresses <= self.updated_messages:
          observation = self._parse_bosch_track(msg_a, msg_b)
        self.bosch_tracks.update(i, observation)
        continue

      # Make sure msg A and B are together
      if msg_a['Index'] != msg_b['Index2']:
        continue

      # Check if it's a valid track
      if not msg_a['Tracked']:
        if i in self.pts:
          del self.pts[i]
        continue

      # New track!
      if i not in self.pts:
        self.pts[i] = structs.RadarData.RadarPoint()
        self.pts[i].trackId = self.track_id
        self.track_id += 1

      # Parse track data
      self.pts[i].dRel = msg_a['LongDist']
      self.pts[i].yRel = msg_a['LatDist'] + self.radar_offset
      self.pts[i].vRel = msg_a['LongSpeed']
      self.pts[i].aRel = msg_a['LongAccel']
      self.pts[i].yvRel = msg_b['LatSpeed']
      self.pts[i].measured = bool(msg_a['Meas'])

    ret.points = self.bosch_tracks.points if self.bosch_radar else list(self.pts.values())
    self.updated_messages.clear()
    return ret

  def _parse_bosch_track(self, msg_a, msg_b):
    if msg_a['Index'] != msg_b['Index2'] or not msg_a['Tracked']:
      return None

    if msg_a["LongDist"] > 250.0 or msg_a["LongDist"] <= 0 or msg_a["ProbExist"] < 50.0:
      return None

    return BoschTrackObservation(
      d_rel=msg_a['LongDist'],
      y_rel=msg_a['LatDist'] + self.radar_offset,
      v_rel=msg_a['LongSpeed'],
      a_rel=msg_a['LongAccel'],
      yv_rel=msg_b['LatSpeed'],
      measured=bool(msg_a['Meas']),
    )
