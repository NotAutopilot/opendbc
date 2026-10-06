from opendbc.car import structs
from opendbc.car.tesla.radar_interface import RadarInterface as NapRadarInterface


class RadarInterface(NapRadarInterface):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)
