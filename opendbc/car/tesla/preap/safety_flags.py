from enum import IntFlag


class TeslaPreAPSafetyFlags(IntFlag):
  ENABLE_PEDAL = 1
  RADAR_EMULATION = 2
  RADAR_BEHIND_NOSECONE = 4
  RADAR_DIAGNOSTIC = 8
