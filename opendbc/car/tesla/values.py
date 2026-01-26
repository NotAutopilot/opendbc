from dataclasses import dataclass, field
from enum import Enum, IntFlag
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.lateral import AngleSteeringLimits, ISO_LATERAL_ACCEL
from opendbc.car.structs import CarParams, CarState
from opendbc.car.docs_definitions import CarDocs, CarFootnote, CarHarness, CarParts, Column
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu


class Footnote(Enum):
  HW_TYPE = CarFootnote(
    "Some 2023 model years have HW4. To check which hardware type your vehicle has, look for " +
    "<b>Autopilot computer</b> under <b>Software -> Additional Vehicle Information</b> on your vehicle's touchscreen. </br></br>" +
    "See <a href=\"https://www.notateslaapp.com/news/2173/how-to-check-if-your-tesla-has-hardware-4-ai4-or-hardware-3\">this page</a> for more information.",
    Column.MODEL)

  SETUP = CarFootnote(
    "See more setup details for <a href=\"https://github.com/commaai/openpilot/wiki/tesla\" target=\"_blank\">Tesla</a>.",
    Column.MAKE, setup_note=True)


@dataclass
class TeslaCarDocsHW3(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.tesla_a]))
  footnotes: list[Enum] = field(default_factory=lambda: [Footnote.HW_TYPE, Footnote.SETUP])


@dataclass
class TeslaCarDocsHW4(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.tesla_b]))
  footnotes: list[Enum] = field(default_factory=lambda: [Footnote.HW_TYPE, Footnote.SETUP])


@dataclass
class TeslaPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.party: 'tesla_model3_party'})


class CAR(Platforms):
  TESLA_MODEL_3 = TeslaPlatformConfig(
    [
      # TODO: do we support 2017? It's HW3
      TeslaCarDocsHW3("Tesla Model 3 (with HW3) 2019-23"),
      TeslaCarDocsHW4("Tesla Model 3 (with HW4) 2024-25"),
    ],
    CarSpecs(mass=1899., wheelbase=2.875, steerRatio=12.0),
  )
  TESLA_MODEL_Y = TeslaPlatformConfig(
    [
      TeslaCarDocsHW3("Tesla Model Y (with HW3) 2020-23"),
      TeslaCarDocsHW4("Tesla Model Y (with HW4) 2024-25"),
     ],
    CarSpecs(mass=2072., wheelbase=2.890, steerRatio=12.0),
  )
  TESLA_MODEL_Y_JUNIPER = TeslaPlatformConfig(
    [
      TeslaCarDocsHW4("Tesla Model Y JUNIPER (with HW4) 2025-26"),
     ],
    CarSpecs(mass=2072., wheelbase=2.890, steerRatio=12.0),
  )
  TESLA_MODEL_X = TeslaPlatformConfig(
    [TeslaCarDocsHW4("Tesla Model X (with HW4) 2024")],
    CarSpecs(mass=2495., wheelbase=2.960, steerRatio=12.0),
  )
  TESLA_MODEL_X_HW1 = TeslaPlatformConfig(
    [CarDocs("Tesla Model X (with HW1) 2014-16", "All", car_parts=CarParts.common([CarHarness.tesla_model_x_hw1]))],
    CarSpecs(mass=2447., wheelbase=2.960, steerRatio=15.0),
    {
      Bus.chassis: 'tesla_can',
      Bus.party: 'tesla_can',
      Bus.pt: 'tesla_can',
      Bus.radar: 'tesla_radar_bosch_generated',
    },
  )
  TESLA_MODEL_S = TeslaPlatformConfig(
    [TeslaCarDocsHW4("Tesla Model S (with HW4) 2024")],
    CarSpecs(mass=2166., wheelbase=2.960, steerRatio=12.0),
  )
  TESLA_MODEL_S_HW1 = TeslaPlatformConfig(
    [CarDocs("Tesla Model S (with HW1) 2014-16", "All", car_parts=CarParts.common([CarHarness.tesla_model_s_hw1]))],
    CarSpecs(mass=2100., wheelbase=2.960, steerRatio=15.0),
    {
      Bus.chassis: 'tesla_can',
      Bus.party: 'tesla_can',
      Bus.pt: 'tesla_can',
      Bus.radar: 'tesla_radar_bosch_generated',
    },
  )
  TESLA_MODEL_S_HW2 = TeslaPlatformConfig(
    [CarDocs("Tesla Model S (with HW2) 2017-19", "All", car_parts=CarParts.common([CarHarness.tesla_model_sx_hw2]))],
    CarSpecs(mass=2100., wheelbase=2.960, steerRatio=15.0),
    {
      Bus.chassis: 'tesla_can',
      Bus.party: 'tesla_can',
      Bus.pt: 'tesla_powertrain',
      Bus.radar: 'tesla_radar_bosch_generated',
    },
  )
  TESLA_MODEL_S_HW3 = TeslaPlatformConfig(
    [CarDocs("Tesla Model S (with HW3) 2020-23", "All", car_parts=CarParts.common([CarHarness.tesla_model_sx_hw3]))],
    CarSpecs(mass=2100., wheelbase=2.960, steerRatio=15.0),
    {
      Bus.chassis: 'tesla_can',
      Bus.party: 'tesla_raven_party',
      Bus.pt: 'tesla_powertrain',
      Bus.radar: 'tesla_radar_continental_generated',
    },
  )
  TESLA_MODEL_S_PREAP = TeslaPlatformConfig(
    [CarDocs("Tesla Model S (Pre-AP) 2012-14", "All", car_parts=CarParts.common([CarHarness.tesla_model_s_hw1]))],
    CarSpecs(mass=2100., wheelbase=2.960, steerRatio=15.0),
    {
      Bus.chassis: 'tesla_preap',
      Bus.party: 'tesla_preap',
      Bus.pt: 'tesla_preap',
    },
  )


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.SUPPLIER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.SUPPLIER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    ),
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.UDS_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.UDS_VERSION_RESPONSE],
      rx_offset=0x08,
      bus=0,
    ),
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      rx_offset=0x08,
      bus=0,
    ),
  ]
)


class CANBUS:
  party = 0
  radar = 1
  autopilot_party = 2

  # only needed on raven
  powertrain = 4
  chassis = 5
  autopilot_powertrain = 6


GEAR_MAP = {
  "DI_GEAR_INVALID": CarState.GearShifter.unknown,
  "DI_GEAR_P": CarState.GearShifter.park,
  "DI_GEAR_R": CarState.GearShifter.reverse,
  "DI_GEAR_N": CarState.GearShifter.neutral,
  "DI_GEAR_D": CarState.GearShifter.drive,
  "DI_GEAR_SNA": CarState.GearShifter.unknown,
}


# Add extra tolerance for average banked road since safety doesn't have the roll
AVERAGE_ROAD_ROLL = 0.06  # ~3.4 degrees, 6% superelevation. higher actual roll lowers lateral acceleration


class CarControllerParams:
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    # EPAS faults above this angle
    360,  # deg
    # Tesla uses a vehicle model instead, check carcontroller.py for details
    ([], []),
    ([], []),

    # Vehicle model angle limits
    # Add extra tolerance for average banked road since safety doesn't have the roll
    MAX_LATERAL_ACCEL=ISO_LATERAL_ACCEL + (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL),  # ~3.6 m/s^2
    MAX_LATERAL_JERK=3.0 + (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL),  # ~3.6 m/s^3

    # limit angle rate to both prevent a fault and for low speed comfort (~12 mph rate down to 0 mph)
    MAX_ANGLE_RATE=5,  # deg/20ms frame, EPS faults at 12 at a standstill
  )

  STEER_STEP = 2  # Angle command is sent at 50 Hz
  ACCEL_MAX = 2.0    # m/s^2
  ACCEL_MIN = -3.48  # m/s^2
  JERK_LIMIT_MAX = 4.9  # m/s^3, ACC faults at 5.0
  JERK_LIMIT_MIN = -4.9  # m/s^3, ACC faults at 5.0

  # Pre-AP specific limits (regen braking only, no friction brakes)
  PREAP_ACCEL_MAX = 2.0    # m/s^2 (same as other Teslas)
  PREAP_ACCEL_MIN = -1.5   # m/s^2 (regen braking limit from Tinkla)


class TeslaLegacyParams(IntFlag):
  NO_SDM1 = 1


class TeslaSafetyFlags(IntFlag):
  LONG_CONTROL = 1
  FLAG_EXTERNAL_PANDA = 2
  FLAG_HW1 = 4
  FLAG_HW2 = 8
  FLAG_HW3 = 16
  FLAG_PREAP = 32


class TeslaFlags(IntFlag):
  LONG_CONTROL = 1

DBC = CAR.create_dbc_map()

STEER_THRESHOLD = 1

LEGACY_CARS = (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_S_HW2, CAR.TESLA_MODEL_S_HW3, CAR.TESLA_MODEL_X_HW1, CAR.TESLA_MODEL_S_PREAP)

# Pre-Autopilot cars - no DAS ECU, uses pedal interceptor for longitudinal
PREAP_CARS = (CAR.TESLA_MODEL_S_PREAP,)

# Tesla-specific accel limits (from Tinkla)
TESLA_MAX_ACCEL = 2.0   # m/s^2
TESLA_MIN_ACCEL = -3.48  # m/s^2 (with iBooster), -1.5 for regen only


class CruiseButtons:
    """Cruise control stalk button values from STW_ACTN_RQ.SpdCtrlLvr_Stat"""
    IDLE = 0
    CANCEL = 1
    MAIN = 2  # Pull stalk towards driver
    DECEL_SET = 3  # Push down 1st detent
    DECEL_2ND = 4  # Push down 2nd detent (5 mph/kph increment)
    RES_ACCEL = 5  # Push up 1st detent
    RES_ACCEL_2ND = 6  # Push up 2nd detent (5 mph/kph increment)

    @classmethod
    def should_be_throttled(cls, button):
        """Returns True if button press should be considered human action"""
        return button not in (cls.IDLE,)

    @classmethod
    def is_accel(cls, button):
        return button in (cls.RES_ACCEL, cls.RES_ACCEL_2ND)

    @classmethod
    def is_decel(cls, button):
        return button in (cls.DECEL_SET, cls.DECEL_2ND)


class CruiseState:
    """Cruise state values from DI_state.DI_cruiseState"""
    OFF = 0
    STANDBY = 1
    ENABLED = 2
    STANDSTILL = 3
    OVERRIDE = 4
    FAULT = 5
    PRE_FAULT = 6
    PRE_CANCEL = 7

    @classmethod
    def is_off(cls, state):
        return state in (cls.OFF, cls.STANDBY)

    @classmethod
    def is_enabled_or_standby(cls, state):
        return state in (cls.STANDBY, cls.ENABLED, cls.STANDSTILL, cls.OVERRIDE)


# CAN bus assignments for different Tesla platforms
CAN_CHASSIS = {
    CAR.TESLA_MODEL_S_PREAP: CANBUS.party,
    CAR.TESLA_MODEL_S_HW1: CANBUS.party,
    CAR.TESLA_MODEL_S_HW2: CANBUS.party,
    CAR.TESLA_MODEL_S_HW3: CANBUS.chassis,
    CAR.TESLA_MODEL_X_HW1: CANBUS.party,
}

CAN_POWERTRAIN = {
    CAR.TESLA_MODEL_S_PREAP: CANBUS.party,
    CAR.TESLA_MODEL_S_HW1: CANBUS.party,
    CAR.TESLA_MODEL_S_HW2: CANBUS.powertrain,
    CAR.TESLA_MODEL_S_HW3: CANBUS.powertrain,
    CAR.TESLA_MODEL_X_HW1: CANBUS.party,
}
