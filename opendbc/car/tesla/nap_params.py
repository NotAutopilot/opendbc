"""
NAP (NotAutopilot) Parameter Module

Provides helper functions for reading/writing NAP parameters via openpilot's Params system.
Replaces Tinkla's CFG_module.py with a more robust implementation.
"""

from openpilot.common.params import Params


class NAPParamKeys:
    """NAP-specific parameter keys"""
    # Pedal Interceptor
    PEDAL_ENABLED = "NAPPedalEnabled"
    PEDAL_PROFILE = "NAPPedalProfile"
    PEDAL_CAN_BUS = "NAPPedalCanBus"
    PEDAL_CALIB_MIN = "NAPPedalCalibMin"
    PEDAL_CALIB_MAX = "NAPPedalCalibMax"
    PEDAL_CALIB_FACTOR = "NAPPedalCalibFactor"
    PEDAL_CALIB_ZERO = "NAPPedalCalibZero"
    PEDAL_CALIB_DONE = "NAPPedalCalibDone"

    # Follow Distance
    FOLLOW_DISTANCE = "NAPFollowDistance"

    # Radar
    RADAR_ENABLED = "NAPRadarEnabled"
    RADAR_BEHIND_NOSECONE = "NAPRadarBehindNosecone"

    # iBooster
    IBOOSTER_ENABLED = "NAPiBoosterEnabled"
    BRAKE_FACTOR = "NAPBrakeFactor"

    # Cruise Control
    DISABLE_CRUISE_CONTROL = "NAPDisableCruiseControl"

    # Force Fingerprint
    FORCE_PREAP = "NAPForcePreAP"

    # Speed Limit
    SPEED_LIMIT_OFFSET = "NAPSpeedLimitOffset"
    ADJUST_ACC_WITH_SPEED_LIMIT = "NAPAdjustAccWithSpeedLimit"
    SPEED_LIMIT_USE_RELATIVE = "NAPSpeedLimitUseRelative"
    USE_LONG_CONTROL_DATA = "NAPUseLongControlData"
    AUTOPILOT_DISABLED = "NAPDisableCruiseControl"

    # Human Override
    HSO_ENABLED = "NAPHSOEnabled"
    HSO_NUMB_PERIOD = "NAPHSONumbPeriod"
    HAO_ENABLED = "NAPHAOEnabled"

    # Autoresume
    AUTORESUME_ACC = "NAPAutoresumeAcc"
    ENABLE_JUST_CC = "NAPEnableJustCC"


_params = None


def _get_params():
    """Get or create Params instance"""
    global _params
    if _params is None:
        _params = Params()
    return _params


def save_bool_param(param_name: str, param_value: bool) -> None:
    """Save a boolean parameter"""
    try:
        _get_params().put_bool(param_name, param_value)
    except Exception as e:
        print(f"Failed to save {param_name} with value {param_value}: {e}")


def load_bool_param(param_name: str, param_def_value: bool) -> bool:
    """Load a boolean parameter, initializing with default if not found"""
    try:
        return _get_params().get_bool(param_name)
    except Exception:
        print(f"Initializing {param_name} with value {param_def_value}")
        save_bool_param(param_name, param_def_value)
        return param_def_value


def save_float_param(param_name: str, param_value: float) -> None:
    """Save a float parameter"""
    try:
        _get_params().put(param_name, str(float(param_value)))
    except Exception as e:
        print(f"Failed to save {param_name} with value {param_value}: {e}")


def load_float_param(param_name: str, param_def_value: float) -> float:
    """Load a float parameter, initializing with default if not found"""
    try:
        val = _get_params().get(param_name)
        if val is not None:
            return float(val.decode('utf-8'))
    except Exception:
        pass
    print(f"Initializing {param_name} with value {param_def_value}")
    save_float_param(param_name, param_def_value)
    return param_def_value


def save_int_param(param_name: str, param_value: int) -> None:
    """Save an integer parameter"""
    try:
        _get_params().put(param_name, str(int(param_value)))
    except Exception as e:
        print(f"Failed to save {param_name} with value {param_value}: {e}")


def load_int_param(param_name: str, param_def_value: int) -> int:
    """Load an integer parameter, initializing with default if not found"""
    try:
        val = _get_params().get(param_name)
        if val is not None:
            return int(val.decode('utf-8'))
    except Exception:
        pass
    print(f"Initializing {param_name} with value {param_def_value}")
    save_int_param(param_name, param_def_value)
    return param_def_value
