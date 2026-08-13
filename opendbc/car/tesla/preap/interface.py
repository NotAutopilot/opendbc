from opendbc.car import structs
from opendbc.car.tesla.preap.boot import (
  apply_preap_capabilities,
  apply_preap_identity,
  is_preap_platform,
)


def get_preap_params(ret: structs.CarParams, fingerprint) -> structs.CarParams:
  del fingerprint  # identity is platform-locked; hardware comes from the boot snapshot
  return apply_preap_identity(ret)


def get_preap_params_sp(ret: structs.CarParamsSP) -> structs.CarParamsSP:
  return apply_preap_capabilities(ret)


__all__ = ["get_preap_params", "get_preap_params_sp", "is_preap_platform"]
