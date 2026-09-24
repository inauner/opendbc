from dataclasses import dataclass, field

from opendbc.car import Bus, CanBusBase, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu

class CarControllerParams:
  STEER_STEP = 1
  HUD_2_STEP = 25
  HUD_3_STEP = 100

  # Development limits: within both directions of the observed stock range (-68..34).
  STEER_MAX = 34
  LKA_MAX_REQUEST_NS = 3_500_000_000
  LKA_RAMP_DOWN_NS = 3_000_000_000
  LKA_ZERO_TORQUE_NS = 3_200_000_000
  LKA_RAMP_DELTA = 2
  LKA_RESET_NS = 2_000_000_000
  STEER_DRIVER_ALLOWANCE = 80
  STEER_DRIVER_MULTIPLIER = 3  # weight driver torque heavily
  STEER_DRIVER_FACTOR = 1  # from dbc
  STEER_DELTA_UP = 4
  STEER_DELTA_DOWN = 4

  def __init__(self, CP):
    pass


class CanBus(CanBusBase):
  def __init__(self, CP=None, fingerprint=None) -> None:
    super().__init__(CP, fingerprint)

  @property
  def pt(self) -> int:
    return self.offset

  @property
  def cam(self) -> int:
    return self.offset + 2


@dataclass
class FcaGiorgioPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'fca_giorgio'})


@dataclass(frozen=True, kw_only=True)
class FcaGiorgioCarSpecs(CarSpecs):
  centerToFrontRatio: float = 0.45
  steerRatio: float = 14.2


@dataclass
class FcaGiorgioCarDocs(CarDocs):
  package: str = "Adaptive Cruise Control (ACC) & Lane Assist"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.vw_a]))


class CAR(Platforms):
  config: FcaGiorgioPlatformConfig

  # ALFA_ROMEO_STELVIO_1ST_GEN = FcaGiorgioPlatformConfig(
  #   [FcaGiorgioCarDocs("Alfa Romeo Stelvio 2017-24")],
  #   FcaGiorgioCarSpecs(mass=1660, wheelbase=2.82),
  # )

  RAM_PROMASTER = FcaGiorgioPlatformConfig(
    [FcaGiorgioCarDocs("RAM ProMaster 2024-26")],
    # rough specs for Promaster 2500 with medium wheelbase
    FcaGiorgioCarSpecs(mass=2177, wheelbase=3.45),
  )

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.UDS_VERSION_REQUEST],
      [StdQueries.UDS_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.eps],
      bus=0,
    ),
  ],
)


DBC = CAR.create_dbc_map()
