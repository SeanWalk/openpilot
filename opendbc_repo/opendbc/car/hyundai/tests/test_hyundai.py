import unittest

from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.structs import CarParams
from opendbc.car.fw_versions import build_fw_dict, match_fw_to_car
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.radar_interface import RADAR_START_ADDR
from opendbc.car.hyundai.values import CAR, DBC, DATE_FW_ECUS, Buttons, \
                                         FW_QUERY_CONFIG, CANFD_FUZZY_WHITELIST, \
                                         PLATFORM_CODE_ECUS, HYUNDAI_VERSION_REQUEST_LONG, \
                                         HyundaiFlags, get_platform_codes, HyundaiSafetyFlags, \
                                         NON_SCC_CAR
from opendbc.sunnypilot.car.hyundai.longitudinal.helpers import get_longitudinal_tune
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP, HyundaiSafetyFlagsSP
from opendbc.car.hyundai.fingerprints import FW_VERSIONS
from opendbc.testing import fuzzy_test

Ecu = CarParams.Ecu

# Some platforms have date codes in a different format we don't yet parse (or are missing).
# For now, assert list of expected missing date cars
NO_DATES_PLATFORMS = {
  # CAN FD
  CAR.KIA_SPORTAGE_5TH_GEN,
  CAR.HYUNDAI_SANTA_CRUZ_1ST_GEN,
  CAR.HYUNDAI_TUCSON_4TH_GEN,
  # CAN
  CAR.HYUNDAI_ELANTRA,
  CAR.HYUNDAI_ELANTRA_GT_I30,
  CAR.KIA_CEED,
  CAR.KIA_CEED_PHEV,
  CAR.KIA_FORTE,
  CAR.KIA_OPTIMA_G4,
  CAR.KIA_OPTIMA_G4_FL,
  CAR.KIA_SORENTO,
  CAR.HYUNDAI_KONA,
  CAR.HYUNDAI_KONA_EV,
  CAR.HYUNDAI_KONA_EV_2022,
  CAR.HYUNDAI_KONA_HEV,
  CAR.HYUNDAI_SONATA_LF,
  CAR.HYUNDAI_VELOSTER,
  CAR.HYUNDAI_KONA_2022,
}

CANFD_EXPECTED_ECUS = {Ecu.fwdCamera, Ecu.fwdRadar}

# CAN-only feature flags that should not appear on CAN FD platforms
CAN_FEATURE_FLAGS = (HyundaiFlags.CLUSTER_GEARS | HyundaiFlags.TCU_GEARS | HyundaiFlags.CHECKSUM_CRC8 |
                     HyundaiFlags.CHECKSUM_6B | HyundaiFlags.LEGACY | HyundaiFlags.UNSUPPORTED_LONGITUDINAL |
                     HyundaiFlags.CAMERA_SCC)


def cars_with(flags):
  return {c for c in CAR if c.config.flags & flags}


class TestHyundaiFingerprint(unittest.TestCase):
  def test_mufasa_firmware_match_without_eps(self):
    car_fw = [
      CarParams.CarFw(ecu=Ecu.fwdRadar, address=0x7d0, brand="hyundai",
                      fwVersion=b'\xf1\x00NU2_ RDR -----      1.00 1.00 99110-MA300         '),
      CarParams.CarFw(ecu=Ecu.fwdRadar, address=0x7d0, brand="hyundai",
                      fwVersion=b'\xf1\x8b #\x07\x16'),
      CarParams.CarFw(ecu=Ecu.combinationMeter, address=0x7c6, brand="hyundai", fwVersion=b'\xf1\x002C6'),
      CarParams.CarFw(ecu=Ecu.fwdCamera, address=0x7c4, brand="hyundai",
                      fwVersion=b'\xf1\x00NU2 MFC  AT EUR LHD 1.00 1.02 99211-MA300 230502'),
      CarParams.CarFw(ecu=Ecu.hvac, address=0x7b3, brand="hyundai",
                      fwVersion=b'\xf1\x00NU2\x00\x00\x0097255-MA000UNIT-HEATER CONTROL\x001.00.11NU2DATC(-)0.1\xaa\xaa\xaa\xaa\xaa'),
    ]
    _, matches = match_fw_to_car(car_fw, "", allow_fuzzy=False, log=False)
    assert matches == {CAR.HYUNDAI_MUFASA}

    fingerprint = gen_empty_fingerprint()
    fingerprint[CanBus(None, fingerprint).CAM] = [0x110]
    CP = CarInterface.get_params(CAR.HYUNDAI_MUFASA, fingerprint, car_fw, False, False, False)
    assert CP.pcmCruise
    assert CP.flags & HyundaiFlags.CANFD
    assert CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG

  def test_feature_detection(self):
    # LKA steering
    for lka_steering in (True, False):
      fingerprint = gen_empty_fingerprint()
      if lka_steering:
        cam_can = CanBus(None, fingerprint).CAM
        fingerprint[cam_can] = [0x50, 0x110]  # LKA steering messages
      CP = CarInterface.get_params(CAR.KIA_EV6, fingerprint, [], False, False, False)
      assert bool(CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG) == lka_steering

    # radar available
    for radar in (True, False):
      fingerprint = gen_empty_fingerprint()
      if radar:
        fingerprint[1][RADAR_START_ADDR] = 8
      CP = CarInterface.get_params(CAR.HYUNDAI_SONATA, fingerprint, [], False, False, False)
      assert CP.radarUnavailable != radar

  def test_mufasa_alternate_cruise_buttons_with_lka(self):
    fingerprint = gen_empty_fingerprint()
    fingerprint[1][0x1aa] = 8  # CRUISE_BUTTONS_ALT on ECAN
    fingerprint[2][0x110] = 8  # LKAS_ALT on camera CAN
    CP = CarInterface.get_params(CAR.HYUNDAI_MUFASA, fingerprint, [], False, False, False)
    CAN = CanBus(CP)
    assert (CAN.ECAN, CAN.ACAN, CAN.CAM) == (1, 0, 2)
    assert CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS
    assert CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG
    assert CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_LKA_STEER_MSG
    assert CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_ALT_BUTTONS
    assert not CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CAMERA_SCC
    assert not CP.flags & HyundaiFlags.CANFD_LFA_SUPPRESS

    # LFA suppression is enabled only when the camera block frame exists.
    fingerprint[2][0x362] = 32
    CP_WITH_SUPPRESS = CarInterface.get_params(CAR.HYUNDAI_MUFASA, fingerprint, [], False, False, False)
    assert CP_WITH_SUPPRESS.flags & HyundaiFlags.CANFD_LFA_SUPPRESS
    assert not CP_WITH_SUPPRESS.flags & (HyundaiFlags.HYBRID | HyundaiFlags.CLUSTER_GEARS)

  def test_mufasa_longitudinal_tuning_and_o_button(self):
    fingerprint = gen_empty_fingerprint()
    fingerprint[0][0x1aa] = 8  # CRUISE_BUTTONS_ALT on ECAN (LFA steering)
    CP = CarInterface.get_params(CAR.HYUNDAI_MUFASA, fingerprint, [], True, False, False)
    CP_SP = CarInterface.get_params_sp(CP, CAR.HYUNDAI_MUFASA, fingerprint, [], True, False, False)
    CP_SP.flags |= HyundaiFlagsSP.LONGITUDINAL_MAIN_CRUISE_TOGGLEABLE.value

    assert CP.openpilotLongitudinalControl
    assert not CP.pcmCruise
    self.assertAlmostEqual(CP.longitudinalActuatorDelay, 0.65, places=5)
    self.assertAlmostEqual(CP.stopAccel, -2.5, places=5)
    assert list(CP.longitudinalTuning.kiBP) == [0., 5., 35.]
    for actual, expected in zip(CP.longitudinalTuning.kiV, [0.35, 0.25, 0.15], strict=True):
      self.assertAlmostEqual(actual, expected, places=5)
    assert CP_SP.safetyParam & HyundaiSafetyFlagsSP.CANCEL_BTN_RESUME

    # Dynamic/Predictive tuning helper must preserve Mufasa's 0.65s actuator delay
    get_longitudinal_tune(CP)
    self.assertAlmostEqual(CP.longitudinalActuatorDelay, 0.65, places=5)

    # Stock longitudinal should not enable CANCEL_BTN_RESUME
    CP_STOCK = CarInterface.get_params(CAR.HYUNDAI_MUFASA, fingerprint, [], False, False, False)
    CP_STOCK_SP = CarInterface.get_params_sp(CP_STOCK, CAR.HYUNDAI_MUFASA, fingerprint, [], False, False, False)
    assert not (CP_STOCK_SP.safetyParam & HyundaiSafetyFlagsSP.CANCEL_BTN_RESUME)

    # Test Mufasa O-button (0x1AA btn=4) Pause/Resume state machine
    CS = CarState(CP, CP_SP)
    parsers = CS.get_can_parsers(CP, CP_SP)
    packer = CANPacker(DBC[CAR.HYUNDAI_MUFASA][Bus.pt])
    ButtonType = structs.CarState.ButtonEvent.Type

    t = 0
    def step(cruise_btn=Buttons.NONE, main_btn=0, acc_req=0):
      nonlocal t
      t += 10_000_000
      msgs = [
        packer.make_can_msg("TCS", 0, {"ACCEnable": 0, "ACC_REQ": acc_req}),
        packer.make_can_msg("CRUISE_BUTTONS_ALT", 0, {
          "CRUISE_BUTTONS": cruise_btn,
          "ADAPTIVE_CRUISE_MAIN_BTN": main_btn,
        }),
      ]
      parsers[Bus.pt].update([(t, msgs)])
      parsers[Bus.cam].update([(t, [])])
      ret, _ = CS.update(parsers)
      return ret

    # Initial frame: main cruise off (available=False)
    ret = step()
    assert not ret.cruiseState.available

    # O-button while main cruise off -> cancel
    ret = step(cruise_btn=Buttons.CANCEL)
    assert [(e.type, e.pressed) for e in ret.buttonEvents] == [(ButtonType.cancel, True)]
    ret = step(cruise_btn=Buttons.NONE)
    assert [(e.type, e.pressed) for e in ret.buttonEvents] == [(ButtonType.cancel, False)]

    # Press mainCruise -> enters standby (available=True, enabled=False)
    ret = step(main_btn=1)
    assert ret.cruiseState.available and not ret.cruiseState.enabled
    step(main_btn=0)

    # O-button while paused standby with unset speed -> decelCruise (set current speed)
    ret = step(cruise_btn=Buttons.CANCEL)
    assert [(e.type, e.pressed) for e in ret.buttonEvents] == [(ButtonType.decelCruise, True)]
    ret = step(cruise_btn=Buttons.NONE)
    assert [(e.type, e.pressed) for e in ret.buttonEvents] == [(ButtonType.decelCruise, False)]

    # Now engaged (ACC_REQ=1): O-button press disengages with cancel, and stays cancel on release even after ACC_REQ drops
    ret = step(acc_req=1)
    assert ret.cruiseState.available and ret.cruiseState.enabled
    ret = step(cruise_btn=Buttons.CANCEL, acc_req=1)
    assert [(e.type, e.pressed) for e in ret.buttonEvents] == [(ButtonType.cancel, True)]
    ret = step(cruise_btn=Buttons.NONE, acc_req=0)
    assert [(e.type, e.pressed) for e in ret.buttonEvents] == [(ButtonType.cancel, False)]
    assert ret.cruiseState.available and not ret.cruiseState.enabled

    # Now paused standby with speed already set -> O-button resumes with accelCruise
    ret = step(cruise_btn=Buttons.CANCEL, acc_req=0)
    assert [(e.type, e.pressed) for e in ret.buttonEvents] == [(ButtonType.accelCruise, True)]
    ret = step(cruise_btn=Buttons.NONE, acc_req=0)
    assert [(e.type, e.pressed) for e in ret.buttonEvents] == [(ButtonType.accelCruise, False)]

    # Press mainCruise again -> turns main cruise off and resets stored speed state
    ret = step(main_btn=1, acc_req=0)
    assert not ret.cruiseState.available
    assert not CS.mufasa_v_cruise_set

  def test_alternate_limits(self):
    # Alternate lateral control limits, for high torque cars, verify Panda safety mode flag is set
    fingerprint = gen_empty_fingerprint()
    for car_model in CAR:
      CP = CarInterface.get_params(car_model, fingerprint, [], False, False, False)
      assert bool(CP.flags & HyundaiFlags.ALT_LIMITS) == bool(CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.ALT_LIMITS)

  def test_can_features(self):
    for car_model in CAR:
      flags = car_model.config.flags
      # Test no EV/HEV with cluster/TCU gears (should all use ELECT_GEAR)
      if flags & (HyundaiFlags.HYBRID | HyundaiFlags.EV):
        assert not (flags & (HyundaiFlags.CLUSTER_GEARS | HyundaiFlags.TCU_GEARS))

      # Test CAN FD car not in CAN feature lists
      if flags & HyundaiFlags.CANFD:
        assert not (flags & CAN_FEATURE_FLAGS), "CAN FD car unexpectedly has a CAN feature flag"

  def test_hybrid_ev_flags(self):
    for car_model in CAR:
      flags = car_model.config.flags
      assert not (flags & HyundaiFlags.HYBRID and flags & HyundaiFlags.EV), "Shared cars between hybrid and EV"
      assert not (flags & HyundaiFlags.CANFD and flags & HyundaiFlags.HYBRID), \
        "Hard coding CAN FD cars as hybrid is no longer supported"

  def test_canfd_ecu_whitelist(self):
    # Asserts only expected Ecus can exist in database for CAN-FD cars
    for car_model in CAR:
      if not (car_model.config.flags & HyundaiFlags.CANFD):
        continue
      ecus = {fw[0] for fw in FW_VERSIONS[car_model].keys()}
      ecus_not_in_whitelist = ecus - CANFD_EXPECTED_ECUS
      ecu_strings = ", ".join([f"Ecu.{ecu}" for ecu in ecus_not_in_whitelist])
      assert len(ecus_not_in_whitelist) == 0, \
                       f"{car_model}: Car model has unexpected ECUs: {ecu_strings}"

  def test_blacklisted_parts(self):
    # Asserts no ECUs known to be shared across platforms exist in the database.
    # Tucson having Santa Cruz camera and EPS for example
    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        if car_model == CAR.HYUNDAI_SANTA_CRUZ_1ST_GEN:
          raise unittest.SkipTest("Skip checking Santa Cruz for its parts")

        for code, _ in get_platform_codes(ecus[(Ecu.fwdCamera, 0x7c4, None)]):
          if b"-" not in code:
            continue
          part = code.split(b"-")[1]
          assert not part.startswith(b'CW'), "Car has bad part number"

  def test_correct_ecu_response_database(self):
    """
    Assert standard responses for certain ECUs, since they can
    respond to multiple queries with different data
    """
    expected_fw_prefix = HYUNDAI_VERSION_REQUEST_LONG[1:]
    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        for ecu, fws in ecus.items():
          assert all(fw.startswith(expected_fw_prefix) for fw in fws), \
                          f"FW from unexpected request in database: {(ecu, fws)}"

  @fuzzy_test(max_examples=100)
  def test_platform_codes_fuzzy_fw(self, fuzzy):
    """Ensure function doesn't raise an exception"""
    get_platform_codes(fuzzy.list(fuzzy.binary))

  def test_expected_platform_codes(self):
    # Ensures we don't accidentally add multiple platform codes for a car unless it is intentional
    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        for ecu, fws in ecus.items():
          if ecu[0] not in PLATFORM_CODE_ECUS:
            continue

          # Third and fourth character are usually EV/hybrid identifiers
          codes = {code.split(b"-")[0][:2] for code, _ in get_platform_codes(fws)}
          if car_model == CAR.HYUNDAI_PALISADE:
            assert codes == {b"LX", b"ON"}, f"Car has unexpected platform codes: {car_model} {codes}"
          elif car_model == CAR.HYUNDAI_KONA_EV and ecu[0] == Ecu.fwdCamera:
            assert codes == {b"OE", b"OS"}, f"Car has unexpected platform codes: {car_model} {codes}"
          else:
            assert len(codes) == 1, f"Car has multiple platform codes: {car_model} {codes}"

  # Tests for platform codes, part numbers, and FW dates which Hyundai will use to fuzzy
  # fingerprint in the absence of full FW matches:
  def test_platform_code_ecus_available(self):
    # TODO: add queries for these non-CAN FD cars to get EPS
    no_eps_platforms = cars_with(HyundaiFlags.CANFD) | {CAR.KIA_SORENTO, CAR.KIA_OPTIMA_G4, CAR.KIA_OPTIMA_G4_FL, CAR.KIA_OPTIMA_H, CAR.KIA_K7_2017,
                                    CAR.KIA_OPTIMA_H_G4_FL, CAR.HYUNDAI_SONATA_LF, CAR.HYUNDAI_TUCSON, CAR.GENESIS_G90, CAR.GENESIS_G80, CAR.HYUNDAI_ELANTRA}

    # Asserts ECU keys essential for fuzzy fingerprinting are available on all platforms
    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        for platform_code_ecu in PLATFORM_CODE_ECUS:
          if platform_code_ecu in (Ecu.fwdRadar, Ecu.eps) and car_model == CAR.HYUNDAI_GENESIS:
            continue
          if platform_code_ecu == Ecu.eps and car_model in no_eps_platforms:
            continue
          if car_model in NON_SCC_CAR:
            continue
          assert platform_code_ecu in [e[0] for e in ecus]

  def test_fw_format(self):
    # Asserts:
    # - every supported ECU FW version returns one platform code
    # - every supported ECU FW version has a part number
    # - expected parsing of ECU FW dates

    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        for ecu, fws in ecus.items():
          if ecu[0] not in PLATFORM_CODE_ECUS:
            continue

          if car_model in NON_SCC_CAR:
            continue

          codes = set()
          for fw in fws:
            result = get_platform_codes([fw])
            assert 1 == len(result), f"Unable to parse FW: {fw}"
            codes |= result

          if ecu[0] not in DATE_FW_ECUS or car_model in NO_DATES_PLATFORMS:
            assert all(date is None for _, date in codes)
          else:
            assert all(date is not None for _, date in codes)

          if car_model == CAR.HYUNDAI_GENESIS:
            raise unittest.SkipTest("No part numbers for car model")

          # Hyundai places the ECU part number in their FW versions, assert all parsable
          # Some examples of valid formats: b"56310-L0010", b"56310L0010", b"56310/M6300"
          assert all(b"-" in code for code, _ in codes), \
                          f"FW does not have part number: {fw}"

  def test_platform_codes_spot_check(self):
    # Asserts basic platform code parsing behavior for a few cases
    results = get_platform_codes([b"\xf1\x00DH LKAS 1.1 -150210"])
    assert results == {(b"DH", b"150210")}

    # Some cameras and all radars do not have dates
    results = get_platform_codes([b"\xf1\x00AEhe SCC H-CUP      1.01 1.01 96400-G2000         "])
    assert results == {(b"AEhe-G2000", None)}

    results = get_platform_codes([b"\xf1\x00CV1_ RDR -----      1.00 1.01 99110-CV000         "])
    assert results == {(b"CV1-CV000", None)}

    results = get_platform_codes([
      b"\xf1\x00DH LKAS 1.1 -150210",
      b"\xf1\x00AEhe SCC H-CUP      1.01 1.01 96400-G2000         ",
      b"\xf1\x00CV1_ RDR -----      1.00 1.01 99110-CV000         ",
    ])
    assert results == {(b"DH", b"150210"), (b"AEhe-G2000", None), (b"CV1-CV000", None)}

    results = get_platform_codes([
      b"\xf1\x00LX2 MFC  AT USA LHD 1.00 1.07 99211-S8100 220222",
      b"\xf1\x00LX2 MFC  AT USA LHD 1.00 1.08 99211-S8100 211103",
      b"\xf1\x00ON  MFC  AT USA LHD 1.00 1.01 99211-S9100 190405",
      b"\xf1\x00ON  MFC  AT USA LHD 1.00 1.03 99211-S9100 190720",
    ])
    assert results == {(b"LX2-S8100", b"220222"), (b"LX2-S8100", b"211103"),
                               (b"ON-S9100", b"190405"), (b"ON-S9100", b"190720")}

  def test_fuzzy_excluded_platforms(self):
    # Asserts a list of platforms that will not fuzzy fingerprint with platform codes due to them being shared.
    # This list can be shrunk as we combine platforms and detect features
    excluded_platforms = {
      CAR.GENESIS_G70,            # shared platform code, part number, and date
      CAR.GENESIS_G70_2020,
    }
    excluded_platforms |= cars_with(HyundaiFlags.CANFD) - cars_with(HyundaiFlags.EV) - CANFD_FUZZY_WHITELIST  # shared platform codes
    excluded_platforms |= NO_DATES_PLATFORMS  # date codes are required to match

    platforms_with_shared_codes = set()
    for platform, fw_by_addr in FW_VERSIONS.items():
      car_fw = []
      for ecu, fw_versions in fw_by_addr.items():
        ecu_name, addr, sub_addr = ecu
        for fw in fw_versions:
          car_fw.append(CarParams.CarFw(ecu=ecu_name, fwVersion=fw, address=addr,
                                        subAddress=0 if sub_addr is None else sub_addr))

      if platform in NON_SCC_CAR:
        continue

      CP = CarParams(carFw=car_fw)
      matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(build_fw_dict(CP.carFw), CP.carVin, FW_VERSIONS)
      if len(matches) == 1:
        assert list(matches)[0] == platform
      else:
        platforms_with_shared_codes.add(platform)

    assert platforms_with_shared_codes == excluded_platforms
