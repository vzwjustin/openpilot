import math
from unittest import mock

import openpilot.cereal.messaging as messaging
from openpilot.common.test import OpenpilotTestCase
from openpilot.common.parameterized import parameterized
from opendbc.car.car_helpers import interfaces
from opendbc.car.toyota.values import CAR as TOYOTA
from opendbc.car.nissan.values import CAR as NISSAN
from opendbc.car.ford.values import CAR as FORD
from opendbc.car.tesla.values import CAR as TESLA
from openpilot.selfdrive.controls import controlsd

SUB_SERVICES = ['lateralDelay', 'vehicleParameters', 'lateralTorqueParameters', 'modelV2', 'selfdriveState',
                'extrinsicsCalibration', 'deviceMotion', 'longitudinalPlan', 'lateralManeuverPlan', 'carState', 'carOutput',
                'driverMonitoringState', 'onroadEvents', 'driverAssistance']


class FakeSubMaster:
  def __init__(self, *args, **kwargs):
    self.msgs = {s: messaging.new_message(s, 0) if s == 'onroadEvents' else messaging.new_message(s) for s in SUB_SERVICES}
    self.data = {s: getattr(m, s) for s, m in self.msgs.items()}
    self.valid = dict.fromkeys(SUB_SERVICES, True)
    self.valid['lateralManeuverPlan'] = False  # only valid while lateral_maneuversd runs a maneuver
    self.updated = dict.fromkeys(SUB_SERVICES, False)
    self.logMonoTime = dict.fromkeys(SUB_SERVICES, 0)

  def __getitem__(self, s):
    return self.data[s]

  def all_checks(self, service_list=None):
    return True


class FakePubMaster:
  def __init__(self, *args, **kwargs):
    self.sent = {}

  def send(self, s, dat):
    self.sent[s] = dat


class TestControlsdNonFinite(OpenpilotTestCase):

  def _make_controls(self, car_name):
    CP = interfaces[car_name].get_non_essential_params(car_name)
    params = mock.MagicMock()
    params.get.return_value = CP.to_bytes()
    with mock.patch.object(controlsd, 'Params', return_value=params), \
         mock.patch.object(controlsd.messaging, 'SubMaster', FakeSubMaster), \
         mock.patch.object(controlsd.messaging, 'PubMaster', FakePubMaster):
      controls = controlsd.Controls()

    sm = controls.sm
    sm.data['selfdriveState'].enabled = True
    sm.data['selfdriveState'].active = True
    sm.data['carState'].vEgo = 25.
    sm.data['carState'].canValid = True
    sm.data['vehicleParameters'].steerRatio = CP.steerRatio
    sm.data['vehicleParameters'].stiffnessFactor = 1.
    sm.data['lateralDelay'].lateralDelay = 0.2
    return controls

  def _step(self, controls, desired_curvature):
    controls.sm.data['modelV2'].action.desiredCurvature = desired_curvature
    CC, lac_log = controls.state_control()
    controls.publish(CC, lac_log)
    return CC, controls.pm.sent['controlsState']

  # torque control, angle control (Nissan, and Tesla which uses carOutput for saturation),
  # and Ford (angle type, but its carcontroller sends actuators.curvature)
  @parameterized.expand([(TOYOTA.TOYOTA_RAV4, 'torque'), (NISSAN.NISSAN_LEAF, 'steeringAngleDeg'),
                         (TESLA.TESLA_MODEL_3, 'steeringAngleDeg'), (FORD.FORD_EXPLORER_MK6, 'curvature')])
  def test_non_finite_curvature_does_not_latch(self, car_name, lateral_actuator):
    controls = self._make_controls(car_name)

    for _ in range(100):
      CC, controls_state = self._step(controls, 0.002)
    assert CC.latActive and controls_state.valid
    good_curvature = controls.desired_curvature

    # one bad model frame: hold the last good request and flag controlsState invalid so selfdrived alerts
    CC, controls_state = self._step(controls, float('nan'))
    assert not controls_state.valid
    assert math.isfinite(controls.desired_curvature)
    assert abs(controls.desired_curvature - good_curvature) < 1e-4

    # the model recovers: so must lateral control, without a re-engage or controlsd restart
    for _ in range(200):
      CC, controls_state = self._step(controls, 0.002)
    assert controls_state.valid
    assert math.isfinite(controls.desired_curvature)
    assert abs(controls.desired_curvature - 0.002) < 1e-4
    lateral_output = getattr(CC.actuators, lateral_actuator)
    assert math.isfinite(lateral_output) and abs(lateral_output) > 1e-3
