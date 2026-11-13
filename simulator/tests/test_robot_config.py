import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import yaml
from robot_config import DEFAULT_ROBOT, ROBOT_FIELDS, load_robot, motor_spec_summary
from run import simulate, validate


class RobotConfigTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.path = Path(directory.name) / 'custom-robot.yaml'
        self.data = yaml.safe_load(DEFAULT_ROBOT.read_text())
        self.write()

    def write(self):
        self.path.write_text(yaml.safe_dump(self.data))

    def test_all_robot_defaults_are_loaded_and_trial_overrides_win(self):
        self.data['body']['mass_kg'] = 1.1
        self.data['body']['center_of_mass_m']['x'] = .01
        self.data['imu']['position_m']['z'] = .22
        self.write()
        robot = load_robot(self.path)
        defaults = validate({}, robot)
        self.assertTrue(ROBOT_FIELDS.keys() <= defaults.keys())
        self.assertEqual(defaults['body_mass'], 1.1)
        self.assertEqual(defaults['com_x'], .01)
        self.assertEqual(defaults['imu_z'], .22)
        self.assertEqual(validate({'body_mass': 1.3}, robot)['body_mass'], 1.3)

    def test_yaml_changes_are_reread_and_reach_native_physics(self):
        trial = dict(kp=0, ki=0, kd=0, min_pwm=0, initial_angle=0, duration=1)
        centered = simulate(trial, self.path)
        self.assertEqual(centered['summary']['max_angle'], 0)
        self.data['body']['center_of_mass_m']['x'] = .03
        self.write()
        offset = simulate(trial, self.path)
        self.assertGreater(offset['summary']['max_angle'], 1)
        self.assertNotEqual(centered['robot_config']['sha256'], offset['robot_config']['sha256'])
        self.assertEqual(centered['build'], offset['build'])
        # Sensor position does not alter ideal orientation or mechanical dynamics.
        self.data['imu']['position_m']['x'] = .1
        self.write()
        moved_sensor = simulate(trial, self.path)
        self.assertEqual(moved_sensor['samples'], offset['samples'])

    def test_imu_mounting_changes_pitch_instead_of_only_metadata(self):
        self.data['imu']['mounting_deg']['pitch'] = 30
        self.write()
        trial = dict(kp=0, ki=0, kd=0, min_pwm=0, initial_angle=0, duration=1)
        mounted = simulate(trial, self.path)
        self.assertAlmostEqual(mounted['imu_upright_pitch'], 30)
        self.assertAlmostEqual(mounted['samples'][-1][2], 30, places=4)
        self.data['imu']['mounting_deg']['roll'] = 60
        self.write()
        rolled_sensor = simulate(trial, self.path)
        expected = math.degrees(math.atan2(math.sin(math.radians(30)),
                                          math.cos(math.radians(30))*math.cos(math.radians(60))))
        self.assertAlmostEqual(rolled_sensor['samples'][-1][2], expected, places=4)
        self.data['imu']['mounting_deg']['roll'] = 0
        # Rotation about z reverses sensitivity when mounted facing backwards.
        self.data['imu']['mounting_deg']['yaw'] = 180
        self.write()
        reversed_sensor = simulate(dict(trial, initial_angle=3), self.path)
        first = next(s for s in reversed_sensor['samples'] if s[13])
        self.assertAlmostEqual(first[2], 27, places=3)
        self.data['imu']['mounting_deg']['yaw'] = 0
        self.write()
        forward_sensor = simulate(dict(trial, initial_angle=3), self.path)
        first = next(s for s in forward_sensor['samples'] if s[13])
        self.assertAlmostEqual(first[2], 33, places=3)

    def test_headless_cli_accepts_custom_yaml_from_another_working_directory(self):
        self.data['base']['wheel_radius_m'] = .05
        self.write()
        process = subprocess.run(
            [sys.executable, str(DEFAULT_ROBOT.with_name('run.py')), '--headless', '--robot', str(self.path)],
            cwd=self.path.parent, capture_output=True, text=True, timeout=15, check=True)
        data = json.loads(process.stdout)
        self.assertEqual(data['settings']['wheel_radius'], .05)
        self.assertEqual(data['robot_config']['path'], str(self.path.resolve()))

    def test_missing_invalid_unknown_and_duplicate_values_fail_clearly(self):
        for text, message in [
            ('{}', 'Missing robot YAML fields'),
            (DEFAULT_ROBOT.read_text().replace('mass_kg: 0.7', 'mass_kg: -1'), 'body.mass_kg'),
            (DEFAULT_ROBOT.read_text().replace('mass_kg: 0.7', 'mass_kg: true'), 'finite number'),
            (DEFAULT_ROBOT.read_text().replace('gravity_m_s2: 9.81', 'gravity_m_s2: .nan'), 'finite number'),
            (DEFAULT_ROBOT.read_text()+'\nextra: 42\n', 'Unknown robot YAML fields'),
            (DEFAULT_ROBOT.read_text()+'\nbody: {}\n', 'Duplicate robot YAML key'),
            ('body: [', 'custom-robot.yaml'),
        ]:
            with self.subTest(message=message):
                self.path.write_text(text)
                with self.assertRaisesRegex(ValueError, message):
                    load_robot(self.path)
        self.path.unlink()
        with self.assertRaisesRegex(ValueError, 'custom-robot.yaml'):
            load_robot(self.path)

    def test_undefined_sensor_pitch_is_rejected(self):
        self.data['imu']['mounting_deg'].update(roll=90, pitch=0)
        self.write()
        with self.assertRaisesRegex(ValueError, 'pitch undefined'):
            load_robot(self.path)

    def test_motor_ratings_are_converted_once_at_the_gearbox_output(self):
        robot = load_robot(self.path)
        self.assertAlmostEqual(robot.values['max_force'], 2 * .2941995 / .035)
        self.assertAlmostEqual(robot.values['free_speed'], 150 * 2 * math.pi / 60 * .035)
        specs = motor_spec_summary(robot.motor_specs)
        self.assertAlmostEqual(specs['total_motor_mass_kg'], .202)
        self.assertAlmostEqual(specs['encoder_counts_per_output_rev'], 1632.672)
        self.assertAlmostEqual(specs['stall_output_torque_n_m'], .2941995)
        # Aggregate mass estimates already may include the motors; avoid double-counting.
        self.assertEqual(robot.values['body_mass'], self.data['body']['mass_kg'])
        self.assertEqual(robot.values['axle_mass'], self.data['base']['effective_mass_kg'])

    def test_confirmed_wheels_are_loaded_and_reported_without_double_counting(self):
        data = simulate({'duration': 1}, self.path)
        self.assertEqual(data['settings']['wheel_radius'], .035)
        self.assertAlmostEqual(data['wheel_specs']['diameter_m'], .07)
        self.assertEqual(data['wheel_specs']['width_m'], .008)
        self.assertEqual(data['wheel_specs']['count'], 2)
        self.assertAlmostEqual(data['wheel_specs']['mass_each_kg'], .5*.028349523125, delta=.000001)
        self.assertAlmostEqual(data['wheel_specs']['total_wheel_mass_kg'], .02835)
        self.assertEqual(data['settings']['axle_mass'], self.data['base']['effective_mass_kg'])
        self.assertAlmostEqual(data['settings']['max_force'], 16.8114)
        self.data['base']['wheels']['mass_each_kg'] = -1
        self.write()
        with self.assertRaisesRegex(ValueError, 'base.wheels.mass_each_kg'):
            load_robot(self.path)

    def test_motor_force_and_speed_follow_radius_voltage_and_explicit_overrides(self):
        robot = load_robot(self.path)
        nominal = validate({}, robot)
        larger_wheel = validate({'wheel_radius': .07}, robot)
        self.assertAlmostEqual(larger_wheel['max_force'], nominal['max_force']/2)
        self.assertAlmostEqual(larger_wheel['free_speed'], nominal['free_speed']*2)
        half_voltage = validate({'motor_voltage': 6}, robot)
        self.assertAlmostEqual(half_voltage['max_force'], nominal['max_force']/2)
        self.assertAlmostEqual(half_voltage['free_speed'], nominal['free_speed']/2)
        explicit = validate({'wheel_radius': .07, 'max_force': 8, 'free_speed': 3}, robot)
        self.assertEqual(explicit['max_force'], 8)
        self.assertEqual(explicit['free_speed'], 3)
        self.data['base']['wheel_radius_m'] = .07
        self.data['motors']['supply_voltage_v'] = 6
        self.write()
        modified = load_robot(self.path)
        self.assertAlmostEqual(modified.values['max_force'], nominal['max_force']/4)
        self.assertAlmostEqual(modified.values['free_speed'], nominal['free_speed'])

    def test_invalid_motor_specs_fail_before_derivation(self):
        for key, value in [('rated_voltage_v', 0), ('stall_output_torque_kgf_cm', True),
                           ('no_load_output_speed_rpm', float('inf')),
                           ('encoder_counts_per_motor_rev', 48.5), ('stall_current_a', .05)]:
            with self.subTest(key=key):
                self.data = yaml.safe_load(DEFAULT_ROBOT.read_text())
                self.data['motors']['specification'][key] = value
                self.write()
                with self.assertRaisesRegex(ValueError, key):
                    load_robot(self.path)


if __name__ == '__main__':
    unittest.main()
