import concurrent.futures
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from run import simulate, validate

TUNING = dict(kp=25, ki=80, kd=.3, min_pwm=0, sync_motors=1, prime_pid=1, sample_ms=10)


class SimulatorTests(unittest.TestCase):
    def test_original_sketch_exposes_startup_and_alternating_wheels(self):
        data = simulate({'duration': 1})
        first = next(s for s in data['samples'] if s[12] > 0)
        self.assertEqual(first[6:9], [255, 255, 0])
        self.assertEqual(data['summary']['pid_dt_min'], 20)
        self.assertEqual(data['summary']['pid_dt_max'], 20)

    def test_minimum_pwm_still_applies_at_zero_pid_output(self):
        data = simulate(dict(initial_angle=0, prime_pid=1, sync_motors=1, duration=1))
        first = next(s for s in data['samples'] if s[12] > 0)
        self.assertEqual(first[6:9], [0, 25, 25])

    def test_unpowered_upright_equilibrium_and_unstable_lean(self):
        settings = dict(kp=0, ki=0, kd=0, min_pwm=0, initial_angle=0, duration=1)
        equilibrium = simulate(settings)
        self.assertEqual(equilibrium['summary']['max_angle'], 0)
        falling = simulate(dict(settings, initial_angle=3))
        self.assertLess(falling['summary']['fall_time'], 1)
        self.assertTrue(all(s[7:9] == [0, 0] for s in falling['samples']))

    def test_tuning_example_recovers_from_push_in_assumed_model(self):
        data = simulate(dict(TUNING, duration=20, push_at=2, push=15))
        self.assertIsNone(data['summary']['fall_time'])
        self.assertLess(abs(data['samples'][-1][1]), 1)
        self.assertTrue(all(s[7] == s[8] for s in data['samples']))
        self.assertEqual(data['summary']['pid_dt_max'], 10)
        wrong_sign = simulate(dict(TUNING, polarity=-1))
        self.assertIsNotNone(wrong_sign['summary']['fall_time'])

    def test_dropout_holds_commands_and_sensor_age_increases(self):
        data = simulate(dict(TUNING, duration=1, dropout_at=.2, dropout_ms=200))
        gap = [s for s in data['samples'] if .205 <= s[0] <= .39]
        self.assertGreater(len(gap), 10)
        self.assertTrue(all(s[7:9] == gap[0][7:9] for s in gap))
        self.assertGreater(gap[-1][11], 150)
        self.assertEqual(data['summary']['dropped_samples'], 20)

    def test_corrupt_fifo_packets_are_rejected_by_the_sketch(self):
        data = simulate(dict(corrupt_every=1, duration=1))
        self.assertEqual(data['summary']['imu_reads'], 0)
        self.assertEqual(data['summary']['pid_updates'], 0)
        self.assertGreater(data['summary']['corrupt_samples'], 0)
        self.assertGreater(data['samples'][-1][14], 0)

    def test_runs_are_deterministic_and_process_isolated(self):
        settings = dict(TUNING, noise_deg=.1, duration=2)
        expected = simulate(settings)
        with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
            runs = list(pool.map(simulate, [settings, {'duration': 1}, settings]))
        self.assertEqual(runs[0], expected)
        self.assertEqual(runs[2], expected)
        self.assertTrue(all(math.isfinite(value) for row in expected['samples'] for value in row))

    def test_validation_rejects_invalid_simulation_inputs(self):
        for values in [{'duration': 999}, {'kp': float('nan')}, {'sample_ms': 1.5},
                       {'polarity': 0}, {'kp': True}, {'unknown': 1}, []]:
            with self.subTest(values=values), self.assertRaises(ValueError):
                validate(values)


if __name__ == '__main__':
    unittest.main()
