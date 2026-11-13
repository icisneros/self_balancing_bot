#!/usr/bin/env python3
"""Local UI and process-isolated native Arduino simulation."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import threading
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from robot_config import DEFAULT_ROBOT, PHYSICAL_FIELDS, derive_motor_values, load_robot, motor_spec_summary, validate_physical, wheel_spec_summary

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
BUILD = HERE / '.build'
BUILD_LOCK = threading.Lock()
RUN_LIMIT = threading.BoundedSemaphore(4)

# default, minimum, maximum; shared by API validation and frontend controls.
PARAMETERS = {
    'kp': (40, 0, 200), 'ki': (0, 0, 200), 'kd': (.7, 0, 20),
    'setpoint': (88.5, -180, 180), 'sample_ms': (15, 1, 100),
    'min_pwm': (25, 0, 100), 'sync_motors': (0, 0, 1), 'prime_pid': (0, 0, 1),
    'initial_angle': (3, -30, 30), 'duration': (12, 1, 30),
    'fold_angle': (0, 0, 1),
    'dropout_at': (3, 0, 30), 'dropout_ms': (0, 0, 2000),
    'corrupt_every': (0, 0, 1000), 'push_at': (-1, -1, 30), 'push': (15, -100, 100),
    **{key: (None, low, high) for key, (_, low, high) in PHYSICAL_FIELDS.items()},
}
INTEGERS = {'sample_ms', 'sync_motors', 'prime_pid', 'fold_angle', 'corrupt_every', 'polarity'}


def validate(values, robot=None):
    robot = robot or load_robot()
    if not isinstance(values, dict):
        raise ValueError('Settings must be a JSON object')
    unknown = set(values) - PARAMETERS.keys()
    if unknown:
        raise ValueError(f'Unknown settings: {", ".join(sorted(unknown))}')
    result = {}
    for name, (default, low, high) in PARAMETERS.items():
        value = values.get(name, robot.values.get(name, default))
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'{name} must be a finite number')
        if not low <= value <= high or (name in INTEGERS and value != int(value)):
            raise ValueError(f'{name} must be between {low} and {high}' + (' (integer)' if name in INTEGERS else ''))
        if name == 'polarity' and value not in (-1, 1):
            raise ValueError('polarity must be -1 or 1')
        result[name] = value
    # Recalculate from the resolved trial radius/voltage unless explicitly overridden.
    for name, value in derive_motor_values(robot.motor_specs, result).items():
        if name not in values:
            result[name] = value
    validate_physical(result)
    return result


def build():
    sources = sorted(HERE.glob('*.cpp')) + sorted((HERE / 'compat').rglob('*.h'))
    sources += sorted((HERE / 'vendor').glob('PID_v1.*'))
    sources += [ROOT / 'DCMotor_and_IMU' / 'DCMotor_and_IMU.ino']
    compiler = shutil.which('clang++') or shutil.which('g++')
    if not compiler:
        raise RuntimeError('A C++17 compiler is required: install Xcode Command Line Tools on macOS, or g++ on Linux.')
    digest = hashlib.sha256(compiler.encode() + b'c++17-O2-v1')
    for source in sources:
        digest.update(str(source).encode())
        digest.update(source.read_bytes())
    fingerprint = digest.hexdigest()[:16]
    with BUILD_LOCK:
        BUILD.mkdir(exist_ok=True)
        binary = BUILD / f'robot-sim-{fingerprint}'
        if not binary.exists():
            print('Compiling the Arduino sketch for the simulator…', file=sys.stderr)
            command = [compiler, '-std=c++17', '-O2', '-DARDUINO=100', '-Wno-writable-strings',
                       '-I', str(HERE / 'compat'), str(HERE / 'runner.cpp'), str(HERE / 'pid.cpp'),
                       '-o', str(binary)]
            completed = subprocess.run(command, capture_output=True, text=True, timeout=60)
            if completed.returncode:
                binary.unlink(missing_ok=True)
                raise RuntimeError(completed.stderr)
    return binary, fingerprint


def simulate(values, robot_path=DEFAULT_ROBOT):
    robot = load_robot(robot_path)
    settings = validate(values, robot)
    binary, fingerprint = build()
    args = [str(binary)]
    for name, value in settings.items():
        args += [name, str(value)]
    with RUN_LIMIT:
        result = subprocess.run(args, text=True, capture_output=True, timeout=10)
    if result.returncode:
        raise RuntimeError(result.stderr or 'Native simulation failed')
    data = json.loads(result.stdout)
    data.update(settings=settings, build=fingerprint, motor_specs=motor_spec_summary(robot.motor_specs),
                wheel_specs=wheel_spec_summary(robot.wheel_specs, robot.values['wheel_radius']),
                robot_config={'path': str(robot.path), 'sha256': robot.sha256})
    return data


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(HERE / 'web'), **kwargs)

    def json_response(self, status, value):
        payload = json.dumps(value, allow_nan=False).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(payload)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self):
        if self.path == '/api/defaults':
            try:
                robot = load_robot(self.server.robot_path)
                self.json_response(200, validate({}, robot))
            except (ValueError, RuntimeError) as error:
                self.json_response(500, {'error': str(error)})
        else:
            super().do_GET()

    def do_POST(self):
        if self.path != '/api/simulate':
            self.json_response(404, {'error': 'Not found'})
            return
        try:
            size = int(self.headers.get('Content-Length', '0'))
            if not 0 < size <= 16384:
                raise ValueError('Request must contain at most 16 KB of JSON')
            values = json.loads(self.rfile.read(size))
            data = simulate(values, self.server.robot_path)
            self.json_response(200, data)
        except (ValueError, json.JSONDecodeError) as error:
            self.json_response(400, {'error': str(error)})
        except (RuntimeError, subprocess.TimeoutExpired) as error:
            self.json_response(500, {'error': str(error)})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--headless', action='store_true', help='Print one simulation as JSON and exit')
    parser.add_argument('--settings', type=Path, help='JSON settings for a headless run')
    parser.add_argument('--robot', type=Path, default=DEFAULT_ROBOT, help='Robot YAML file (default: simulator/robot.yaml)')
    args = parser.parse_args()
    if args.headless:
        values = json.loads(args.settings.read_text()) if args.settings else {}
        print(json.dumps(simulate(values, args.robot)))
        return
    load_robot(args.robot)  # Report configuration errors before starting the server.
    build()
    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    server.robot_path = args.robot.resolve()
    print(f'Robot PID sandbox: http://127.0.0.1:{server.server_port}', flush=True)
    print(f'Robot configuration: {server.robot_path}', flush=True)
    print('Runs the existing Arduino sketch. Ctrl+C stops the local server.', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, ValueError, OSError, subprocess.TimeoutExpired) as error:
        print(error, file=sys.stderr)
        sys.exit(1)
