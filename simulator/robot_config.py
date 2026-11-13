"""Load and validate the physical robot description, independently of trial settings."""
from dataclasses import dataclass
import hashlib
import math
from pathlib import Path

try:
    import yaml
except ImportError:
    yaml = None

DEFAULT_ROBOT = Path(__file__).resolve().with_name('robot.yaml')

# Native parameter -> (YAML path, lower limit, upper limit). No physical defaults
# live here: robot.yaml is the single source of defaults for the robot model.
ROBOT_FIELDS = {
    'gravity': ('environment.gravity_m_s2', .1, 30),
    'body_mass': ('body.mass_kg', .1, 5),
    'body_height': ('body.height_m', .05, 1),
    'body_depth': ('body.depth_m', .01, .5),
    'com_x': ('body.center_of_mass_m.x', -.3, .3),
    'com_height': ('body.center_of_mass_m.z', .05, .5),
    'pitch_inertia': ('body.pitch_inertia_kg_m2', .00001, 2),
    'angular_damping': ('body.angular_damping_n_m_s_per_rad', 0, .2),
    'axle_mass': ('base.effective_mass_kg', .1, 5),
    'wheel_radius': ('base.wheel_radius_m', .01, .15),
    'drag': ('base.drag_n_s_per_m', 0, 3),
    'motor_voltage': ('motors.supply_voltage_v', .1, 24),
    'motor_lag_ms': ('motors.response_time_ms', 1, 200),
    'deadband': ('motors.deadband_pwm', 0, 150),
    'polarity': ('motors.polarity', -1, 1),
    'imu_x': ('imu.position_m.x', -1, 1),
    'imu_y': ('imu.position_m.y', -1, 1),
    'imu_z': ('imu.position_m.z', -1, 1),
    'imu_roll_deg': ('imu.mounting_deg.roll', -180, 180),
    'imu_pitch_deg': ('imu.mounting_deg.pitch', -180, 180),
    'imu_yaw_deg': ('imu.mounting_deg.yaw', -180, 180),
    'imu_hz': ('imu.output_rate_hz', 10, 500),
    'imu_delay_ms': ('imu.delay_ms', 0, 100),
    'noise_deg': ('imu.noise_std_deg', 0, 2),
}

# Datasheet facts retained with the robot description. Geometry, mass, current
# and encoder counts are recorded, not inferred to be chassis totals or fed
# into physics subsystems that do not model them.
MOTOR_SPEC_FIELDS = {
    'count': ('motors.count', 2, 2),
    'rated_voltage_v': ('motors.specification.rated_voltage_v', 1, 48),
    'gear_ratio': ('motors.specification.gear_ratio', 1, 1000),
    'mass_each_kg': ('motors.specification.mass_each_kg', .001, 5),
    'diameter_m': ('motors.specification.diameter_m', .001, .5),
    'length_excluding_output_shaft_m': ('motors.specification.length_excluding_output_shaft_m', .001, 1),
    'gearbox_length_m': ('motors.specification.gearbox_length_m', .001, .5),
    'output_shaft_diameter_m': ('motors.specification.output_shaft_diameter_m', .001, .1),
    'output_shaft_extension_m': ('motors.specification.output_shaft_extension_m', .001, .5),
    'no_load_output_speed_rpm': ('motors.specification.no_load_output_speed_rpm', 1, 10000),
    'no_load_current_a': ('motors.specification.no_load_current_a', .001, 100),
    'stall_current_a': ('motors.specification.stall_current_a', .001, 100),
    'stall_output_torque_kgf_cm': ('motors.specification.stall_output_torque_kgf_cm', .001, 100),
    'encoder_counts_per_motor_rev': ('motors.specification.encoder_counts_per_motor_rev', 1, 10000),
}
WHEEL_SPEC_FIELDS = {
    'count': ('base.wheels.count', 2, 2),
    'width_m': ('base.wheels.width_m', .001, .2),
    'mass_each_kg': ('base.wheels.mass_each_kg', .001, 5),
}
DERIVED_FIELDS = {
    'max_force': ('derived combined stall force (N)', .001, 500),
    'free_speed': ('derived wheel no-load speed (m/s)', .001, 20),
}
PHYSICAL_FIELDS = {**ROBOT_FIELDS, **DERIVED_FIELDS}
STANDARD_GRAVITY = 9.80665  # kgf is defined using standard gravity, not environment.gravity_m_s2.


def derive_motor_values(specs, physical):
    scale = physical['motor_voltage'] / specs['rated_voltage_v']
    torque_nm = specs['stall_output_torque_kgf_cm'] * STANDARD_GRAVITY / 100
    radius = physical['wheel_radius']
    return {
        'max_force': specs['count'] * torque_nm * scale / radius,
        'free_speed': specs['no_load_output_speed_rpm'] * 2 * math.pi / 60 * radius * scale,
    }


def motor_spec_summary(specs):
    return {
        **specs,
        'stall_output_torque_n_m': specs['stall_output_torque_kgf_cm'] * STANDARD_GRAVITY / 100,
        'total_motor_mass_kg': specs['count'] * specs['mass_each_kg'],
        'encoder_counts_per_output_rev': specs['encoder_counts_per_motor_rev'] * specs['gear_ratio'],
    }


def wheel_spec_summary(specs, radius):
    return {**specs, 'diameter_m': radius*2,
            'total_wheel_mass_kg': specs['count']*specs['mass_each_kg']}


def read_specification(leaves, schema):
    specs = {key: leaves[field[0]] for key, field in schema.items()}
    for key, (name, low, high) in schema.items():
        value = specs[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'{name} must be a finite number')
        if not low <= value <= high:
            raise ValueError(f'{name} must be between {low} and {high}')
        if key in ('count', 'encoder_counts_per_motor_rev') and value != int(value):
            raise ValueError(f'{name} must be an integer')
    return specs


def validate_physical(values):
    for key, (path, low, high) in PHYSICAL_FIELDS.items():
        value = values[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'{path} must be a finite number')
        if not low <= value <= high or (key == 'polarity' and value not in (-1, 1)):
            raise ValueError(f'{path} must be between {low} and {high}'
                             + (' and equal -1 or 1' if key == 'polarity' else ''))
    roll, pitch = map(math.radians, (values['imu_roll_deg'], values['imu_pitch_deg']))
    if math.hypot(math.sin(pitch), math.cos(pitch)*math.cos(roll)) < 1e-6:
        raise ValueError('imu.mounting_deg makes pitch undefined at upright (sensor y axis is vertical)')


@dataclass(frozen=True)
class RobotConfig:
    values: dict
    path: Path
    sha256: str
    motor_specs: dict
    wheel_specs: dict


def load_robot(path=DEFAULT_ROBOT):
    if yaml is None:
        raise RuntimeError('PyYAML is required. Install simulator/requirements.txt in your Python environment; see simulator/README.md.')

    class UniqueSafeLoader(yaml.SafeLoader):
        def construct_mapping(self, node, deep=False):
            result = {}
            for key_node, value_node in node.value:
                key = self.construct_object(key_node, deep=deep)
                if not isinstance(key, str):
                    raise ValueError('Robot YAML keys must be strings')
                if key in result:
                    raise ValueError(f'Duplicate robot YAML key: {key}')
                result[key] = self.construct_object(value_node, deep=deep)
            return result

    path = Path(path).resolve()
    try:
        content = path.read_bytes()
        data = yaml.load(content, Loader=UniqueSafeLoader)
        if not isinstance(data, dict):
            raise ValueError('Robot YAML must contain a mapping')
        leaves = {}

        def flatten(mapping, prefix=''):
            for key, value in mapping.items():
                name = f'{prefix}.{key}' if prefix else key
                if isinstance(value, dict) and value:
                    flatten(value, name)
                else:
                    leaves[name] = value

        flatten(data)
        expected = {field[0] for field in [*ROBOT_FIELDS.values(), *MOTOR_SPEC_FIELDS.values(), *WHEEL_SPEC_FIELDS.values()]}
        missing, unknown = expected - leaves.keys(), leaves.keys() - expected
        if missing:
            raise ValueError('Missing robot YAML fields: ' + ', '.join(sorted(missing)))
        if unknown:
            raise ValueError('Unknown robot YAML fields: ' + ', '.join(sorted(unknown)))
        values = {key: leaves[field[0]] for key, field in ROBOT_FIELDS.items()}
        specs = read_specification(leaves, MOTOR_SPEC_FIELDS)
        wheel_specs = read_specification(leaves, WHEEL_SPEC_FIELDS)
        if specs['no_load_current_a'] >= specs['stall_current_a']:
            raise ValueError('motors.specification.stall_current_a must exceed no_load_current_a')
        # Validate radius and voltage before using them as divisors/scales.
        for key in ('wheel_radius', 'motor_voltage'):
            value = values[key]
            name, low, high = ROBOT_FIELDS[key]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f'{name} must be a finite number between {low} and {high}')
        values.update(derive_motor_values(specs, values))
        validate_physical(values)
    except (OSError, yaml.YAMLError, ValueError) as error:
        raise ValueError(f'{path}: {error}') from error
    return RobotConfig(values, path, hashlib.sha256(content).hexdigest(), specs, wheel_specs)
