# Arduino balance sandbox

One-time setup, from the repository root:

```sh
python3 -m venv simulator/.venv
simulator/.venv/bin/python -m pip install -r simulator/requirements.txt
```

Start the simulator:

```sh
simulator/.venv/bin/python simulator/run.py
```

Open **http://127.0.0.1:8765**. Requires Python 3.10+ and a C++17 compiler
(`clang++` or `g++`). On macOS the compiler comes with Xcode Command Line Tools.
PyYAML is the only Python dependency; no npm packages or Arduino IDE are required.
After the one-time installation, the simulator works offline.
Use `--port 8766` if the default port is occupied. Ctrl+C stops the server.

The server binds only to the local machine. Each trial runs in a fresh native
process, so Arduino globals, static locals, PID history, and motor state reset
between trials. Compilation is cached under ignored `.build/`; changes to the
sketch, compatibility headers, or PID source trigger a rebuild on the next run.

## Robot configuration

**`simulator/robot.yaml` is the source of physical defaults.** It contains masses,
body dimensions, center-of-mass position, pitch inertia, wheel radius, motor
properties, damping, gravity, and the IMU's location, mounting angles, rate,
delay, and noise. Units are explicit in the YAML field names and comments.
PID gains, the control setpoint, initial tilt, and fault/disturbance scenarios
remain trial settings rather than physical constants.

Edit that file and reload the browser page to populate the controls from it.
There is no C++ rebuild or server restart for YAML edits: the Python script
rereads and validates the selected file on each defaults request and trial.
Controls and imported JSON settings override YAML defaults for a trial without
writing to the file. Existing exported settings include physical parameters;
loading one overrides those parameters even if YAML has subsequently changed.
Each trial includes the YAML path and SHA-256 hash alongside the resolved settings.

### Motor specifications and derived limits

The motor section now describes the user-confirmed **Pololu #3264, 34:1
25Dx64L LP 12 V with 48 CPR encoder (no end cap)**. Sources are the
[archived specifications](https://www.pololu.com/product/3264/specs),
[encoder description](https://www.pololu.com/product/3264), and the
[dimension drawing, page 2](https://www.pololu.com/file/0J1042/25d-metal-gearmotor-dimension-diagram.pdf).

Recorded per motor: 101 g, 34.014:1 reduction, 150 RPM no-load output speed,
100 mA no-load current, 1.1 A extrapolated stall current, and 3 kgf·cm
extrapolated stall output torque at 12 V. The torque is approximately
0.294 N·m; the manufacturer's 43 oz·in figure is a separately rounded rating.
The 25 mm diameter and 64 mm housing/encoder length are nominal catalog
dimensions; the encoder drawing's axial dimensions sum to 63.5 mm. Output
shaft diameter is 4 mm and extension is 12.5 mm. Gearbox length is 21 mm.

The 48 encoder counts are per **motor shaft** revolution with both edges of
both channels counted, giving `48 * 34.014 = 1632.672` per gearbox output
revolution. Encoder feedback is not implemented in the balance sketch or
simulation. The 150 RPM speed and stall torque are already **gearbox output**
ratings and are not multiplied/divided by the gear ratio again.

On loading the YAML, the simulator computes:

```text
torque_Nm = stall_output_torque_kgf_cm * 9.80665 / 100
voltage_scale = supply_voltage_v / rated_voltage_v
max_force = count * torque_Nm * voltage_scale / wheel_radius_m
free_speed = no_load_output_speed_rpm * 2*pi/60 * wheel_radius_m * voltage_scale
```

At the confirmed 35 mm radius and still-assumed 12 V motor-terminal supply, this gives
**16.81 N combined stall force** and **0.550 m/s wheel no-load speed**.
The supply voltage is an explicit operating assumption; the datasheet does
not establish what voltage the robot actually applied after driver losses.
Stall ratings describe a peak extrapolation, not sustained available force.
The motor time constant and deadband remain estimates. Current, mass, and
motor dimensions are retained as reference facts, not used by an electrical
or component-by-component inertia model that the simulator does not include.
The two motors total 202 g, but this is not added to estimated body/base masses
because those aggregates may already include them; their actual placements
and the robot's total mass distribution remain unknown.

Trial/API wheel-radius and voltage overrides recalculate force and speed unless
those limits are explicitly supplied. The browser rescales the displayed force
and speed when changing wheel radius or voltage; both limits remain editable
for experiments. Saved settings with explicit limits retain those overrides.
Raw datasheet fields and converted motor/encoder values accompany headless
trial output as `motor_specs`.

### Confirmed wheels

The user confirmed **Pololu Wheel 70×8mm Pair**, with a 70 mm diameter and
0.5 oz mass **per wheel**, including the tire. The
[manufacturer's specifications](https://www.pololu.com/product/1425/specs)
agree; the color is unspecified and does not change these dimensions/ratings.
`base.wheel_radius_m` is therefore confirmed as 0.035 m. `base.wheels` records
two wheels, 0.008 m width, and approximately 0.014175 kg per wheel (0.028350 kg
per pair). Trial output includes these facts as `wheel_specs`.

The existing radius was already 35 mm, so this confirmation does not change
derived force or speed. Wheel masses are retained without adding them again
to aggregate body/base estimates. Their rotational inertia remains part of
the estimated effective base mass; a measured inertia is not available from
diameter and total mass alone.

Select a different robot description with:

```sh
simulator/.venv/bin/python simulator/run.py --robot /path/to/my-robot.yaml
simulator/.venv/bin/python simulator/run.py --headless --robot /path/to/my-robot.yaml
```

The default path resolves relative to the script, so launch from any working
directory. Custom relative paths resolve from the current working directory.
All YAML fields are required. Unknown/duplicate keys, non-finite values,
invalid types, and out-of-range values produce descriptive errors.

The body frame has its origin at the axle center, with x forward, y left,
and z up at zero body tilt. Center of mass x/z affect the dynamics. Body
height/depth affect the drawing; pitch inertia is specified independently,
so update it when changing a body's mass distribution. The default inertia
retains the original uniform-rod approximation (`m*l²/3`).

IMU mounting is the sensor-to-body rotation `Rz(yaw) * Ry(pitch) * Rx(roll)`
in right-handed degrees. Ideal sensor pitch is calculated by transforming
gravity into the sensor frame and projecting it with `atan2(-g_x, g_z)`;
the continuous result is unwrapped around the upright reading. The legacy
folded output option is applied afterward. All three mounting angles affect
this transformation; yaw here is a fixed mount orientation, not simulated
robot turning. Pitch-undefined upright mounts (sensor y vertical) are rejected.
The control setpoint remains independent of mounting: adjust it if changing
the IMU's upright reading. The default 88.5° pitch preserves the earlier
assumption about the project's missing custom angle remapping.

IMU position x/z changes the side-view marker; y is retained but lies outside
that view. Position does **not** affect ideal orientation measurements: the
simulator does not model lever-arm acceleration or DMP fusion. The measured
plot is pitch relative to the configured upright reading; arbitrary mounting
can change its sign and sensitivity relative to physical body tilt.

## What runs

`runner.cpp` directly includes `../DCMotor_and_IMU/DCMotor_and_IMU.ino` and calls
its real `setup()` and `loop()`. This is a native host build, not a JavaScript
translation, AVR instruction emulator, or electrical simulation. A few forward
declarations replace the Arduino IDE's automatic prototype generation.

Compatibility headers supply Arduino time, a successful Wire connection, a
virtual MPU6050 FIFO, and motor objects that retain the sketch's signed PWM
commands. A pinned copy of the actual upstream PID_v1 1.2.1 library performs
the calculations. See `vendor/README.md` for provenance and the historical
dependency uncertainty. The simulator compiles the current firmware directly.

The firmware defaults use:

- Both wheels updated in the same control iteration, with minimum PWM disabled.
- Zero PID output stops both wheels, including when an optional nonzero PWM floor is configured.
- Gains 40 / 0 / 0.7, setpoint 88.5°, and 15 ms PID gating.
- FIFO rejection logic, use of stale pitch on failed reads, and the angle cutoff.

The frontend can override gains, sample interval, setpoint, and minimum PWM.
An optional harness setting primes the PID with the initial sensor angle.
The sketch itself updates both wheels; no harness synchronization override is
applied. The legacy `sync_motors` setting is accepted for saved-file compatibility
and has no effect. The same current sketch executes in both presets.

## First experiment

1. Run **Firmware defaults**. Inspect the startup output, matching wheel commands,
   and actual 20 ms PID interval with a 100 Hz IMU.
2. Select **Tuning example · assumed robot**, then run again. This example uses
   Kp 25, Ki 80, Kd 0.3, minimum PWM 0, initialized PID, and
   10 ms samples. It changes multiple settings; use individual controls for
   experiments that isolate a cause.
3. Pause, scrub, replay at slower speed, or push the robot. The push button
   recomputes the deterministic trial with one +15°/s angular impulse shortly
   after the playback cursor, then resumes from that cursor. It replaces any
   previously scheduled push. This is trial playback, not a live hardware loop.
4. Change gains, sensor delay/noise, motor lag/deadband, or dimensions and run
   again. Settings take effect only on a new run. Plots show the whole trial;
   the vertical cursor follows playback. Trials end at 60° of physical tilt.
5. Export CSV and save settings for reproducible comparisons. Load settings
   to repeat a trial. Exports always refer to the last computed trial, including
   when the controls have unsaved changes.

RMS tilt and saturation are sampled every 5 ms. Balance time is elapsed time
until the 60° fall threshold, not time continuously within a small angle band.
PID interval excludes the first calculation. Sensor age measures time since
the last successful FIFO read; configured transport delay is additional.

## Physics and assumptions

This is a planar, coupled wheel-base / inverted-pendulum approximation. Body
mass `m` has center of mass `(a, l)` relative to a translating axle mass `M`,
and pitch inertia `I_COM` specified in the robot YAML.
Angle `theta` is positive when the top leans forward. The model includes the
equal/opposite wheel-drive torque on the body. Wheel rotational inertia is
absorbed into the effective base mass; yaw and differential-drive turning
are omitted. Unequal motor commands contribute their average to pitch motion.

The coupled equations are:

```text
h = l*cos(theta) - a*sin(theta)
p = a*cos(theta) + l*sin(theta)

(M+m)*x_dd + m*h*theta_dd
    = F - drag*x_d + m*p*theta_d²

m*h*x_dd + (I_COM + m*(a²+l²))*theta_dd
    = m*g*p - angular_damping*theta_d - F*wheel_radius
```

Motor commands pass through a configurable PWM deadband. Combined target
force is `max_force * (average_normalized_command - axle_velocity/free_speed)`;
force follows this target with a first-order response. This is a simplified
back-EMF model, not a measured torque-speed curve. The lag is integrated
analytically over each step and the mechanical state uses RK4 at **0.5 ms**.
Sensor events and `loop()` are serviced on this same grid, so timestamps are
quantized to 0.5 ms (and PID `millis()` to 1 ms).

Default assumptions: body 0.7 kg, effective base 0.5 kg, center of mass 0.15 m
above axle, 12 V motor-terminal supply, motor lag 35 ms,
IMU 100 Hz, angle delay 10 ms. Initial tilt is 3°. Motor force/speed limits
are derived from the confirmed motor ratings and wheel radius (0.035 m), and assumed voltage
(16.81 N and 0.550 m/s). Other robot values remain illustrative estimates.

The IMU adapter supplies delayed/noisy final pitch values directly, rather than
reimplementing DMP fusion. With the default mounting, continuous pitch is
`88.5° + physical tilt`. Other mountings use the rotation described above.
The legacy folded mapping is selectable.
Injected malformed packets exercise the sketch's rejection path; dropouts
omit events and leave the motors holding their last commands. These are
controlled fault scenarios, not simulations of the underlying electrical faults.

I2C transfer duration, serial blocking, MCU float precision/overflow behavior,
battery sag, brownouts, sensor fusion drift, tire slip, gear backlash, body
contact after falling, remote control, and encoder feedback are not simulated.
Host `double` is generally 64 bits, unlike AVR `double`. Simulation gains should
not be treated as validated gains for the physical robot.

## Headless runs and checks

```sh
simulator/.venv/bin/python simulator/run.py --headless
simulator/.venv/bin/python simulator/run.py --headless --settings balance-settings.json
simulator/.venv/bin/python -m unittest discover -s simulator/tests -v
```

Headless mode prints a JSON object with settings, build fingerprint, column
names, 5 ms telemetry samples, and summary metrics. It can be used for offline
parameter sweeps. The tests exercise startup, minimum PWM, scheduling, gravity,
disturbance recovery, polarity, dropouts, FIFO rejection, and run isolation.
