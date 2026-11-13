# self_balancing_bot
Arduino-powered self balancing robot.

A local simulator compiles and runs the balance sketch against simulated IMU
and motor interfaces, with a browser UI for PID tuning:

```sh
python3 -m venv simulator/.venv
simulator/.venv/bin/python -m pip install -r simulator/requirements.txt
simulator/.venv/bin/python simulator/run.py
```

Open **http://127.0.0.1:8765**. See [simulator/README.md](simulator/README.md)
for setup, model assumptions, and limitations. The Arduino firmware is unchanged.
Physical parameters and IMU mounting are configured in [simulator/robot.yaml](simulator/robot.yaml).
