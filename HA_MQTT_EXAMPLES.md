# Home Assistant MQTT examples (Discovery disabled)

Assumes base topic: rv/pico1

## Switches
You can create MQTT switches in HA for heater/pump/lights.

Example (YAML):
switch:
  - platform: mqtt
    name: "RV Water Heater"
    command_topic: "rv/pico1/heater/set"
    state_topic: "rv/pico1/heater/state"
    payload_on: "ON"
    payload_off: "OFF"
    retain: true

## Awning (hold-to-run)
The firmware stops the awning unless it keeps receiving EXTEND/RETRACT commands.
In HA you typically implement this with a script + button that repeats while pressed
(or a dashboard control that sends repeats). If you want, we can provide a full HA blueprint.

Command topic:
- rv/pico1/awning/set payload: EXTEND | RETRACT | STOP
State topic:
- rv/pico1/awning/state payload: stop | extend | retract

## Furnace
Set mode:
- rv/pico1/furnace/mode/set payload: AUTO | ON | OFF
State:
- rv/pico1/furnace/mode/state payload: AUTO | ON | OFF

Setpoint (°F):
- rv/pico1/furnace/setpoint_f/set payload: 68.0
State:
- rv/pico1/furnace/setpoint_f payload: 68.0

Temperature (°F):
- rv/pico1/furnace/temp_f payload: 71.3

Call-for-heat:
- rv/pico1/furnace/state payload: ON | OFF

## Tank levels
Each is 1 (active) / 0 (inactive), retained:
- rv/pico1/tank/black/one_third
- rv/pico1/tank/black/two_third
- rv/pico1/tank/black/full
... etc
