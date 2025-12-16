# Wiring Layout (Current Build)

## Pico W GPIO Map (in order)
- GP0: reserved (UART/I2C future)
- GP1: reserved (UART/I2C future)
- GP2:  Black 1/3 (PC817 OUT, active-low)
- GP3:  Black 2/3
- GP4:  Black Full
- GP5:  Grey 1/3
- GP6:  Grey 2/3
- GP7:  Grey Full
- GP8:  Fresh 1/3
- GP9:  Fresh 2/3
- GP10: Fresh Full (Board B IN1)
- GP11–GP17: reserved for more opto inputs
- GP18: Awning EXT relay drive (piggyback COM→EXT)
- GP19: Awning RET relay drive (piggyback COM→RET)
- GP20: Furnace CALL relay drive (piggyback thermostat contact closure)
- GP21: Water Heater relay drive (piggyback rocker)
- GP22: Water Pump relay drive (piggyback rocker)
- GP23: Tank TEST Black (+12V injection enable)
- GP24: Tank TEST Grey (+12V injection enable)
- GP25: Tank TEST Fresh (+12V injection enable)
- GP26: Indoor Lights output
- GP27: Outdoor Lights output
- GP28 (ADC2): NTC 100k thermistor input (furnace temperature)

## Power
- RV +12V (fused 2–5A) into enclosure
- 12V→5V buck converter to power Pico W (VSYS/5V input depends on your carrier)
- Common GND between RV, Pico, relay drivers, opto output-side GND.

## Tank level inputs (PC817 opto boards)
Input side:
- VIN+ → RV +12V
- VIN− → RV GND
- IN+ → tank sense wire (12V signal during TEST)
- IN− → RV GND

Output side:
- OUT → Pico GPIO (GP2..GP10)
- GND → Pico GND
Outputs are active-low.

## Tank TEST outputs (+12V injection)
Use 3 high-side +12V output channels (P-MOSFET high-side recommended) driven by:
- GP23 (Black), GP24 (Grey), GP25 (Fresh)

Each channel OUT is tied to the same node the physical TEST button energizes.

## Water heater + water pump piggyback (dry contact)
For each rocker:
- Relay COM → rocker terminal A
- Relay NO  → rocker terminal B

## Awning piggyback (momentary 3-position switch)
At the awning switch:
- Identify COM, EXT, RET
Add two relays (dry contact):
- EXT relay: COM→switch COM, NO→switch EXT
- RET relay: COM→switch COM, NO→switch RET
Firmware uses hold-to-run deadman: requires repeated EXT/RET commands; otherwise STOP.

## Furnace (propane) thermostat piggyback
Relay contact in parallel with thermostat heat-call:
- Furnace relay COM → thermostat wire A
- Furnace relay NO  → thermostat wire B

### Thermistor divider (NTC 100k)
- 3.3V → 100k fixed resistor → ADC node → NTC 100k → GND
- ADC node → GP28 (ADC2)
Optional: 0.01–0.1 µF from ADC node to GND.
