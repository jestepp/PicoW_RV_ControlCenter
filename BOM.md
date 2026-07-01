# BOM (core items)

## Control / Compute
- Raspberry Pi Pico W
- 12V→5V buck converter (RV-rated, ≥1A recommended)
- Enclosure, terminal blocks, ferrules, heatshrink, wiring

## Inputs (tank levels)
- PC817 8-channel optocoupler boards (2 boards if you expand beyond 8 inputs)

## Outputs
- 8-channel opto-isolated relay board (for heater/pump/awning/furnace and future)
- OR high-side +12V switch channels (P-MOSFET based) for Tank TEST and other +12V injection needs

## Furnace temperature sensing
- NTC 100k thermistor (B3950 typical)
- 100k resistor (1% preferred) for divider
- Optional 0.01–0.1 µF capacitor to filter ADC noise

## Indoor temperature sensing
- DS18B20 temperature sensor
- 4.7k resistor for the OneWire pull-up

## Protection / Best practice
- Inline fuse (2–5A) on the 12V feed into the control box
- Flyback diodes if you drive bare relay coils (relay boards usually include them)
