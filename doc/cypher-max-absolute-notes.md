# Cypher Max absolute PWM notes

- The encoder always emits its absolute position on the PWM wire; there is no hardware switch to change modes. Quadrature A/B/I are provided simultaneously on separate pins.
- Use the side switch to match the supply voltage to the control hub's digital input level. For REV hubs, select **3.3V** to avoid overstressing the 3.3V-tolerant digital ports.
- A full PWM frame is 4,119 pulses at ~546 Hz. It begins with 12 high start pulses, then 0–4,095 high data pulses encoding the 12-bit position, followed by 8 low end pulses.
- To align the mechanical zero with the factory-calibrated zero, measure the raw count at your mechanical reference and configure an offset in `CypherMaxAbsoluteEncoder` (see `setZeroOffsetCounts` / `setZeroOffsetDegrees`).
- Near the wrap point (counts close to 0 or 4,095), apply the zero offset so the reported "zeroed" angles remain continuous around the factory mark instead of jumping across 0/360.
