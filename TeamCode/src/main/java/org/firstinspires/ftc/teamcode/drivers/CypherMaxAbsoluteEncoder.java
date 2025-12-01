package org.firstinspires.ftc.teamcode.drivers;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * Utility class for reading the Studica Cypher Max Through Bore encoder
 * in absolute PWM mode. The encoder outputs a ~546 Hz PWM signal where the
 * duty cycle represents the 12-bit position (0-4095). This helper polls the
 * digital input, measures the duty cycle, and exposes the position in
 * raw counts or degrees.
 */
public class CypherMaxAbsoluteEncoder {

    private static final double MAX_COUNTS = 4095.0;

    private final DigitalChannel pwmInput;
    private boolean lastState;
    private long lastRisingTimeNs;
    private long lastFallingTimeNs;
    private double dutyCycle;
    private double lastPositionCounts;

    /**
     * Create a new Cypher Max reader.
     *
     * @param hardwareMap the OpMode hardware map
     * @param deviceName  the configured digital input name connected to the PWM wire
     */
    public CypherMaxAbsoluteEncoder(HardwareMap hardwareMap, String deviceName) {
        pwmInput = hardwareMap.get(DigitalChannel.class, deviceName);
        pwmInput.setMode(DigitalChannel.Mode.INPUT);

        lastState = pwmInput.getState();
        long now = System.nanoTime();
        lastRisingTimeNs = now;
        lastFallingTimeNs = now;
    }

    /**
     * Sample the digital input and update the cached duty cycle/position. Call
     * this once per loop during init and tele-op.
     */
    public void update() {
        boolean state = pwmInput.getState();
        long now = System.nanoTime();

        // Rising edge: compute period and duty cycle using the previous high window
        if (state && !lastState) {
            long periodNs = now - lastRisingTimeNs;
            if (periodNs > 0 && lastFallingTimeNs > lastRisingTimeNs) {
                long highNs = lastFallingTimeNs - lastRisingTimeNs;
                dutyCycle = (double) highNs / periodNs;
                lastPositionCounts = Range.clip(dutyCycle * MAX_COUNTS, 0, MAX_COUNTS);
            }
            lastRisingTimeNs = now;
        }

        // Falling edge: capture the time spent high for the current PWM frame
        if (!state && lastState) {
            lastFallingTimeNs = now;
        }

        lastState = state;
    }

    /**
     * @return last measured duty cycle (0.0-1.0)
     */
    public double getDutyCycle() {
        return dutyCycle;
    }

    /**
     * @return absolute position in raw encoder counts (0-4095)
     */
    public double getPositionCounts() {
        return lastPositionCounts;
    }

    /**
     * @return absolute position in degrees (0-360)
     */
    public double getPositionDegrees() {
        return (lastPositionCounts / MAX_COUNTS) * 360.0;
    }
}
