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

    /**
     * The encoder transmits 12 high start pulses followed by 0-4095 high data pulses
     * inside a 4119-pulse frame. Use these values to convert duty cycle into counts.
     */
    private static final double MAX_COUNTS = 4095.0;
    private static final double START_PULSES = 12.0;
    private static final double FRAME_PULSES = 4119.0;

    private final DigitalChannel pwmInput;
    private boolean lastState;
    private long lastRisingTimeNs;
    private long lastFallingTimeNs;
    private double dutyCycle;
    private double lastPositionCounts;
    private double zeroOffsetCounts;
    private boolean validReading;

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
                double encodedPulses = dutyCycle * FRAME_PULSES;
                double decodedCounts = encodedPulses - START_PULSES;
                validReading = encodedPulses >= START_PULSES && encodedPulses <= START_PULSES + MAX_COUNTS;
                lastPositionCounts = Range.clip(decodedCounts, 0, MAX_COUNTS);
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
     * @return absolute position in raw counts adjusted by any configured zero offset.
     */
    public double getZeroedPositionCounts() {
        double adjusted = lastPositionCounts - zeroOffsetCounts;
        if (adjusted < 0) {
            adjusted += MAX_COUNTS + 1;
        }
        return Range.clip(adjusted, 0, MAX_COUNTS);
    }

    /**
     * @return absolute position in degrees (0-360)
     */
    public double getPositionDegrees() {
        return (lastPositionCounts / MAX_COUNTS) * 360.0;
    }

    /**
     * @return absolute position in degrees (0-360) adjusted by any configured zero offset.
     */
    public double getZeroedPositionDegrees() {
        return (getZeroedPositionCounts() / MAX_COUNTS) * 360.0;
    }

    /**
     * Apply an offset that is subtracted from the raw absolute reading. Use this to
     * align the factory-calibrated zero with your mechanical zero reference.
     *
     * @param offsetCounts value in counts (0-4095)
     */
    public void setZeroOffsetCounts(double offsetCounts) {
        zeroOffsetCounts = Range.clip(offsetCounts, 0, MAX_COUNTS);
    }

    /**
     * Apply an offset in degrees that is subtracted from the raw absolute reading.
     *
     * @param offsetDegrees value in degrees (0-360)
     */
    public void setZeroOffsetDegrees(double offsetDegrees) {
        double offsetCounts = (offsetDegrees / 360.0) * MAX_COUNTS;
        setZeroOffsetCounts(offsetCounts);
    }

    /**
     * @return true if the measured duty cycle fell within the expected frame window
     *         (i.e., at least the 12 start pulses and no more than the total frame)
     */
    public boolean isValid() {
        return validReading;
    }
}
