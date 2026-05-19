package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WrappedServo {
    /// Data

    private final Servo servo;

    private double openPosition = 0;
    private double closePosition = 1;
    private double lastPosition = 0;


    /// Constructors


    /**
     * Create a new wrapped Servo class.
     * @param servo {@link Servo}   The servo to be used.
     */
    public WrappedServo(Servo servo) {
        this.servo = servo;
    }

    /**
     * Create a new wrapped Servo class using HardwareMap.
     * @param hardwareMap {@link HardwareMap}   The Hardware Map to use.
     * @param servoName {@link String}          The name of the servo inside the station config.
     */
    public WrappedServo(HardwareMap hardwareMap, String servoName) {
        this.servo = hardwareMap.get(Servo.class, servoName);
    }

    /**
     * Create a new wrapped Servo class with open and closed positions.
     * @param servo {@link Servo}   The servo to be used.
     * @param openPosition          The position of the open position. [0-1]
     * @param closePosition         The position of the close position. [0-1]
     */
    public WrappedServo(Servo servo, double openPosition, double closePosition) {
        this.servo = servo;
        this.openPosition = openPosition;
        this.closePosition = closePosition;
    }

    /**
     * Create a new wrapped Servo class using HardwareMap with open and closed positions.
     * @param hardwareMap {@link HardwareMap}   The Hardware Map to use.
     * @param servoName {@link String}          The name of the servo inside the station config.
     * @param openPosition                      The position of the open position. [0-1]
     * @param closePosition                     The position of the close position. [0-1]
     */
    public WrappedServo(HardwareMap hardwareMap, String servoName, double openPosition, double closePosition) {
        this.servo = hardwareMap.get(Servo.class, servoName);
        this.openPosition = openPosition;
        this.closePosition = closePosition;
    }


    /// Methods


    /**
     * Set the open position.
     * @param position      The position to the open position to. [0-1]
     */
    public void setOpenPosition(double position) {
        openPosition = position;
    }

    /**
     * Set the open position.
     * @return  The position of the open position. [0-1]
     */
    public double getOpenPosition() {
        return openPosition;
    }

    /**
     * Set the position of the servo to the open position.
     */
    public void open() {
        if (getPosition() != openPosition) setPosition(openPosition);
    }

    /**
     * Set the close position.
     * @param position      The position to the close position to. [0-1]
     */
    public void setClosedPosition(double position) {
        closePosition = position;
    }

    /**
     * Set the open position.
     * @return      The position of the close position. [0-1]
     */
    public double getClosePosition() {
        return closePosition;
    }

    /**
     * Set the position of the servo to the close position.
     */
    public void close() {
        if (getPosition() != closePosition) setPosition(closePosition);
    }

    /**
     * Set the direction of the servo.
     * @param direction {@link Servo.Direction}     The direction to set it to.
     */
    public void setDirection(Servo.Direction direction) {
        servo.setDirection(direction);
    }

    /**
     * Get the current direction of the servo.
     * @return {@link Servo.Direction}  The direction the servo is using.
     */
    public Servo.Direction getDirection() {
        return servo.getDirection();
    }

    /**
     * Set the position of the servo.
     * Doesn't allow position to be set if the last position provided is the same as the position that's being set.
     * @param position    The position to be set to. [0.0-1.0]
     */
    public void setPosition(double position) {
        if (lastPosition != position) {
            lastPosition = position;
            servo.setPosition(position);
        }
    }

    /**
     * Get the position of the servo.
     * Will return Double.NaN if position is unknown.
     * @return    The current position of the servo.
     */
    public double getPosition() {
        return servo.getPosition();
    }

    /**
     * Scales the available movement range of the servo to be a subset of its maximum range.
     * Subsequent positioning calls will operate within that subset range.
     * This is useful if your servo has only a limited useful range of movement.
     * @param min   The lower limit of the range. [0-1]
     * @param max   The upper limit of the range. [0-1]
     */
    public void scaleRange(double min, double max) {
        servo.scaleRange(min, max);
    }
}