package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class WrappedMotor {
    /// Data


    private final DcMotor motor;

    private double lastPower = 0;
    private int targetPosition = 0;


    /// Constructors


    /**
     * Create a new wrapped DcMotor class.
     * @param motor {@link DcMotor}   The motor to be used.
     */
    public WrappedMotor(DcMotor motor) {
        this.motor = motor;
    }

    /**
     * Create a new wrapped DcMotor class using HardwareMap.
     * @param hardwareMap {@link HardwareMap}   The Hardware Map to use.
     * @param motorName {@link String}          The name of the motor inside the station config.
     */
    public WrappedMotor(HardwareMap hardwareMap, String motorName) {
        this.motor = hardwareMap.get(DcMotor.class, motorName);
    }


    /// Methods


    /**
     * Set the power of the motor.
     * Doesn't allow power to be set if the last power provided is the same as the power that's being set.
     * @param power     The power to set the motor to.
     */
    public void setPower(double power) {
        if (power != lastPower) {
            motor.setPower(power);
            lastPower = power;
        }
    }

    /**
     * Get the current power of the motor.
     * @return    The power of the motor.
     */
    public double getPower() {
        return lastPower;
    }

    /**
     * Set the direction of the motor.
     * @param direction {@link DcMotorSimple.Direction}     The direction to set it to.
     */
    public void setDirection(DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
    }

    /**
     * Get the current direction of the motor.
     * @return {@link DcMotorSimple.Direction}  The direction the motor is using.
     */
    public DcMotorSimple.Direction getDirection() {
        return motor.getDirection();
    }

    /**
     * Set the zero power behavior of the motor.
     * @param behavior {@link DcMotor.ZeroPowerBehavior}    The behavior that will be used.
     */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        motor.setZeroPowerBehavior(behavior);
    }

    /**
     * Set the target position.
     * Doesn't allow target position to be set if the last target position provided is the same as the target position that's being set.
     * @param target     The target position to use.
     */
    public void setTargetPosition(int target) {
        if (targetPosition != target) {
            targetPosition = target;
            motor.setTargetPosition(target);
        }
    }

    /**
     * Get the current target position.
     * @return   The target position being used.
     */
    public int getTargetPosition() {
        return targetPosition;
    }

    /**
     * Get the current position of the motor.
     * @return    The current position of the motor.
     */
    public double getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    /**
     * Checks if the current position is close enough to the target position.
     * @return   If it's close enough.
     */
    public boolean atTarget() {
        return atTarget(10);
    }

    /**
     * Checks if the current position is close enough to the target position.
     * @param tolerance     The tolerance where it can be within range.
     * @return              If it's close enough.
     */
    public boolean atTarget(int tolerance) {
        return Math.abs(targetPosition - getCurrentPosition()) < tolerance;
    }

    /**
     * Check if the motor is busy.
     * @return  If it's busy.
     */
    public boolean isBusy() {
        return motor.isBusy();
    }

    /**
     * Get the motor being used.
     * @return {@link DcMotor}  The motor of the WrappedMotorEx.
     */
    public DcMotor getMotor() {
        return motor;
    }
}
