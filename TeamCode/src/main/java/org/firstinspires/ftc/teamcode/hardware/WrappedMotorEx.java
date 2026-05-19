package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class WrappedMotorEx {
    /// Data


    private final DcMotorEx motor;

    private double lastPower = 0;
    private double lastAngularRate = 0;
    private AngleUnit lastAngleUnit;
    private int targetPosition = 0;


    /// Constructors


    /**
     * Create a new wrapped DcMotorEx class.
     * @param motor {@link DcMotorEx}   The motor to be used.
     */
    public WrappedMotorEx(DcMotorEx motor) {
        this.motor = motor;
    }

    /**
     * Create a new wrapped DcMotorEx class using HardwareMap.
     * @param hardwareMap {@link HardwareMap}   The Hardware Map to use.
     * @param motorName {@link String}          The name of the motor inside the station config.
     */
    public WrappedMotorEx(HardwareMap hardwareMap, String motorName) {
        this.motor = hardwareMap.get(DcMotorEx.class, motorName);
    }


    /// Methods


    /**
     * Set the power of the motor.
     * Doesn't allow power to be set if the last power provided is the same as the power that's being set.
     * @param power      The power to set the motor to.
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
     * Set the velocity of the motor.
     * @param angularRate     The Angular Rate using units per second.
     */
    public void setVelocity(double angularRate) {
        if (lastAngularRate != angularRate) {
            lastAngularRate = angularRate;
            motor.setVelocity(angularRate);
        }
    }

    /**
     * Set the velocity of the motor.
     * @param angularRate                   The Angular Rate using units per second.
     * @param angleUnit {@link AngleUnit}   The units that Angular Rate is expressed in.
     */
    public void setVelocity(double angularRate, AngleUnit angleUnit) {
        if (lastAngularRate != angularRate || lastAngleUnit != angleUnit) {
            lastAngularRate = angularRate;
            lastAngleUnit = angleUnit;
            motor.setVelocity(angularRate, angleUnit);
        }
    }

    /**
     * Get the current velocity of the motor.
     * @return    The current velocity of the motor.
     */
    public double getVelocity() {
        return motor.getVelocity();
    }

    /**
     * Get the current velocity of the motor using a specific angle unit.
     * @param angleUnit {@link AngleUnit}   The units that velocity will be expressed in.
     * @return                              The current velocity of the motor using angleUnit.
     */
    public double getVelocity(AngleUnit angleUnit) {
        return motor.getVelocity(angleUnit);
    }

    /**
     * Set the motor's PIDF Coefficients.
     * @param runMode {@link DcMotor.RunMode}               The RunMode that will be used.
     * @param pidfCoefficients {@link PIDFCoefficients}     The PIDF Coefficients that will be used.
     */
    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients pidfCoefficients) {
        motor.setPIDFCoefficients(runMode, pidfCoefficients);
    }

    /**
     * Set the velocity PIDF Coefficients.
     * @param p   The proportional. Corrects based on current error by applying power.
     * @param i   The integral. Corrects collected past errors to get rid of drifts or offsets.
     * @param d   The derivative. Predicts future error by reacting to the rate of change.
     * @param f   The feed forward. Provides a constant power guess based on the target value.
     */
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        motor.setVelocityPIDFCoefficients(p, i, d, f);
    }

    /**
     * Set the position PIDF Coefficients.
     * @param p   The proportional. Corrects based on current error by applying power.
     */
    public void setPositionPIDFCoefficients(double p) {
        motor.setPositionPIDFCoefficients(p);
    }

    /**
     * Get the current PIDF Coefficients.
     * @param runMode {@link DcMotor.RunMode}   The RunMode to be used.
     * @return {@link PIDFCoefficients}         The current PIDF Coefficients being used.
     */
    public PIDFCoefficients getPIDFCoefficients(DcMotor.RunMode runMode) {
        return motor.getPIDFCoefficients(runMode);
    }

    /**
     * Set the target position.
     * Doesn't allow target position to be set if the last target position provided is the same as the target position that's being set.
     * @param target    The target position to use.
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
    public int getCurrentPosition() {
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
     * @return {@link DcMotorEx}    The motor of the WrappedMotorEx.
     */
    public DcMotorEx getMotor() {
        return motor;
    }
}
