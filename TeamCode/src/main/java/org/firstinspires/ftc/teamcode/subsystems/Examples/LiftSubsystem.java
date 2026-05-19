package org.firstinspires.ftc.teamcode.subsystems.Examples;

import org.firstinspires.ftc.teamcode.hardware.WrappedMotorEx;

public class LiftSubsystem {
    /// Data

    private final WrappedMotorEx motor;
    private int target = 0;

    /// Constructor

    /**
     * Uses a WrappedMotorEx class to make a simple lift system.
     * @param motor {@link WrappedMotorEx}  The motor to be used.
     */
    public LiftSubsystem(WrappedMotorEx motor) {
        this.motor = motor;
    }

    /// Methods

    /**
     * Move the lift to a certain position.
     * @param position  The position for the lift to move to.
     */
    public void moveTo(int position) {
        target = position;
    }

    /**
     * Update the lifts power so it can reach its target.
     * This should be looped so that {@link #moveTo} works.
     */
    public void update() {
        int error = target - motor.getCurrentPosition();
        double power = error * 0.005;
        power = Math.max(-1, Math.min(1, power));
        motor.setPower(power);
    }

    /**
     * Check if the motor is at the target position.
     * @return If it's at position.
     */
    public boolean atTarget() {
        return Math.abs(target - motor.getCurrentPosition()) < 20;
    }
}
