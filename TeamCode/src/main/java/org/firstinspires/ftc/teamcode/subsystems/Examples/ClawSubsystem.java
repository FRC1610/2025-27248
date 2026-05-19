package org.firstinspires.ftc.teamcode.subsystems.Examples;

import org.firstinspires.ftc.teamcode.hardware.WrappedServo;

public class ClawSubsystem {
    /// Data
    private final WrappedServo servo;
    private boolean open = false;

    /// Constructor

    /**
     * Create a ClawSubsystem using a WrappedServo class.
     * @param servo {@link WrappedServo} The servo to use.
     */
    public ClawSubsystem(WrappedServo servo) {
        this.servo = servo;
    }

    /// Methods

    /**
     * Toggle the claw.
     */
    public void toggle() {
        open = !open;
        if (open) servo.open();
        else servo.close();
    }

    /**
     * Open the claw.
     */
    public void open() {
        servo.open();
    }

    /**
     * Close the claw.
     */
    public void close() {
        servo.close();
    }
}
