package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.*;
import org.firstinspires.ftc.teamcode.subsystems.*;

public class RobotHardware {
    /// OpMode
    private final OpMode opMode;

    /// Subsystems
    public DriveSubsystem drive;

    /// Motor, Servos, Etc
    public WrappedMotor frontLeft;
    public WrappedMotor frontRight;
    public WrappedMotor backLeft;
    public WrappedMotor backRight;

    /// Constructor

    public RobotHardware(OpMode opMode) {
        this.opMode = opMode;
    }


    /// Methods


    public void init() {
        // Motor Setup
        frontLeft = new WrappedMotor(opMode.hardwareMap, "frontLeft");
        backLeft = new WrappedMotor(opMode.hardwareMap, "backLeft");
        frontRight = new WrappedMotor(opMode.hardwareMap, "frontRight");
        backRight = new WrappedMotor(opMode.hardwareMap, "backRight");

        // Servo Setup

        // Subsystem Setup
        drive = new DriveSubsystem(frontLeft, frontRight, backLeft, backRight);
    }

    public void update() {}
}
