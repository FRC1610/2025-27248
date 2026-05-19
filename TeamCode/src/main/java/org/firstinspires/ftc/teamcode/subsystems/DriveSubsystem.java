package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.hardware.WrappedMotor;

public class DriveSubsystem {
    private final WrappedMotor frontLeft;
    private final WrappedMotor frontRight;
    private final WrappedMotor backLeft;
    private final WrappedMotor backRight;

    public DriveSubsystem(WrappedMotor frontLeft, WrappedMotor frontRight, WrappedMotor backLeft, WrappedMotor backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
    }

    public void fieldCentricDrive(double y, double header) {
        double left = y + header;
        double right = y - header;
        frontLeft.setPower(left);
        backLeft.setPower(left);
        frontRight.setPower(right);
        backRight.setPower(right);
    }
}
