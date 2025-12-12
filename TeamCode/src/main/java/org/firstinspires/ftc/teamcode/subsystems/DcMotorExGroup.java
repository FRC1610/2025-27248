package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class DcMotorExGroup {
    private DcMotorEx[] motors;

    public DcMotorExGroup(DcMotorEx... motors) {
        this.motors = motors;
    }

    public void setMode(DcMotor.RunMode mode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(mode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(behavior);
        }
    }

    public void setTargetPositionTolerance(int tolerance) {
        for (DcMotorEx motor : motors) {
            motor.setTargetPositionTolerance(tolerance);
        }
    }

    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        for (DcMotorEx motor : motors) {
            motor.setVelocityPIDFCoefficients(p, i, d, f);
        }
    }

    public void setPower(double power) {
        for (DcMotorEx motor : motors) {
            motor.setPower(power);
        }
    }

    public double getPower() {
        double sum = 0;
        for (DcMotorEx motor : motors) {
            sum += motor.getPower();
        }
        return sum / motors.length;
    }

    public void setVelocity(double angularRate) {
        for (DcMotorEx motor : motors) {
            motor.setVelocity(angularRate);
        }
    }

    public double getVelocity() {
        double sum = 0;
        for (DcMotorEx motor : motors) {
            sum += motor.getVelocity();
        }
        return sum / motors.length;
    }
}