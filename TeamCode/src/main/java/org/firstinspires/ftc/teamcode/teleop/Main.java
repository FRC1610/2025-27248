package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.RobotHardware;

@TeleOp(name = "Main", group = "TeleOp")
public class Main extends OpMode {
    private RobotHardware robot;

    @Override
    public void init() {
        robot = new RobotHardware(this);
    }

    @Override
    public void loop() {
        robot.drive.fieldCentricDrive(-gamepad1.left_stick_y, gamepad1.right_stick_x);
        robot.update();
        telemetry.addLine("hi i am telemetry");
        telemetry.update();
    }
}
