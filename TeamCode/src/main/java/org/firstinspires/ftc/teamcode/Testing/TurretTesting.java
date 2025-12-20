package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name = "Turret Testing", group = "Test")
public class TurretTesting extends LinearOpMode {

    private final RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        robot.init();

        waitForStart();

        if (robot.turret == null) {
            telemetry.addLine("ERROR: turret motor is NULL!");
            telemetry.update();
            return;
        }

        int turretTarget = robot.turret.getCurrentPosition();

        while (opModeIsActive()) {
            if (gamepad1.dpadRightWasPressed()) {
                turretTarget += 25;
            }

            if (gamepad1.dpadLeftWasPressed()) {
                turretTarget -= 25;
            }

            robot.turret.setTargetPosition(turretTarget);
            robot.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.turret.setPower(0.40);

            telemetry.addData("Turret Target", turretTarget);
            telemetry.addData("Turret Position", robot.turret.getCurrentPosition());
            telemetry.update();

            idle();
        }
    }
}
