package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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
        boolean leftPressedLast = false;
        boolean rightPressedLast = false;

        while (opModeIsActive()) {
            boolean leftPressed = gamepad1.dpad_left;
            boolean rightPressed = gamepad1.dpad_right;

            if (rightPressed && !rightPressedLast) {
                turretTarget += 25;
            }

            if (leftPressed && !leftPressedLast) {
                turretTarget -= 25;
            }

            rightPressedLast = rightPressed;
            leftPressedLast = leftPressed;

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
