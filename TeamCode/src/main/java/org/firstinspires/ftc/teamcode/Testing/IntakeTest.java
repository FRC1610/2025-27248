package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerController;

@Disabled
@TeleOp(name = "Intake Test", group = "Test")
public class IntakeTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(this);
        robot.init();

        SpindexerController spindexerController = new SpindexerController(robot, telemetry);

        spindexerController.toggleAuto();

        waitForStart();

        while (opModeIsActive()) {
            spindexerController.update();

            if (gamepad1.right_bumper && !spindexerController.isSpindexerFull()) {
                robot.runIntake(RobotHardware.IntakeDirection.IN);
            } else if (gamepad1.left_bumper) {
                robot.runIntake(RobotHardware.IntakeDirection.OUT);
            } else {
                robot.runIntake(RobotHardware.IntakeDirection.STOP);
            }

            // Spindexer positions on gamepad1 B/X/Y
            if (gamepad1.b) {
                spindexerController.setPosition(0);
            } else if (gamepad1.y) {
                spindexerController.setPosition(1);
            } else if (gamepad1.x) {
                spindexerController.setPosition(2);
            }

            telemetry.update();
        }

        robot.runIntake(RobotHardware.IntakeDirection.STOP);
    }
}
