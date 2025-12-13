package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ArtifactTracker;
import org.firstinspires.ftc.teamcode.subsystems.IntakeController;

@Disabled
@TeleOp(name = "Intake Test", group = "Test")
public class IntakeTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(this);
        robot.init();

        ArtifactTracker artifactTracker = new ArtifactTracker(robot, telemetry);
        IntakeController intakeController = new IntakeController(robot, artifactTracker, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            intakeController.update();

            if (gamepad1.right_bumper && !intakeController.isSpindexerFull()) {
                robot.runIntake(RobotHardware.IntakeDirection.IN);
            } else if (gamepad1.left_bumper) {
                robot.runIntake(RobotHardware.IntakeDirection.OUT);
            } else {
                robot.runIntake(RobotHardware.IntakeDirection.STOP);
            }

            // Spindexer positions on gamepad1 B/X/Y
            if (gamepad1.b) {
                robot.spindexer.setPosition(Constants.spindexer1);
                robot.spindexerPos = Constants.spindexer1;
            } else if (gamepad1.y) {
                robot.spindexer.setPosition(Constants.spindexer2);
                robot.spindexerPos = Constants.spindexer2;
            } else if (gamepad1.x) {
                robot.spindexer.setPosition(Constants.spindexer3);
                robot.spindexerPos = Constants.spindexer3;
            }

            telemetry.update();
        }

        robot.runIntake(RobotHardware.IntakeDirection.STOP);
    }
}
