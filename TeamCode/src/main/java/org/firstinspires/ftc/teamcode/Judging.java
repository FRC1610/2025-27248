package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ArtifactTracker;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.subsystems.TurretTracker;

@TeleOp(name = "Judging", group = "Test")
public class Judging extends LinearOpMode {

    private final RobotHardware robot = new RobotHardware(this);
    private TurretTracker turretTracker;
    private FlywheelController flywheelController;
    private ArtifactTracker artifactTracker;

    @Override
    public void runOpMode() {
        boolean backButtonPreviouslyPressed = false;
        boolean startButtonPreviouslyPressed = false;
        boolean turretTrackingEnabled = false;

        robot.init();
        turretTracker = new TurretTracker(robot, telemetry);
        flywheelController = new FlywheelController(robot, telemetry);
        artifactTracker = new ArtifactTracker(robot, telemetry);

        telemetry.addLine("Judging OpMode Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            robot.refreshLimelightResult();
            artifactTracker.update();

            // Toggle turret tracking on gamepad1 back button
            boolean backButtonPressed = gamepad1.back;
            if (backButtonPressed && !backButtonPreviouslyPressed) {
                turretTrackingEnabled = !turretTrackingEnabled;
            }
            backButtonPreviouslyPressed = backButtonPressed;

            // Toggle flywheel enable on gamepad1 start button
            boolean startButtonPressed = gamepad1.start;
            if (startButtonPressed && !startButtonPreviouslyPressed) {
                flywheelController.toggle();
            }
            startButtonPreviouslyPressed = startButtonPressed;

            // Run turret tracking when enabled
            if (turretTrackingEnabled) {
                turretTracker.update();
                robot.headlight.setPosition(Constants.headlightPower);
            } else {
                robot.turret.setPower(0);
                robot.headlight.setPosition(0.0);
            }

            // Flywheel updates based on Limelight distance
            flywheelController.update();

            // Intake control on gamepad1 bumpers
            if (gamepad1.right_bumper) {
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

            // Telemetry for sensors and indicators
            telemetry.addData("Alliance", robot.allianceColorRed ? "RED" : "BLUE");
            telemetry.addData("Turret Tracking", turretTrackingEnabled);
            telemetry.addData("Flywheel Enabled", flywheelController.isEnabled());
            telemetry.addData("Flywheel Target RPM", "%.0f", flywheelController.getTargetRpm());
            telemetry.addData("Flywheel Measured RPM", "%.0f", flywheelController.getCurrentRpm());
            telemetry.update();
        }
    }
}
