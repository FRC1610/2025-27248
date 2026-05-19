package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelPidfConfig;
import org.firstinspires.ftc.teamcode.subsystems.ShootingController;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerController;
import org.firstinspires.ftc.teamcode.subsystems.TurretTracker;

import java.util.Locale;

//@Disabled
@TeleOp(name = "Competition Main (SOLO MODE)", group = "TeleOp")
public class CompetitionSolo extends LinearOpMode {

    final RobotHardware robot = new RobotHardware(this);

    private boolean dpadUpPreviouslyPressed = false;
    private boolean dpadDownPreviouslyPressed = false;
    private boolean dpadLeftPreviouslyPressed = false;
    private boolean dpadRightPreviouslyPressed = false;

    @Override
    public void runOpMode() {
        boolean backButtonPreviouslyPressed = false;
        boolean rightBumperPreviouslyPressed = false;


        robot.init();  //Hardware configuration in RobotHardware.java

        TurretTracker turretTracker = new TurretTracker(robot, telemetry);
        FlywheelController flywheelController = new FlywheelController(robot, telemetry);
        SpindexerController spindexerController = new SpindexerController(robot, telemetry);
        ShootingController shootingController = new ShootingController(robot, flywheelController, spindexerController, telemetry);

        spindexerController.init();

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {

            robot.refreshLimelightResult();

            LLResult result = robot.getLatestLimelightResult();
            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("tx/ty", "tx: %.2f ty: %.2f", result.getTx(), result.getTy());
                    telemetry.addData("Botpose", botpose.toString());
                }
            }

            robot.pinpoint.update();
            Pose2D pos = robot.pinpoint.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            double VelX = robot.pinpoint.getVelX(DistanceUnit.MM);
            double VelY = robot.pinpoint.getVelY(DistanceUnit.MM);
            double headingVel = robot.pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

            telemetry.addData("Velocities (mm/s,deg/s)", "X: %.0f  Y: %.0f  H: %.1f", VelX, VelY, headingVel);

            robot.updateHeadingOffsetFromAllianceButton();
            double botHeading = robot.pinpoint.getHeading(AngleUnit.RADIANS);
            double adjustedHeading = robot.applyHeadingOffset(botHeading);

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            robot.FieldCentricDrive(x, y, rx, adjustedHeading);

            /// BUTTON MAPPING
            // D-Pad left/right = turret manual rotate
            // Trigger left/right = (hold) intake forward/reverse

            // Run turret tracking when the flywheel is active or Start is held for manual testing
            boolean trackingActive = flywheelController.isEnabled() || gamepad1.start;
            if (trackingActive) {
                turretTracker.update();
                robot.headlight.setPosition(Constants.headlightPower); //Set light power here
            } else {
                robot.turret.setPower(0);
                robot.headlight.setPosition(0.0);
            }

            // Flywheel toggle on gamepad1 back
            boolean backButtonPressed = gamepad1.back;
            if (backButtonPressed && !backButtonPreviouslyPressed) {
                flywheelController.toggle();
            }
            backButtonPreviouslyPressed = backButtonPressed;

            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;
            boolean dpadLeft = gamepad1.dpad_left;
            boolean dpadRight = gamepad1.dpad_right;

            if (dpadUp && !dpadUpPreviouslyPressed) {
                flywheelController.adjustRpmTolerance(10.0);
            }

            if (dpadDown && !dpadDownPreviouslyPressed) {
                flywheelController.adjustRpmTolerance(-10.0);
            }

            if (dpadRight && !dpadRightPreviouslyPressed) {
                flywheelController.adjustLauncherFeedforward(1.0);
            }

            if (dpadLeft && !dpadLeftPreviouslyPressed) {
                flywheelController.adjustLauncherFeedforward(-1.0);
            }

            dpadUpPreviouslyPressed = dpadUp;
            dpadDownPreviouslyPressed = dpadDown;
            dpadLeftPreviouslyPressed = dpadLeft;
            dpadRightPreviouslyPressed = dpadRight;

            boolean rightBumperPressed = gamepad1.a;
            if (rightBumperPressed && !rightBumperPreviouslyPressed && shootingController.isIdle()
                    && flywheelController.isEnabled() && flywheelController.getTargetRpm() > 0) {
                shootingController.startShootSequence();
            }
            rightBumperPreviouslyPressed = rightBumperPressed;

            flywheelController.update();
            shootingController.update();

            ///INTAKE
            //IntakeDirection
            boolean IntakeForwardPressed = gamepad1.right_bumper; //Check if button pressed
            boolean IntakeReversePressed = gamepad1.left_bumper; //Check if button pressed

            if (IntakeForwardPressed){
                robot.runIntake(RobotHardware.IntakeDirection.IN);
            } else if (IntakeReversePressed) {
                robot.runIntake(RobotHardware.IntakeDirection.OUT);
            } else {
                robot.runIntake(RobotHardware.IntakeDirection.STOP);
            }

            if (shootingController.isIdle()) {
                if (gamepad1.left_trigger >= 0.5) {
                    robot.kicker.setPosition(Constants.KICKER_UP);
                } else {
                    robot.kicker.setPosition(Constants.KICKER_DOWN);
                }

                //Spindexer Manual Control
                if (gamepad1.b) {
                    spindexerController.setPosition(0);
                } else if (gamepad1.y) {
                    spindexerController.setPosition(1);
                } else if (gamepad1.x) {
                    spindexerController.setPosition(2);
                }
            }

            telemetry.addData("Flywheel Tolerance", "%.0f rpm", flywheelController.getRpmTolerance());
            telemetry.addData("Launcher F", "%.0f", FlywheelPidfConfig.launcherF);
            robot.flushPanelsTelemetry(telemetry);
            telemetry.update();
        }
    }

}
