package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drivers.rgbIndicator;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelPidfConfig;
import org.firstinspires.ftc.teamcode.subsystems.ShootingController;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerController;
import org.firstinspires.ftc.teamcode.subsystems.TurretTracker;

import java.util.Locale;

//@Disabled
@TeleOp(name = "Competition (TEST)", group = "TeleOp")
public class CompetitionTest extends LinearOpMode {
    final RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        robot.init();  //Hardware configuration in RobotHardware.java

        TurretTracker turretTracker = new TurretTracker(robot, telemetry);
        FlywheelController flywheelController = new FlywheelController(robot, telemetry);
        SpindexerController spindexerController = new SpindexerController(robot, telemetry);
        ShootingController shootingController = new ShootingController(robot, flywheelController, spindexerController, telemetry);

        boolean kickerStandToggled = false;

        spindexerController.init();

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {

            robot.refreshLimelightResult();

            //Limelight Data
            LLResult result = robot.getLatestLimelightResult();
            if (result != null) {
                if (result.isValid()) {
                    Pose3D botPose = result.getBotpose();
                    telemetry.addData("tx/ty", "tx: %.2f ty: %.2f", result.getTx(), result.getTy());
                    telemetry.addData("Bot Pose", botPose.toString());
                }
            }

            //Odometry
            robot.pinpoint.update(); //Update odometry
            Pose2D pos = robot.pinpoint.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addLine("--- ROBOT DATA ---");

            telemetry.addData("Position", data);
            double VelX = robot.pinpoint.getVelX(DistanceUnit.MM);
            double VelY = robot.pinpoint.getVelY(DistanceUnit.MM);
            double headingVel = robot.pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

            telemetry.addData("Wheel Power Dampening", robot.getPowerDampener());
            telemetry.addData("Velocities (mm/s,deg/s)", "X: %.0f  Y: %.0f  H: %.1f", VelX, VelY, headingVel);
            telemetry.addLine("---------------------------");

            robot.updateHeadingOffsetFromAllianceButton();
            double botHeading = robot.pinpoint.getHeading(AngleUnit.RADIANS);
            double adjustedHeading = robot.applyHeadingOffset(botHeading);

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            robot.FieldCentricDrive(x, y, rx, adjustedHeading);

            /// DPad Gamepad 1
            if (gamepad1.dpadUpWasPressed()) {
                flywheelController.adjustRpmTolerance(10.0);
            }

            if (gamepad1.dpadDownWasPressed()) {
                flywheelController.adjustRpmTolerance(-10.0);
            }

            if (gamepad1.dpadLeftWasPressed()) {
                flywheelController.adjustLauncherFeedforward(1.0);
            }

            if (gamepad1.dpadRightWasPressed()) {
                flywheelController.adjustLauncherFeedforward(-1.0);
            }

            if (gamepad1.leftStickButtonWasPressed()) {
                robot.setPowerDampener(0.5);
            }

            /// GAMEPAD 2

            /// Gamepad 2 Intake
            boolean intakeIn = gamepad2.right_bumper; //Check if button is currently held
            boolean intakeOut = gamepad2.left_bumper; //Check if button is currently held

            if (intakeIn){
                if (!spindexerController.isEnabled() || !spindexerController.isSpindexerFull()) {
                    robot.runIntake(RobotHardware.IntakeDirection.IN);
                }
            } else if (intakeOut) {
                robot.runIntake(RobotHardware.IntakeDirection.OUT);
            } else {
                robot.runIntake(RobotHardware.IntakeDirection.STOP);
            }

            /// Gamepad 2 Sticks
            if (gamepad2.leftStickButtonWasPressed()) {
                spindexerController.advanceSpindexer();
            }
            if (gamepad2.rightStickButtonWasPressed()) {
                spindexerController.reverseSpindexer();
            }

            /// Gamepad 2 FlyWheel toggle
            if (gamepad2.backWasPressed()) {
                flywheelController.toggle();
            }

            /// Gamepad 2 DPad
            if (gamepad2.dpadUpWasPressed()) {
                spindexerController.toggleAuto();
            }

            /// Gamepad 2 Tracking
            boolean trackingActive = flywheelController.isEnabled() || gamepad2.start;
            if (trackingActive) {
                turretTracker.update();
                robot.headlight.setPosition(Constants.headlightPower);
            } else {
                robot.turret.setPower(0);
                robot.headlight.setPosition(0.0);
            }

            /// Gamepad 2 Shoot Sequence
            if (gamepad2.aWasPressed() && shootingController.isIdle()
                    && flywheelController.isEnabled() && flywheelController.getTargetRpm() > 0) {
                shootingController.startShootSequence();
            }

            /// Gamepad 1 & 2 Spindexer Manuals
            if (shootingController.isIdle()) {
                // Kicker Manual
                if (gamepad1.a) {
                    robot.kicker.setPosition(Constants.KICKER_UP);
                } else {
                    robot.kicker.setPosition(Constants.KICKER_DOWN);
                }

                // Spindexer Manual
                if (gamepad2.b) {
                    spindexerController.setPosition(0);
                } else if (gamepad2.y) {
                    spindexerController.setPosition(1);
                } else if (gamepad2.x) {
                    spindexerController.setPosition(2);
                }
            }

            /*
            ///  Gamepad 1 Toggle Kicker Stand
            if (gamepad1.rightBumperWasPressed()) {
                if (kickerStandToggled) {
                    robot.setKickStandPosition(Constants.KICKERSTAND_NORMAL);
                } else {
                    robot.setKickStandPosition(Constants.KICKERSTAND_RETRACTED);
                }
                kickerStandToggled = !kickerStandToggled;
            }

            if (kickerStandToggled) {
                robot.setColorOfBackLights(rgbIndicator.LEDColors.INDIGO);
            } else {
                flywheelController.update();
                spindexerController.update();
                shootingController.update(false);
            }

             */

            telemetry.addLine("--- FLYWHEEL DATA ---");
            telemetry.addData("Flywheel Tolerance", "%.0f rpm", flywheelController.getRpmTolerance());
            telemetry.addData("Launcher F", "%.0f", FlywheelPidfConfig.launcherF);
            telemetry.addLine("--------------------------------");
            robot.flushPanelsTelemetry(telemetry);
            telemetry.update();
        }
    }

}
