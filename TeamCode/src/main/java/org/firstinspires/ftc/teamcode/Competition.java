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
@TeleOp(name = "Competition Main", group = "TeleOp")
public class Competition extends LinearOpMode {
    final RobotHardware robot = new RobotHardware(this);

    private boolean dpadUpPreviouslyPressed = false;
    private boolean dpadDownPreviouslyPressed = false;
    private boolean dpadLeftPreviouslyPressed = false;
    private boolean dpadRightPreviouslyPressed = false;
    private boolean gamepad1LeftStickPreviouslyPressed = false;

    private boolean leftStickPreviouslyPressed = false;

    private boolean rightStickPreviouslyPressed = false;
    private boolean dpadUpGamepad2PreviouslyPressed = false;

    @Override
    public void runOpMode() {

        ///Variable Setup
        //Odometry
        //double oldTime = 0;

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

            /// BUTTON MAPPING
            // D-Pad left/right = turret manual rotate
            // Trigger left/right = (hold) intake forward/reverse

            // Run turret tracking when the flywheel is active or Start is held for manual testing
            boolean trackingActive = flywheelController.isEnabled() || gamepad2.start;
            if (trackingActive) {
                turretTracker.update();
                robot.headlight.setPosition(Constants.headlightPower); //Set light power here
            } else {
                robot.turret.setPower(0);
                robot.headlight.setPosition(0.0);
            }

            // Flywheel toggle on gamepad2 back
            boolean backButtonPressed = gamepad2.back;
            if (backButtonPressed && !backButtonPreviouslyPressed) {
                flywheelController.toggle();
            }
            backButtonPreviouslyPressed = backButtonPressed;

            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;
            boolean dpadLeft = gamepad1.dpad_left;
            boolean dpadRight = gamepad1.dpad_right;

            boolean gamepad1LeftStickDown = gamepad1.left_stick_button;

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

            if (gamepad1LeftStickDown && !gamepad1LeftStickPreviouslyPressed) {
                robot.setPowerDampener(0.5);
            }

            gamepad1LeftStickPreviouslyPressed = gamepad1LeftStickDown;

            dpadUpPreviouslyPressed = dpadUp;
            dpadDownPreviouslyPressed = dpadDown;
            dpadLeftPreviouslyPressed = dpadLeft;
            dpadRightPreviouslyPressed = dpadRight;

            boolean rightBumperPressed = gamepad2.a;
            if (rightBumperPressed && !rightBumperPreviouslyPressed && shootingController.isIdle()
                    && flywheelController.isEnabled() && flywheelController.getTargetRpm() > 0) {
                shootingController.startShootSequence();
            }
            rightBumperPreviouslyPressed = rightBumperPressed;

            flywheelController.update();
            spindexerController.update();
            shootingController.update();

            ///INTAKE
            //IntakeDirection
            boolean IntakeForwardPressed = gamepad2.right_bumper; //Check if button pressed
            boolean IntakeReversePressed = gamepad2.left_bumper; //Check if button pressed

            if (IntakeForwardPressed){
                robot.runIntake(RobotHardware.IntakeDirection.IN);
            } else if (IntakeReversePressed) {
                robot.runIntake(RobotHardware.IntakeDirection.OUT);
            } else {
                robot.runIntake(RobotHardware.IntakeDirection.STOP);
            }

            boolean gamepad2DpadUpPressed = gamepad2.dpad_up;

            if (gamepad2DpadUpPressed && !dpadUpGamepad2PreviouslyPressed) {
                spindexerController.toggleAuto();
            }

            dpadUpGamepad2PreviouslyPressed = gamepad2DpadUpPressed;

            boolean leftStickDown = gamepad2.left_stick_button;
            boolean rightStickDown = gamepad2.right_stick_button;

            if (leftStickDown && !leftStickPreviouslyPressed) {
                spindexerController.advanceSpindexer();
            }

            if (rightStickDown && !rightStickPreviouslyPressed) {
                spindexerController.reverseSpindexer();
            }

            leftStickPreviouslyPressed = leftStickDown;
            rightStickPreviouslyPressed = rightStickDown;



            if (shootingController.isIdle()) {
                //Manual Lift Control
                if (gamepad1.a) {
                    robot.kicker.setPosition(Constants.KICKER_UP);
                } else {
                    robot.kicker.setPosition(Constants.KICKER_DOWN);
                }

                // Spindexer Manual Control
                if (gamepad2.b) {
                    spindexerController.setPosition(0);
                } else if (gamepad2.y) {
                    spindexerController.setPosition(1);
                } else if (gamepad2.x) {
                    spindexerController.setPosition(2);
                }
            }

            telemetry.addLine("--- FLYWHEEL DATA ---");
            telemetry.addData("Flywheel Tolerance", "%.0f rpm", flywheelController.getRpmTolerance());
            telemetry.addData("Launcher F", "%.0f", FlywheelPidfConfig.launcherF);
            telemetry.addLine("--------------------------------");
            robot.flushPanelsTelemetry(telemetry);
            telemetry.update();
        }
    }

}
