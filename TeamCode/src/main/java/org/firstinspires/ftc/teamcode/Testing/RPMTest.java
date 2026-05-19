package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

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
import org.firstinspires.ftc.teamcode.subsystems.LauncherMotorGroup;
import org.firstinspires.ftc.teamcode.subsystems.ShootingController;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerController;
import org.firstinspires.ftc.teamcode.subsystems.TurretTracker;

import java.util.Locale;

// <3.5 ft - 2,200 rpm
// 5-6ft(?) - 2,350 rpm
// max feet - 2,650 rpm

//@Disabled
@TeleOp(name = "RPM Test Mode", group = "TeleOp")
public class RPMTest extends LinearOpMode {
    private static final double TICKS_PER_REV = 28.0;

    final RobotHardware robot = new RobotHardware(this);

    private boolean dpadUpPreviouslyPressed = false;
    private boolean dpadDownPreviouslyPressed = false;
    private boolean dpadLeftPreviouslyPressed = false;
    private boolean dpadRightPreviouslyPressed = false;

    private boolean dpadUpPreviouslyPressed2 = false;
    private boolean dpadDownPreviouslyPressed2 = false;
    private boolean dpadLeftPreviouslyPressed2 = false;
    private boolean dpadRightPreviouslyPressed2 = false;

    private double targetRpm = 0.0;
    private double spinupSetpointRpm = 0.0;
    private final ElapsedTime spinupTimer = new ElapsedTime();
    private boolean measuringSpinup = false;
    private FlywheelController flywheelController;

    private double rpmToTicksPerSecond(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }

    private void setFlywheelRpm(double rpm) {
        if (rpm > 0 && targetRpm <= 0) {
            spinupSetpointRpm = rpm;
            spinupTimer.reset();
            measuringSpinup = true;
        }

        targetRpm = rpm;
        LauncherMotorGroup launcherGroup = robot.launcherGroup;
        if (launcherGroup == null) {
            telemetry.addLine("ERROR: launcher motor is NULL!");
            return;
        }

        double ticksPerSecond = rpmToTicksPerSecond(targetRpm);
        launcherGroup.group.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherGroup.group.setVelocity(ticksPerSecond);
    }

    public double getCurrentRpm() {
        LauncherMotorGroup launcherGroup = robot.launcherGroup;
        if (launcherGroup == null) {
            telemetry.addLine("ERROR: launcher motor is NULL!");
            return 0.0;
        }
        return (launcherGroup.group.getVelocity() * 60.0) / TICKS_PER_REV;
    }

    private boolean isAtSpeed() {
        return (getCurrentRpm() >= (targetRpm - ((double) 85 /2))) && ( getCurrentRpm() <= (targetRpm + 85) );
    }
    private void setFrontLedColor(double color) {
        if (robot.frontLED != null) {
            robot.frontLED.setColor(color);
        }
    }

    private void updateFrontLedColor() {
        if (robot.frontLED == null) {
            return;
        }

        if (!flywheelController.isEnabled()) {
            setFrontLedColor(rgbIndicator.LEDColors.OFF);
            return;
        }

        if (targetRpm == Constants.DEFAULT_RPM) {
            setFrontLedColor(rgbIndicator.LEDColors.VIOLET);
            return;
        }

        double currentRpm = Math.abs(targetRpm);
        double minimumRpm = targetRpm - 85;
        double maxRpm = targetRpm + 85;

        if ( isAtSpeed() ) {
            setFrontLedColor(rgbIndicator.LEDColors.GREEN);
        } else if (currentRpm > maxRpm ) {
            setFrontLedColor(rgbIndicator.LEDColors.RED);
        } else if (currentRpm >= minimumRpm * 0.75) {
            setFrontLedColor(rgbIndicator.LEDColors.ORANGE);
        } else if (currentRpm >= minimumRpm * 0.5) {
            setFrontLedColor(rgbIndicator.LEDColors.YELLOW);
        } else {
            setFrontLedColor(rgbIndicator.LEDColors.RED);
        }
    }

    public void update() {
        robot.launcherGroup.refreshLauncherPIDFFromConfig();

        if (!flywheelController.isEnabled()) {
            setFrontLedColor(rgbIndicator.LEDColors.OFF);
            flywheelController.publishPanelsFlywheelTelemetry(targetRpm, getCurrentRpm());
            return;
        }

        if (robot.launcherGroup == null) {
            telemetry.addLine("ERROR: launcher motor is NULL!");
            setFrontLedColor(rgbIndicator.LEDColors.OFF);
            return;
        }

        updateFrontLedColor();

        flywheelController.publishPanelsFlywheelTelemetry(targetRpm, getCurrentRpm());

        if (measuringSpinup && isAtSpeed()) {
            double elapsedSeconds = spinupTimer.seconds();
            RobotLog.ii("FlywheelController", "Spin-up to %.0f RPM reached in %.2f s", spinupSetpointRpm, elapsedSeconds);
            measuringSpinup = false;
        }
    }

    @Override
    public void runOpMode() {
        boolean backButtonPreviouslyPressed = false;


        robot.init();

        TurretTracker turretTracker = new TurretTracker(robot, telemetry);
        SpindexerController spindexerController = new SpindexerController(robot, telemetry);
        flywheelController = new FlywheelController(robot, telemetry);
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
                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("tx/ty", "tx: %.2f ty: %.2f", result.getTx(), result.getTy());
                    telemetry.addData("Botpose", botpose.toString());
                }
            }

            //Odometry
            robot.pinpoint.update(); //Update odometry
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
                setFlywheelRpm(targetRpm + 100);
            }

            if (dpadDown && !dpadDownPreviouslyPressed) {
                setFlywheelRpm(targetRpm - 100);
            }

            if (dpadRight && !dpadRightPreviouslyPressed) {
                setFlywheelRpm(targetRpm - 10);
            }

            if (dpadLeft && !dpadLeftPreviouslyPressed) {
                setFlywheelRpm(targetRpm + 10);
            }

            boolean dpadUp2 = gamepad1.dpad_up;
            boolean dpadDown2 = gamepad1.dpad_down;
            boolean dpadLeft2 = gamepad1.dpad_left;
            boolean dpadRight2 = gamepad1.dpad_right;

            if (dpadUp2 && !dpadUpPreviouslyPressed2) {
                flywheelController.adjustLauncherFeedforward(1.0);
            }

            if (dpadDown2 && !dpadDownPreviouslyPressed2) {
                flywheelController.adjustLauncherFeedforward(-1.0);
            }

            if (dpadRight2 && !dpadRightPreviouslyPressed2) {
                flywheelController.adjustLauncherFeedforward(10.0);
            }

            if (dpadLeft2 && !dpadLeftPreviouslyPressed2) {
                flywheelController.adjustLauncherFeedforward(-10.0);
            }

            dpadUpPreviouslyPressed = dpadUp;
            dpadDownPreviouslyPressed = dpadDown;
            dpadLeftPreviouslyPressed = dpadLeft;
            dpadRightPreviouslyPressed = dpadRight;

            dpadUpPreviouslyPressed2 = dpadUp2;
            dpadDownPreviouslyPressed2 = dpadDown2;
            dpadLeftPreviouslyPressed2 = dpadLeft2;
            dpadRightPreviouslyPressed2 = dpadRight2;

            update();
            shootingController.update();

            if (shootingController.isIdle()) {
                //Manual Lift Control
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

            telemetry.addData("Flywheel RPM", "%.0f rpm", targetRpm);
            telemetry.addData("Flywheel Tolerance", "%.0f rpm", flywheelController.getRpmTolerance());
            telemetry.addData("Launcher F", "%.0f", FlywheelPidfConfig.launcherF);
            robot.flushPanelsTelemetry(telemetry);
            telemetry.update();
        }
    }

}
