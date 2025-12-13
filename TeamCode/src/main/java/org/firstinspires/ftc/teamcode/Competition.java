package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.ArtifactTracker;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelPidfConfig;
import org.firstinspires.ftc.teamcode.subsystems.ShootingController;
import org.firstinspires.ftc.teamcode.subsystems.TurretTracker;
//import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;

import java.util.Locale;

//@Disabled
@TeleOp(name = "Competition Main", group = "TeleOp")
public class Competition extends LinearOpMode {

    //GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    RobotHardware robot = new RobotHardware(this);
    private TurretTracker turretTracker;
    private FlywheelController flywheelController;
    private ShootingController shootingController;
    private ArtifactTracker artifactTracker;

    private boolean dpadUpPreviouslyPressed = false;
    private boolean dpadDownPreviouslyPressed = false;
    private boolean dpadLeftPreviouslyPressed = false;
    private boolean dpadRightPreviouslyPressed = false;

    private final double[] spindexerPositions = new double[]{Constants.spindexer1, Constants.spindexer2, Constants.spindexer3};
    private int spindexerIndex = 0;

    @Override
    public void runOpMode() {

        ///Variable Setup
        //Odometry
        double oldTime = 0;

        //Mecanum Drive
        double x;
        double y;
        double rotation;

        //boolean aPressed = false;
        //boolean bPressed = false;

        // boolean dpadLeft2PreviouslyPressed = false;
        // dpadRight2PreviouslyPressed = false;
        boolean backButtonPreviouslyPressed = false;
        boolean rightBumperPreviouslyPressed = false;


        robot.init();  //Hardware configuration in RobotHardware.java

        robot.spindexer.setPosition(spindexerPositions[spindexerIndex]);
        robot.spindexerPos = spindexerPositions[spindexerIndex];

        turretTracker = new TurretTracker(robot, telemetry);
        flywheelController = new FlywheelController(robot, telemetry);
        shootingController = new ShootingController(robot, flywheelController, telemetry);
        artifactTracker = new ArtifactTracker(robot, telemetry);

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {

            robot.refreshLimelightResult();
            artifactTracker.update();

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
            double newTime = getRuntime();
            double loopTime = newTime - oldTime;
            double frequency = 1 / loopTime;
            oldTime = newTime;
            Pose2D pos = robot.pinpoint.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            double VelX = robot.pinpoint.getVelX(DistanceUnit.MM);
            double VelY = robot.pinpoint.getVelY(DistanceUnit.MM);
            double headingVel = robot.pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

            telemetry.addData("Velocities (mm/s,deg/s)", "X: %.0f  Y: %.0f  H: %.1f", VelX, VelY, headingVel);

            //telemetry.addData("Status", robot.pinpoint.getDeviceStatus());
            //telemetry.addData("Pinpoint Frequency", robot.pinpoint.getFrequency()); //prints/gets the current refresh rate of the Pinpoint
            //telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate

            ///MECANUM DRIVE

            // Get joystick inputs
            y = -gamepad1.left_stick_y * 0.90; // Forward/backward - multiply by 0.90 to scale speed down
            x = gamepad1.left_stick_x * 0.90;  // Strafe - multiply by 0.90 to scale speed down
            if (gamepad1.right_stick_button) {
                rotation = gamepad1.right_stick_x * 0.50; //Slow rotation mode when button pressed in
            } else {
                rotation = gamepad1.right_stick_x * 0.75; // Rotation - multiply by 0.75 to scale speed down
            }

            robot.mecanumDrive(x, y, rotation);

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

            boolean rightBumperPressed = gamepad2.a;
            if (rightBumperPressed && !rightBumperPreviouslyPressed && shootingController.isIdle()
                    && flywheelController.isEnabled() && flywheelController.getTargetRpm() > 0) {
                shootingController.startShootSequence();
            }
            rightBumperPreviouslyPressed = rightBumperPressed;

            flywheelController.update();
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

            /*
            // ----- Spindexer test control -----

            boolean dpadLeft2  = gamepad2.dpad_left;
            boolean dpadRight2 = gamepad2.dpad_right;

            // Edge trigger LEFT (-0.05)
            if (dpadLeft2 && !dpadLeft2PreviouslyPressed) {
                robot.adjustSpindexer(-0.01);
            }

            // Edge trigger RIGHT (+0.05)
            if (dpadRight2 && !dpadRight2PreviouslyPressed) {
                robot.adjustSpindexer(0.01);
            }

            // update previous
            dpadLeft2PreviouslyPressed  = dpadLeft2;
            dpadRight2PreviouslyPressed = dpadRight2;

             */

            if (shootingController.isIdle()) {
                //Manual Lift Control
                if (gamepad1.a) {
                    robot.kicker.setPosition(Constants.kickerUp);
                } else {
                    robot.kicker.setPosition(Constants.kickerDown);
                }

                //Spindexer Manual Control
                if (gamepad2.b) {
                    robot.spindexer.setPosition(Constants.spindexer1);
                    robot.spindexerPos = Constants.spindexer1;
                } else if (gamepad2.y) {
                    robot.spindexer.setPosition(Constants.spindexer2);
                    robot.spindexerPos = Constants.spindexer2;
                } else if (gamepad2.x) {
                    robot.spindexer.setPosition(Constants.spindexer3);
                    robot.spindexerPos = Constants.spindexer3;
                }
            }

//            telemetry.addData("Turret", "enabled=%b  targetPos=%d  currentPos=%d", flywheelController.isEnabled(), robot.getTurretTarget(), robot.getTurretPosition());
//            telemetry.addData("Shooter", shootingController.getShootState());
//            telemetry.addData("Spindexer", "pos=%.2f", robot.spindexerPos);
            telemetry.addData("Flywheel Tolerance", "%.0f rpm", flywheelController.getRpmTolerance());
            telemetry.addData("Launcher F", "%.0f", FlywheelPidfConfig.launcherF);
            robot.flushPanelsTelemetry(telemetry);
            telemetry.update();
        }
    }

}
