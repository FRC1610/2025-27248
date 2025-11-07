package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.StateMachine.State;
//import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;

import java.util.Locale;

@Disabled
@TeleOp(name = "Competition Main", group = "TeleOp")
public class Competition extends LinearOpMode {

    //GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    RobotHardware robot = new RobotHardware(this);

    StateMachine StateMachine;

    @Override
    public void runOpMode() {

        StateMachine = new StateMachine(robot);

        ///Variable Setup
        //Odometry
        double oldTime = 0;

        //Mecanum Drive
        double x;
        double y;
        double rotation;

        boolean aPressed = false;
        boolean bPressed = false;
        boolean xPressed = false;
        boolean yPressed = false;

        robot.init();  //Hardware configuration in RobotHardware.java

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {

            //Limelight Data
            LLResult result = robot.limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
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

            telemetry.addData("X Vel (mm/s)", VelX);
            telemetry.addData("Y Vel (mm/s)", VelY);
            telemetry.addData("Heading Vel (rad/s)", headingVel);

            telemetry.addData("Status", robot.pinpoint.getDeviceStatus());
            telemetry.addData("Pinpoint Frequency", robot.pinpoint.getFrequency()); //prints/gets the current refresh rate of the Pinpoint
            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate

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

            boolean TurretLeftPressed = gamepad1.dpad_left;
            boolean TurretRightPressed = gamepad1.dpad_right;

            if (TurretLeftPressed){
                robot.rotateTurret(RobotHardware.TurretDirection.LEFT);
            } else if (TurretRightPressed) {
                robot.rotateTurret(RobotHardware.TurretDirection.RIGHT);
            } else {
                robot.rotateTurret(RobotHardware.TurretDirection.STOP);
            }

            // --- Toggle Close Shot ---
            if (gamepad1.a && !aPressed) {
                robot.toggleFlywheel(Constants.launcherClose);
                aPressed = true;
            } else if (!gamepad1.a) {
                aPressed = false;
            }

            // --- Toggle Far Shot ---
            if (gamepad1.b && !bPressed) {
                robot.toggleFlywheel(Constants.launcherFar);
                bPressed = true;
            } else if (!gamepad1.b) {
                bPressed = false;
            }

            // --- Decrease RPM ---
            if (gamepad1.x && !xPressed) {
                robot.adjustRPM(-100);
                xPressed = true;
            } else if (!gamepad1.x) {
                xPressed = false;
            }

            // --- Increase RPM ---
            if (gamepad1.y && !yPressed) {
                robot.adjustRPM(100);
                yPressed = true;
            } else if (!gamepad1.y) {
                yPressed = false;
            }

            ///STATE CHANGE BUTTON SETUP
            /// START
            if (gamepad1.start) {
                StateMachine.setState(State.HOME);  //START = HOME Position
            /// X
            } else if (gamepad1.x) {
                //StateMachine.setState(State.WALL_PICKUP);
            ///A
            }

            StateMachine.update(); //Update state machine in case of long running tasks
            telemetry.addData("State", StateMachine.getState());
            telemetry.addData("Intake Vel", robot.intake.getVelocity());
            telemetry.addData("Target RPM", robot.getTargetRPM());
            telemetry.addData("Current RPM", "%.1f", robot.getCurrentRPM());
            telemetry.addData("Flywheel On", robot.isFlywheelOn());
            telemetry.update();
        }
    }
}