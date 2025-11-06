package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.StateMachine.State;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;

import java.util.Locale;

@Disabled
@TeleOp(name = "Competition Main", group = "TeleOp")
public class Competition extends LinearOpMode {

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

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

        //Elevator
        boolean manualControl = false; // Default to position-based control
        boolean backButtonPreviouslyPressed = false; // To track toggle state
        double elevatorPower = 0;

        //Intake
        //boolean IntakeClosed = true;
        //boolean IntakeButtonWasPressed = false;
        boolean RightBumperPressed = false;
        boolean aPressed = false;
        boolean leftBumperPressed = false;
        //double PosChange = 0.0;

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
            robot.odo.update(); //Update odometry
            double newTime = getRuntime();
            double loopTime = newTime - oldTime;
            double frequency = 1 / loopTime;
            oldTime = newTime;
            Pose2D pos = robot.odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            Pose2D vel = robot.odo.getVelocity();
            String velocity = String.format(Locale.US, "{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.INCH), vel.getY(DistanceUnit.INCH), vel.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Velocity", velocity);
            telemetry.addData("Status", robot.odo.getDeviceStatus());
            telemetry.addData("Pinpoint Frequency", robot.odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint
            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate

            ///MECANUM DRIVE

            // Get joystick inputs
            y = -gamepad1.left_stick_y * 0.80; // Forward/backward - multiply by 0.75 to scale speed down
            x = gamepad1.left_stick_x * 0.80;  // Strafe - multiply by 0.75 to scale speed down
            if (gamepad1.right_stick_button) {
                rotation = gamepad1.right_stick_x * 0.45; //Slow rotation mode when button pressed in
            } else {
                rotation = gamepad1.right_stick_x * 0.75; // Rotation - multiply by 0.75 to scale speed down
            }

            robot.mecanumDrive(x, y, rotation);

            ///INTAKE

            //Intake Pincher
            boolean IntakeButtonPressed = gamepad1.left_bumper; //Check if button pressed

            /// Elevator Pincher Rotation Test
            //TODO Remove this once State Machine handles this
            if (gamepad2.a){
                //robot.ElevatorPivot(0.01);
            } else if (gamepad2.b) {
                //robot.ElevatorPivot(-0.01);
            } else {
                //robot.ElevatorPivot(0);
            }

            //TODO Remove this once State Machine handles this
            if (gamepad2.x){
                //robot.setElevatorPincher(0.01);
            } else if (gamepad2.y) {
                //robot.setElevatorPincher(-0.01);
            } else {
                //robot.setElevatorPincher(0);
            }

            //telemetry.addData("Elev Pivot", robot.elevatorPivot.getPosition());
            //telemetry.addData("Elev Pinch Rotate", robot.elevatorPincherRotate.getPosition());
            //telemetry.addData("Elev Pinch Pos", robot.elevatorPincher.getPosition());

            ///STATE CHANGE BUTTON SETUP
            //TODO Maybe rearrange this
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


            telemetry.addData("Elevator Pos", robot.intake.getCurrentPosition());

            telemetry.update();
        }
    }
}