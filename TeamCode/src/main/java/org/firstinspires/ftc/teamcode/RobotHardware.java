package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.drivers.rgbIndicator;
import org.firstinspires.ftc.teamcode.drivers.rgbIndicator.LEDColors;

public class RobotHardware {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;
    public DcMotorEx intake = null;
    public DcMotorEx launcher = null;
    public Servo spindexer = null;
    public Servo turret = null;
    public Servo hood = null;
    public Servo kicker = null;
    public boolean allianceColorRed = false;
    public boolean allianceColorBlue = false;

    public enum IntakeDirection {
        IN,
        OUT,
        STOP
    }

    public enum TurretDirection {
        LEFT,
        RIGHT,
        STOP
    }

    Limelight3A limelight = null;
    GoBildaPinpointDriver pinpoint = null; // Declare OpMode member for the Odometry Computer
    rgbIndicator rgbIndicatorMain = null;
    private DigitalChannel allianceButton = null;
    private double targetRPM = 0;
    private boolean flywheelOn = false;

    // Example: GoBilda 5202/5203/5204 encoder = 28 ticks/rev
    private static final double TICKS_PER_REV = 28.0;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init()    {

        rgbIndicatorMain = new rgbIndicator(myOpMode.hardwareMap, "rgbLight");
        rgbIndicatorMain.setColor(LEDColors.YELLOW);

        allianceButton = myOpMode.hardwareMap.get(DigitalChannel.class, "allianceButton");
        if (allianceButton.getState()){
            allianceColorRed = true;
            rgbIndicatorMain.setColor(LEDColors.RED);
        } else {
            allianceColorBlue = true;
            rgbIndicatorMain.setColor(LEDColors.BLUE);
        }

        ///GoBilda Odometry Pod Setup
        //Deploy to Control Hub to make Odometry Pod show in hardware selection list
        pinpoint = myOpMode.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-100, -65, DistanceUnit.MM);
        //odo.setOffsets(-100.0, -65.0);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();

        ///Drive Motor Setup
        // Define and Initialize Drive Motors
        leftFront  = myOpMode.hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = myOpMode.hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = myOpMode.hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = myOpMode.hardwareMap.get(DcMotorEx.class, "rightBack");

        // Drive motor brake mode
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Run Using Encoder
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set Directions
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        //INTAKE
        intake = myOpMode.hardwareMap.get(DcMotorEx.class,"intake");
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setTargetPositionTolerance(5);
        //intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //LAUNCHER
        launcher = myOpMode.hardwareMap.get(DcMotorEx.class,"launcher");
        launcher.setDirection(DcMotor.Direction.REVERSE);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Define and initialize ALL installed servos.

        spindexer = myOpMode.hardwareMap.get(Servo.class, "spindexer");

        turret = myOpMode.hardwareMap.get(Servo.class, "turret");
        turret.setPosition(Constants.turretHome);

        hood = myOpMode.hardwareMap.get(Servo.class, "hood");
        hood.setPosition(Constants.hoodMinimum);

        kicker = myOpMode.hardwareMap.get(Servo.class, "kicker");
        kicker.setPosition(Constants.kickerDown);

        //Limelight Setup
        limelight = myOpMode.hardwareMap.get(Limelight3A.class, "limelight");
        //odo.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        //Telemetry Data
        myOpMode.telemetry.addData("Status", "Initialized");
        //myOpMode.telemetry.addData("X offset", odo.getXOffset());
        //myOpMode.telemetry.addData("Y offset", odo.getYOffset());
        myOpMode.telemetry.addData("Device Version Number:", pinpoint.getDeviceVersion());
        myOpMode.telemetry.addData("Device Scalar", pinpoint.getYawScalar());
        myOpMode.telemetry.update();
    }

    /**
     * Calculates the left/right motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param x     x-axis power
     * @param y     y-axis power
     * @param rotation Rotation
     */
    public void mecanumDrive(double x, double y, double rotation) {
        // Combine drive and turn for blended motion.
        double leftFrontPower = y + x + rotation;
        double rightFrontPower = y - x - rotation;
        double leftBackPower = y - x + rotation;
        double rightBackPower = y + x - rotation;

        // Normalize power values to keep them between -1 and 1
        double maxPower = Math.max(1.0, Math.abs(leftFrontPower));
        maxPower = Math.max(maxPower, Math.abs(rightFrontPower));
        maxPower = Math.max(maxPower, Math.abs(leftBackPower));
        maxPower = Math.max(maxPower, Math.abs(rightBackPower));

        leftFrontPower /= maxPower;
        rightFrontPower /= maxPower;
        leftBackPower /= maxPower;
        rightBackPower /= maxPower;

        // Use existing function to drive the wheels.
        setDrivePower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    public void FieldCentricDrive(double x, double y, double rx, double botHeading){
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double leftFrontPower = (rotY + rotX + rx) / denominator;
        double leftBackPower = (rotY - rotX + rx) / denominator;
        double rightFrontPower = (rotY - rotX - rx) / denominator;
        double rightBackPower = (rotY + rotX - rx) / denominator;

        setDrivePower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param leftFrontPower Left Front Power
     * @param rightFrontPower Right Front Power
     * @param leftBackPower Left Back Power
     * @param rightBackPower Right Back Power
     */
    public void setDrivePower(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        // Output the values to the motor drives.
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public boolean isFlywheelOn() {
        return flywheelOn;
    }

    public void toggleFlywheel(double rpm) {
        if (flywheelOn && Math.abs(targetRPM - rpm) < 10) {
            stopFlywheel();
        } else {
            setTargetRPM(rpm);
        }
    }

    public void stopFlywheel() {
        launcher.setVelocity(0);
        targetRPM = 0;
        flywheelOn = false;
    }

    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
        flywheelOn = rpm > 0;

        if (flywheelOn) {
            double ticksPerSecond = rpmToTicksPerSecond(rpm);
            launcher.setVelocity(ticksPerSecond);
        } else {
            launcher.setVelocity(0);
        }
    }

    public void adjustRPM(double delta) {
        if (flywheelOn) {
            setTargetRPM(Math.max(0, targetRPM + delta));
        }
    }

    private double rpmToTicksPerSecond(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }

    public double getCurrentRPM() {
        return (launcher.getVelocity() * 60.0) / TICKS_PER_REV;
    }


    public void rotateTurret(TurretDirection Direction){
        if (Direction == TurretDirection.LEFT){
            turret.setPosition(0.0);
        } else if (Direction == TurretDirection.RIGHT) {
            turret.setPosition(1.0);
        } else if (Direction == TurretDirection.STOP) {
            turret.setPosition(0.5);
        }
    }

    public void runIntake(IntakeDirection Direction) {
            if (Direction == IntakeDirection.OUT) {
                intake.setPower(Constants.intakeReversePower);
            } else if (Direction == IntakeDirection.IN){
                intake.setPower(Constants.intakeForwardPower);
            } else if (Direction == IntakeDirection.STOP) {
                intake.setPower(0.0);
            }
        }
    }

