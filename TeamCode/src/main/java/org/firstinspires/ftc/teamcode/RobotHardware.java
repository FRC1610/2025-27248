package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.drivers.rgbIndicator;
import org.firstinspires.ftc.teamcode.drivers.rgbIndicator.LEDColors;

public class RobotHardware {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;
    public DcMotorEx intake;
    public DcMotorEx launcher;
    public DcMotorEx turret;
    public Servo spindexer;
    public Servo kicker;
    public Servo headlight;  //PWM Controlled LED
    public boolean allianceColorRed = false;
    public boolean allianceColorBlue = false;
    public ColorSensor color1;
    public DistanceSensor distance1;
    //private AnalogInput turretPos;

    public enum IntakeDirection {
        IN,
        OUT,
        STOP
    }

    public Limelight3A limelight;
    public GoBildaPinpointDriver pinpoint; // Declare OpMode member for the Odometry Computer
    public rgbIndicator rgbIndicatorMain;
    private DigitalChannel allianceButton;
    private double targetRPM = 0;
    private boolean flywheelOn = false;
    public int targetTagID;
    private int turretTargetPosition = 0;
    public double spindexerPos = Constants.spindexerStart;

    // Example: GoBilda 5202/5203/5204 encoder = 28 ticks/rev
    private static final double TICKS_PER_REV = 28.0;
    private LLResult latestLimelightResult;

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

        if (allianceColorRed) {
            targetTagID = 24; // example tag ID for red alliance
        } else if (allianceColorBlue) {
            targetTagID = 20; // example tag ID for blue alliance
        }

        ///GoBilda Odometry Pod Setup
        //Deploy to Control Hub to make Odometry Pod show in hardware selection list
        pinpoint = myOpMode.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-100, -65, DistanceUnit.MM);  //TODO update offsets
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
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //LAUNCHER
        launcher = myOpMode.hardwareMap.get(DcMotorEx.class,"launcher");
        launcher.setDirection(DcMotor.Direction.FORWARD);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //TURRET
        turret = myOpMode.hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setTargetPositionTolerance(5);

        // Define and initialize ALL installed servos.

        spindexer = myOpMode.hardwareMap.get(Servo.class, "spindexer");
        spindexer.setPosition(Constants.spindexerStart);

        kicker = myOpMode.hardwareMap.get(Servo.class, "kicker");
        kicker.setPosition(Constants.kickerDown);

        Servo hood = myOpMode.hardwareMap.get(Servo.class, "hood");
        hood.setPosition(Constants.hoodMinimum);

        Servo kicker = myOpMode.hardwareMap.get(Servo.class, "kicker");
        kicker.setPosition(Constants.kickerDown);

        //Turret LED
        headlight = myOpMode.hardwareMap.get(Servo.class, "headlight");
        headlight.setPosition(0.0);

        //Limelight Setup
        limelight = myOpMode.hardwareMap.get(Limelight3A.class, "limelight");
        //odo.setMsTransmissionInterval(11);
        selectAllianceLimelightPipeline();
        limelight.start();

        // Prime cached Limelight data
        refreshLimelightResult();

        //Color Sensor Setup
        color1 = myOpMode.hardwareMap.get(RevColorSensorV3.class, "color1");
        distance1 = myOpMode.hardwareMap.get(DistanceSensor.class, "color1");

        //Telemetry Data
        myOpMode.telemetry.addData("Status", "Initialized");
        myOpMode.telemetry.addData("X offset", pinpoint.getXOffset(DistanceUnit.MM));
        myOpMode.telemetry.addData("Y offset", pinpoint.getYOffset(DistanceUnit.MM));
        myOpMode.telemetry.addData("Device Version Number:", pinpoint.getDeviceVersion());
        myOpMode.telemetry.addData("Device Scalar", pinpoint.getYawScalar());
        myOpMode.telemetry.update();
    }

    /**
     * Fetch and cache the most recent Limelight result so subsystems can reuse
     * the same frame data instead of polling the camera independently.
     */
    public void refreshLimelightResult() {
        if (limelight == null) {
            latestLimelightResult = null;
            myOpMode.telemetry.addLine("ERROR: Limelight not initialized");
            return;
        }

        latestLimelightResult = limelight.getLatestResult();
    }

    public LLResult getLatestLimelightResult() {
        return latestLimelightResult;
    }

    public void selectAllianceLimelightPipeline() {
        if (limelight == null) {
            myOpMode.telemetry.addLine("ERROR: Limelight not initialized");
            return;
        }

        if (allianceColorBlue) {
            limelight.pipelineSwitch(0);   // Blue → Tag 20
        } else if (allianceColorRed) {
            limelight.pipelineSwitch(4);   // Red → Tag 24
        }
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

    public void adjustSpindexer(double delta) {
        spindexerPos += delta;

        // Clamp between valid servo range
        spindexerPos = Math.max(0.0, Math.min(1.0, spindexerPos));

        spindexer.setPosition(spindexerPos);

        myOpMode.telemetry.addData("Spindexer Pos", spindexerPos);
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


    public int getTurretTarget() {
        return turretTargetPosition;
    }

    public int getTurretPosition() {
        return turret.getCurrentPosition();
    }

    public void adjustTurret(int deltaTicks) {
        turretTargetPosition += deltaTicks;

        // Clamp safe range
        turretTargetPosition = Math.max(Constants.turret_MIN, Math.min(Constants.turret_MAX, turretTargetPosition));

        turret.setTargetPosition(turretTargetPosition);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0.40);
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

