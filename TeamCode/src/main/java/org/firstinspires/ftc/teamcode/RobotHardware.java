package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drivers.rgbIndicator;
import org.firstinspires.ftc.teamcode.drivers.rgbIndicator.LEDColors;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroConstants;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelPidfConfig;
import org.firstinspires.ftc.teamcode.subsystems.LauncherMotorGroup;
import org.firstinspires.ftc.teamcode.subsystems.TurretAimConfig;

public class RobotHardware {

    /* Declare OpMode members. */
    private final LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;
    public DcMotorEx intake;
    public LauncherMotorGroup launcherGroup;
    public DcMotorEx turret;
    public Servo spindexer;

    //public Servo kickStand1;
    //public Servo kickStand2;
    public Servo kicker;
    public Servo headlight;  //PWM Controlled LED
    public boolean allianceColorRed = false;
    public boolean allianceColorBlue = false;
    public ColorSensor color1;
    public DistanceSensor distance1;
    public ColorSensor color2;
    public DistanceSensor distance2;
    public ColorSensor color3;
    public DistanceSensor distance3;

    private double powerDampener = 1;

    private TelemetryManager panelsTelemetry;

    public enum IntakeDirection {
        IN,
        OUT,
        STOP
    }

    public Limelight3A limelight;
    public GoBildaPinpointDriver pinpoint; // Declare OpMode member for the Odometry Computer
    public rgbIndicator rgbIndicatorMain;
    public rgbIndicator frontLED;
    public rgbIndicator rearRGB1;
    public rgbIndicator rearRGB2;
    public rgbIndicator rearRGB3;
    private DigitalChannel allianceButton;
    private double headingOffsetRadians = 0.0;

    // Example: GoBilda 5202/5203/5204 encoder = 28 ticks/rev
    private static final double TICKS_PER_REV = 28.0;
    private LLResult latestLimelightResult;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware(LinearOpMode opMode) {
        myOpMode = opMode;
    }

    public TelemetryManager getPanelsTelemetry() {
        return panelsTelemetry;
    }

    /**
     * Flush any queued Panels telemetry entries. Call this once per loop after all subsystems
     * have contributed their debug values so the dashboard renders a single coherent packet.
     */
    public void flushPanelsTelemetry(Telemetry telemetry) {
        if (panelsTelemetry == null) {
            panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        }
        panelsTelemetry.update(telemetry);
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init()    {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        rgbIndicatorMain = new rgbIndicator(myOpMode.hardwareMap, "rgbLight");
        rgbIndicatorMain.setColor(LEDColors.YELLOW);

        frontLED = new rgbIndicator(myOpMode.hardwareMap, "frontLED");
        frontLED.setColor(LEDColors.YELLOW);

        rearRGB1 = new rgbIndicator(myOpMode.hardwareMap, "rearRGB1");
        rearRGB2 = new rgbIndicator(myOpMode.hardwareMap, "rearRGB2");
        rearRGB3 = new rgbIndicator(myOpMode.hardwareMap, "rearRGB3");
        setColorOfBackLights(LEDColors.YELLOW);

        allianceButton = myOpMode.hardwareMap.get(DigitalChannel.class, "allianceButton");
        allianceButton.setMode(DigitalChannel.Mode.INPUT);
        configureAllianceFromSwitch();



        ///GoBilda Odometry Pod Setup
        //Deploy to Control Hub to make Odometry Pod show in hardware selection list
        pinpoint = myOpMode.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        // Apply the Pedro Pathing tuner odometry offsets (recorded in inches) to the Pinpoint localizer.
        // The Pinpoint firmware stores offsets in millimeters, but its driver converts when given a DistanceUnit
        // so we can supply inch measurements directly. Forward pod is the Y offset, strafe pod is the X offset.
        double forwardPodOffsetInches = PedroConstants.FORWARD_POD_Y;
        double strafePodOffsetInches = PedroConstants.STRAFE_POD_X;
        pinpoint.setOffsets(forwardPodOffsetInches, strafePodOffsetInches, DistanceUnit.INCH);
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
        DcMotorEx launcher = myOpMode.hardwareMap.get(DcMotorEx.class,"launcher");
        DcMotorEx launcher2 = myOpMode.hardwareMap.get(DcMotorEx.class,"launcher2");
        launcherGroup = new LauncherMotorGroup(myOpMode.telemetry, panelsTelemetry, launcher, launcher2);
        launcherGroup.applyLauncherPIDFTuning();

        //TURRET
        turret = myOpMode.hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setTargetPositionTolerance(5);

        // Define and initialize ALL installed servos.

        spindexer = myOpMode.hardwareMap.get(Servo.class, "spindexer");
        spindexer.setPosition(Constants.SPINDEXER_1);

        kicker = myOpMode.hardwareMap.get(Servo.class, "kicker");
        kicker.setPosition(Constants.KICKER_DOWN);

        /*
        kickStand1 = myOpMode.hardwareMap.get(Servo.class, "kickStand1");
        kickStand1.setPosition(Constants.KICKERSTAND_NORMAL);
        kickStand2 = myOpMode.hardwareMap.get(Servo.class, "kickStand2");
        kickStand2.setPosition(Constants.KICKERSTAND_NORMAL);
         */

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
        color2 = myOpMode.hardwareMap.get(RevColorSensorV3.class, "color2");
        distance2 = myOpMode.hardwareMap.get(DistanceSensor.class, "color2");
        color3 = myOpMode.hardwareMap.get(RevColorSensorV3.class, "color3");
        distance3 = myOpMode.hardwareMap.get(DistanceSensor.class, "color3");

        PanelsConfigurables.INSTANCE.refreshClass(FlywheelPidfConfig.class);
        PanelsConfigurables.INSTANCE.refreshClass(TurretAimConfig.class);
        launcherGroup.refreshLauncherPIDFFromConfig();
        flushPanelsTelemetry(myOpMode.telemetry);

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

    /**
     * Read the alliance selector switch and set alliance colors with a blue default.
     */
    public void configureAllianceFromSwitch() {
        refreshAllianceFromSwitchState();
        rgbIndicatorMain.setColor(allianceColorRed ? LEDColors.RED : LEDColors.BLUE);
        frontLED.setColor(allianceColorRed ? LEDColors.RED : LEDColors.BLUE);
        setColorOfBackLights(allianceColorRed ? LEDColors.RED : LEDColors.BLUE);

    }

    /**
     * Refresh the cached alliance color using the same logic as {@link #configureAllianceFromSwitch()}.
     * This helper avoids updating indicators so utilities can fetch the alliance without extra side effects.
     *
     * @return true if the alliance switch indicates red; false otherwise.
     */
    public boolean refreshAllianceFromSwitchState() {
        // Default to blue. If the alliance switch is wired active-low (ground = red),
        // interpret a low signal as red and a high or absent signal as blue.
        allianceColorBlue = true;
        allianceColorRed = false;

        if (allianceButton != null && allianceButton.getMode() == DigitalChannel.Mode.INPUT) {
            boolean rawSwitchState = allianceButton.getState();
            allianceColorRed = !rawSwitchState;
            allianceColorBlue = rawSwitchState;
        }

        return allianceColorRed;
    }

    public void selectAllianceLimelightPipeline() {
        if (limelight == null) {
            myOpMode.telemetry.addLine("ERROR: Limelight not initialized");
            return;
        }

        // Default to blue if the switch is absent or read as low
        int pipeline = allianceColorRed ? 4 : 0;
        limelight.pipelineSwitch(pipeline);
        boolean rawSwitchState = allianceButton != null && allianceButton.getState();
        myOpMode.telemetry.addData("Alliance switch state (raw)", rawSwitchState);
        myOpMode.telemetry.addData("Alliance inferred", allianceColorRed ? "RED" : "BLUE");
        myOpMode.telemetry.addData("Selected pipeline", pipeline);
    }

    public void updateHeadingOffsetFromAllianceButton() {
        refreshAllianceFromSwitchState();
        // The robot always starts parked sideways relative to the drivers.
        // Blue alliance: bot front points to the right (+X), so add +90 degrees to align field-forward.
        // Red alliance: bot front points to the left (-X), so add -90 degrees to align field-forward.
        headingOffsetRadians = allianceColorRed ? -Math.PI / 2.0 : Math.PI / 2.0;
    }

    public double getHeadingOffsetRadians() {
        return headingOffsetRadians;
    }

    public double applyHeadingOffset(double headingRadians) {
        return headingRadians + headingOffsetRadians;
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
        leftFront.setPower(leftFrontPower * powerDampener);
        rightFront.setPower(rightFrontPower * powerDampener);
        leftBack.setPower(leftBackPower * powerDampener);
        rightBack.setPower(rightBackPower * powerDampener);
    }

    public void setPowerDampener(double delta) {
        if (powerDampener == delta) {
            powerDampener = 1;
        } else {
            powerDampener = delta;
        }
    }

    public double getPowerDampener() {
        return powerDampener;
    }

    private double rpmToTicksPerSecond(double rpm) {
        // Convert desired flywheel RPM to the motor-side encoder rate using the gear reduction.
        double motorRpm = rpm * Constants.LAUNCHER_GEAR_REDUCTION;
        return (motorRpm * TICKS_PER_REV) / 60.0;
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

    /*
    public void setKickStandPosition(double position) {
        kickStand1.setPosition(position);
        kickStand2.setPosition(position);
    }
     */

    public void setColorOfBackLights(double color) {
        rearRGB1.setColor(color);
        rearRGB2.setColor(color);
        rearRGB3.setColor(color);
    }
}