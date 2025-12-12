package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
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
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drivers.rgbIndicator;
import org.firstinspires.ftc.teamcode.drivers.rgbIndicator.LEDColors;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelPidfConfig;
import org.firstinspires.ftc.teamcode.subsystems.LauncherMotorGroup;
import org.firstinspires.ftc.teamcode.subsystems.TurretAimConfig;

public class RobotHardware {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;
    public DcMotorEx intake;
    public LauncherMotorGroup launcherGroup;

    public DcMotorEx launcher;
    public DcMotorEx launcher2;
    public DcMotorEx turret;
    public Servo spindexer;
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
    //private AnalogInput turretPos;

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
    private double targetRPM = 0;
    private boolean flywheelOn = false;
    private int turretTargetPosition = 0;
    public double spindexerPos = Constants.spindexerStart;

    // Example: GoBilda 5202/5203/5204 encoder = 28 ticks/rev
    private static final double TICKS_PER_REV = 28.0;
    private LLResult latestLimelightResult;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
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
        rearRGB1.setColor(LEDColors.YELLOW);
        rearRGB2.setColor(LEDColors.YELLOW);
        rearRGB3.setColor(LEDColors.YELLOW);

        allianceButton = myOpMode.hardwareMap.get(DigitalChannel.class, "allianceButton");
        allianceButton.setMode(DigitalChannel.Mode.INPUT);
        configureAllianceFromSwitch();



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
        DcMotorEx launcher = myOpMode.hardwareMap.get(DcMotorEx.class,"launcher");
        DcMotorEx launcher2 = myOpMode.hardwareMap.get(DcMotorEx.class,"launcher2");
        launcherGroup = new LauncherMotorGroup(myOpMode.telemetry, panelsTelemetry, launcher); //, launcher2
        launcherGroup.applyLauncherPIDFTuning();
        launcher = myOpMode.hardwareMap.get(DcMotorEx.class,"launcher");
        launcher.setDirection(DcMotor.Direction.FORWARD);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setTargetPositionTolerance(25);

        launcher2 = myOpMode.hardwareMap.get(DcMotorEx.class,"launcher2");
        launcher2.setDirection(DcMotor.Direction.REVERSE);
        launcher2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher2.setTargetPositionTolerance(25);

        applyLauncherPidfTuning();

        //TURRET
        turret = myOpMode.hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setTargetPositionTolerance(5);

        // Define and initialize ALL installed servos.

        spindexer = myOpMode.hardwareMap.get(Servo.class, "spindexer");
        spindexer.setPosition(Constants.spindexerStart);

        kicker = myOpMode.hardwareMap.get(Servo.class, "kicker");
        kicker.setPosition(Constants.kickerDown);

        Servo hood = myOpMode.hardwareMap.get(Servo.class, "hood");
        hood.setPosition(Constants.hoodMinimum);

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
        myOpMode.telemetry.addData("RESULT", latestLimelightResult != null && latestLimelightResult.isValid());
        return latestLimelightResult;
    }

    /**
     * Read the alliance selector switch and set alliance colors with a blue default.
     */
    public void configureAllianceFromSwitch() {
        refreshAllianceFromSwitchState();
        rgbIndicatorMain.setColor(allianceColorRed ? LEDColors.RED : LEDColors.BLUE);
        frontLED.setColor(allianceColorRed ? LEDColors.RED : LEDColors.BLUE);
        rearRGB1.setColor(allianceColorRed ? LEDColors.RED : LEDColors.BLUE);
        rearRGB2.setColor(allianceColorRed ? LEDColors.RED : LEDColors.BLUE);
        rearRGB3.setColor(allianceColorRed ? LEDColors.RED : LEDColors.BLUE);

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

    /**
     * Apply starting PIDF gains for the 6000 RPM Yellow Jacket launcher.
     * These values are derived from the motor's free speed and provide a
     * responsive baseline to counter RPM droop when a note is launched.
     */
    private void applyLauncherPidfTuning() {
        if (launcher == null) {
            myOpMode.telemetry.addLine("ERROR: launcher motor is NULL!");
            return;
        }

        if (launcher2 == null) {
            myOpMode.telemetry.addLine("ERROR: launcher2 motor is NULL!");
        }

        // Scale the motor-side PIDF gains by the gear reduction so the feedforward
        // and proportional response still match the flywheel-side setpoints that are
        // converted into motor ticks/second.
        double gearScaledP = FlywheelPidfConfig.launcherP * Constants.LAUNCHER_GEAR_REDUCTION;
        double gearScaledI = FlywheelPidfConfig.launcherI * Constants.LAUNCHER_GEAR_REDUCTION;
        double gearScaledD = FlywheelPidfConfig.launcherD * Constants.LAUNCHER_GEAR_REDUCTION;
        double gearScaledF = FlywheelPidfConfig.launcherF * Constants.LAUNCHER_GEAR_REDUCTION;

        PIDFCoefficients pidf = new PIDFCoefficients(
                gearScaledP,
                gearScaledI,
                gearScaledD,
                gearScaledF);

        launcher.setVelocityPIDFCoefficients(
                pidf.p,
                pidf.i,
                pidf.d,
                pidf.f);

        if (launcher2 != null) {
            launcher2.setVelocityPIDFCoefficients(
                    pidf.p,
                    pidf.i,
                    pidf.d,
                    pidf.f);
        }

        lastLauncherBaseP = FlywheelPidfConfig.launcherP;
        lastLauncherBaseI = FlywheelPidfConfig.launcherI;
        lastLauncherBaseD = FlywheelPidfConfig.launcherD;
        lastLauncherBaseF = FlywheelPidfConfig.launcherF;
        lastLauncherScaledP = pidf.p;
        lastLauncherScaledI = pidf.i;
        lastLauncherScaledD = pidf.d;
        lastLauncherScaledF = pidf.f;

        publishLauncherPidfTelemetry();
    }

    /**
     * Reapply launcher PIDF values if the Panels-configured base values changed.
     * Always publishes the latest scaled/base values to Panels so they appear live
     * even when the setpoint is not being updated.
     */
    public void refreshLauncherPidfFromConfig() {
        boolean baseChanged = FlywheelPidfConfig.launcherP != lastLauncherBaseP
                || FlywheelPidfConfig.launcherI != lastLauncherBaseI
                || FlywheelPidfConfig.launcherD != lastLauncherBaseD
                || FlywheelPidfConfig.launcherF != lastLauncherBaseF;

        if (baseChanged || !Double.isFinite(lastLauncherScaledP) || !Double.isFinite(lastLauncherScaledF)) {
            applyLauncherPidfTuning();
            return;
        }

        publishLauncherPidfTelemetry();
    }

    private void publishLauncherPidfTelemetry() {
        if (panelsTelemetry == null || !Double.isFinite(lastLauncherScaledP) || !Double.isFinite(lastLauncherScaledF)) {
            return;
        }

        panelsTelemetry.debug("Launcher PIDF scaled (P,I,D,F)",
                String.format("P=%.3f I=%.3f D=%.3f F=%.3f",
                        lastLauncherScaledP, lastLauncherScaledI, lastLauncherScaledD, lastLauncherScaledF));
        panelsTelemetry.debug("Launcher PIDF base (P,I,D,F)",
                String.format("P=%.3f I=%.3f D=%.3f F=%.3f",
                        lastLauncherBaseP, lastLauncherBaseI, lastLauncherBaseD, lastLauncherBaseF));
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
        launcherGroup.group.setVelocity(0);
        launcher.setVelocity(0);
        if (launcher2 != null) {
            launcher2.setVelocity(0);
        }
        targetRPM = 0;
        flywheelOn = false;
    }

    public void setTargetRPM(double rpm) {
        launcherGroup.applyLauncherPIDFTuning();

        targetRPM = rpm;
        flywheelOn = rpm > 0;

        if (flywheelOn) {
            double ticksPerSecond = rpmToTicksPerSecond(rpm);
            launcherGroup.group.setVelocity(ticksPerSecond);
        } else {
            launcherGroup.group.setVelocity(0);
            launcher.setVelocity(ticksPerSecond);
            if (launcher2 != null) {
                launcher2.setVelocity(ticksPerSecond);
            }
        } else {
            launcher.setVelocity(0);
            if (launcher2 != null) {
                launcher2.setVelocity(0);
            }
        }
    }

    public void adjustRPM(double delta) {
        if (flywheelOn) {
            setTargetRPM(Math.max(0, targetRPM + delta));
        }
    }

    private double rpmToTicksPerSecond(double rpm) {
        // Convert desired flywheel RPM to the motor-side encoder rate using the gear reduction.
        double motorRpm = rpm * Constants.LAUNCHER_GEAR_REDUCTION;
        return (motorRpm * TICKS_PER_REV) / 60.0;
    }

    public double getCurrentRPM() {
        // Convert motor-side encoder velocity back to flywheel RPM.
        return (launcherGroup.group.getVelocity() * 60.0) / (TICKS_PER_REV * Constants.LAUNCHER_GEAR_REDUCTION);
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