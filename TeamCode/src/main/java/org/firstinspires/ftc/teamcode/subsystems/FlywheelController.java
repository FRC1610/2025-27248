package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drivers.rgbIndicator.LEDColors;

import java.util.List;

/**
 * Flywheel controller that maps Limelight AprilTag distance to RPM setpoints.
 * The Limelight pipeline is selected in {@link RobotHardware#selectAllianceLimelightPipeline()}
 * so the correct tag is isolated for the current alliance. The launcher motor in
 * {@link RobotHardware#launcher} is used to spin the flywheel.
 */
public class FlywheelController {

    private static final double TICKS_PER_REV = 28.0;
    private static final double MOTOR_TO_FLYWHEEL_RATIO = 24.0 / 16.0;

    private static final double MID_ZONE_DISTANCE_FT = 3.5;
    private static final double FAR_ZONE_DISTANCE_FT = 5.5;
    private static final double FAR_FAR_ZONE_DISTANCE_FT = 8.0;

    private final RobotHardware robot;
    private final Telemetry telemetry;
    private TelemetryManager panelsTelemetry;
    private boolean flywheelEnabled = false;
    private double targetRpm = 0.0;

    private final ElapsedTime spinupTimer = new ElapsedTime();
    private boolean measuringSpinup = false;
    private double spinupSetpointRpm = 0.0;

    private double lastPidfP = Double.NaN;
    private double lastPidfI = Double.NaN;
    private double lastPidfD = Double.NaN;
    private double lastPidfF = Double.NaN;

    public FlywheelController(RobotHardware robot,
                              Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.panelsTelemetry = robot.getPanelsTelemetry();
    }

     /**
     * Toggle the flywheel on/off using the launcher motor.
     */
    public void toggle() {
        flywheelEnabled = !flywheelEnabled;

        if (flywheelEnabled) {
            setFlywheelRpm(Constants.DEFAULT_RPM);
        } else {
            stop();
        }
    }

    public boolean isEnabled() {
        return flywheelEnabled;
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getCurrentRpm() {
        DcMotorEx launcherMotor = robot.launcher;
        if (launcherMotor == null) {
            telemetry.addLine("ERROR: launcher motor is NULL!");
            return 0.0;
        }

        double motorRpm = (launcherMotor.getVelocity() * 60.0) / TICKS_PER_REV;
        return motorRpm * MOTOR_TO_FLYWHEEL_RATIO;
    }

    public boolean isAtSpeed(double tolerance) {
        return Math.abs(getCurrentRpm() - targetRpm) <= tolerance;
    }

    /**
     * Call every loop to update the RPM based on the detected AprilTag.
     */
    public void update() {
        applyPanelsPidfIfChanged();

        if (!flywheelEnabled) {
            setFrontLedColor(LEDColors.OFF);
            publishPanelsFlywheelTelemetry(targetRpm, getCurrentRpm());
            return;
        }

        if (robot.launcher == null) {
            telemetry.addLine("ERROR: launcher motor is NULL!");
            setFrontLedColor(LEDColors.OFF);
            return;
        }

        double rpm = Constants.DEFAULT_RPM;

        LLResult result = robot.getLatestLimelightResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                LLResultTypes.FiducialResult fid = fiducials.get(0);
                Pose3D pose = fid.getRobotPoseTargetSpace();
                Position position = pose != null ? pose.getPosition() : null;

                if (position != null) {
                    Position metersPosition = position.toUnit(DistanceUnit.METER);
                    double xMeters = metersPosition.x;
                    double yMeters = metersPosition.y;
                    double zMeters = metersPosition.z;
                    // Use full 3D translation magnitude to avoid underestimating range.
                    double distanceMeters = Math.sqrt(xMeters * xMeters + yMeters * yMeters + zMeters * zMeters);
                    double distanceFeet = distanceMeters * 3.28084;

                    if (distanceFeet >= FAR_FAR_ZONE_DISTANCE_FT) {
                        rpm = Constants.LAUNCH_ZONE_FAR_FAR_RPM;
                    } else {
                        double clampedDistance = Range.clip(distanceFeet, MID_ZONE_DISTANCE_FT, FAR_ZONE_DISTANCE_FT);
                        double distanceRatio = (clampedDistance - MID_ZONE_DISTANCE_FT) / (FAR_ZONE_DISTANCE_FT - MID_ZONE_DISTANCE_FT);
                        rpm = Constants.LAUNCH_ZONE_MID_RPM
                                + distanceRatio * (Constants.LAUNCH_ZONE_FAR_RPM - Constants.LAUNCH_ZONE_MID_RPM);
                    }

                    telemetry.addData("Flywheel Distance (ft)", "%.2f", distanceFeet);
                }
            }
        }

        rpm = Math.max(rpm, Constants.DEFAULT_RPM);
        setFlywheelRpm(rpm);

        updateFrontLedColor();

        publishPanelsFlywheelTelemetry(targetRpm, getCurrentRpm());

        if (measuringSpinup && isAtSpeed(Constants.FLYWHEEL_TOLERANCE_RPM)) {
            double elapsedSeconds = spinupTimer.seconds();
            RobotLog.ii("FlywheelController", "Spin-up to %.0f RPM reached in %.2f s", spinupSetpointRpm, elapsedSeconds);
            measuringSpinup = false;
        }
    }

    private void stop() {
        targetRpm = 0.0;
        measuringSpinup = false;
        DcMotorEx launcherMotor = robot.launcher;
        if (launcherMotor != null) {
            launcherMotor.setVelocity(0);
            launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setFrontLedColor(LEDColors.OFF);
        DcMotorEx launcherFollower = robot.launcher2;
        if (launcherFollower != null) {
            launcherFollower.setVelocity(0);
            launcherFollower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        publishPanelsFlywheelTelemetry(targetRpm, getCurrentRpm());
    }

    private void setFlywheelRpm(double rpm) {
        if (rpm > 0 && targetRpm <= 0) {
            spinupSetpointRpm = rpm;
            spinupTimer.reset();
            measuringSpinup = true;
        }

        targetRpm = rpm;
        DcMotorEx launcherMotor = robot.launcher;
        if (launcherMotor == null) {
            telemetry.addLine("ERROR: launcher motor is NULL!");
            return;
        }

        double ticksPerSecond = rpmToMotorTicksPerSecond(rpm);
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setVelocity(ticksPerSecond);

        DcMotorEx launcherFollower = robot.launcher2;
        if (launcherFollower != null) {
            launcherFollower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcherFollower.setVelocity(ticksPerSecond);
        }
    }

    private double rpmToMotorTicksPerSecond(double flywheelRpm) {
        double motorRpm = flywheelRpm / MOTOR_TO_FLYWHEEL_RATIO;
        return (motorRpm * TICKS_PER_REV) / 60.0;
    }

    private void publishPanelsFlywheelTelemetry(double target, double current) {
        if (panelsTelemetry == null) {
            panelsTelemetry = robot.getPanelsTelemetry();
        }

        if (panelsTelemetry == null) {
            return;
        }

        panelsTelemetry.debug("Flywheel RPM (target/current)", String.format("%.0f / %.0f", target, current));
    }

    /**
     * Pull the latest PIDF values from Panels and apply them directly (no scaling) to both flywheel motors.
     */
    private void applyPanelsPidfIfChanged() {
        PanelsConfigurables.INSTANCE.refreshClass(FlywheelPidfConfig.class);

        double p = FlywheelPidfConfig.launcherP;
        double i = FlywheelPidfConfig.launcherI;
        double d = FlywheelPidfConfig.launcherD;
        double f = FlywheelPidfConfig.launcherF;

        boolean changed = p != lastPidfP || i != lastPidfI || d != lastPidfD || f != lastPidfF;
        if (!changed) {
            return;
        }

        PIDFCoefficients pidf = new PIDFCoefficients(p, i, d, f);

        if (robot.launcher != null) {
            robot.launcher.setVelocityPIDFCoefficients(pidf.p, pidf.i, pidf.d, pidf.f);
        }

        if (robot.launcher2 != null) {
            robot.launcher2.setVelocityPIDFCoefficients(pidf.p, pidf.i, pidf.d, pidf.f);
        }

        if (panelsTelemetry != null) {
            panelsTelemetry.debug(
                    "Launcher PIDF (P,I,D,F)",
                    String.format("P=%.3f I=%.3f D=%.3f F=%.3f", pidf.p, pidf.i, pidf.d, pidf.f));
        }

        lastPidfP = p;
        lastPidfI = i;
        lastPidfD = d;
        lastPidfF = f;
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

        if (!flywheelEnabled) {
            setFrontLedColor(LEDColors.OFF);
            return;
        }

        if (targetRpm == Constants.DEFAULT_RPM) {
            setFrontLedColor(LEDColors.VIOLET);
            return;
        }

        double error = Math.abs(getCurrentRpm() - targetRpm);

        if (error > 250.0) {
            setFrontLedColor(LEDColors.RED);
        } else if (error <= 50.0) {
            setFrontLedColor(LEDColors.GREEN);
        } else if (error <= 100.0) {
            setFrontLedColor(LEDColors.YELLOW);
        } else if (error <= 175.0) {
            setFrontLedColor(LEDColors.ORANGE);
        } else {
            setFrontLedColor(LEDColors.RED);
        }
    }
}
