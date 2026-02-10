package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;
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

import java.util.ArrayDeque;
import java.util.List;

/**
 * Flywheel controller that maps Limelight AprilTag distance to RPM setpoints.
 * The Limelight pipeline is selected in {@link RobotHardware#selectAllianceLimelightPipeline()}
 * so the correct tag is isolated for the current alliance. The launcher motor in
 * {@link RobotHardware#launcherGroup} is used to spin the flywheel.
 */
public class FlywheelController {

    private static final double TICKS_PER_REV = 28.0;

    private static final double MID_ZONE_DISTANCE_FT = 3.5;
    private static final double FAR_ZONE_DISTANCE_FT = 6.0;
    private static final int LIMELIGHT_DISTANCE_WINDOW = 5;
    private static final double LIMELIGHT_DISTANCE_HOLD_SECONDS = 0.25;

    private final RobotHardware robot;
    private final Telemetry telemetry;
    private TelemetryManager panelsTelemetry;
    private boolean flywheelEnabled = false;
    private double targetRpm = 0.0;
    private double rpmTolerance = Constants.FLYWHEEL_TOLERANCE_RPM;

    private final ElapsedTime spinupTimer = new ElapsedTime();
    private final ElapsedTime flywheelAdjustmentTimer = new ElapsedTime();
    private final ElapsedTime limelightHoldTimer = new ElapsedTime();
    private final ArrayDeque<Double> limelightDistancesFeet = new ArrayDeque<>();
    private double lastSmoothedDistanceFeet = Double.NaN;
    private boolean measuringSpinup = false;
    private double spinupSetpointRpm = 0.0;

    public FlywheelController(RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.panelsTelemetry = robot.getPanelsTelemetry();

        resetDriverTuningFromConstants();
        limelightHoldTimer.reset();
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

    public void setTargetRPM(double delta) {
        targetRpm = delta;
    }

    public boolean isEnabled() {
        return flywheelEnabled;
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getCurrentRpm() {
        LauncherMotorGroup launcherGroup = robot.launcherGroup;
        if (launcherGroup == null) {
            telemetry.addLine("ERROR: launcher motor is NULL!");
            return 0.0;
        }
        return (launcherGroup.group.getVelocity() * 60.0) / TICKS_PER_REV;
    }

    public boolean isAtSpeed() {
        // EXAMPLE:
        // RPM : 2,200
        // RPM TOLERANCE : 100
        // RANGE : 2,300 - 2,150
        return (getCurrentRpm() >= (targetRpm - (rpmTolerance/2))) && ( getCurrentRpm() <= (targetRpm + rpmTolerance) ); //return Math.abs(getCurrentRpm() - targetRpm) <= rpmTolerance;
    }

    public double getDistanceFromAprilTag() {
        LLResult result = robot.getLatestLimelightResult();
        double distanceFeet = extractDistanceFeet(result);
        return Double.isFinite(distanceFeet) ? distanceFeet : 0.0;
    }

    public double getRpmTolerance() {
        return rpmTolerance;
    }

    public void adjustRpmTolerance(double delta) {
        rpmTolerance = Math.max(0.0, rpmTolerance + delta);
    }

    public void adjustLauncherFeedforward(double delta) {
        FlywheelPidfConfig.launcherF = Math.max(0.0, FlywheelPidfConfig.launcherF + delta);
    }

    public void setLauncherFeedforward(double delta) {
        FlywheelPidfConfig.launcherF = delta;
    }

    /**
     * Restore driver-tunable values to their default Constants-based settings.
     */
    public void resetDriverTuningFromConstants() {
        rpmTolerance = Constants.FLYWHEEL_TOLERANCE_RPM;
        FlywheelPidfConfig.launcherP = Constants.LAUNCHER_P;
        FlywheelPidfConfig.launcherI = Constants.LAUNCHER_I;
        FlywheelPidfConfig.launcherD = Constants.LAUNCHER_D;
        FlywheelPidfConfig.launcherF = Constants.LAUNCHER_F;
    }

    /**
     * Call every loop to update the RPM based on the detected AprilTag.
     */
    public void update(boolean jsDoTarget) {
        robot.launcherGroup.refreshLauncherPIDFFromConfig();

        if (!flywheelEnabled) {
            setFrontLedColor(LEDColors.OFF);
            publishPanelsFlywheelTelemetry(targetRpm, getCurrentRpm());
            return;
        }

        if (robot.launcherGroup == null) {
            telemetry.addLine("ERROR: launcher motor is NULL!");
            setFrontLedColor(LEDColors.OFF);
            return;
        }

        if (jsDoTarget) {
            setFlywheelRpm(targetRpm);
            updateFrontLedColor();
            publishPanelsFlywheelTelemetry(targetRpm, getCurrentRpm());
            return;
        }

        double rpm = Constants.DEFAULT_RPM;

        LLResult result = robot.getLatestLimelightResult();
        double distanceFeet = getSmoothedDistanceFeet(result);
        if (Double.isFinite(distanceFeet)) {
            if (distanceFeet >= Constants.FAR_DEADZONE_FT) {
                double clampedDistance = Range.clip(distanceFeet, Constants.FAR_DEADZONE_FT, 14.5);
                double distanceRatio = (clampedDistance - Constants.FAR_DEADZONE_FT) / (14.5 - Constants.FAR_DEADZONE_FT);
                rpm = Constants.LAUNCH_ZONE_FAR_FAR_RPM
                        + distanceRatio * (2900 - Constants.LAUNCH_ZONE_FAR_FAR_RPM);

            } else if (distanceFeet < MID_ZONE_DISTANCE_FT) {
                rpm = Constants.LAUNCH_ZONE_MID_RPM;
            } else {
                double clampedDistance = Range.clip(distanceFeet, MID_ZONE_DISTANCE_FT, FAR_ZONE_DISTANCE_FT);
                double distanceRatio = (clampedDistance - MID_ZONE_DISTANCE_FT) / (FAR_ZONE_DISTANCE_FT - MID_ZONE_DISTANCE_FT);
                rpm = Constants.LAUNCH_ZONE_MID_RPM
                        + distanceRatio * (Constants.LAUNCH_ZONE_FAR_RPM - Constants.LAUNCH_ZONE_MID_RPM);
            }

            telemetry.addData("Flywheel Distance (ft)", "%.2f", distanceFeet);
        }

        if (targetRpm == 0 || result == null || !result.isValid()) {
            setLauncherFeedforward(31.0);
            flywheelAdjustmentTimer.reset();
        } else if (flywheelAdjustmentTimer.seconds() >= Constants.FLYWHEEL_ADJUSTMENT_TIME) {
            if (getCurrentRpm() >= targetRpm+rpmTolerance) {
                adjustLauncherFeedforward(-Constants.FLYWHEEL_ADJUSTMENT_INCREMENT);
            } else if (getCurrentRpm() <= targetRpm-rpmTolerance) {
                adjustLauncherFeedforward(Constants.FLYWHEEL_ADJUSTMENT_INCREMENT);
            }
            flywheelAdjustmentTimer.reset();
        }

        rpm = Math.max(rpm, Constants.DEFAULT_RPM);
        setFlywheelRpm(rpm);

        updateFrontLedColor();

        publishPanelsFlywheelTelemetry(targetRpm, getCurrentRpm());

        if (measuringSpinup && isAtSpeed()) {
            double elapsedSeconds = spinupTimer.seconds();
            RobotLog.ii("FlywheelController", "Spin-up to %.0f RPM reached in %.2f s", spinupSetpointRpm, elapsedSeconds);
            measuringSpinup = false;
        }
    }

    public void update() { update(false); }
    private double extractDistanceFeet(LLResult result) {
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
                    double distanceMeters = Math.sqrt(xMeters * xMeters + yMeters * yMeters + zMeters * zMeters);
                    return distanceMeters * 3.28084;
                }
            }
        }
        return Double.NaN;
    }

    private double getSmoothedDistanceFeet(LLResult result) {
        double distanceFeet = extractDistanceFeet(result);
        if (Double.isFinite(distanceFeet)) {
            limelightDistancesFeet.addLast(distanceFeet);
            while (limelightDistancesFeet.size() > LIMELIGHT_DISTANCE_WINDOW) {
                limelightDistancesFeet.removeFirst();
            }
            double sum = 0.0;
            for (double sample : limelightDistancesFeet) {
                sum += sample;
            }
            lastSmoothedDistanceFeet = sum / limelightDistancesFeet.size();
            limelightHoldTimer.reset();
            return lastSmoothedDistanceFeet;
        }

        if (Double.isFinite(lastSmoothedDistanceFeet)
                && limelightHoldTimer.seconds() < LIMELIGHT_DISTANCE_HOLD_SECONDS) {
            return lastSmoothedDistanceFeet;
        }

        limelightDistancesFeet.clear();
        lastSmoothedDistanceFeet = Double.NaN;
        return Double.NaN;
    }

    private void stop() {
        targetRpm = 0.0;
        measuringSpinup = false;
        LauncherMotorGroup launcherGroup = robot.launcherGroup;
        if (launcherGroup != null) {
            launcherGroup.group.setVelocity(0);
            launcherGroup.group.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        publishPanelsFlywheelTelemetry(targetRpm, getCurrentRpm());
    }

    public void setFlywheelRpm(double rpm) {
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

        double ticksPerSecond = rpmToTicksPerSecond(rpm);
        launcherGroup.group.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherGroup.group.setVelocity(ticksPerSecond);
    }

    private double rpmToTicksPerSecond(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }

    @SuppressLint("DefaultLocale")
    public void publishPanelsFlywheelTelemetry(double target, double current) {
        if (panelsTelemetry == null) {
            panelsTelemetry = robot.getPanelsTelemetry();
        }

        if (panelsTelemetry == null) {
            return;
        }

        panelsTelemetry.addLine("--- FLYWHEEL RPM ---");
        panelsTelemetry.debug("Flywheel RPM (target/current)", String.format("%.0f / %.0f", target, current));
        panelsTelemetry.addLine("-------------------------------");
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

        double currentRpm = Math.abs(getCurrentRpm());
        double minimumRpm = targetRpm - rpmTolerance;
        double maxRpm = targetRpm + rpmTolerance;

        if (isAtSpeed() ) {
            setFrontLedColor(LEDColors.GREEN);
        } else if (currentRpm > maxRpm ) {
            setFrontLedColor(LEDColors.AZURE);
        } else if (currentRpm >= minimumRpm * 0.75) {
            setFrontLedColor(LEDColors.ORANGE);
        } else if (currentRpm >= minimumRpm * 0.5) {
            setFrontLedColor(LEDColors.YELLOW);
        } else {
            setFrontLedColor(LEDColors.RED);
        }
    }
}
