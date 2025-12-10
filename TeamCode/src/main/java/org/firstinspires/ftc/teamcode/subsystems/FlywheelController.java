package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;

import java.util.List;

/**
 * Flywheel controller that maps Limelight AprilTag distance to RPM setpoints.
 * The Limelight pipeline is selected in {@link RobotHardware#selectAllianceLimelightPipeline()}
 * so the correct tag is isolated for the current alliance. The launcher motor in
 * {@link RobotHardware#launcher} is used to spin the flywheel.
 */
public class FlywheelController {

    private static final double TICKS_PER_REV = 28.0;

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

        return (launcherMotor.getVelocity() * 60.0) / TICKS_PER_REV;
    }

    public boolean isAtSpeed(double tolerance) {
        return Math.abs(getCurrentRpm() - targetRpm) <= tolerance;
    }

    /**
     * Call every loop to update the RPM based on the detected AprilTag.
     */
    public void update() {
        robot.refreshLauncherPidfFromConfig();

        if (!flywheelEnabled) {
            publishPanelsFlywheelTelemetry(targetRpm, getCurrentRpm());
            return;
        }

        if (robot.launcher == null) {
            telemetry.addLine("ERROR: launcher motor is NULL!");
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

        double ticksPerSecond = rpmToTicksPerSecond(rpm);
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setVelocity(ticksPerSecond);
    }

    private double rpmToTicksPerSecond(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
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
}
