package org.firstinspires.ftc.teamcode.Testing;

import com.bylazar.configurables.PanelsConfigurables;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelPidfConfig;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

/**
 * Standalone flywheel testing OpMode that mirrors the flywheel controller logic
 * while allowing PIDF tuning from Panels without scaling.
 *
 * Button mapping:
 * - A: toggle the flywheel on/off.
 * - X/Y/B: switch flywheel RPM setpoints between 2000/2200/2600.
 */
@TeleOp(name = "Flywheel Test", group = "Test")
public class FlywheelTest extends LinearOpMode {

    private static final double TICKS_PER_REV = 28.0;

    private static final double RPM_LOW = 2000;
    private static final double RPM_MID = 2200;
    private static final double RPM_HIGH = 2600;

    private static final double MID_ZONE_DISTANCE_FT = 3.5;
    private static final double FAR_ZONE_DISTANCE_FT = 5.5;
    private static final double FAR_FAR_ZONE_DISTANCE_FT = 8.0;

    private final RobotHardware robot = new RobotHardware(this);

    private boolean flywheelEnabled = false;
    private double rpmSetpoint = RPM_LOW;

    private double lastPidfP = Double.NaN;
    private double lastPidfI = Double.NaN;
    private double lastPidfD = Double.NaN;
    private double lastPidfF = Double.NaN;

    @Override
    public void runOpMode() {
        robot.init();

        applyPanelsPidfIfChanged();

        waitForStart();

        if (robot.launcher == null) {
            telemetry.addLine("ERROR: launcher motor is NULL!");
            telemetry.update();
            return;
        }

        boolean lastA = false;
        boolean lastX = false;
        boolean lastY = false;
        boolean lastB = false;

        while (opModeIsActive()) {
            applyPanelsPidfIfChanged();

            if (gamepad1.a && !lastA) {
                flywheelEnabled = !flywheelEnabled;
            }

            if (gamepad1.x && !lastX) {
                rpmSetpoint = RPM_LOW;
            }

            if (gamepad1.y && !lastY) {
                rpmSetpoint = RPM_MID;
            }

            if (gamepad1.b && !lastB) {
                rpmSetpoint = RPM_HIGH;
            }

            updateFlywheel();

            telemetry.addData("Flywheel Enabled", flywheelEnabled);
            telemetry.addData("Flywheel Target RPM", "%.0f", flywheelEnabled ? rpmSetpoint : 0.0);
            telemetry.addData("Flywheel Current RPM", "%.0f", getCurrentRpm());
            telemetry.update();

            robot.flushPanelsTelemetry(telemetry);

            lastA = gamepad1.a;
            lastX = gamepad1.x;
            lastY = gamepad1.y;
            lastB = gamepad1.b;

            idle();
        }
    }

    private void updateFlywheel() {
        if (!flywheelEnabled) {
            stopFlywheel();
            return;
        }

        double dynamicRpm = Math.max(rpmSetpoint, calculateAutoRpm());
        double ticksPerSecond = rpmToTicksPerSecond(dynamicRpm);

        if (robot.launcher != null) {
            robot.launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.launcher.setVelocity(ticksPerSecond);
        }

        if (robot.launcher2 != null) {
            robot.launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.launcher2.setVelocity(ticksPerSecond);
        }

        if (robot.getPanelsTelemetry() != null) {
            robot.getPanelsTelemetry().debug(
                    "Flywheel RPM (target/current)",
                    String.format("%.0f / %.0f", dynamicRpm, getCurrentRpm()));
        }
    }

    private double calculateAutoRpm() {
        LLResult result = robot.getLatestLimelightResult();
        if (result == null || !result.isValid()) {
            return Constants.DEFAULT_RPM;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            return Constants.DEFAULT_RPM;
        }

        LLResultTypes.FiducialResult fid = fiducials.get(0);
        Pose3D pose = fid.getRobotPoseTargetSpace();
        Position position = pose != null ? pose.getPosition() : null;

        if (position == null) {
            return Constants.DEFAULT_RPM;
        }

        Position metersPosition = position.toUnit(DistanceUnit.METER);
        double xMeters = metersPosition.x;
        double yMeters = metersPosition.y;
        double zMeters = metersPosition.z;
        double distanceMeters = Math.sqrt(xMeters * xMeters + yMeters * yMeters + zMeters * zMeters);
        double distanceFeet = distanceMeters * 3.28084;

        if (distanceFeet >= FAR_FAR_ZONE_DISTANCE_FT) {
            return Constants.LAUNCH_ZONE_FAR_FAR_RPM;
        }

        double clampedDistance = Math.max(MID_ZONE_DISTANCE_FT, Math.min(distanceFeet, FAR_ZONE_DISTANCE_FT));
        double distanceRatio = (clampedDistance - MID_ZONE_DISTANCE_FT) / (FAR_ZONE_DISTANCE_FT - MID_ZONE_DISTANCE_FT);
        return Constants.LAUNCH_ZONE_MID_RPM
                + distanceRatio * (Constants.LAUNCH_ZONE_FAR_RPM - Constants.LAUNCH_ZONE_MID_RPM);
    }

    private void stopFlywheel() {
        if (robot.launcher != null) {
            robot.launcher.setVelocity(0);
            robot.launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (robot.launcher2 != null) {
            robot.launcher2.setVelocity(0);
            robot.launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (robot.getPanelsTelemetry() != null) {
            robot.getPanelsTelemetry().debug("Flywheel RPM (target/current)", "0 / 0");
        }
    }

    private double rpmToTicksPerSecond(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }

    private double getCurrentRpm() {
        DcMotorEx launcherMotor = robot.launcher;
        if (launcherMotor == null) {
            return 0.0;
        }

        return (launcherMotor.getVelocity() * 60.0) / TICKS_PER_REV;
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

        if (robot.getPanelsTelemetry() != null) {
            robot.getPanelsTelemetry().debug(
                    "Launcher PIDF (P,I,D,F)",
                    String.format("P=%.3f I=%.3f D=%.3f F=%.3f", pidf.p, pidf.i, pidf.d, pidf.f));
        }

        lastPidfP = p;
        lastPidfI = i;
        lastPidfD = d;
        lastPidfF = f;
    }
}
