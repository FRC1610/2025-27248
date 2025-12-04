package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.SharedState;
import org.firstinspires.ftc.teamcode.RobotHardware;
import java.util.List;
import java.util.Locale;
import java.util.function.BooleanSupplier;

/**
 * Utility that uses the Limelight to read the "obelisk" AprilTag and derive the
 * scoring color pattern. The detected pattern is cached statically so it can be
 * reused across autonomous and teleop OpModes.
 */
public class ReadObelisk {

    public enum ArtifactColor {
        GREEN,
        PURPLE
    }

    public static class ObeliskPattern {
        private final ArtifactColor[] colors;

        public ObeliskPattern(ArtifactColor first, ArtifactColor second, ArtifactColor third) {
            this.colors = new ArtifactColor[]{first, second, third};
        }

        public ArtifactColor[] getColors() {
            return colors.clone();
        }

        @Override
        public String toString() {
            return String.format(Locale.US, "%s, %s, %s", colors[0], colors[1], colors[2]);
        }
    }

    private static volatile ObeliskPattern cachedPattern;

    private final RobotHardware robot;
    private final LinearOpMode opMode;
    private final Telemetry telemetry;

    public ReadObelisk(RobotHardware robot, LinearOpMode opMode) {
        this.robot = robot;
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
    }

    public static ObeliskPattern getCachedPattern() {
        if (cachedPattern == null) {
            cachedPattern = SharedState.loadObeliskPattern();
        }
        return cachedPattern;
    }

    /**
     * Attempt to decode the latest Limelight result and cache the detected pattern for
     * reuse across OpModes.
     *
     * @return the detected {@link ObeliskPattern}, or {@code null} if no valid tag is visible
     */
    public ObeliskPattern decodeLatestAndCache() {
        robot.refreshLimelightResult();
        ObeliskPattern pattern = decodePattern(robot.getLatestLimelightResult());
        if (pattern != null) {
            cachedPattern = pattern;
            SharedState.saveObeliskPattern(pattern);
        }
        return pattern;
    }

    public ObeliskPattern scanForPattern() {
        return scanForPattern(() -> false);
    }

    public ObeliskPattern scanForPattern(BooleanSupplier shouldCancel) {
        if (robot.limelight == null) {
            telemetry.addLine("ERROR: Limelight not initialized");
            telemetry.update();
            return cachedPattern;
        }

        if (robot.turret == null) {
            telemetry.addLine("ERROR: Turret motor not initialized");
            telemetry.update();
            return cachedPattern;
        }

        boolean allianceColor = robot.refreshAllianceFromSwitchState();
        String sweepDirection = allianceColor ? "LEFT (negative)" : "RIGHT (positive)";

        robot.limelight.pipelineSwitch(1);
        robot.refreshLimelightResult();

        int targetTicks = allianceColor ? Constants.turret_OBELISK_LEFT_LIMIT : Constants.turret_OBELISK_RIGHT_LIMIT;

        ObeliskPattern detected = cachedPattern;
        boolean targetFound = detected != null;

        driveTurretTo(targetTicks);

        while (opMode.opModeIsActive() && robot.turret.isBusy() && !shouldCancel.getAsBoolean()) {
            robot.refreshLimelightResult();
            detected = decodePattern(robot.getLatestLimelightResult());
            targetFound = detected != null;
            if (targetFound) {
                cachedPattern = detected;
                SharedState.saveObeliskPattern(cachedPattern);
            }

            telemetry.addData("Alliance Color", allianceColor ? "RED" : "BLUE");
            telemetry.addData("Turret Sweep", sweepDirection);
            telemetry.addData("Turret Position", robot.turret.getCurrentPosition());
            telemetry.addData("Target Found", targetFound);
            if (targetFound) {
                telemetry.addData("Pattern", cachedPattern);
            }
            telemetry.update();

            opMode.idle();
            if (targetFound) {
                break;
            }
        }

        if (cachedPattern == null) {
            cachedPattern = decodePattern(robot.getLatestLimelightResult());
            if (cachedPattern != null) {
                SharedState.saveObeliskPattern(cachedPattern);
                targetFound = true;
            }
        }

        driveTurretTo(Constants.turretHome);

        telemetry.addData("Alliance Color", allianceColor ? "RED" : "BLUE");
        telemetry.addData("Turret Sweep", "Returning to home");
        telemetry.addData("Turret Position", robot.turret.getCurrentPosition());
        telemetry.addData("Target Found", targetFound);
        if (cachedPattern != null) {
            telemetry.addData("Pattern", cachedPattern.toString());
        } else {
            telemetry.addLine("No valid obelisk tag detected");
        }
        telemetry.update();

        return cachedPattern;
    }

    private void driveTurretTo(int targetTicks) {
        DcMotorEx turret = robot.turret;
        turret.setTargetPosition(targetTicks);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0.35);
    }

    private ObeliskPattern decodePattern(LLResult result) {
        if (result == null || !result.isValid()) {
            return null;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            return null;
        }

        int id = fiducials.get(0).getFiducialId();
        switch (id) {
            case 21:
                return new ObeliskPattern(ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE);
            case 22:
                return new ObeliskPattern(ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.PURPLE);
            case 23:
                return new ObeliskPattern(ArtifactColor.PURPLE, ArtifactColor.PURPLE, ArtifactColor.GREEN);
            default:
                return null;
        }
    }
}
