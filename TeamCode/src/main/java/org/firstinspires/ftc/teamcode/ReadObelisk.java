package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.List;
import java.util.Locale;

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
        return cachedPattern;
    }

    public ObeliskPattern scanForPattern() {
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

        robot.limelight.pipelineSwitch(1);
        robot.refreshLimelightResult();

        int targetTicks = allianceColor ? Constants.turret_OBELISK_LEFT_LIMIT : Constants.turret_OBELISK_RIGHT_LIMIT;

        driveTurretTo(targetTicks);

        while (opMode.opModeIsActive() && robot.turret.isBusy()) {
            robot.refreshLimelightResult();
            ObeliskPattern detected = decodePattern(robot.getLatestLimelightResult());
            if (detected != null) {
                cachedPattern = detected;
                break;
            }
            opMode.idle();
        }

        if (cachedPattern == null) {
            cachedPattern = decodePattern(robot.getLatestLimelightResult());
        }

        driveTurretTo(Constants.turretHome);

        if (cachedPattern != null) {
            telemetry.addData("Obelisk Pattern", cachedPattern.toString());
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
        return switch (id) {
            case 21 -> new ObeliskPattern(ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE);
            case 22 -> new ObeliskPattern(ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.PURPLE);
            case 23 -> new ObeliskPattern(ArtifactColor.PURPLE, ArtifactColor.PURPLE, ArtifactColor.GREEN);
            default -> null;
        };
    }
}
