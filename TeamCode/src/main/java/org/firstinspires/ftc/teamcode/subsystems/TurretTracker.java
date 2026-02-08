package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import java.util.ArrayDeque;
import java.util.Locale;
import java.util.List;

public class TurretTracker {

    private static final int LIMELIGHT_TX_WINDOW = 5;
    private static final double LIMELIGHT_TX_HOLD_SECONDS = 0.2;

    private final RobotHardware robot;
    private final Telemetry telemetry;

    private double lastError = 0;
    private double integral = 0;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime limelightHoldTimer = new ElapsedTime();
    private final ArrayDeque<Double> txSamples = new ArrayDeque<>();
    private double lastSmoothedTx = Double.NaN;
    private double lastSmoothedDistanceFeet = Double.NaN;

    public TurretTracker(RobotHardware robot, Telemetry telemetry) {

        this.robot = robot;
        this.telemetry = telemetry;
        limelightHoldTimer.reset();
    }

    public void update() {

        // SAFETY: limelight not initialized
        // SAFETY: turret not initialized
        if (robot.turret == null) {
            telemetry.addLine("ERROR: turret motor is NULL!");
            return;
        }

        // Get latest frame
        LLResult result = robot.getLatestLimelightResult();

        SmoothedLimelightData smoothedData = getSmoothedLimelightData(result);
        if (smoothedData == null) {
            robot.turret.setPower(0);
            return;
        }

        double tx = smoothedData.txDegrees;
        double distanceFeet = smoothedData.distanceFeet;

        double aimOffset = 0.0;
        if (Double.isFinite(distanceFeet) && distanceFeet > Constants.TURRET_FAR_AIM_DISTANCE_FEET) {
            aimOffset = robot.allianceColorRed
                    ? TurretAimConfig.turretFarAimAdjustRed
                    : TurretAimConfig.turretFarAimAdjustBlue;
            tx += aimOffset;
        }

        // PID timing
        double dt = timer.seconds();
        timer.reset();

        // PID compute
        double error = tx;
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double kP = 0.015;
        double kI = 0.0;
        double kD = 0.0;

        double power = kP * error + kI * integral + kD * derivative;

        // Turret encoder limits
        double pos = robot.turret.getCurrentPosition();
        if ((pos <= Constants.TURRET_MIN && power < 0) ||
                (pos >= Constants.TURRET_MAX && power > 0)) {
            power = 0;
        }

        // Apply power safely
        power = Range.clip(power, -0.75, 0.75);
        robot.turret.setPower(power);

        // Telemetry
        String distanceText = Double.isFinite(distanceFeet)
                ? String.format(Locale.US, "%.2f ft", distanceFeet)
                : "n/a";
        telemetry.addData("Turret", "id=%d dist=%s aim=%.3f power=%.3f",
                smoothedData.fiducialId, distanceText, aimOffset, power);
    }

    public int getTurretPosition() {
        return robot.turret.getCurrentPosition();
    }

    private SmoothedLimelightData getSmoothedLimelightData(LLResult result) {
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                LLResultTypes.FiducialResult fid = fiducials.get(0);

                double tx = fid.getTargetXDegrees();
                Pose3D cameraSpacePose = fid.getTargetPoseCameraSpace();
                double distanceFeet = Double.NaN;
                if (cameraSpacePose != null) {
                    Position position = cameraSpacePose.getPosition();
                    if (position != null) {
                        Position positionMeters = position.toUnit(DistanceUnit.METER);
                        double x = positionMeters.x;
                        double y = positionMeters.y;
                        double z = positionMeters.z;
                        double distanceMeters = Math.sqrt(x * x + y * y + z * z);
                        distanceFeet = distanceMeters * 3.28084;
                    }
                }

                txSamples.addLast(tx);
                while (txSamples.size() > LIMELIGHT_TX_WINDOW) {
                    txSamples.removeFirst();
                }
                double sum = 0.0;
                for (double sample : txSamples) {
                    sum += sample;
                }
                lastSmoothedTx = sum / txSamples.size();
                lastSmoothedDistanceFeet = distanceFeet;
                limelightHoldTimer.reset();
                return new SmoothedLimelightData(fid.getFiducialId(), lastSmoothedTx, lastSmoothedDistanceFeet);
            }
        }

        if (Double.isFinite(lastSmoothedTx)
                && limelightHoldTimer.seconds() < LIMELIGHT_TX_HOLD_SECONDS) {
            return new SmoothedLimelightData(-1, lastSmoothedTx, lastSmoothedDistanceFeet);
        }

        txSamples.clear();
        lastSmoothedTx = Double.NaN;
        lastSmoothedDistanceFeet = Double.NaN;
        return null;
    }

    private static class SmoothedLimelightData {
        private final int fiducialId;
        private final double txDegrees;
        private final double distanceFeet;

        private SmoothedLimelightData(int fiducialId, double txDegrees, double distanceFeet) {
            this.fiducialId = fiducialId;
            this.txDegrees = txDegrees;
            this.distanceFeet = distanceFeet;
        }
    }
}
