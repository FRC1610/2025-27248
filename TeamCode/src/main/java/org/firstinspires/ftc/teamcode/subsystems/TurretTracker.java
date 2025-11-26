package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import java.util.List;

public class TurretTracker {

    private final RobotHardware robot;
    private final Telemetry telemetry;

    private double lastError = 0;
    private double integral = 0;
    private final ElapsedTime timer = new ElapsedTime();

    public TurretTracker(RobotHardware robot, Telemetry telemetry,
                         boolean allianceColorRed, boolean allianceColorBlue) {

        this.robot = robot;
        this.telemetry = telemetry;

        // -----------------------------
        // Select pipeline by alliance
        // -----------------------------
        if (allianceColorBlue) {
            robot.limelight.pipelineSwitch(0);   // Blue → Tag 20
        } else if (allianceColorRed) {
            robot.limelight.pipelineSwitch(4);   // Red → Tag 24
        }
    }

    public void update() {

        // SAFETY: limelight not initialized
        if (robot.limelight == null) {
            robot.turret.setPower(0);
            telemetry.addLine("Limelight is NULL!");
            return;
        }

        // SAFETY: turret not initialized
        if (robot.turret == null) {
            telemetry.addLine("ERROR: turret motor is NULL!");
            return;
        }

        // Get latest frame
        LLResult result = robot.limelight.getLatestResult();

        // SAFETY: result missing or invalid
        if (result == null || !result.isValid()) {
            robot.turret.setPower(0);
            return;
        }

        // Get fiducials (FTC API)
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            robot.turret.setPower(0);
            return;
        }

        // Since the pipeline already filters the tag ID,
        // the first fiducial is always our target.
        LLResultTypes.FiducialResult fid = fiducials.get(0);

        // Horizontal angle offset (tx)
        double tx = fid.getTargetXDegrees();

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
        if ((pos <= Constants.turret_MIN && power < 0) ||
                (pos >= Constants.turret_MAX && power > 0)) {
            power = 0;
        }

        // Apply power safely
        power = Range.clip(power, -0.4, 0.4);
        robot.turret.setPower(power);

        // Telemetry
        telemetry.addData("TagID", fid.getFiducialId());
        telemetry.addData("tx", tx);
        telemetry.addData("Power", power);
        telemetry.addData("TurretPos", pos);
    }
}