package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;

import java.util.List;

/**
 * Helper that sweeps the turret toward the alliance goal and lightly centers the tag once detected.
 */
public class FindGoal {

    private static final double SEARCH_POWER = 0.25;
    private static final double ALIGN_KP = 0.05;
    private static final double TARGET_TOLERANCE_PERCENT = 0.15;
    private static final double TARGET_TOLERANCE_DEGREES = 15.0;

    private final RobotHardware robot;

    public FindGoal(RobotHardware robot) {
        this.robot = robot;
    }

    /**
     * Drive the turret toward the alliance goal until the Limelight target is centered within tolerance.
     *
     * @return {@code true} once the target has been centered or if the turret cannot move
     */
    public boolean updateAndIsDone() {
        DcMotorEx turret = robot.turret;

        if (turret == null) {
            return true;
        }

        robot.refreshLimelightResult();
        LLResult result = robot.getLatestLimelightResult();

        boolean targetVisible = result != null && result.isValid();
        if (targetVisible) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            targetVisible = fiducials != null && !fiducials.isEmpty();
        }

        double power;

        if (targetVisible) {
            double txPercent = result.getTx();
            double error;

            if (!Double.isNaN(txPercent)) {
                error = txPercent;
                if (Math.abs(error) <= TARGET_TOLERANCE_PERCENT) {
                    turret.setPower(0);
                    return true;
                }
            } else {
                double txDegrees = result.getFiducialResults().get(0).getTargetXDegrees();
                error = txDegrees;
                if (Math.abs(error) <= TARGET_TOLERANCE_DEGREES) {
                    turret.setPower(0);
                    return true;
                }
            }

            power = ALIGN_KP * error;
        } else {
            power = robot.allianceColorBlue ? -SEARCH_POWER : SEARCH_POWER;
        }

        power = Range.clip(power, -SEARCH_POWER, SEARCH_POWER);

        double pos = turret.getCurrentPosition();
        if ((pos <= Constants.turret_MIN && power < 0) || (pos >= Constants.turret_MAX && power > 0)) {
            turret.setPower(0);
            return false;
        }

        turret.setPower(power);
        return false;
    }
}
