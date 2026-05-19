package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;

import java.util.List;

public class ShootingController {

    public enum ShootState {
        IDLE,
        WAIT_FOR_SPINUP,
        ADVANCE,
        NEXT_ARTIFACT,
        FIRE,
        RETRACT,
        FINISH,
    }

    private final RobotHardware robot;
    private final FlywheelController flywheelController;
    private final SpindexerController spindexerController;
    private final Telemetry telemetry;
    private final ElapsedTime shootTimer = new ElapsedTime();
    private ShootState shootState = ShootState.IDLE;
    private int artifactCount = 3;

    public ShootingController(RobotHardware robot, FlywheelController flywheelController, SpindexerController spindexerController, Telemetry telemetry) {
        this.robot = robot;
        this.flywheelController = flywheelController;
        this.spindexerController = spindexerController;
        this.telemetry = telemetry;
    }

    public void startShootSequence() {
        shootTimer.reset();
        spindexerController.setPosition(0);
        robot.kicker.setPosition(Constants.KICKER_DOWN);
        shootState = ShootState.WAIT_FOR_SPINUP;
    }

    public void update(boolean checkArtifacts) {
        if (!flywheelController.isEnabled() || flywheelController.getTargetRpm() <= 0) return;

        switch (shootState) {
            case WAIT_FOR_SPINUP:
                // Check if spindexer is in position
                if (!spindexerController.isFinished()) return;

                if (artifactCount == 0) {
                    shootState = ShootState.FINISH;
                    return;
                }

                // Check if flywheel is at speed and is aimed at the target
                if (flywheelController.isAtSpeed() && isAimedAtTarget()) {
                    shootTimer.reset();
                    shootState = ShootState.FIRE;
                    artifactCount -= 1;
                }
                break;
            case ADVANCE:
                spindexerController.advanceSpindexer();
                shootTimer.reset();
                shootState = ShootState.NEXT_ARTIFACT;
                break;
            case NEXT_ARTIFACT:
                if (!spindexerController.isFinished()) return;

                if (artifactCount == 0) {
                    shootState = ShootState.FINISH;
                    return;
                }

                shootTimer.reset();
                shootState = ShootState.WAIT_FOR_SPINUP;

                break;
            case FIRE:
                robot.kicker.setPosition(Constants.KICKER_UP);
                shootTimer.reset();
                shootState = ShootState.RETRACT;
                break;
            case RETRACT:
                if (shootTimer.milliseconds() >= Constants.SHOOT_RETRACT_DURATION_MS) {
                    robot.kicker.setPosition(Constants.KICKER_DOWN);
                    if (artifactCount > 0) {
                        shootTimer.reset();
                        shootState = ShootState.ADVANCE;
                    } else {
                        shootTimer.reset();
                        shootState = ShootState.FINISH;
                    }
                }
                break;
            case FINISH:
                robot.kicker.setPosition(Constants.KICKER_DOWN);
                artifactCount = 3;
                spindexerController.setPosition(0);
                shootState = ShootState.IDLE;
                break;
            default:
                shootState = ShootState.IDLE;
                break;
        }

        telemetry.addData("State", shootState);
    }

    public void update() {
        update(true);
    }

    /**
     * Update the shooting sequence and report when the full firing cycle has finished.
     *
     * @return true once the controller has completed the queued shots and returned to IDLE
     */
    public boolean updateAndIsComplete(boolean checkArtifacts) {
        update(checkArtifacts);
        return shootState == ShootState.IDLE && spindexerController.isSpindexerEmpty();
    }

    public boolean isIdle() {
        return shootState == ShootState.IDLE;
    }

    public ShootState getShootState() {
        return shootState;
    }

    private boolean isAimedAtTarget() {
        LLResult result = robot.getLatestLimelightResult();
        if (result == null || !result.isValid()) {
            return false;
        }

        double txPercent = result.getTx();
        if (!Double.isNaN(txPercent)) {
            telemetry.addData("*** TX PERCENT", Math.abs(txPercent));
            return Math.abs(txPercent) <= 1.5; // USE TO BE <=, AND 0.075
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            return false;
        }

        double txDegrees = fiducials.get(0).getTargetXDegrees();
        telemetry.addData("*** TX DEGREES", txDegrees);
        return Math.abs(txDegrees) <= 10.0;
    }
}
