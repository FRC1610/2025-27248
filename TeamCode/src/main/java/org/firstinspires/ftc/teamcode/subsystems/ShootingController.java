package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;

import java.util.List;

public class ShootingController {

    public enum ShootState {
        IDLE,
        FIRE,
        RETRACT,
        ADVANCE,
        WAIT_FOR_SPINUP,
        PRIME_NEXT
    }

    private final RobotHardware robot;
    private final FlywheelController flywheelController;
    private final Telemetry telemetry;

    private final double[] spindexerPositions = new double[]{Constants.spindexer1, Constants.spindexer2, Constants.spindexer3};
    private int spindexerIndex = 0;
    private final ElapsedTime shootTimer = new ElapsedTime();
    private boolean hasFiredShot = false;
    private ShootState shootState = ShootState.IDLE;

    public ShootingController(RobotHardware robot, FlywheelController flywheelController, Telemetry telemetry) {
        this.robot = robot;
        this.flywheelController = flywheelController;
        this.telemetry = telemetry;
    }

    public void startShootSequence() {
        shootState = ShootState.WAIT_FOR_SPINUP;
        shootTimer.reset();
        hasFiredShot = false;
        robot.kicker.setPosition(Constants.kickerDown);
    }

    public void update() {
        if (shootState == ShootState.IDLE) {
            return;
        }

        if (!flywheelController.isEnabled() || flywheelController.getTargetRpm() <= 0) {
            robot.kicker.setPosition(Constants.kickerDown);
            shootState = ShootState.IDLE;
            hasFiredShot = false;
            return;
        }

        switch (shootState) {
            case WAIT_FOR_SPINUP:
                if (flywheelController.isAtSpeed(Constants.FLYWHEEL_TOLERANCE_RPM) && isAimedAtTarget()) {
                    robot.kicker.setPosition(Constants.kickerUp);
                    shootTimer.reset();
                    if (!hasFiredShot) {
                        shootState = ShootState.FIRE;
                        hasFiredShot = true;
                    } else {
                        shootState = ShootState.PRIME_NEXT;
                    }
                }
                break;
            case FIRE:
                if (shootTimer.milliseconds() >= 500) {
                    robot.kicker.setPosition(Constants.kickerDown);
                    shootTimer.reset();
                    shootState = ShootState.RETRACT;
                }
                break;
            case RETRACT:
                if (shootTimer.milliseconds() >= 250) {
                    advanceSpindexer();
                    shootTimer.reset();
                    shootState = ShootState.ADVANCE;
                }
                break;
            case ADVANCE:
                if (shootTimer.milliseconds() >= 500) {
                    shootState = ShootState.WAIT_FOR_SPINUP;
                }
                break;
            case PRIME_NEXT:
                if (shootTimer.milliseconds() >= 500) {
                    robot.kicker.setPosition(Constants.kickerDown);
                    shootState = ShootState.IDLE;
                }
                break;
            default:
                shootState = ShootState.IDLE;
                break;
        }

        telemetry.addData("Shoot State", shootState);
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
            return Math.abs(txPercent) <= 0.05;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            return false;
        }

        double txDegrees = fiducials.get(0).getTargetXDegrees();
        return Math.abs(txDegrees) <= 5.0;
    }

    private void advanceSpindexer() {
        spindexerIndex = (spindexerIndex + 1) % spindexerPositions.length;
        robot.spindexerPos = spindexerPositions[spindexerIndex];
        robot.spindexer.setPosition(robot.spindexerPos);
    }
}
