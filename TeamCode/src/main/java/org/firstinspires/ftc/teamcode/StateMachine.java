package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.DecodePaths;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.subsystems.FindGoal;
import org.firstinspires.ftc.teamcode.subsystems.ShootingController;
import org.firstinspires.ftc.teamcode.subsystems.TurretTracker;

import java.util.HashMap;
import java.util.Map;

public class StateMachine {
    public enum State {
        HOME,
        AUTO_HOME_NEAR,
        AUTO_NEAR,
        AUTO_HOME_FAR,
        AUTO_FAR,
        AUTO_LEAVE_NEAR,
        AUTO_LEAVE_FAR,
    }

    public enum AUTO_PATHS {
        NEAR_PATH_TO_SHOOT_AREA,
        NEAR_SHOOT_AREA_TO_SPIKE1,
        NEAR_GOTO_SHOOT_SPIKE1,
        NEAR_PICKUP_SPIKE1,
        FAR_START_TO_SHOOT,
        FAR_SHOOT_TO_SPIKE3,
        FAR_SHOOT_ROTATE_TO_SPIKE3,
        NEAR_START_TO_LEAVE,
        FAR_START_TO_LEAVE,
    }

    private State currentState;
    private final RobotHardware robot;
    private final Follower follower;
    private final ShootingController shootingController;
    private final FlywheelController flywheelController;
    private final TurretTracker turretTracker;
    private final FindGoal findGoal;

    private Timer pathTimer = new Timer();
    private int autoNearSubStep = 0;
    private int autoFarSubStep = 0;
    private boolean shootStarted = false;

    private final double[] spindexerPositions = new double[]{org.firstinspires.ftc.teamcode.Constants.spindexer1, org.firstinspires.ftc.teamcode.Constants.spindexer2, org.firstinspires.ftc.teamcode.Constants.spindexer3};
    private int spindexerIndex = 0;

    private Map<AUTO_PATHS, PathChain> paths = new HashMap<>();

    public StateMachine(RobotHardware hardware, Follower follower, ShootingController shootingController,
                       FlywheelController flywheelController, TurretTracker turretTracker) {
        this.robot = hardware;
        this.follower = follower;
        this.shootingController = shootingController;
        this.flywheelController = flywheelController;
        this.turretTracker = turretTracker;
        this.findGoal = new FindGoal(hardware);
        setState(State.HOME, true);
    }
    public void setState(State state, boolean doUpdate) {
        currentState = state;

        // reset blah blah

        pathTimer.resetTimer();

        autoNearSubStep = 0;
        autoFarSubStep = 0;

        shootStarted = false;

        if (doUpdate) { update(); }
    }

    public void setState(State state) {
        setState(state, false);
    }

    public State getState() { return currentState; }

    private void buildPath(AUTO_PATHS name, Pose blueFrom, Pose blueTo, Pose redFrom, Pose redTo) {
        Pose from = robot.allianceColorBlue ? blueFrom : redFrom;
        Pose to = robot.allianceColorBlue ? blueTo : redTo;
        paths.put(name, Constants.buildPath(follower, from, to));
    }

    public void init() {
        // NEAR
        buildPath(AUTO_PATHS.NEAR_PATH_TO_SHOOT_AREA,
                DecodePaths.BLUE_NEAR_START, DecodePaths.BLUE_NEAR_SHOOT,
                DecodePaths.RED_NEAR_START, DecodePaths.RED_NEAR_SHOOT);

        buildPath(AUTO_PATHS.NEAR_SHOOT_AREA_TO_SPIKE1,
                DecodePaths.BLUE_NEAR_SHOOT, DecodePaths.BLUE_NEAR_GOTO_ARTIFACTS,
                DecodePaths.RED_NEAR_SHOOT, DecodePaths.RED_NEAR_GOTO_ARTIFACTS);

        buildPath(AUTO_PATHS.NEAR_PICKUP_SPIKE1,
                DecodePaths.BLUE_NEAR_GOTO_ARTIFACTS, DecodePaths.BLUE_NEAR_PICKUP_ARTIFACTS,
                DecodePaths.RED_NEAR_GOTO_ARTIFACTS, DecodePaths.RED_NEAR_PICKUP_ARTIFACTS);

        buildPath(AUTO_PATHS.NEAR_GOTO_SHOOT_SPIKE1,
                DecodePaths.BLUE_NEAR_PICKUP_ARTIFACTS, DecodePaths.BLUE_NEAR_GO_INSIDE_ZONE,
                DecodePaths.RED_NEAR_PICKUP_ARTIFACTS, DecodePaths.RED_NEAR_GO_INSIDE_ZONE);
//
//        buildPath(AUTO_PATHS.NEAR_LEAVE_SHOOT_AREA,
//                DecodePaths.BLUE_NEAR_SHOOT, DecodePaths.BLUE_NEAR_LEAVE,
//                DecodePaths.RED_NEAR_SHOOT, DecodePaths.RED_NEAR_LEAVE);

        // FAR
        buildPath(AUTO_PATHS.FAR_START_TO_SHOOT,
                DecodePaths.BLUE_FAR_START, DecodePaths.BLUE_FAR_TO_SHOOT_AREA,
                DecodePaths.RED_FAR_START, DecodePaths.RED_FAR_TO_SHOOT_AREA);

        buildPath(AUTO_PATHS.FAR_SHOOT_TO_SPIKE3,
                DecodePaths.BLUE_FAR_TO_SHOOT_AREA, DecodePaths.BLUE_FAR_TO_SPIKE3,
                DecodePaths.RED_FAR_TO_SHOOT_AREA, DecodePaths.RED_FAR_TO_SPIKE3);

//        buildPath(AUTO_PATHS.FAR_SHOOT_ROTATE_TO_SPIKE3,
//                DecodePaths.BLUE_FAR_TO_SPIKE3, DecodePaths.BLUE_FAR_ROTATE_TO_SPIKE3,
//                DecodePaths.RED_FAR_TO_SPIKE3, DecodePaths.RED_FAR_ROTATE_TO_SPIKE3);

//        buildPath(AUTO_PATHS.FAR_LEAVE_TO_PICKUP,
//                DecodePaths.BLUE_FAR_LEAVE, DecodePaths.BLUE_FAR_GET_ARTIFACTS,
//                DecodePaths.RED_FAR_LEAVE, DecodePaths.RED_FAR_GET_ARTIFACTS);
//
//        buildPath(AUTO_PATHS.FAR_PICKUP_TO_START,
//                DecodePaths.BLUE_FAR_GET_ARTIFACTS, DecodePaths.BLUE_FAR_START,
//                DecodePaths.RED_FAR_GET_ARTIFACTS, DecodePaths.RED_FAR_START);
    }

    private boolean shoot() {
        if (shootingController == null) {
            return true;
        }

        if (turretTracker != null) {
            robot.refreshLimelightResult();
            turretTracker.update();
        }

        if (flywheelController != null) {
            flywheelController.update();
        }

        if (!shootStarted
                && flywheelController != null
                && flywheelController.isEnabled()
                && flywheelController.getTargetRpm() > 0) {
            shootingController.startShootSequence();
            shootStarted = true;
        }

        boolean finishedShooting = shootStarted && shootingController.updateAndIsComplete();
        if (finishedShooting) {
            shootStarted = false;
            return true;
        }
        return false;
    }

    private void runFlywheel() {
        if (flywheelController != null) {
            if (!flywheelController.isEnabled()) {
                flywheelController.toggle();
            }
            flywheelController.update();
        }
    }

    private void stopFlywheel() {
        if (flywheelController != null && flywheelController.isEnabled()) {
            flywheelController.toggle();
        }
    }

    private void setSpindexPosition(int position) {
        spindexerIndex = position;
        robot.spindexer.setPosition(spindexerPositions[spindexerIndex]);
        robot.spindexerPos = spindexerPositions[spindexerIndex];
    }

    public void update() {
        switch (currentState) {
            case HOME:
                robot.spindexer.setPosition(spindexerPositions[spindexerIndex]);
                robot.spindexerPos = spindexerPositions[spindexerIndex];
                break;
            case AUTO_HOME_NEAR:
                Pose startPoseNear = robot.allianceColorBlue ? DecodePaths.BLUE_NEAR_START : DecodePaths.RED_NEAR_START;
                follower.setStartingPose(startPoseNear);
                follower.setPose(startPoseNear);
                break;
            case AUTO_HOME_FAR:
                Pose startPoseFar = robot.allianceColorBlue ? DecodePaths.BLUE_FAR_START : DecodePaths.RED_FAR_START;
                follower.setStartingPose(startPoseFar);
                follower.setPose(startPoseFar);
                break;
            case AUTO_NEAR:
                switch (autoNearSubStep) {
                    case 0:
                        runFlywheel();
                        follower.followPath(paths.get(AUTO_PATHS.NEAR_PATH_TO_SHOOT_AREA), true);
                        autoNearSubStep++;
                        break;
                    case 1:
                        boolean turretReady = findGoal.updateAndIsDone();
                        if (turretReady) {
                            autoNearSubStep++;
                        }
                        break;
                    case 2:
                        // Auto shoot preloaded artifacts
                        if (!follower.isBusy()) {
                            boolean shot = shoot();
                            if (shot) {
                                autoNearSubStep++;
                            }
                        }
                        break;
                    case 3:
                        if (!follower.isBusy()) {
                            stopFlywheel();
                            follower.followPath(paths.get(AUTO_PATHS.NEAR_SHOOT_AREA_TO_SPIKE1), true);
                            setSpindexPosition(0);
                            pathTimer.resetTimer();
                            autoNearSubStep++;
                        }
                        break;
                    case 4:
                        if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.0) {
                            robot.runIntake(RobotHardware.IntakeDirection.IN);
                            follower.followPath(paths.get(AUTO_PATHS.NEAR_PICKUP_SPIKE1), true);
                            runFlywheel();
                            pathTimer.resetTimer();
                            autoNearSubStep++;
                        }
                        break;
                    case 5:
                        if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1) {
                            follower.followPath(paths.get(AUTO_PATHS.NEAR_GOTO_SHOOT_SPIKE1), true);
                            pathTimer.resetTimer();
                            autoNearSubStep++;
                        }
                        break;
                    case 6:
                        if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.5) {
                            robot.runIntake(RobotHardware.IntakeDirection.STOP);
                            autoNearSubStep++;
                        }
                        break;
                    case 7:
                        if (!follower.isBusy()) {
                            boolean shot = shoot();
                            if (shot) {
                                stopFlywheel();
                                autoNearSubStep++;
                            }
                        }
                        break;
                    default:
                        break;
                }
                break;
            case AUTO_FAR:
                switch (autoFarSubStep) {
                    case 0:
                        follower.followPath(paths.get(AUTO_PATHS.FAR_START_TO_SHOOT), true);
                        runFlywheel();
                        autoFarSubStep++;
                        break;
                    case 1:
                        boolean turretReady = findGoal.updateAndIsDone();
                        if (turretReady) {
                            autoFarSubStep++;
                        }
                        break;
                    case 2:
                        if (!follower.isBusy()) {
                            boolean shot = shoot();
                            if (shot) {
                                follower.followPath(paths.get(AUTO_PATHS.FAR_SHOOT_TO_SPIKE3), true);
                                stopFlywheel();
                                autoFarSubStep++;
                            }
                        }
                        break;
                    default:
                        break;
                }
                break;
        }
    }
}
