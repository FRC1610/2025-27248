package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

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
    }

    public enum AUTO_PATHS {
        NEAR_PATH_TO_SHOOT_AREA,
        NEAR_SHOOT_AREA_TO_ARTIFACTS,
        FAR_START_TO_SHOOT,
        FAR_SHOOT_TO_ARTIFACTS,
    }

    private State currentState;
    private final RobotHardware robot;
    private final Follower follower;
    private final ShootingController shootingController;
    private final FlywheelController flywheelController;
    private final TurretTracker turretTracker;
    private final FindGoal findGoal;

    private int autoNearSubStep = 0;
    private int autoFarSubStep = 0;
    private boolean shootStarted = false;
    private boolean trackerEnabled = false;

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

        buildPath(AUTO_PATHS.NEAR_SHOOT_AREA_TO_ARTIFACTS,
                DecodePaths.BLUE_NEAR_SHOOT, DecodePaths.BLUE_NEAR_GOTO_ARTIFACTS,
                DecodePaths.RED_NEAR_SHOOT, DecodePaths.RED_NEAR_GOTO_ARTIFACTS);

//        buildPath(AUTO_PATHS.NEAR_PICKUP_CLOSEST_ARTIFACTS,
//                DecodePaths.BLUE_NEAR_GOTO_ARTIFACTS, DecodePaths.BLUE_NEAR_PICKUP_ARTIFACTS,
//                DecodePaths.RED_NEAR_GOTO_ARTIFACTS, DecodePaths.RED_NEAR_PICKUP_ARTIFACTS);

//        buildPath(AUTO_PATHS.NEAR_PICKUP_TO_SHOOT_AREA,
//                DecodePaths.BLUE_NEAR_PICKUP_ARTIFACTS, DecodePaths.BLUE_NEAR_SHOOT,
//                DecodePaths.RED_NEAR_PICKUP_ARTIFACTS, DecodePaths.RED_NEAR_SHOOT);
//
//        buildPath(AUTO_PATHS.NEAR_LEAVE_SHOOT_AREA,
//                DecodePaths.BLUE_NEAR_SHOOT, DecodePaths.BLUE_NEAR_LEAVE,
//                DecodePaths.RED_NEAR_SHOOT, DecodePaths.RED_NEAR_LEAVE);

        // FAR
        buildPath(AUTO_PATHS.FAR_START_TO_SHOOT,
                DecodePaths.BLUE_FAR_START, DecodePaths.BLUE_FAR_TO_SHOOT_AREA,
                DecodePaths.RED_FAR_START, DecodePaths.RED_FAR_TO_SHOOT_AREA);

        buildPath(AUTO_PATHS.FAR_SHOOT_TO_ARTIFACTS,
                DecodePaths.BLUE_FAR_TO_SHOOT_AREA, DecodePaths.BLUE_FAR_TO_CLOSEST_ARTIFACT,
                DecodePaths.RED_FAR_TO_SHOOT_AREA, DecodePaths.RED_FAR_TO_CLOSEST_ARTIFACT);

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
                            follower.followPath(paths.get(AUTO_PATHS.NEAR_SHOOT_AREA_TO_ARTIFACTS), true);
                            autoNearSubStep++;
                        }
                        break;
                    case 4:
//                        if (!follower.isBusy()) {
//                            follower.followPath(paths.get(AUTO_PATHS.NEAR_PICKUP_CLOSEST_ARTIFACTS), true);
//                            //TODO: stop intake after path is finished
//                            autoNearSubStep++;
//                        }
                        break;
                    case 5:
//                        if (!follower.isBusy()) {
//                            follower.followPath(paths.get(AUTO_PATHS.NEAR_PICKUP_TO_SHOOT_AREA), true);
//                            //TODO: Shoot sequence here
//                            autoNearSubStep++;
//                        }
                        break;
                    case 6:
//                        if (!follower.isBusy()) {
//                            this.follower.followPath(paths.get(AUTO_PATHS.NEAR_LEAVE_SHOOT_AREA), true);
//                            autoNearSubStep++;
//                        }
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
                                follower.followPath(paths.get(AUTO_PATHS.FAR_SHOOT_TO_ARTIFACTS));
                                stopFlywheel();
                                robot.turret.setTargetPosition(0);
                                autoFarSubStep++;
                            }
                        }
                        break;
                    case 3:
                        if (!follower.isBusy()) {
                            //follower.followPath(paths.get(AUTO_PATHS.FAR_LEAVE_TO_PICKUP), true);
                            //TODO: run intake at a certain point
                            //TODO: stop after pickup is complete
                            //autoFarSubStep++;
                        }
                        break;
                    case 4:
                        if (!follower.isBusy()) {
                            runFlywheel();
                            //follower.followPath(paths.get(AUTO_PATHS.FAR_PICKUP_TO_START), true);
                            autoFarSubStep++;
                        }
                        break;
                    case 5:
                        boolean turretReady2 = findGoal.updateAndIsDone();
                        if (turretReady2) {
                            autoFarSubStep++;
                        }
                        break;
                    case 6:
                        if (!follower.isBusy()) {
                            boolean shot2 = shoot();
                            if (shot2) {
                                //follower.followPath(paths.get(AUTO_PATHS.FAR_START_TO_LEAVE), true);
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
