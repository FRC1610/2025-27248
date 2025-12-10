package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.DecodePaths;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.subsystems.ShootingController;
import org.firstinspires.ftc.teamcode.subsystems.TurretTracker;

import java.util.HashMap;
import java.util.Map;

public class StateMachine {
    public enum State {
        HOME,
        AUTO_NEAR,
        AUTO_HOME_NEAR,
    }

    public enum AUTO_PATHS {
        NEAR_PATH_TO_SHOOT_AREA,
        NEAR_SHOOT_AREA_TO_CLOSEST_ARTIFACTS,
        NEAR_PICKUP_CLOSEST_ARTIFACTS,
        NEAR_PICKUP_TO_SHOOT_AREA,
        NEAR_LEAVE_SHOOT_AREA,
    }

    private State currentState;
    private final RobotHardware robot;
    private final Follower follower;
    private final ShootingController shootingController;
    private final FlywheelController flywheelController;
    private final TurretTracker turretTracker;

    private int autoNearSubStep = 0;
    private boolean autoNearShootStarted = false;

    private final double[] spindexerPositions = new double[]{org.firstinspires.ftc.teamcode.Constants.spindexer1, org.firstinspires.ftc.teamcode.Constants.spindexer2, org.firstinspires.ftc.teamcode.Constants.spindexer3};
    private int spindexerIndex = 0;

    private Map<AUTO_PATHS, PathChain> paths = new HashMap<>();

    public StateMachine(RobotHardware hardware, Follower follower, ShootingController shootingController,
                       FlywheelController flywheelController, TurretTracker turretTracker) {
        this.robot = hardware;
        this.currentState = State.HOME;
        this.follower = follower;
        this.shootingController = shootingController;
        this.flywheelController = flywheelController;
        this.turretTracker = turretTracker;
    }

    public StateMachine(RobotHardware hardware, Follower follower, ShootingController shootingController) {
        this(hardware, follower, shootingController, null, null);
    }

    public StateMachine(RobotHardware hardware, Follower follower) {
        this(hardware, follower, null, null, null);
    }

    public StateMachine(RobotHardware hardware) {
        this(hardware, null, null, null, null);
    }
    public void setState(State state, boolean doUpdate) {
        this.currentState = state;

        // reset blah blah
        autoNearSubStep = 0;
        autoNearShootStarted = false;

        if (doUpdate) { update(); }
    }

    public void setState(State state) {
        setState(state, false);
    }

    public State getState() { return currentState; }

    private void buildPath(AUTO_PATHS name, Pose blueFrom, Pose blueTo, Pose redFrom, Pose redTo) {
        Pose from = robot.allianceColorBlue ? blueFrom : redFrom;
        Pose to = robot.allianceColorBlue ? blueTo : redTo;
        paths.put(name, Constants.buildPath(this.follower, from, to));
    }

    public void init() {
        buildPath(AUTO_PATHS.NEAR_PATH_TO_SHOOT_AREA,
                DecodePaths.BLUE_NEAR_START, DecodePaths.BLUE_NEAR_SHOOT,
                DecodePaths.RED_NEAR_START, DecodePaths.RED_NEAR_SHOOT);

        buildPath(AUTO_PATHS.NEAR_SHOOT_AREA_TO_CLOSEST_ARTIFACTS,
                DecodePaths.BLUE_NEAR_SHOOT, DecodePaths.BLUE_NEAR_GOTO_ARTIFACTS,
                DecodePaths.RED_NEAR_SHOOT, DecodePaths.RED_NEAR_GOTO_ARTIFACTS);

        buildPath(AUTO_PATHS.NEAR_PICKUP_CLOSEST_ARTIFACTS,
                DecodePaths.BLUE_NEAR_GOTO_ARTIFACTS, DecodePaths.BLUE_NEAR_PICKUP_ARTIFACTS,
                DecodePaths.RED_NEAR_GOTO_ARTIFACTS, DecodePaths.RED_NEAR_PICKUP_ARTIFACTS);

        buildPath(AUTO_PATHS.NEAR_PICKUP_TO_SHOOT_AREA,
                DecodePaths.BLUE_NEAR_PICKUP_ARTIFACTS, DecodePaths.BLUE_NEAR_SHOOT,
                DecodePaths.RED_NEAR_PICKUP_ARTIFACTS, DecodePaths.RED_NEAR_SHOOT);

        buildPath(AUTO_PATHS.NEAR_LEAVE_SHOOT_AREA,
                DecodePaths.BLUE_NEAR_SHOOT, DecodePaths.BLUE_NEAR_LEAVE,
                DecodePaths.RED_NEAR_SHOOT, DecodePaths.RED_NEAR_LEAVE);
    }

    public void update() {
        switch (currentState) {
            case HOME:
                break;
            case AUTO_HOME_NEAR:
                Pose startPose = robot.allianceColorBlue ? DecodePaths.BLUE_NEAR_START : DecodePaths.RED_NEAR_START;
                this.follower.setStartingPose(startPose);
                //follower.setPose(startPose);
                // reset the spindexer here if its needed
                this.robot.spindexer.setPosition(spindexerPositions[spindexerIndex]);
                this.robot.spindexerPos = spindexerPositions[spindexerIndex];
                break;
            case AUTO_NEAR:
                switch (autoNearSubStep) {
                    case 0:
                        // Go from near goal to shooting point
                        this.follower.followPath(paths.get(AUTO_PATHS.NEAR_PATH_TO_SHOOT_AREA), true);
                        autoNearSubStep++;
                        break;
                    case 1:
                        // Auto shoot preloaded artifacts
                        if (!this.follower.isBusy()) {
                            if (turretTracker != null) {
                                turretTracker.update();
                            }

                            if (flywheelController != null) {
                                if (!flywheelController.isEnabled()) {
                                    flywheelController.toggle();
                                }
                                flywheelController.update();
                            }

                            if (shootingController != null) {
                                if (!autoNearShootStarted
                                        && flywheelController != null
                                        && flywheelController.isEnabled()
                                        && flywheelController.getTargetRpm() > 0) {
                                    shootingController.startShootSequence();
                                    autoNearShootStarted = true;
                                }

                                boolean finishedShooting = autoNearShootStarted && shootingController.updateAndIsComplete();
                                if (finishedShooting) {
                                    autoNearSubStep++;
                                    autoNearShootStarted = false;
                                }
                            } else {
                                autoNearSubStep++;
                            }
                        }
                        break;
                    case 2:
                        if (!this.follower.isBusy()) {
                            this.follower.followPath(paths.get(AUTO_PATHS.NEAR_SHOOT_AREA_TO_CLOSEST_ARTIFACTS), true);
                            //TODO: run intake
                            autoNearSubStep++;
                        }
                        break;
                    case 3:
                        if (!this.follower.isBusy()) {
                            this.follower.followPath(paths.get(AUTO_PATHS.NEAR_PICKUP_CLOSEST_ARTIFACTS), true);
                            //TODO: stop intake after path is finished
                            autoNearSubStep++;
                        }
                        break;
                    case 4:
                        if (!this.follower.isBusy()) {
                            this.follower.followPath(paths.get(AUTO_PATHS.NEAR_PICKUP_TO_SHOOT_AREA), true);
                            //TODO: Shoot sequence here
                            autoNearSubStep++;
                        }
                        break;
                    case 5:
                        if (!this.follower.isBusy()) {
                            this.follower.followPath(paths.get(AUTO_PATHS.NEAR_LEAVE_SHOOT_AREA), true);
                            autoNearSubStep++;
                        }
                        break;
                    default:
                        break;
                }
        }
    }


}
