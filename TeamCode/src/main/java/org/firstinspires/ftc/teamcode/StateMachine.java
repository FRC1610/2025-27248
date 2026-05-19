package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.DecodePaths;
import org.firstinspires.ftc.teamcode.subsystems.FindGoal;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.subsystems.ShootingController;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerController;
import org.firstinspires.ftc.teamcode.subsystems.TurretTracker;

import java.util.Map;

public class StateMachine {
    public enum State {
        STOP,
        HOME,
        AUTO_HOME_NEAR,
        AUTO_NEAR,
        AUTO_HOME_FAR,
        AUTO_FAR,
        AUTO_LEAVE_NEAR,
        AUTO_LEAVE_FAR,
    }

    private State currentState;
    private final RobotHardware robot;
    private final Follower follower;
    private final ShootingController shootingController;
    private final FlywheelController flywheelController;
    private final TurretTracker turretTracker;
    private final SpindexerController spindexerController;
    private final FindGoal findGoal;

    private final Timer pathTimer = new Timer();
    private final Timer autoTimer = new Timer();
    private int autoNearSubStep = 0;
    private int autoFarSubStep = 0;
    private boolean shootStarted = false;

    private Map<DecodePaths.AUTO_PATHS, PathChain> paths;

    public StateMachine(RobotHardware hardware, Follower follower, ShootingController shootingController,
                        FlywheelController flywheelController, TurretTracker turretTracker,
                        SpindexerController spindexerController) {
        this.robot = hardware;
        this.follower = follower;
        this.shootingController = shootingController;
        this.flywheelController = flywheelController;
        this.turretTracker = turretTracker;
        this.spindexerController = spindexerController;
        this.findGoal = new FindGoal(hardware);
        setState(State.HOME, true);
    }
    public void setState(State state, boolean doUpdate) {
        currentState = state;

        pathTimer.resetTimer();

        autoNearSubStep = 0;
        autoFarSubStep = 0;

        shootStarted = false;

        if (doUpdate) { update(); }
    }
    public void setState(State state) { setState(state, false); }
    public State getState() { return currentState; }

    public void init() {
        paths = DecodePaths.buildPaths(robot, follower);
    }

    private boolean shoot(DecodePaths.AUTO_PATHS breakPath, boolean checkArtifacts) {
        if (breakPath != null) {
            if (breakOutAuto(breakPath)) return false;
        }

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

        boolean finishedShooting = shootStarted && shootingController.updateAndIsComplete(checkArtifacts);
        if (finishedShooting) {
            shootStarted = false;
            return true;
        }
        return false;
    }

    private boolean shoot(DecodePaths.AUTO_PATHS breakPath) {
        return shoot(breakPath, true);
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

    private boolean breakOutAuto(DecodePaths.AUTO_PATHS breakPath) {
//        if (autoTimer.getElapsedTimeSeconds() >= 28.75) {
//            robot.runIntake(RobotHardware.IntakeDirection.STOP);
//            spindexerController.setPosition(0);
//            spindexerController.disableAuto();
//            stopFlywheel();
//            followPath(breakPath);
//            return true;
//        }
        return false;
    }

    private void followPath(DecodePaths.AUTO_PATHS path, boolean hold) {
        PathChain toFollow = paths.get(path);
        if (toFollow == null) {
            return;
        }
        follower.followPath(toFollow, hold);
    }
    private void followPath(DecodePaths.AUTO_PATHS path) { followPath(path, false); }

    private void setPose(Pose bluePose, Pose redPose) {
        Pose pose = robot.allianceColorBlue ? bluePose : redPose;
        follower.setStartingPose(pose);
        follower.setPose(pose);
    }

    private boolean completedPath() { return !follower.isBusy(); }

    private boolean completePathWithDelay(double seconds) { return completedPath() && pathTimer.getElapsedTimeSeconds() >= seconds; }

    public void update() {
        switch (currentState) {
            case HOME:
                spindexerController.setPosition(0);
                break;
            case AUTO_HOME_NEAR:
                setPose(DecodePaths.BLUE_NEAR_START, DecodePaths.RED_NEAR_START);
                break;
            case AUTO_HOME_FAR:
                setPose(DecodePaths.BLUE_FAR_START, DecodePaths.RED_FAR_START);
                break;
            case AUTO_NEAR:
                // case 0-2 first shoot sequence
                // case 3-5 start intake sequence for getting Spike 1
                // case 6-7 shoot last 3
                // case 8 kill robot
                // est : artifact count 3-5 maybe 6

                switch (autoNearSubStep) {
                    case 0:
                        flywheelController.setLauncherFeedforward(30);
                        //spindexerController.enableAuto();
                        autoTimer.resetTimer();
                        runFlywheel();
                        followPath(DecodePaths.AUTO_PATHS.NEAR_PATH_TO_SHOOT_AREA, true);
                        autoNearSubStep++;
                        break;
                    case 1:
                        if (findGoal.updateAndIsDone()) autoNearSubStep++;
                        break;
                    case 2:
                        if ( completedPath() )
                            if ( shoot(DecodePaths.AUTO_PATHS.NEAR_SHOOT_LEAVE, false)) autoNearSubStep++;
                        break;
                    case 3:
                        if ( completedPath() ) {
                            followPath(DecodePaths.AUTO_PATHS.NEAR_SHOOT_AREA_TO_SPIKE1, true);
                            pathTimer.resetTimer();
                            autoNearSubStep++;
                        }
                        break;
                    case 4:
                        if ( completePathWithDelay(1.0) ) {
                            robot.runIntake(RobotHardware.IntakeDirection.IN);
                            followPath(DecodePaths.AUTO_PATHS.NEAR_PICKUP_SPIKE1_PART1, true);
                            pathTimer.resetTimer();
                            autoNearSubStep++;
                        }
                        break;
                    case 5:
                        if ( completePathWithDelay(1.0) ) {
                            followPath(DecodePaths.AUTO_PATHS.NEAR_PICKUP_SPIKE1_PART2, true);
                            pathTimer.resetTimer();
                            autoNearSubStep++;
                        }
                        break;
                    case 6:
                        if ( completePathWithDelay(1.5) ) {
                            robot.runIntake(RobotHardware.IntakeDirection.STOP);
                            followPath(DecodePaths.AUTO_PATHS.NEAR_GOTO_SHOOT_SPIKE1, true);
                            autoNearSubStep++;
                        }
                        break;
                    case 7:
                        if ( completedPath() ) {
                            if ( shoot(DecodePaths.AUTO_PATHS.NEAR_SHOOT_LEAVE) ) {
                                stopFlywheel();
                                autoNearSubStep++;
                            }
                        }
                        break;
                    case 8:
                        if (completedPath()) setState(State.STOP);
                        break;
                    default:
                        break;
                }
                break;
            case AUTO_FAR:
                // case 0-2 first shoot sequence
                // case 3-5 start intake sequence for getting Spike 3
                // case 6-7 shoot last 3
                // case 8 kill robot
                // est : artifact count 3-5 maybe 6

                switch (autoFarSubStep) {
                    case 0:
                        flywheelController.setLauncherFeedforward(29);
                        //spindexerController.enableAuto();
                        autoTimer.resetTimer();
                        runFlywheel();
                        followPath(DecodePaths.AUTO_PATHS.FAR_START_TO_SHOOT, true);
                        autoFarSubStep++;
                        break;
                    case 1:
                        if ( findGoal.updateAndIsDone() ) autoFarSubStep++;
                        break;
                    case 2:
                        if ( completedPath() ) {
                            if ( shoot(DecodePaths.AUTO_PATHS.FAR_SHOOT_LEAVE, false) ) {
                                robot.runIntake(RobotHardware.IntakeDirection.IN);
                                followPath(DecodePaths.AUTO_PATHS.FAR_SHOOT_TO_SPIKE3_LINEUP, true);
                                autoFarSubStep++;
                            }
                        }
                        break;
                    case 3:
                        if ( completedPath() ) {
                            followPath(DecodePaths.AUTO_PATHS.FAR_SPIKE3_PICKUP_PART1, true);
                            pathTimer.resetTimer();
                            autoFarSubStep++;
                        }
                        break;
                    case 4:
                        if ( completePathWithDelay(2.0) ) {
                            spindexerController.setPosition(2);
                            pathTimer.resetTimer();
                            autoFarSubStep++;
                        }
                        break;
                    case 5:
                        if ( completePathWithDelay(1.25) ) {
                            spindexerController.setPosition(1);
                            pathTimer.resetTimer();
                            autoFarSubStep++;
                        }
                        break;
                    case 6:
                        if ( completePathWithDelay(1.375) ) {
                            followPath(DecodePaths.AUTO_PATHS.FAR_SPIKE3_PICKUP_PART2, true);
                            pathTimer.resetTimer();
                            autoFarSubStep++;
                        }
                        break;
                    case 7:
                        if ( completePathWithDelay(1.45) ) {
                            robot.runIntake(RobotHardware.IntakeDirection.STOP);
                            followPath(DecodePaths.AUTO_PATHS.FAR_SPIKE3_TO_SHOOT, true);
                            autoFarSubStep++;
                        }
                        break;
                    case 8:
                        if ( completedPath() ) {
                            if ( shoot(DecodePaths.AUTO_PATHS.FAR_SHOOT_LEAVE) ) {
                                followPath(DecodePaths.AUTO_PATHS.FAR_SHOOT_LEAVE, true);
                                stopFlywheel();
                                autoFarSubStep++;
                            }
                        }
                        break;
                    case 9:
                        if (completedPath()) setState(State.STOP);
                        break;
                    default:
                        break;
                }
                break;
            case AUTO_LEAVE_NEAR:
                followPath(DecodePaths.AUTO_PATHS.NEAR_SHOOT_TO_WALL, false);
                setState(State.STOP);
                break;
            case AUTO_LEAVE_FAR:
                followPath(DecodePaths.AUTO_PATHS.FAR_SHOOT_LEAVE, false);
                setState(State.STOP);
                break;
            default:
                break;
        }
    }
}