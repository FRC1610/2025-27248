package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.DecodePaths;

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

    private Timer pathTimer = new Timer();
    private Timer opModeTimer = new Timer();

    private int autoNearSubStep = 0;

    private final double[] spindexerPositions = new double[]{org.firstinspires.ftc.teamcode.Constants.spindexer1, org.firstinspires.ftc.teamcode.Constants.spindexer2, org.firstinspires.ftc.teamcode.Constants.spindexer3};
    private int spindexerIndex = 0;

    private Map<AUTO_PATHS, PathChain> paths = new HashMap<>();

    public StateMachine(RobotHardware hardware, Follower follower) {
        this.robot = hardware;
        this.currentState = State.HOME;
        this.follower = follower;
    }

    public StateMachine(RobotHardware hardware) {
        this(hardware, null);
    }
    public void setState(State state, boolean doUpdate) {
        this.currentState = state;

        // reset blah blah
        autoNearSubStep = 0;

        pathTimer.resetTimer();
        opModeTimer.resetTimer();

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

        buildPath(AUTO_PATHS.NEAR_PATH_TO_SHOOT_AREA,
                DecodePaths.BLUE_NEAR_PICKUP_ARTIFACTS, DecodePaths.BLUE_NEAR_SHOOT,
                DecodePaths.RED_NEAR_PICKUP_ARTIFACTS, DecodePaths.RED_NEAR_SHOOT);

        buildPath(AUTO_PATHS.NEAR_PATH_TO_SHOOT_AREA,
                DecodePaths.BLUE_NEAR_SHOOT, DecodePaths.BLUE_NEAR_LEAVE,
                DecodePaths.RED_NEAR_SHOOT, DecodePaths.RED_NEAR_LEAVE);
    }

    public void update() {
        switch (currentState) {
            case HOME:
                break;
            case AUTO_HOME_NEAR:
                follower.setPose(DecodePaths.BLUE_NEAR_START);
                // reset the spindexer here if its needed
                robot.spindexer.setPosition(spindexerPositions[spindexerIndex]);
                robot.spindexerPos = spindexerPositions[spindexerIndex];
                break;
            case AUTO_NEAR:
                //TODO: adjust auto pathTimer values, and add sleeps if needed
                //TODO: what i wrote above
                switch (autoNearSubStep) {
                    case 0:
                        // Go from near goal to shooting point
                        this.follower.followPath(paths.get(AUTO_PATHS.NEAR_PATH_TO_SHOOT_AREA), true);
                        autoNearSubStep++;
                        break;
                    case 1:
                        // Auto shoot preloaded artifacts
                        if (!this.follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                            //TODO: Shoot sequence here
                            autoNearSubStep++;
                        }
                        break;
                    case 2:
                        if (!this.follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 6.5) {
                            this.follower.followPath(paths.get(AUTO_PATHS.NEAR_SHOOT_AREA_TO_CLOSEST_ARTIFACTS), true);
                            //TODO: run intake
                            autoNearSubStep++;
                        }
                        break;
                    case 3:
                        if (!this.follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 7.5) {
                            this.follower.followPath(paths.get(AUTO_PATHS.NEAR_PICKUP_CLOSEST_ARTIFACTS), true);
                            //TODO: stop intake after path is finished
                            autoNearSubStep++;
                        }
                        break;
                    case 4:
                        if (!this.follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 8.5) {
                            this.follower.followPath(paths.get(AUTO_PATHS.NEAR_PICKUP_TO_SHOOT_AREA), true);
                            //TODO: Shoot sequence here
                            autoNearSubStep++;
                        }
                        break;
                    case 5:
                        if (!this.follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 10) {
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
