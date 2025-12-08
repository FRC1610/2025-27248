package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.DecodePaths;

public class StateMachine {
    public enum State {
        HOME,
        AUTO_HOME_FAR,
        AUTO_HOME_NEAR,
        AUTO_DRIVE_SHOOT_POSITION,
        AUTO_SHOOT_PRELOADED,
        // dk if we're past here yet
        AUTO_NEAR_GOTO_CLOSEST_ARTIFACTS,
        AUTO_NEAR_PICKUP_CLOSEST_ARTIFACTS,
        AUTO_REVERSE_NEAR_CLOSEST_ARTIFACTS,
        AUTO_SHOOT_POSITION,
        AUTO_SHOOT_LOADED
    }

    private State currentState;
    private final RobotHardware robot;
    private final Follower follower;

    private Timer pathTimer = new Timer();
    private Timer opModeTimer = new Timer();

    private final double[] spindexerPositions = new double[]{org.firstinspires.ftc.teamcode.Constants.spindexer1, org.firstinspires.ftc.teamcode.Constants.spindexer2, org.firstinspires.ftc.teamcode.Constants.spindexer3};
    private int spindexerIndex = 0;

    public StateMachine(RobotHardware hardware, Follower follower) {
        this.robot = hardware;
        this.currentState = State.HOME;
        this.follower = follower;
    }

    public StateMachine(RobotHardware hardware) {
        this(hardware, null);
    }
    public void setState(State state) {
        this.currentState = state;

        pathTimer.resetTimer();
        opModeTimer.resetTimer();
        // reset blah blah
    }

    public State getState() { return currentState; }

    public void update() {
        //TODO: adjust auto pathTimer values, and add sleeps if needed
        //TODO: currently thisll put it in infinite recursion of getting the "closest" balls to the goal, while it should do the first step correctly, i want to minimize time needed to go from a->b, and possibly make time to get another set
        //TODO: the above might require me to merge the auto states into 1, which is no big deal, but its 1:30am so ill redo like half of this tmrw
        //TODO: oh yeah almost forgot, i need to make it so it can get its color to determine which path to use
        switch (currentState) {
            case HOME:
                break;
            case AUTO_HOME_NEAR:
                follower.setPose(DecodePaths.BLUE_NEAR_START);
                // reset the spindexer here if its needed
                robot.spindexer.setPosition(spindexerPositions[spindexerIndex]);
                robot.spindexerPos = spindexerPositions[spindexerIndex];
            case AUTO_DRIVE_SHOOT_POSITION:
                PathChain pathToNearShoot = Constants.buildPath(this.follower, DecodePaths.BLUE_NEAR_START, DecodePaths.BLUE_NEAR_SHOOT);
                this.follower.followPath(pathToNearShoot, true);
                setState(State.AUTO_SHOOT_PRELOADED);
                break;
            case AUTO_SHOOT_PRELOADED:
                if (!this.follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    // TODO: add shoot sequence
                    setState(State.AUTO_NEAR_GOTO_CLOSEST_ARTIFACTS);
                }
                break;
            case AUTO_NEAR_GOTO_CLOSEST_ARTIFACTS:
                if (!this.follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    // TODO: run intake
                    PathChain pathToNearestArtifacts = Constants.buildPath(this.follower, DecodePaths.BLUE_NEAR_SHOOT, DecodePaths.BLUE_PICKUP_NEAR_ARTIFACTS_TOP_STEP_1AND3);
                    this.follower.followPath(pathToNearestArtifacts, true);
                    // TODO: stop intake
                    setState(State.AUTO_NEAR_PICKUP_CLOSEST_ARTIFACTS);
                }
                break;
            case AUTO_NEAR_PICKUP_CLOSEST_ARTIFACTS:
                if (!this.follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    PathChain pathToNearestArtifacts = Constants.buildPath(this.follower, DecodePaths.BLUE_PICKUP_NEAR_ARTIFACTS_TOP_STEP_1AND3, DecodePaths.BLUE_PICKUP_NEAR_ARTIFACTS_TOP_STEP_2);
                    this.follower.followPath(pathToNearestArtifacts, true);
                    // hopefully reuse a state
                    setState(State.AUTO_REVERSE_NEAR_CLOSEST_ARTIFACTS);
                }
                break;
            case AUTO_REVERSE_NEAR_CLOSEST_ARTIFACTS:
                if (!this.follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    PathChain pathToNearestArtifacts = Constants.buildPath(this.follower, DecodePaths.BLUE_PICKUP_NEAR_ARTIFACTS_TOP_STEP_2, DecodePaths.BLUE_PICKUP_NEAR_ARTIFACTS_TOP_STEP_1AND3);
                    this.follower.followPath(pathToNearestArtifacts, true);
                    setState(State.AUTO_DRIVE_SHOOT_POSITION);
                }
            default:
                break;
        }
    }


}
