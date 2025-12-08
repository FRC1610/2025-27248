package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.StateMachine;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

// testing if im publishing this right
@Autonomous(name = "Auto Proto", group = "Auto Test")
public class BlueNear extends LinearOpMode {
    RobotHardware hardware = new RobotHardware(this);

    @Override
    public void runOpMode() {
        hardware.init();
        Follower follower = Constants.createFollower(hardwareMap);
        StateMachine stateMachine = new StateMachine(hardware, follower);
        stateMachine.init();

        // set our home position
        stateMachine.setState(StateMachine.State.AUTO_HOME_NEAR, true);
        //follower.setStartingPose(DecodePaths.BLUE_NEAR_START);

        waitForStart();

        // start our auto state
        stateMachine.setState(StateMachine.State.AUTO_NEAR);

        while (opModeIsActive()) {
            follower.update();
            stateMachine.update();

            telemetry.addData("STATE", stateMachine.getState());
            telemetry.addData("X POS", follower.getPose().getX());
            telemetry.addData("Y POS", follower.getPose().getY());
            telemetry.addData("HEADING", follower.getPose().getHeading());
        }
    }
}