package org.firstinspires.ftc.teamcode.autonomous;

import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
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
    private TelemetryManager panelsTelemetry;

    @Override
    public void runOpMode() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        PanelsField.INSTANCE.getField().setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());

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
            stateMachine.update();
            follower.update();

            panelsTelemetry.debug("State", stateMachine.getState());
            panelsTelemetry.debug("Timer", stateMachine.getTimerInSeconds());
            panelsTelemetry.debug("Pose X", follower.getPose().getX());
            panelsTelemetry.debug("Pose Y", follower.getPose().getY());
            panelsTelemetry.debug("Heading", follower.getPose().getHeading());

            telemetry.addData("TIMER", stateMachine.getTimerInSeconds());
            telemetry.addData("STATE", stateMachine.getState());
            telemetry.addData("X POS", follower.getPose().getX());
            telemetry.addData("Y POS", follower.getPose().getY());
            telemetry.addData("HEADING", follower.getPose().getHeading());
            telemetry.update();

            hardware.flushPanelsTelemetry(telemetry);
        }
    }
}