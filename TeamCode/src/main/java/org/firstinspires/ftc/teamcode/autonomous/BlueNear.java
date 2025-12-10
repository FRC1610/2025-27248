package org.firstinspires.ftc.teamcode.autonomous;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.StateMachine;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.subsystems.ShootingController;
import org.firstinspires.ftc.teamcode.subsystems.TurretTracker;

// testing if im publishing this right
@Autonomous(name = "Auto Proto", group = "Auto Test")
public class BlueNear extends LinearOpMode {
    RobotHardware hardware = new RobotHardware(this);
    private TelemetryManager panelsTelemetry;
    private FieldManager panelsField;

    @Override
    public void runOpMode() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        panelsField = PanelsField.INSTANCE.getField();
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());

        hardware.init();
        FlywheelController flywheelController = new FlywheelController(hardware, telemetry);
        ShootingController shootingController = new ShootingController(hardware, flywheelController, telemetry);
        TurretTracker turretTracker = new TurretTracker(hardware, telemetry);
        Follower follower = Constants.createFollower(hardwareMap);
        StateMachine stateMachine = new StateMachine(hardware, follower, shootingController, flywheelController, turretTracker);
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
            if (flywheelController.isEnabled()) {
                turretTracker.update();
            }
            flywheelController.update();

            drawRobot(panelsField, follower.getPose());

            panelsTelemetry.debug("State", stateMachine.getState());
            panelsTelemetry.debug("Pose X", follower.getPose().getX());
            panelsTelemetry.debug("Pose Y", follower.getPose().getY());
            panelsTelemetry.debug("Heading", follower.getPose().getHeading());

            panelsTelemetry.update(telemetry);

            telemetry.addData("STATE", stateMachine.getState());
            telemetry.addData("X POS", follower.getPose().getX());
            telemetry.addData("Y POS", follower.getPose().getY());
            telemetry.addData("HEADING", follower.getPose().getHeading());
            telemetry.update();
        }
    }

    private void drawRobot(FieldManager field, Pose pose) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }

        Style robotLook = new Style("", "#3F51B5", 0.75);
        field.setStyle(robotLook);
        field.moveCursor(pose.getX(), pose.getY());
        field.circle(9);

        double headingX = Math.cos(pose.getHeading());
        double headingY = Math.sin(pose.getHeading());
        double x1 = pose.getX() + headingX * 4.5;
        double y1 = pose.getY() + headingY * 4.5;
        double x2 = pose.getX() + headingX * 9;
        double y2 = pose.getY() + headingY * 9;

        field.setStyle(robotLook);
        field.moveCursor(x1, y1);
        field.line(x2, y2);
        field.update();
    }
}