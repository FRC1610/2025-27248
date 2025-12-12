package org.firstinspires.ftc.teamcode.autonomous;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.StateMachine;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.subsystems.ShootingController;
import org.firstinspires.ftc.teamcode.subsystems.TurretTracker;

@Autonomous(name = "Auto Near", group = "Auto V1")
public class Near extends LinearOpMode {
    RobotHardware hardware = new RobotHardware(this);

    TelemetryManager panelsTelemetry;
    FieldManager panelsField;

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
        Drawer drawer = new Drawer(panelsField, follower);

        stateMachine.init();

        stateMachine.setState(StateMachine.State.AUTO_HOME_NEAR, true);

        drawer.draw();

        waitForStart();
        stateMachine.setState(StateMachine.State.AUTO_NEAR);

        while (opModeIsActive()) {
            stateMachine.update();
            follower.update();
            drawer.draw();

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
}