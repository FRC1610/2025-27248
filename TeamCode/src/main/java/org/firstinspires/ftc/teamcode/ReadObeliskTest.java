package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.ReadObelisk;

/**
 * TeleOp helper to exercise the ReadObelisk helper in isolation. Drives the turret toward
 * the appropriate alliance sweep limit, reports progress over telemetry, and caches any
 * detected obelisk pattern for reuse in other modes.
 */
@TeleOp(name = "Read Obelisk Test", group = "Test")
public class ReadObeliskTest extends LinearOpMode {

    private RobotHardware robot = new RobotHardware(this);
    private ReadObelisk obeliskReader = new ReadObelisk(robot, this);

    @Override
    public void runOpMode() {
        robot.init();

        waitForStart();

        if (robot.limelight == null || robot.turret == null) {
            telemetry.addLine("ERROR: Limelight and turret must be initialized for obelisk testing.");
            telemetry.update();
            return;
        }

        // Ensure the Limelight is on the obelisk-reading pipeline.
        robot.limelight.pipelineSwitch(1);

        boolean allianceRed = robot.refreshAllianceFromSwitchState();
        int sweepTarget = allianceRed ? Constants.turret_OBELISK_LEFT_LIMIT : Constants.turret_OBELISK_RIGHT_LIMIT;
        String sweepDirection = allianceRed ? "LEFT (negative)" : "RIGHT (positive)";

        DcMotorEx turret = robot.turret;
        ReadObelisk.ObeliskPattern detectedPattern = ReadObelisk.getCachedPattern();
        boolean targetFound = detectedPattern != null;

        // Sweep toward the appropriate alliance limit while checking for the tag.
        turret.setTargetPosition(sweepTarget);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0.35);

        while (opModeIsActive() && turret.isBusy() && !targetFound) {
            detectedPattern = obeliskReader.decodeLatestAndCache();
            targetFound = detectedPattern != null;

            telemetry.addData("Alliance Color", allianceRed ? "RED" : "BLUE");
            telemetry.addData("Turret Sweep", sweepDirection);
            telemetry.addData("Turret Position", turret.getCurrentPosition());
            telemetry.addData("Target Found", targetFound);
            if (targetFound) {
                telemetry.addData("Pattern", detectedPattern);
            }
            telemetry.update();
            idle();
        }

        // One last decode in case the tag appeared as motion stopped.
        if (!targetFound) {
            detectedPattern = obeliskReader.decodeLatestAndCache();
            targetFound = detectedPattern != null;
        }

        // Return the turret to home for the next mode.
        turret.setTargetPosition(Constants.turretHome);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0.35);

        while (opModeIsActive() && turret.isBusy()) {
            telemetry.addData("Alliance Color", allianceRed ? "RED" : "BLUE");
            telemetry.addData("Turret Sweep", "Returning to home");
            telemetry.addData("Turret Position", turret.getCurrentPosition());
            telemetry.addData("Target Found", targetFound);
            if (targetFound) {
                telemetry.addData("Pattern", detectedPattern);
            }
            telemetry.update();
            idle();
        }

        telemetry.addData("Alliance Color", allianceRed ? "RED" : "BLUE");
        telemetry.addData("Turret Sweep", "Complete");
        telemetry.addData("Turret Position", turret.getCurrentPosition());
        telemetry.addData("Target Found", targetFound);
        if (targetFound) {
            telemetry.addData("Pattern", detectedPattern);
        }
        telemetry.update();
    }
}

