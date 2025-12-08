package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.concurrent.atomic.AtomicBoolean;

import org.firstinspires.ftc.teamcode.RobotHardware;
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

        boolean backPressedLast = false;
        AtomicBoolean scanEnabled = new AtomicBoolean(false);
        boolean scanning = false;
        ReadObelisk.ObeliskPattern detectedPattern = ReadObelisk.getCachedPattern();

        while (opModeIsActive()) {
            boolean backPressed = gamepad2.back;
            if (backPressed && !backPressedLast) {
                scanEnabled.set(!scanEnabled.get());
            }
            backPressedLast = backPressed;

            if (scanEnabled.get() && !scanning) {
                scanning = true;
                ReadObelisk.ObeliskPattern pattern = obeliskReader.scanForPattern(
                        () -> !opModeIsActive() || !scanEnabled.get());
                if (pattern != null) {
                    detectedPattern = pattern;
                }
                scanning = false;
                scanEnabled.set(false);
            }

            boolean allianceColor = robot.refreshAllianceFromSwitchState();
            String sweepDirection = allianceColor ? "LEFT (negative)" : "RIGHT (positive)";

            telemetry.addData("Alliance Color", allianceColor ? "RED" : "BLUE");
            telemetry.addData("Turret Sweep", scanning ? sweepDirection : "Idle");
            telemetry.addData("Turret Position", robot.turret.getCurrentPosition());
            telemetry.addData("Target Found", detectedPattern != null);
            if (detectedPattern != null) {
                telemetry.addData("Pattern", detectedPattern);
            }
            telemetry.update();

            idle();
        }
    }
}

