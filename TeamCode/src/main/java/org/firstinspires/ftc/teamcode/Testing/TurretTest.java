package org.firstinspires.ftc.teamcode.Testing;

import com.bylazar.configurables.PanelsConfigurables;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelPidfConfig;

/**
 * Combined turret positioning and flywheel RPM testing OpMode.
 * <p>
 * Button mapping:
 * - D-pad left/right: nudge turret target position by +/-25 ticks.
 * - A: toggle the flywheel on/off.
 * - X/Y/B: switch flywheel RPM setpoints between 2000/2200/2600.
 */
@TeleOp(name = "Turret Test", group = "Test")
public class TurretTest extends LinearOpMode {

    private static final double TICKS_PER_REV = 28.0;

    private static final double RPM_LOW = 2000;
    private static final double RPM_MID = 2200;
    private static final double RPM_HIGH = 2600;

    private final RobotHardware robot = new RobotHardware(this);

    private boolean flywheelEnabled = false;
    private double rpmSetpoint = RPM_LOW;

    private double lastPidfP = Double.NaN;
    private double lastPidfI = Double.NaN;
    private double lastPidfD = Double.NaN;
    private double lastPidfF = Double.NaN;

    @Override
    public void runOpMode() {
        robot.init();

        applyPanelsPidfIfChanged();

        waitForStart();

        if (robot.turret == null) {
            telemetry.addLine("ERROR: turret motor is NULL!");
            telemetry.update();
            return;
        }

        boolean lastLeft = false;
        boolean lastRight = false;
        boolean lastA = false;
        boolean lastX = false;
        boolean lastY = false;
        boolean lastB = false;

        int turretTarget = robot.turret.getCurrentPosition();

        while (opModeIsActive()) {
            applyPanelsPidfIfChanged();

            boolean leftPressed = gamepad1.dpad_left;
            boolean rightPressed = gamepad1.dpad_right;

            if (rightPressed && !lastRight) {
                turretTarget += 25;
            }

            if (leftPressed && !lastLeft) {
                turretTarget -= 25;
            }

            robot.turret.setTargetPosition(turretTarget);
            robot.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.turret.setPower(0.40);

            if (gamepad1.a && !lastA) {
                flywheelEnabled = !flywheelEnabled;
            }

            if (gamepad1.x && !lastX) {
                rpmSetpoint = RPM_LOW;
            }

            if (gamepad1.y && !lastY) {
                rpmSetpoint = RPM_MID;
            }

            if (gamepad1.b && !lastB) {
                rpmSetpoint = RPM_HIGH;
            }

            updateFlywheel();

            telemetry.addData("Turret Target", turretTarget);
            telemetry.addData("Turret Position", robot.turret.getCurrentPosition());
            telemetry.addData("Flywheel Enabled", flywheelEnabled);
            telemetry.addData("Flywheel Target RPM", "%.0f", flywheelEnabled ? rpmSetpoint : 0.0);
            telemetry.addData("Flywheel Current RPM", "%.0f", getCurrentRpm());
            telemetry.update();

            robot.flushPanelsTelemetry(telemetry);

            lastLeft = leftPressed;
            lastRight = rightPressed;
            lastA = gamepad1.a;
            lastX = gamepad1.x;
            lastY = gamepad1.y;
            lastB = gamepad1.b;

            idle();
        }
    }

    private void updateFlywheel() {
        if (!flywheelEnabled) {
            stopFlywheel();
            return;
        }

        double ticksPerSecond = rpmToTicksPerSecond(rpmSetpoint);

        if (robot.launcher != null) {
            robot.launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.launcher.setVelocity(ticksPerSecond);
        }

        if (robot.launcher2 != null) {
            robot.launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.launcher2.setVelocity(ticksPerSecond);
        }

        if (robot.getPanelsTelemetry() != null) {
            robot.getPanelsTelemetry().debug(
                    "Flywheel RPM (target/current)",
                    String.format("%.0f / %.0f", rpmSetpoint, getCurrentRpm()));
        }
    }

    private void stopFlywheel() {
        if (robot.launcher != null) {
            robot.launcher.setVelocity(0);
            robot.launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (robot.launcher2 != null) {
            robot.launcher2.setVelocity(0);
            robot.launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (robot.getPanelsTelemetry() != null) {
            robot.getPanelsTelemetry().debug("Flywheel RPM (target/current)", "0 / 0");
        }
    }

    private double rpmToTicksPerSecond(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }

    private double getCurrentRpm() {
        DcMotorEx launcherMotor = robot.launcher;
        if (launcherMotor == null) {
            return 0.0;
        }

        return (launcherMotor.getVelocity() * 60.0) / TICKS_PER_REV;
    }

    /**
     * Pull the latest PIDF values from Panels and apply them directly (no scaling) to both flywheel motors.
     */
    private void applyPanelsPidfIfChanged() {
        PanelsConfigurables.INSTANCE.refreshClass(FlywheelPidfConfig.class);

        double p = FlywheelPidfConfig.launcherP;
        double i = FlywheelPidfConfig.launcherI;
        double d = FlywheelPidfConfig.launcherD;
        double f = FlywheelPidfConfig.launcherF;

        boolean changed = p != lastPidfP || i != lastPidfI || d != lastPidfD || f != lastPidfF;
        if (!changed) {
            return;
        }

        PIDFCoefficients pidf = new PIDFCoefficients(p, i, d, f);

        if (robot.launcher != null) {
            robot.launcher.setVelocityPIDFCoefficients(pidf.p, pidf.i, pidf.d, pidf.f);
        }

        if (robot.launcher2 != null) {
            robot.launcher2.setVelocityPIDFCoefficients(pidf.p, pidf.i, pidf.d, pidf.f);
        }

        if (robot.getPanelsTelemetry() != null) {
            robot.getPanelsTelemetry().debug(
                    "Launcher PIDF (P,I,D,F)",
                    String.format("P=%.3f I=%.3f D=%.3f F=%.3f", pidf.p, pidf.i, pidf.d, pidf.f));
        }

        lastPidfP = p;
        lastPidfI = i;
        lastPidfD = d;
        lastPidfF = f;
    }
}
