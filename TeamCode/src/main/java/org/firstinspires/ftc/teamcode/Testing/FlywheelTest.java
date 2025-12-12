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
 * Standalone flywheel testing OpMode that mirrors the flywheel controller logic
 * while allowing PIDF tuning from Panels without scaling.
 *
 * Button mapping:
 * - A: toggle the flywheel on/off.
 * - X/Y/B: switch flywheel RPM setpoints between 2000/2200/2600.
 */
@TeleOp(name = "Flywheel Test", group = "Test")
public class FlywheelTest extends LinearOpMode {

    private static final double TICKS_PER_REV = 28.0;
    private static final double MOTOR_TO_FLYWHEEL_RATIO = 24.0 / 16.0;

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

        if (robot.launcher == null) {
            telemetry.addLine("ERROR: launcher motor is NULL!");
            telemetry.update();
            return;
        }

        boolean lastA = false;
        boolean lastX = false;
        boolean lastY = false;
        boolean lastB = false;

        while (opModeIsActive()) {
            applyPanelsPidfIfChanged();

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

            telemetry.addData("Flywheel Enabled", flywheelEnabled);
            telemetry.addData("Flywheel Target RPM", "%.0f", flywheelEnabled ? rpmSetpoint : 0.0);
            telemetry.addData("Flywheel Current RPM", "%.0f", getCurrentRpm());
            telemetry.update();

            robot.flushPanelsTelemetry(telemetry);

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

        double ticksPerSecond = rpmToMotorTicksPerSecond(rpmSetpoint);

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

    private double rpmToMotorTicksPerSecond(double flywheelRpm) {
        double motorRpm = flywheelRpm / MOTOR_TO_FLYWHEEL_RATIO;
        return (motorRpm * TICKS_PER_REV) / 60.0;
    }

    private double getCurrentRpm() {
        DcMotorEx launcherMotor = robot.launcher;
        if (launcherMotor == null) {
            return 0.0;
        }

        double motorRpm = (launcherMotor.getVelocity() * 60.0) / TICKS_PER_REV;
        return motorRpm * MOTOR_TO_FLYWHEEL_RATIO;
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
