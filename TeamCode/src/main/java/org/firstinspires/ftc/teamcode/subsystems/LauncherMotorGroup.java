package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class LauncherMotorGroup {
    private DcMotorEx launcher1;
    private DcMotorEx launcher2;
    private TelemetryManager panels;
    private Telemetry telemetry;

    private double lastLauncherBaseP = Double.NaN;
    private double lastLauncherBaseI = Double.NaN;
    private double lastLauncherBaseD = Double.NaN;
    private double lastLauncherBaseF = Double.NaN;
    private double lastLauncherScaledP = Double.NaN;
    private double lastLauncherScaledI = Double.NaN;
    private double lastLauncherScaledD = Double.NaN;
    private double lastLauncherScaledF = Double.NaN;

    public LauncherMotorGroup(DcMotorEx launcher1, DcMotorEx launcher2, Telemetry telemetry, TelemetryManager panels) {
        if (launcher1 == null || launcher2 == null) {
            telemetry.addLine("ERROR: launcher motor is NULL!");
            return;
        }

        this.telemetry = telemetry;
        this.panels = panels;

        this.launcher1 = launcher1;
        launcher1.setDirection(DcMotor.Direction.FORWARD);
        setLauncherSettings(launcher1);
        this.launcher2 = launcher2;
        launcher2.setDirection(DcMotorSimple.Direction.REVERSE);
        setLauncherSettings(launcher2);
    }

    private void setLauncherSettings(DcMotorEx launcher) {
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setTargetPositionTolerance(25);
    }

    public void setPower(double power) {
        launcher1.setPower(power);
        launcher2.setPower(power);
    }

    public double getPower() {
        return (launcher1.getPower() + launcher2.getPower())/2;
    }

    public void setMode(DcMotor.RunMode mode) {
        launcher1.setMode(mode);
        launcher2.setMode(mode);
    }

    public void setVelocity(double velocity) {
        launcher1.setVelocity(velocity);
        launcher2.setVelocity(velocity);
    }

    public double getVelocity() {
        return (launcher1.getVelocity() + launcher2.getVelocity())/2;
    }

    public void applyLauncherPidfTuning() {
        double gearScaledP = FlywheelPidfConfig.launcherP * Constants.LAUNCHER_GEAR_REDUCTION;
        double gearScaledI = FlywheelPidfConfig.launcherI * Constants.LAUNCHER_GEAR_REDUCTION;
        double gearScaledD = FlywheelPidfConfig.launcherD * Constants.LAUNCHER_GEAR_REDUCTION;
        double gearScaledF = FlywheelPidfConfig.launcherF * Constants.LAUNCHER_GEAR_REDUCTION;

        PIDFCoefficients pidf = new PIDFCoefficients(gearScaledP, gearScaledI, gearScaledD, gearScaledF);

        launcher1.setVelocityPIDFCoefficients(pidf.p, pidf.i, pidf.d, pidf.f);
        launcher2.setVelocityPIDFCoefficients(pidf.p, pidf.i, pidf.d, pidf.f);

        lastLauncherBaseP = FlywheelPidfConfig.launcherP;
        lastLauncherBaseI = FlywheelPidfConfig.launcherI;
        lastLauncherBaseD = FlywheelPidfConfig.launcherD;
        lastLauncherBaseF = FlywheelPidfConfig.launcherF;
        lastLauncherScaledP = pidf.p;
        lastLauncherScaledI = pidf.i;
        lastLauncherScaledD = pidf.d;
        lastLauncherScaledF = pidf.f;

        publishLauncherPidfTelemetry();
    }

    public void refreshLauncherPidfFromConfig() {
        boolean baseChanged = FlywheelPidfConfig.launcherP != lastLauncherBaseP
                || FlywheelPidfConfig.launcherI != lastLauncherBaseI
                || FlywheelPidfConfig.launcherD != lastLauncherBaseD
                || FlywheelPidfConfig.launcherF != lastLauncherBaseF;

        if (baseChanged || !Double.isFinite(lastLauncherScaledP) || !Double.isFinite(lastLauncherScaledF)) {
            applyLauncherPidfTuning();
            return;
        }

        publishLauncherPidfTelemetry();
    }

    private void publishLauncherPidfTelemetry() {
        if (panels == null || !Double.isFinite(lastLauncherScaledP) || !Double.isFinite(lastLauncherScaledF)) { return; }

        panels.debug("Launcher PIDF scaled (P,I,D,F)",
                String.format("P=%.3f I=%.3f D=%.3f F=%.3f",
                        lastLauncherScaledP, lastLauncherScaledI, lastLauncherScaledD, lastLauncherScaledF));
        panels.debug("Launcher PIDF base (P,I,D,F)",
                String.format("P=%.3f I=%.3f D=%.3f F=%.3f",
                        lastLauncherBaseP, lastLauncherBaseI, lastLauncherBaseD, lastLauncherBaseF));
    }
}
