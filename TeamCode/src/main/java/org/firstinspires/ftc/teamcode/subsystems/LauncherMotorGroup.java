package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class LauncherMotorGroup {
    private TelemetryManager telemetryPanels;

    public DcMotorExGroup group;

    private double lastLauncherBaseP = Double.NaN;
    private double lastLauncherBaseI = Double.NaN;
    private double lastLauncherBaseD = Double.NaN;
    private double lastLauncherBaseF = Double.NaN;
    private double lastLauncherScaledP = Double.NaN;
    private double lastLauncherScaledF = Double.NaN;

    public LauncherMotorGroup(Telemetry telemetry, TelemetryManager telemetryPanels, DcMotorEx launcher1, DcMotorEx launcher2) {
        if (launcher1 == null || launcher2 == null) {
            telemetry.addLine("ERROR: launcher motor is NULL!");
            return;
        }

        launcher1.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher2.setDirection(DcMotorSimple.Direction.REVERSE);

        this.telemetryPanels = telemetryPanels;
        this.group = new DcMotorExGroup(launcher1, launcher2);

        this.group.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.group.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.group.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.group.setTargetPositionTolerance(25);
    }

    public void applyLauncherPIDFTuning() {
        double gearScaledP = FlywheelPidfConfig.launcherP * Constants.LAUNCHER_GEAR_REDUCTION;
        double gearScaledI = FlywheelPidfConfig.launcherI * Constants.LAUNCHER_GEAR_REDUCTION;
        double gearScaledD = FlywheelPidfConfig.launcherD * Constants.LAUNCHER_GEAR_REDUCTION;
        double gearScaledF = FlywheelPidfConfig.launcherF * Constants.LAUNCHER_GEAR_REDUCTION;

        PIDFCoefficients pidf = new PIDFCoefficients(gearScaledP, gearScaledI, gearScaledD, gearScaledF);

        group.setVelocityPIDFCoefficients(pidf.p, pidf.i, pidf.d, pidf.f);

        lastLauncherBaseP = FlywheelPidfConfig.launcherP;
        lastLauncherBaseI = FlywheelPidfConfig.launcherI;
        lastLauncherBaseD = FlywheelPidfConfig.launcherD;
        lastLauncherBaseF = FlywheelPidfConfig.launcherF;
        lastLauncherScaledP = pidf.p;
        lastLauncherScaledF = pidf.f;

        publishLauncherPIDFTelemetry();
    }

    public void refreshLauncherPIDFFromConfig() {
        boolean baseChanged = FlywheelPidfConfig.launcherP != lastLauncherBaseP
                || FlywheelPidfConfig.launcherI != lastLauncherBaseI
                || FlywheelPidfConfig.launcherD != lastLauncherBaseD
                || FlywheelPidfConfig.launcherF != lastLauncherBaseF;

        if (baseChanged || !Double.isFinite(lastLauncherScaledP) || !Double.isFinite(lastLauncherScaledF)) {
            applyLauncherPIDFTuning();
            return;
        }

        publishLauncherPIDFTelemetry();
    }

    @SuppressLint("DefaultLocale")
    private void publishLauncherPIDFTelemetry() {
        if (telemetryPanels == null || !Double.isFinite(lastLauncherScaledP) || !Double.isFinite(lastLauncherScaledF)) {
            return;
        }
        telemetryPanels.addLine("--- PIDF ---");
        telemetryPanels.debug("Launcher PIDF base (P,I,D,F)",
                String.format("P=%.3f I=%.3f D=%.3f F=%.3f",
                        lastLauncherBaseP, lastLauncherBaseI, lastLauncherBaseD, lastLauncherBaseF));
        telemetryPanels.addLine("----------------");
    }
}